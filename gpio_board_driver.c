#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "gpio_board_driver.h"

#include "s_smbus.h"
#include "smbus.h"

#include <stdio.h>
#include <string.h>

#ifndef BIT
#define BIT(x) (0x01 << x)
#endif

static const char * TAG = "gpio_driver";

TaskHandle_t task_handle;

mcp23x17_t * mcp23x17_info_malloc(void)
{
    mcp23x17_t * mcp23x17_info = malloc(sizeof(*mcp23x17_info));
    if (mcp23x17_info != NULL)
    {
        memset(mcp23x17_info, 0, sizeof(*mcp23x17_info));
        ESP_LOGD(TAG, "malloc rotairy %p", mcp23x17_info);
    }
    else
    {
        ESP_LOGE(TAG, "malloc rotairy failed");
    }
    return mcp23x17_info;
}

gpio_board_interrupt_cfg_t * gpio_board_interrupt_cfg_malloc(void)
{
    gpio_board_interrupt_cfg_t * gpio_board_interrupt_cfg = malloc(sizeof(*gpio_board_interrupt_cfg));
    if (gpio_board_interrupt_cfg != NULL)
    {
        memset(gpio_board_interrupt_cfg, 0, sizeof(*gpio_board_interrupt_cfg));
        ESP_LOGD(TAG, "malloc rotairy %p", gpio_board_interrupt_cfg);
    }
    else
    {
        ESP_LOGE(TAG, "malloc rotairy failed");
    }
    return gpio_board_interrupt_cfg;
}

esp_err_t mcp23x17_read_reg(mcp23x17_t * mcp23x17_info, 
                                uint8_t command, 
                                uint8_t * data)
{
    return s_smbus_read_byte(mcp23x17_info->smbus_info, 
                                mcp23x17_info->general_i2c_xMutex, 
                                command, 
                                data);
}

esp_err_t mcp23x17_write_reg(mcp23x17_t * mcp23x17_info, 
                                uint8_t command, 
                                uint8_t data)
{
    return s_smbus_write_byte(mcp23x17_info->smbus_info, 
                                mcp23x17_info->general_i2c_xMutex, 
                                command, 
                                data);
}

esp_err_t mcp23x17_toggle_pin(mcp23x17_t * mcp23x17_info, 
                                uint8_t pin, 
                                uint8_t reg, 
                                bool value){
    uint8_t io_dir_reg;
    uint8_t gpio_reg;
    uint8_t io_dir_reg_value;
    uint8_t gpio_reg_value;
    
    if(reg == REG_GPIOA){
        io_dir_reg = REG_IODIRA;
        gpio_reg = REG_GPIOA;
    } else if (reg == REG_GPIOB){
        io_dir_reg = REG_IODIRB;
        gpio_reg = REG_GPIOB;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    // reads DDR of PORT.
    esp_err_t error = s_smbus_read_byte(mcp23x17_info->smbus_info, 
                                        mcp23x17_info->general_i2c_xMutex, 
                                        io_dir_reg, &io_dir_reg_value);

    if(error == ESP_FAIL){
        ESP_LOGE(TAG, "[*] error reading: %d", io_dir_reg);
        return ESP_FAIL;
    }

    // sbi of pin.
    uint8_t pin_mask = BIT(pin);

    bool pin_dir = ((io_dir_reg_value & pin_mask) != 0);

    ESP_LOGI(TAG, "[1] previous pin dir: %d", pin_dir);

    // if needed: set the DDR to right value.
    if(pin_dir){
        uint8_t new_io_dir_reg = io_dir_reg & ~pin_mask;
        error = s_smbus_write_byte(mcp23x17_info->smbus_info, 
                                    mcp23x17_info->general_i2c_xMutex, 
                                    io_dir_reg, new_io_dir_reg);
    }

    if(error == ESP_FAIL){
        ESP_LOGE(TAG, "[*] error writing: %d", io_dir_reg);
        return ESP_FAIL;
    }

    // read gpio values of PORT.
    error = s_smbus_read_byte(mcp23x17_info->smbus_info, 
                                mcp23x17_info->general_i2c_xMutex, 
                                gpio_reg, 
                                &gpio_reg_value);

    ESP_LOGI(TAG, "[2] read gpio value: %d", gpio_reg_value);

    // check desired pin current value
    bool cur_gpio_pin_value = ((gpio_reg_value & pin_mask )!= 0);

    ESP_LOGI(TAG, "[3] previous pin val: %d", cur_gpio_pin_value);

    // if the current value equals the desired value, it will return.
    if(cur_gpio_pin_value == value){
        ESP_LOGE(TAG, "[*] pin value already set: %d", cur_gpio_pin_value);
        return ESP_OK;
    }

    uint8_t new_gpio_reg_value;

    // edit our current gpio reg to desired grio reg.
    if(value){
        new_gpio_reg_value = gpio_reg_value | pin_mask;
    } else {
        new_gpio_reg_value = gpio_reg_value & ~pin_mask;
    }
    
    ESP_LOGI(TAG, "[4] going to write: %d", new_gpio_reg_value);
    
    // write new reg.
    return s_smbus_write_byte(mcp23x17_info->smbus_info, 
                                mcp23x17_info->general_i2c_xMutex, gpio_reg, 
                                new_gpio_reg_value);
}

void gpio_driver_event_task(void * pvParameters){
    // desired configuration for our task.
    gpio_board_interrupt_cfg_t * cfg = (gpio_board_interrupt_cfg_t *)pvParameters;

    if(cfg == NULL){
        vTaskDelete(NULL);
    }

    esp_err_t error;
    mcp23x17_t * gpio_board_info = cfg->gpio_board_info;
    uint8_t cfg_reg_a = cfg->cfg_reg_a;
    uint8_t cfg_reg_b = cfg->cfg_reg_b;
    uint8_t event_trigger_reg_a = cfg->event_trigger_reg_a;
    uint8_t event_trigger_reg_b = cfg->event_trigger_reg_b;
    
    // sets io dir if needed PORTA.
    if(cfg_reg_a > 0){
        uint8_t curr_io_dir_reg;
        error = mcp23x17_read_reg(gpio_board_info, REG_IODIRA, &curr_io_dir_reg);

        if(error == ESP_FAIL){
            vTaskDelete(NULL);
        }

        mcp23x17_write_reg(gpio_board_info, REG_IODIRA, (curr_io_dir_reg | cfg_reg_a));
    }


    // sets io dir if needed PORTB.
    if(cfg_reg_b > 0){
        uint8_t curr_io_dir_reg;
        error = mcp23x17_read_reg(gpio_board_info, REG_IODIRA, &curr_io_dir_reg);

        if(error == ESP_FAIL){
            vTaskDelete(NULL);
        }

        mcp23x17_write_reg(gpio_board_info, REG_IODIRB, (curr_io_dir_reg | cfg_reg_b));
    }

    uint8_t prev_reg_a;
    uint8_t prev_reg_b;
    mcp23x17_read_reg(gpio_board_info, REG_GPIOA, &prev_reg_a);
    mcp23x17_read_reg(gpio_board_info, REG_GPIOB, &prev_reg_b);

    while(1){

        // for PORTA.
        if(cfg_reg_a > 0){

            uint8_t gpio_reg_a;
            error = mcp23x17_read_reg(gpio_board_info, REG_GPIOA, &gpio_reg_a);

            if(error == ESP_OK){
                for(int pin = 0; pin < 8; pin++){

                    uint8_t pin_mask = BIT(pin);
                    
                    if((prev_reg_a & pin_mask) == (gpio_reg_a & pin_mask)){
                        continue;
                    }

                    if(((cfg_reg_a & pin_mask) > 0) 
                        && ((event_trigger_reg_a & pin_mask) > 0) 
                        && ((gpio_reg_a & pin_mask) > 0)){
                        // event trigger when the pin has the desired (in cfg) values. 
                        cfg->event(pin, event_trigger_rising_edge, REG_GPIOA);

                    } else if((cfg_reg_a & pin_mask) > 0 
                        && (event_trigger_reg_a & pin_mask) == 0 
                        && ((gpio_reg_a & pin_mask) == 0)){

                        // event trigger when the pin has the desired (in cfg) values. 
                        cfg->event(pin, event_trigger_falling_edge, REG_GPIOA);
                    }
                }
            }
            if(prev_reg_a != gpio_reg_a){
                prev_reg_a = gpio_reg_a;
            }
        }
        // for PORTB.
        if(cfg_reg_b > 0){
            uint8_t gpio_reg_b;
            error = mcp23x17_read_reg(gpio_board_info, REG_GPIOB, &gpio_reg_b);

            if(error == ESP_OK){
                for(int pin = 0; pin < 8; pin++){

                    uint8_t pin_mask = BIT(pin);

                    if((prev_reg_b & pin_mask) == (gpio_reg_b & pin_mask)){
                        continue;
                    }

                    if(((cfg_reg_b & pin_mask) > 0) 
                        && ((event_trigger_reg_b & pin_mask) > 0) 
                        && ((gpio_reg_b & pin_mask) > 0)){

                        // event trigger when the pin has the desired (in cfg) values.
                        cfg->event(pin, event_trigger_rising_edge, REG_GPIOB);

                    } else if((cfg_reg_b & pin_mask) > 0 
                        && (event_trigger_reg_b & pin_mask) == 0
                        && ((gpio_reg_b & pin_mask) == 0)){

                        // event trigger when the pin has the desired (in cfg) values.
                        cfg->event(pin, event_trigger_falling_edge, REG_GPIOB);
                    }
                }
            }
            if(prev_reg_b != gpio_reg_b){
                prev_reg_b = gpio_reg_b;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(GPIO_DRIVER_EVENT_TASK_TIME_MS));
    }
    vTaskDelete(NULL);
}

mcp23x17_t * current_metronome_board;

void vTimer_callback( xTimerHandle xTimer ){

    uint8_t prev_reg_state;
    // read previous value.
    esp_err_t error = mcp23x17_read_reg(current_metronome_board, 
                                            REG_GPIOB, 
                                            &prev_reg_state);

    if(error == ESP_FAIL){
        ESP_LOGE(TAG, "metronome read failed");
        return;
    }

    // shift light.
    prev_reg_state <<= 1;

    // if register = 0 set it back to 1. 10000000 << 1 = 00000000 -> 00000001
    if(prev_reg_state == 0){
        prev_reg_state = 1;
    }

    // set edited value.
    error = mcp23x17_write_reg(current_metronome_board, REG_GPIOB, prev_reg_state);

    if(error == ESP_FAIL){
        ESP_LOGE(TAG, "metronome write failed");
    }
}

TimerHandle_t metronome;

esp_err_t gpio_board_driver_start_metronome(mcp23x17_t * mcp23x17_info ,
                                                uint8_t BPM){
    
    ESP_LOGI(TAG, "starting metronome bpm: %d", BPM);

    current_metronome_board = mcp23x17_info;

    // convert BPM to interval time in MS.
    double timer_ms = ((60.0 / BPM) * 1000);

    metronome = xTimerCreate("metronome", 
                                            pdMS_TO_TICKS(timer_ms), 
                                            pdTRUE, 
                                            (void *)0, 
                                            &vTimer_callback);

    // set all 8 pins from reg B to OUTPUT.
    esp_err_t error = mcp23x17_write_reg(current_metronome_board, 
                                            REG_IODIRB, 
                                            0);

    if(error == ESP_FAIL)
        return error;

    // set all 8 pins LOW.
    mcp23x17_write_reg(current_metronome_board, REG_GPIOB, 0);

    if(error == ESP_FAIL)
        return error;

    // start timer handle.
    if(xTimerStart(metronome, 0) != pdPASS){
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t gpio_board_driver_stop_metronome(){
    if(current_metronome_board != NULL){
        if(xTimerStop(metronome, 0) == pdTRUE){
            ESP_LOGI(TAG, "Stopped metronome");
            current_metronome_board = NULL;
            return ESP_OK;
        }
    }
    return ESP_FAIL;
}

esp_err_t gpio_board_driver_start_task(gpio_board_interrupt_cfg_t * cfg){
    if(task_handle == NULL){
        ESP_LOGI(TAG, "Starting task");
        if(xTaskCreate(&gpio_driver_event_task, 
                    "gpio_driver_event_task", 
                    4 * 1024, 
                    (void *)cfg, 
                    10, 
                    &task_handle) 
                    == pdPASS){
                        return ESP_OK;
                    }
    }
    return ESP_FAIL;
}

esp_err_t gpio_board_driver_stop_task(){
    if(task_handle != NULL){
        ESP_LOGI(TAG, "Stopping task");
        vTaskDelete(task_handle);
        task_handle = NULL;
        return ESP_OK;
    }
    return ESP_FAIL;
}