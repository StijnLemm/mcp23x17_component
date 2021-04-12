#ifndef MCP23X17_H
#define MCP23X17_H

#include "esp_log.h"
#include "esp_err.h"

#include "smbus.h"

#define REG_IODIRA   0x00
#define REG_IODIRB   0x01
#define REG_IPOLA    0x02
#define REG_IPOLB    0x03
#define REG_GPINTENA 0x04
#define REG_GPINTENB 0x05
#define REG_DEFVALA  0x06
#define REG_DEFVALB  0x07
#define REG_INTCONA  0x08
#define REG_INTCONB  0x09
#define REG_IOCON    0x0A
#define REG_GPPUA    0x0C
#define REG_GPPUB    0x0D
#define REG_INTFA    0x0E
#define REG_INTFB    0x0F
#define REG_INTCAPA  0x10
#define REG_INTCAPB  0x11
#define REG_GPIOA    0x12
#define REG_GPIOB    0x13
#define REG_OLATA    0x14
#define REG_OLATB    0x15

#define BIT_IOCON_INTPOL 1
#define BIT_IOCON_ODR    2
#define BIT_IOCON_HAEN   3
#define BIT_IOCON_DISSLW 4
#define BIT_IOCON_SEQOP  5
#define BIT_IOCON_MIRROR 6
#define BIT_IOCON_BANK   7

#define GPIO_DRIVER_EVENT_TASK_TIME_MS 50

typedef enum
{
    MCP23X17_GPIO_OUTPUT = 0,
    MCP23X17_GPIO_INPUT
} mcp23x17_gpio_mode_t;

typedef enum
{
    event_trigger_falling_edge = 0,
    event_trigger_rising_edge
} event_trigger_arg;

// I2C address.
#define MCP23X17_ADDR_BASE 0x20

typedef struct
{
    // i2c info.
    smbus_info_t * smbus_info;
    // general i2c mutex for mutual i2c exclusion.
    SemaphoreHandle_t * general_i2c_xMutex;
} mcp23x17_t;

typedef struct 
{
    // register representing pin INT. 1 = INT on, 0 = INT off.
    uint8_t cfg_reg_a;
    uint8_t cfg_reg_b;
    // register representing rising or falling. 1 = rising, 0 = falling.
    uint8_t event_trigger_reg_a;
    uint8_t event_trigger_reg_b;
    // event called when INT is triggered.
    void(*event)(uint8_t, uint8_t, uint8_t);
    // which board to poll.
    mcp23x17_t * gpio_board_info; 
} gpio_board_interrupt_cfg_t;

/**
 * @brief
 * allocates memory for a info struct.
 * @return
 * a pointer to some space what can be used to store info.
 */
mcp23x17_t * mcp23x17_info_malloc(void);
/**
 * @brief
 * allocates memory for a config struct.
 * @return
 * a pointer to some space what can be used to store info.
 */
gpio_board_interrupt_cfg_t * gpio_board_interrupt_cfg_malloc(void);
/**
 * @brief                                                       ^
 * Reads register from gpio board. Check register commands above|
 * @param[in] mcp23x17_info info for smbus and mutex.
 * @param[in] command which register to read.
 * @param[in, out] data returns a 8-bit int value representing aksed for reg.
 * @return ESP_OK when succeeded. ESP_FAIL when board can't be read.
 */
esp_err_t mcp23x17_read_reg(mcp23x17_t * mcp23x17_info, uint8_t command, uint8_t * data);
/**
 * @brief                                                      ^
 * Writes register to gpio board. Check register commands above|
 * @param[in] mcp23x17_info info for smbus and mutex.
 * @param[in] command which register to write to.
 * @param[in] data sends a 8-bit int value to desired register.
 * @return ESP_OK when succeeded. ESP_FAIL when board can't be written to.
 */
esp_err_t mcp23x17_write_reg(mcp23x17_t * mcp23x17_info, uint8_t command, uint8_t data);
/**
 * @brief
 * Function will set desired pin to desired state if its not already. Also, this Function
 * will set the pin direction of the desired pin if its still on input <- *WARING*                                             
 * @param[in] mcp23x17_info info for smbus and mutex.
 * @param[in] pin which pin to toggle.
 * @param[in] reg which of the two gpio registers to choose from.
 * @param[in] value set pin high = true, set pin low = false.
 * @return ESP_OK when succeeded. ESP_FAIL when board can't be written to.
 */
esp_err_t mcp23x17_toggle_pin(mcp23x17_t * mcp23x17_info, uint8_t pin, uint8_t reg, bool value);
/**
 * @brief
 * Function will start a polling task wich triggers events corresponding to the given config.
 * The config has a interrupt enable register and a rise or falling event. Depending on these
 * registers, the task will create events.
 * @param[in] cfg config for intterupt.
 * @return
 * ESP_OK when function completed succesfully. ESP_FAIL when 
 * the task is already running or when the task coun't be 
 * created by freeRTOS.
 */
esp_err_t gpio_board_driver_start_task(gpio_board_interrupt_cfg_t * cfg);
/**
 * @brief
 * Function will stop the task if running.
 * @return
 * ESP_OK when task is stopped, ESP_FAIL when no task was running.
 */
esp_err_t gpio_board_driver_stop_task();
/**
 * @brief
 * Function will start a metronome on PORTB
 * @param[in] mcp23x17_info info for smbus and mutex.
 * @param[in] BPM bpm of metronome.
 * @return ESP_OK when function completed succesfuly. ESP_FAIL when timer 
 * could not be created.
 */
esp_err_t gpio_board_driver_start_metronome(mcp23x17_t * mcp23x17_info ,uint8_t BPM);
/**
 * @brief
 * stop the running metronome.
 * @return ESP_OK when succesfuly stopped the running metronome.
 * ESP_FAIL when there is no metronome running or when freeRTOS can't
 * stop it.
 */
esp_err_t gpio_board_driver_stop_metronome();
#endif 