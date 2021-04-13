# mcp23x17 component
This repo contains a application interface for the mcp23x17 board. This driver can be used in the esp idf envoirement.

## Component uitleg (NL)
Dit component is een driver / api voor het mcp23x17 GPIO board. Er kan naar registers geschreven worden met deze driver. Ook is het mogelijk om een polling task te starten die naar een bepaalde configuratie kijkt of er een software interrupt vereist is. Tot slot is het mogelijk om een metronoom aan te zetten. Deze zal telkens het register bit shiften om een bepaalde tijd. Deze tijd is aangegeven in BPM. In de header staat uitgebreid beschreven hoe de functies en structs werken. Het gehele component is geschreven door: Stijn Lemm.

## Component achtergrond (NL)
Dit component komt uit het project: SmArtist. Dit component diende voor het weergeven van de metronoom. Echter kan deze driver een stuk meer, wat niet in het project wordt gebruikt.
