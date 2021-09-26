PZEM-EDL - PZEM event-driven library
======

__[BUILD](/docs/BUILD.md) | [EXAMPLES](/examples/README.md) | [CHANGELOG](/CHANGELOG.md) | [DOCS](/docs/README.md)__


This is an ESP32/Arduino framework library implementing communication interface for [PeaceFair](https://peacefair.aliexpress.com/store/1773456) **PZEM-004Tv3.0**/**PZEM-003** Energy monitor modules via MODBUS proto.

**_currently lib is in it's early develompent state, so expect bugs, issues and sporadic API changes any time_**

The main idea is that lib uses FreeRTOS features to implement non-blocking event-driven operation when exchanging messages via serial port. It is written to run multiple PZEM devices simultaneously sharing same uart port(s) without blocking on rx/tx operations and at the same time to keep a strict message exchange ordering to prevent transmit collisions.

Feel free to open a discussion topic on any questions.

### Main features
 * meant to run multiple PZEM devices, although works fine with a single device also
 * non-blocking code on PZEM request-reply exchange, UART runs in event-mode, no while() loops or polling on rx-read
 * no loop() hooks, loop blocking, etc... actually no loop-dependend code at all
 * background auto-polling driven by RTOS timers
 * event/callback API for user-code hooks
 * Class objects for managing single device/port instance
 * or Pool of multiple PZEM devices groupped on single/multiple Serial port(s)
 * [pzem_cli](/examples/pzem_cli) - a small sketch to interact with pzem via terminal cli


### Supported modules
 - PZEM-004Tv3.0 (same as PZEM-014/PZEM-016) AC 10/100 Amps meter
 - PZEM-003 (same as PZEM-017) DC 10/50/100/200/300 Amps meter (not tested)


### Limitations
 * ESP32 platform only
 * Hardware Serial only (not a problem for esp32, it has 3 of it with pin remapping)

### TODO list
 * Collector module
    * collecting series of metrics data
    * circular buffers
    * PSRAM support
    * time series
 * Export module
    * JSON
    * binary
    * csv
 * PZEM017 support


### How it works
A classic design for Arduino code work like this:
 1) user code asks the lib _"hey, send a message to PZEM and give me the reply"_
 2) lib replies - "OK pls, wait right here, I'm sending a message and awaiting responce", than it makes a req message and sends it to PZEM via uart
 3) than lib blocks the whole loop() code and starts listening for uart RX data awaiting PZEM to craft a responce and return back
 4) lib code waits and waits while data slowly, byte by byte, arrives via uart everytime checking if it has enough data to proceed with a message
 5) when there is enough bytes received lib decodes message and returns control to the user code
 6) worst case scenario - PZEM is not responding and the whole code is blocked untill timeout expires
 7) only now user code is able to process the reply and make a new request

An average RTT that takes PZEM to respond is about 70ms. Multiply this sequence by 3 (or more) devices at 1 second poll rate and it will turn into MCU have to spend 20-25% of it's time just sit and waiting for UART TX/RX.

Now with event-driven approach:
1) user code asks the lib _"hey, send a message to PZEM and give me the reply"_
2) lib replies immidiately _"hey, this will take too long to wait, go mind your own business, just leave me a hook and I will ping you, once reply received"_
3) then it crafts the packet and places it into the queue with an instruction like _"hey, RTOS, pls pipe this data over serial when available and ping me when you got some reply data, I will take a nap meanwhile :)"_
4) user-code: _"I know, it's too early, but here is another request for another PZEM..."_
5) lib: _"no problem, leave it here, I will send it once uart is free "_
5) than lib sleep waits until an event from uart comes that there is some data received
6) it retreives data from buffer and pings user-code - _"here is your data, pick it up anytime"_


### History
I needed to run 3 PZEM devices from one MCU for my [ESPEM](https://github.com/vortigont/espem) project and found that existing implementations are pretty poor on running multiple pzem devs, have issues with WiFi and blocking code. So deciced to make my own implementation along with learning some new features about RTOS and ESP IDF framework.
My sincere regards to the authors of original PZEM libs - [olehs](https://github.com/olehs/PZEM004T) and [mandulaj](https://github.com/mandulaj/PZEM-004T-v30). I've been using their libs for a long time and it inspired me to go on with a new design.

### License
Library code is available under [GNU General Public License v3.0](LICENSE)

### Links
[PZEM-004Tv3.0 datasheet](/docs/PZEM-004T-V3.0-Datasheet-User-Manual.pdf) - User manual, datasheet and MODBUS commands specs

[olehs](https://github.com/olehs/PZEM004T) - This is were it all started. An arduino lib for the legacy version of PZEM004t device

[mandulaj](https://github.com/mandulaj/PZEM-004T-v30) - Jakub's version of the library for newer PZEM-004T-v3.0 devices supporting MODBUS proto

[TheHWcave](https://github.com/TheHWcave/Peacefair-PZEM-004T-) - reverse-engineered schematics and interface software (python)

[zbx-sadman / zabbuino](https://github.com/zbx-sadman/zabbuino) - A Zabbix agent firmware for Arduino supporting PZEM meters

[PZEM on TASMOTA](https://tasmota.github.io/docs/PZEM-0XX/) - Tasmota has it's own embeded support for PZEM

[PZEM-017-v1](https://github.com/maxzerker/PZEM-017-v1) - lib for the DC version PZEM-017-v1