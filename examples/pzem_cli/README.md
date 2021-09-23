PZEM-EDL - CLI tool
======

A simple sketch that allows to interact with PZEM over serial console

provides the following features
 - PZEM metrics reading
 - read/change MODBUS address
 - read/change Power Alarm threshold
 - reset Energy counter


### Build and run
Build it with [platformio](https://platformio.org/), flash and connect to esp32 with any terminal (serial speed speed is 115200)
Hook up PZEM to serial pins and use text menu to interact with PZEM.

there are 3 env to build

```
pio run
```
will build usual version

```
pio run -e debug
```
will build version which dumps RX/TX packets to the terminal

```
pio run -e verbose
```
will build version which produces more verbose debug output
