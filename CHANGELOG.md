# Change Log

## v 1.0.2 (2022-04-06)
 * fix stale data calculation
 * fix IDF build with timeseries feature for IDF 3/4 and arduino core 1.x and 2.x 

## v 1.0.1 (2022-01-30)
 - fix incorrect byte shift for 4-byte values
## v 1.0.0 (2021-12-28)
 - Abstracted Message Queue class
 - Add TimeSeries feature
   - RingBuffer class for storing data structs (support mem allocation in PSRAM)
   - Iterator class to traverse ring buffers
   - TimeSeries object and container for custom sets of TS data
 - a pool class could manage mixed pzem device types simultaneously
 - support for PZEM003
 - building under esp-idf
 - pzem_cli - added 'energy reset', 'alarm get/set' features
 - examples for Sigle/Multiple PZEMs
 - pzem_cli example
 - initial version