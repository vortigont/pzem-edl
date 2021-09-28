### External Libs
The only external lib used is [vortigont/LinkedList](https://github.com/vortigont/LinkedList#vortigont). It is already included into library manifest for Platformio, so should be picked up automatically.


### Build-time defines
It is possible to enable some debugging output from the lib. To do so it is required to enable some build-time flags:

*PZEM_EDL_DEBUG*    - enables pzem-edl lib debug output using ESP32 Core debug logging facilities
*CORE_DEBUG_LEVEL*=ARDUHAL_LOG_LEVEL_DEBUG  - must be set to raise ESP32 core debug level
or
*CORE_DEBUG_LEVEL*=4

For Platformio project ini file this could be done in environment sections like this:

```[env:debug]
build_flags =
  -DCORE_DEBUG_LEVEL=ARDUHAL_LOG_LEVEL_DEBUG
  -DPZEM_EDL_DEBUG
```
