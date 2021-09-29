### External Libs
The only external lib used is [vortigont/LinkedList](https://github.com/vortigont/LinkedList#vortigont). It is already included into library manifest for Platformio, so should be picked up automatically.


### Build-time defines
It is possible to enable debugging output from the lib. To do so it is required to enable one (or both) of the following build-time flags:

__PZEM_EDL_DEBUG__    - enables pzem-edl lib debug output using ESP32 Core debug logging facilities

__CORE_DEBUG_LEVEL__=ARDUHAL_LOG_LEVEL_DEBUG  - must be set to raise ESP32 core debug level

or the same as

__CORE_DEBUG_LEVEL__=4

For Platformio project ini file this could be done in environment sections like this:

```[env:debug]
build_flags =
  -DCORE_DEBUG_LEVEL=ARDUHAL_LOG_LEVEL_DEBUG
  -DPZEM_EDL_DEBUG
```
