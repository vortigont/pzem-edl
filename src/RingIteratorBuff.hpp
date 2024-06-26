#pragma once

/*
  I need c++14 for iterator class. Default toolchain flags for esp32 idf < 2.0.0 is still c++11
  could be tweaked via simple build flags while I'm waiting for an inspiration for a suitable workaround
*/
#if __cplusplus < 201402L  // less than c++14
	#error "Sorry, but this lib requires c++14 compatible compiler to build"
	#error "check supplied examples on how to add proper build flags for platformio"
	#include "no_c++14"
#endif

#include <cstdlib>
#include <list>

// PSRAM support
#include <esp_heap_caps.h>

#include "esp_idf_version.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
	#include <esp32/spiram.h>
#else
	#include <esp_spiram.h>	 // for older IDF core
#endif
// #include "psalloc.hpp"

#include "pzem_modbus.hpp"

// forward declarations
template <typename T>
class RingBuff;
template <class T>
class AveragingFunction;


	}
	auto rend() {
		return Iterator(this, 1);
	}
};

// Unary predicate for ID match
template <class T>
class MatchID : public std::unary_function<T, bool> {
	uint8_t _id;

   public:
	explicit MatchID(uint8_t id)
		: _id(id) {
	}
	bool operator()(T val) {
		// T is a shared_ptr
		return val->id == _id;
	}
};




