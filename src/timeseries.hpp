/*
PZEM EDL - PZEM Event Driven Library

This code implements communication and data exchange with PZEM004T V3.0 module using MODBUS proto
and provides an API for energy metrics monitoring and data processing.

This file is part of the 'PZEM event-driven library' project.

Copyright (C) Emil Muratov, 2021
GitHub: https://github.com/vortigont/pzem-edl
*/

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
#include "esp_idf_version.h"
#include <esp_heap_caps.h>

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    #include "esp_psram.h"
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
    #include <esp32/spiram.h>
#else
    #include <esp_spiram.h>     // for older IDF core
#endif

// #include "psalloc.hpp"

#include "pzem_modbus.hpp"

// forward declarations
template <typename T>
class RingBuff;
template <class T>
class AveragingFunction;

#include "TS_RingIteratorBuff.hpp"
#include "TS_Average.hpp"


/**
 * @brief ring buffer container for time series data
 * derives from a RingBuff class
 *
 * @tparam T type of stored data
 * @param _s - container size (number of elemens stored)
 * @param start_time - atomic timestamps (an increasing counter)
 *                     used to track missed data, could be provided as any time marks, i.e. millis(), micros(), epoch timestamp, etc...
 * @param period     - time series period, in units of time.
 *                     any intermediate samples (less than period time after previous) are skipped. (should be averaged, but not implemented yet)
 */
template <typename T>
class TimeSeries : public RingBuff<T> {
	uint32_t							  tstamp;	 // last update timestamp mark
	uint32_t							  interval;	 // time interval between series
	const char*							  _descr;	 // Mnemonic name for the instance
	std::unique_ptr<AveragingFunction<T>> _avg;		 // averaging instance

   public:
	const uint8_t id;  // TimeSeries unique ID

	TimeSeries(uint8_t id, size_t s, uint32_t start_time, uint32_t inverval = 1, const char* name = NULL)
		: RingBuff<T>(s), tstamp(start_time), interval(inverval), _descr(name), id(id) {
	}
	// virtual ~TimeSeries(){};

	/**
	 * @brief reset buffer to empty state and set last update time mark
	 *
	 * @param t - current starting point (timestamp)
	 */
	void	 clear(uint32_t t);

	/**
	 * @brief put a new element into buffer
	 * time mark is checked to validate insertion period
	 * @param val - object of stored type T
	 * @param time - timestamp for current value
	 * timestamp rollover for uint32_t is handled properly as long as interval between timestamps is consistent
	 */
	void	 push(const T& val, uint32_t time);

	uint32_t getTstamp() const {
		return tstamp;
	}

	uint32_t getInterval() const {
		return interval;
	}

	const char* getDescr() const {
		return _descr;
	}

	// Setters
	void setInterval(uint32_t _interval, uint32_t newtime);

	void setAverager(std::unique_ptr<AveragingFunction<T>>&& rhs) {
		_avg = std::move(rhs);
	};
};



// 
//  ===== Implementation follows below =====

template <typename T>
void TimeSeries<T>::clear(uint32_t t) {
	tstamp = t;
	RingBuff<T>::clear();
	if (_avg) _avg->reset();
}

template <typename T>
void TimeSeries<T>::push(const T& val, uint32_t time) {
	uint32_t _t = time;

	time -= tstamp;	 // разница времени с прошлой выборкой

	// промежуточные выборки либо усредняем либо отбрасываем
	if (time < interval) {
		if (_avg) _avg->push(val);
		return;
	}

	if (time >= 2 * interval) {							// пропущено несколько выборок
		if (time / interval > RingBuff<T>::capacity) {	// пропустили выборок больше чем весь текущий буфер - сбрасываем всё
			clear(_t);
		} else {
			// заполняем пропуски последним известным значением, это неверные данные, но других всё равно нет
			do {
				RingBuff<T>::push_back(val);
				time -= interval;
			} while (time > interval);
		}
	}

	// if we have averaging - use it
	if (_avg && _avg->getCnt()) {
		_avg->push(val);
		RingBuff<T>::push_back(_avg->get());
		_avg->reset();
		_avg->push(val);
	} else
		RingBuff<T>::push_back(val);

	tstamp = _t;  // обновляем метку времени
}

template <typename T>
void TimeSeries<T>::setInterval(uint32_t _interval, uint32_t newtime) {
	if (interval > 0) {
		interval = _interval;
		clear(newtime);
	}
}





template <typename T>
class TSContainer {
   public:
	TSContainer<T>(){};
	//~TSContainer();

	/**
	 * @brief get a pointer to TS object with specified ID
	 *
	 * @param id
	 * @return TimeSeries<T>* - or nullptr if TS with specified ID does not exit
	 */
	const TimeSeries<T>* getTS(uint8_t id) const;

	/**
	 * @brief get a pointer to TS object with specified ID
	 * a cast wraper around const get()
	 * @param id
	 * @return TimeSeries<T>*  - or nullptr if TS with specified ID does not exit
	 */
	TimeSeries<T>*		 getTS(uint8_t id) {
		  return const_cast<TimeSeries<T>*>(const_cast<const TSContainer<T>*>(this)->getTS(id));
	};

	/**
	 * @brief add new TimeSeries object to the stack of series data
	 *
	 * @param s - number of entries to keep
	 * @param start_time - timestamp of TS creation
	 * @param period - sampling period, all samples that are pushed to TS with lesser interval will be either dropped or averaged if averaging function is provided
	 * @param descr - mnemonic description (pointer MUST be valid for the duraion of life-time, it won't be deep-copied)
	 * @param id - desired ID
	 * @return uint8_t - assigned ID, returns 0 if TS has failed, possibly due to requested ID already exist.
	 *                  If 0 is provided, then next available id will be autoassing
	 */
	uint8_t addTS(size_t s, uint32_t start_time, uint32_t period = 1, const char* descr = nullptr, uint8_t id = 0);

	/**
	 * @brief remove specific TS object
	 *
	 * @param id TimeSeries ID
	 */
	void	removeTS(uint8_t id) {
		   tschain.remove_if(MatchID<TimeSeries<T>>(id));
	}

	/**
	 * @brief destroy ALL TS's in chain and release memory
	 *
	 */
	void purge() {
		tschain.clear();
	};

	/**
	 * @brief clear data for the entire container
	 * all TS's are cleared without releasing memory, reseting it's size to 0
	 *
	 */
	void clear();

	/**
	 * @brief push new value to the TimeSeries chain
	 *
	 * @param val - value
	 * @param time - current timestamp
	 */
	void push(const T& val, uint32_t time);

	bool setTSinterval(uint8_t id, uint32_t _interval, uint32_t newtime);

	/**
	 * @brief Set the Averager object to process data between between time intervals
	 *
	 * @param id - id of an TS container
	 * @param rhs - rvalue for the unique pointer of the Averager instance, TS containder will own the object
	 */
	void setAverager(uint8_t id, std::unique_ptr<AveragingFunction<T>>&& rhs);

	/**
	 * @brief get TS size by id
	 * return current number of elements in TimeSeries object.
	 * it could be less than maximum capacity size depending on amount of samples being stored
	 * @param id - TS object id
	 * @return int number of elements
	 */
	int	 getTSsize(uint8_t id) const;

	/**
	 * @brief get total TS containner size
	 * return current number of elements in all TimeSeries objects.
	 * it could be less than maximum capacity size depending on amount of samples being stored
	 * @return int number of elements
	 */
	int	 getTSsize() const;

	/**
	 * @brief get TS capacity by id
	 * return max number of elements in TimeSeries object
	 * @param id - TS object id
	 * @return int number of elements
	 */
	int	 getTScap(uint8_t id) const;

	/**
	 * @brief get total TS container capacity
	 * return total max number of elements in TSContainer object
	 * @return int number of elements
	 */
	int	 getTScap() const;

	int	 getTScnt() const {
		 return tschain.size();
	};

   protected:
	std::list<std::shared_ptr<TimeSeries<T>>> tschain;	// time-series chain
};


template <typename T>
const TimeSeries<T>* TSContainer<T>::getTS(uint8_t id) const {
	if (!tschain.size())
		return nullptr;

	for (auto _i = tschain.begin(); _i != tschain.end(); ++_i)
		if (_i->get()->id == id)
			return _i->get();

	return nullptr;
}


template <typename T>
uint8_t TSContainer<T>::addTS(size_t s, uint32_t start_time, uint32_t period, const char* descr, uint8_t id) {
	if (id && getTS(id)) {	// check if provided id is already exist
		return 0;
	}

	if (!id) {	// if provided id is 0 - than find next free one
		const TimeSeries<T>* n;
		do {
			n = getTS(++id);
		} while (n && id);
		if (!id) return 0;
	}

	tschain.emplace_back(std::make_shared<TimeSeries<T>>(id, s, start_time, period, descr));
	if (period > 1)
		setAverager(id, std::make_unique<MeanAverage<T>>());
                // 2540 mod setAverager(id, std::make_unique<MeanAveragePZ004>());
	return id;
}

template <typename T>
bool TSContainer<T>::setTSinterval(uint8_t id, uint32_t _interval, uint32_t newtime) {
	auto ts = getTS(id);

	if (ts) ts->setInterval(_interval, newtime);

	return ts;
}

template <typename T>
void TSContainer<T>::clear() {
	for (auto i = tschain.begin(); i != tschain.end(); ++i) {
		i->get()->clear();
	}
}

template <typename T>
void TSContainer<T>::push(const T& val, uint32_t time) {
	for (auto i = tschain.begin(); i != tschain.end(); ++i)
		i->get()->push(val, time);
}

template <typename T>
int TSContainer<T>::getTSsize(uint8_t id) const {
	const auto ts = getTS(id);
	return ts ? ts->getSize() : 0;
}

template <typename T>
int TSContainer<T>::getTScap(uint8_t id) const {
	const auto ts = getTS(id);
	return ts ? ts->capacity : 0;
}

template <typename T>
int TSContainer<T>::getTSsize() const {
	int s = 0;
	for (auto i = tschain.cbegin(); i != tschain.cend(); ++i)
		s += i->get()->getSize();

	return s;
}

template <typename T>
int TSContainer<T>::getTScap() const {
	int s = 0;

	for (auto i = tschain.cbegin(); i != tschain.cend(); ++i)
		s += i->get()->capacity;

	return s;
}

template <typename T>
void TSContainer<T>::setAverager(uint8_t id, std::unique_ptr<AveragingFunction<T>>&& rhs) {
	auto ts = getTS(id);
	if (ts) ts->setAverager(std::move(rhs));
}



//////////////////////////////// add 2540 start

// template <>
// class TSContainer<pz003::metrics> {
//    public:
// 	TSContainer<pz003::metrics>(){};
// 	//~TSContainer();

// 	const TimeSeries<pz003::metrics>* getTS(uint8_t id) const;

// 	TimeSeries<pz003::metrics>*		 getTS(uint8_t id) {
// 		  return const_cast<TimeSeries<pz003::metrics>*>(const_cast<const TSContainer<pz003::metrics>*>(this)->getTS(id));
// 	};

// 	uint8_t addTS(size_t s, uint32_t start_time, uint32_t period = 1, const char* descr = nullptr, uint8_t id = 0);

// 	void	removeTS(uint8_t id) {
// 		   tschain.remove_if(MatchID<TimeSeries<pz003::metrics>>(id));
// 	}

// 	void purge() {
// 		tschain.clear();
// 	};

// 	void clear();

// 	void push(const pz003::metrics& val, uint32_t time);

// 	bool setTSinterval(uint8_t id, uint32_t _interval, uint32_t newtime);

// 	void setAverager(uint8_t id, std::unique_ptr<AveragingFunction<pz003::metrics>>&& rhs);

// 	int	 getTSsize(uint8_t id) const;

// 	int	 getTSsize() const;

// 	int	 getTScap(uint8_t id) const;

// 	int	 getTScap() const;

// 	int	 getTScnt() const {
// 		 return tschain.size();
// 	};

//    protected:
// 	std::list<std::shared_ptr<TimeSeries<pz003::metrics>>> tschain;	// time-series chain
// };

////////////////////////////// add 2540 end
