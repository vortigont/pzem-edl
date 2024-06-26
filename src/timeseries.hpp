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

/**
 * @brief Iterator class to traverse RingBuffer data
 * Requires c++14 to build
 *
 * @tparam T - RingBuffer data type
 * @tparam Const - template key for const/non-const version
 *
 * // quite a usefull discussion on iterrators https://stackoverflow.com/questions/2150192/how-to-avoid-code-duplication-implementing-const-and-non-const-iterators?rq=1
 */
template <typename T, bool Const = false>
struct RingIterator {
	using iterator_category			  = std::random_access_iterator_tag;  // bidirectional_iterator_tag;
	using difference_type			  = std::ptrdiff_t;
	// using value_type        = T;
	using value_type				  = typename std::remove_cv<T>::type;
	using pointer					  = typename std::conditional_t<Const, T const*, T*>;
	using reference					  = typename std::conditional_t<Const, T const&, T&>;
	using container					  = typename std::conditional_t<Const, RingBuff<T> const, RingBuff<T>>;

	// c-tors
	RingIterator(const RingIterator&) = default;

	template <bool c = Const, class = std::enable_if_t<c>>
	RingIterator(const RingIterator<T, false>& rhs)
		: m_ptr(rhs.m_ptr), m_idx(rhs.m_idx) {
	}

	RingIterator(container* ptr, int idx)
		: m_ptr(ptr), m_idx(idx) {
		if (m_ptr->size) idx %= m_ptr->size;
	}

	// const
	template <bool c = Const>
	std::enable_if_t<c, reference> operator*() const noexcept {
		return *get(m_idx);
	}

	template <bool c = Const>
	std::enable_if_t<c, pointer> operator->() const noexcept {
		return get(m_idx);
	}

	// non-const
	template <bool c = Const>
	std::enable_if_t<!c, reference> operator*() const noexcept {
		return *get(m_idx);
	}

	template <bool c = Const>
	std::enable_if_t<!c, pointer> operator->() const noexcept {
		return get(m_idx);
	}

	// Incrementers
	RingIterator& operator++() {
		++m_idx;
		return *this;
	}
	RingIterator operator++(int) {
		RingIterator tmp = *this;
		++(*this);
		return tmp;
	}
	RingIterator& operator+=(const difference_type& d) {
		m_idx += d;
		return *this;
	}

	// Decrementers
	RingIterator& operator--() {
		--m_idx;
		return *this;
	}
	RingIterator operator--(int) {
		RingIterator tmp = *this;
		--(*this);
		return tmp;
	}
	RingIterator& operator-=(const difference_type& d) {
		m_idx -= d;
		return *this;
	}
	RingIterator operator-(const difference_type& d) const {
		return RingIterator(m_ptr, m_idx - d);
	}
	difference_type operator-(const RingIterator& a) const {
		return (m_idx - a.m_idx);
	}

	// Comparators
	bool operator==(const RingIterator& a) const {
		return (m_ptr == a.m_ptr && m_idx == a.m_idx);
	};
	bool operator!=(const RingIterator& a) const {
		return !(*this == a);
	};
	bool operator<(const RingIterator& a) const {
		assert(m_ptr == a.m_ptr);
		return (m_idx < a.m_idx);
	}
	bool operator>(const RingIterator& a) const {
		assert(m_ptr == a.m_ptr);
		return (m_idx > a.m_idx);
	}
	bool operator<=(const RingIterator& a) const {
		return (*this == a || *this < a);
	}
	bool operator>=(const RingIterator& a) const {
		return (*this == a || *this > a);
	}

	// friend bool operator== (const RingIterator& a, const RingIterator& b) { return a.m_ctr == b.m_ctr; };
	// friend bool operator!= (const RingIterator& a, const RingIterator& b) { return a.m_ctr != b.m_ctr; };

   protected:
	container* m_ptr;  // ringbuffer object pointer
	int		   m_idx;  // offset from head pointer

   private:
	inline pointer get(int idx) const {
		return m_ptr->at(std::abs(m_idx));	// offset from head
	}
};

/**
 * @brief ring buffer container for arbitrary data
 * It tries to allocate memory from PSRAM if available, then falls back to malloc() if PSRAM fails
 *
 * @tparam T type of stored data
 * @param _s - container size (number of elemens stored)
 */
template <typename T>
class RingBuff {
	int									  head = 0;
	std::unique_ptr<T[], decltype(free)*> data{nullptr, free};
	inline int							  tail() const {
		   return (head + size) % capacity;
	}

	using Iterator		= RingIterator<T, false>;
	using ConstIterator = RingIterator<T, true>;

	friend Iterator;
	friend ConstIterator;

	// safety check for ConstIterator
	static_assert(std::is_trivially_copy_constructible<ConstIterator>::value, "ConstIterator<> failed is_trivially_copy_constructible<> check");

   protected:
	int size = 0;  // current buffer size

   public:
	const size_t capacity;	// max buffer capacity

	explicit RingBuff(size_t _s)
		: capacity(_s) {
		auto p = static_cast<T*>(heap_caps_malloc(_s * sizeof(T), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));  // try to alloc SPI ram first

		if (!p)
			p = static_cast<T*>(malloc(_s * sizeof(T)));  // try any available RAM otherwise

		if (p) {  // OK, we were able to allocate mem
			data.reset(p);
		}
	}

	// D-tor
	virtual ~RingBuff(){};

	T* at(int offset) const {
		if (!size)
			return nullptr;

		offset %= size;
		if (offset < 0)
			offset += size;

		return &data[(head + offset) % capacity];  // offset from head
	}

	/**
	 * @brief reset buffer to initial state
	 * no data changed actually, only iterators and pointers are invalided
	 */
	void clear() {
		head = 0;
		size = 0;
	};

	/**
	 * @brief return current size of the buffer
	 * i.e. a number of elements stored, could be less or equal to capacity
	 *
	 * @return int
	 */
	int getSize() const {
		return this->size;
	}

	void push_back(T const& val);

	// T* pop_front(){};

	// Const iterator methods
	auto cbegin() const {
		return ConstIterator(this, 0);
	}
	auto cend() const {
		return ConstIterator(this, size);
	}

	auto crbegin() const {
		return ConstIterator(this, 1 - size);
	}
	auto crend() const {
		return ConstIterator(this, 1);
	}

	// Mutable iterator methods
	auto begin() {
		return Iterator(this, 0);
	}
	auto end() {
		return Iterator(this, size);
	}

	auto rbegin() {
		return Iterator(this, 1 - size);
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

	/**
	 * Class constructor
	 */
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

////////////////////////////////
template <>
class TSContainer<pz003::metrics> {
   public:
	TSContainer<pz003::metrics>(){};
	//~TSContainer();

	/**
	 * @brief get a pointer to TS object with specified ID
	 *
	 * @param id
	 * @return TimeSeries<T>* - or nullptr if TS with specified ID does not exit
	 */
	const TimeSeries<pz003::metrics>* getTS(uint8_t id) const;

	/**
	 * @brief get a pointer to TS object with specified ID
	 * a cast wraper around const get()
	 * @param id
	 * @return TimeSeries<T>*  - or nullptr if TS with specified ID does not exit
	 */
	TimeSeries<pz003::metrics>*		 getTS(uint8_t id) {
		  return const_cast<TimeSeries<pz003::metrics>*>(const_cast<const TSContainer<pz003::metrics>*>(this)->getTS(id));
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
		   tschain.remove_if(MatchID<TimeSeries<pz003::metrics>>(id));
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
	void push(const pz003::metrics& val, uint32_t time);

	bool setTSinterval(uint8_t id, uint32_t _interval, uint32_t newtime);

	/**
	 * @brief Set the Averager object to process data between between time intervals
	 *
	 * @param id - id of an TS container
	 * @param rhs - rvalue for the unique pointer of the Averager instance, TS containder will own the object
	 */
	void setAverager(uint8_t id, std::unique_ptr<AveragingFunction<pz003::metrics>>&& rhs);

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
	std::list<std::shared_ptr<TimeSeries<pz003::metrics>>> tschain;	// time-series chain
};

//////////////////////////////

template <class T>
class AveragingFunction {
   public:
	virtual ~AveragingFunction(){};

	virtual void   push(const T&) = 0;
	virtual T	   get()		  = 0;
	virtual void   reset()		  = 0;
	virtual size_t getCnt() const = 0;
};

class MeanAveragePZ004 : public AveragingFunction<pz004::metrics> {
	unsigned v{0}, c{0}, p{0}, e{0}, f{0}, pf{0}, _cnt{0};

   public:
	void		   push(const pz004::metrics& m) override;
	pz004::metrics get() override;
	void		   reset() override;
	size_t		   getCnt() const override {
		return _cnt;
	};
};

class MeanAveragePZ003 : public AveragingFunction<pz003::metrics> {
	unsigned v{0}, c{0}, p{0}, e{0}, _cnt{0};

   public:
	void		   push(const pz003::metrics& m) override;
	pz003::metrics get() override;
	void		   reset() override;
	size_t		   getCnt() const override {
		return _cnt;
	};
};

//  ===== Implementation follows below =====
template <typename T>
void RingBuff<T>::push_back(const T& val) {
	if (!data)
		return;

	data[tail()] = val;
	if (size != capacity)
		++size;
	else if (++head == capacity)
		head = 0;
}

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
		setAverager(id, std::make_unique<MeanAveragePZ004>());
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
