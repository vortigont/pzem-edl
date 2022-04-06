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
#if __cplusplus < 201402L     // less than c++14
#error "Sorry, but this lib requires c++14 compatible compiler to build"
#error "check supplied examples on how to add proper build flags for platformio"
#include "no_c++14"
#endif

#include <cstdlib>
#include "LList.h"

// PSRAM support
#include "esp_idf_version.h"
#include <esp_heap_caps.h>

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
#include <esp32/spiram.h>
#else
#include <esp_spiram.h>     // for older IDF core
#endif
//#include "psalloc.hpp"

// forward declarations
template <typename T> class RingBuff;

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
    using iterator_category = std::random_access_iterator_tag;  // bidirectional_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    //using value_type        = T;
    using value_type        = typename std::remove_cv<T>::type;
    using pointer           = typename std::conditional_t<Const, T const *, T *>;
    using reference         = typename std::conditional_t<Const, T const &, T &>;
    using container         = typename std::conditional_t<Const, RingBuff<T> const, RingBuff<T>>;

    // c-tors
    RingIterator(const RingIterator&) = default;

    template<bool c = Const, class = std::enable_if_t<c>>
    RingIterator(const RingIterator<T, false>& rhs) : m_ptr(rhs.m_ptr), m_idx(rhs.m_idx) {}

    RingIterator(container *ptr, int idx) : m_ptr(ptr), m_idx(idx) { if (m_ptr->size) idx %= m_ptr->size;  }


    // const
    template< bool c = Const >
    std::enable_if_t< c, reference > operator*()  const noexcept { return *get(m_idx); }

    template< bool c = Const >
    std::enable_if_t< c, pointer >   operator->() const noexcept { return get(m_idx); }


    // non-const
    template< bool c = Const >
    std::enable_if_t< !c, reference > operator*()  const noexcept { return *get(m_idx); }

    template< bool c = Const >
    std::enable_if_t< !c, pointer >   operator->() const noexcept { return get(m_idx); }

    // Incrementers
    RingIterator& operator++() { ++m_idx; return *this; }
    RingIterator  operator++(int) { RingIterator tmp = *this; ++(*this); return tmp; }
    RingIterator& operator+=(const difference_type& d) { m_idx+=d; return *this; }

    // Decrementers
    RingIterator& operator--() { --m_idx; return *this; }
    RingIterator  operator--(int) { RingIterator tmp = *this; --(*this); return tmp; }
    RingIterator& operator-=(const difference_type& d) { m_idx-=d; return *this; }
    RingIterator  operator- (const difference_type& d) const { return RingIterator(m_ptr, m_idx - d); }
    difference_type operator- (const RingIterator& a) const { return (m_idx - a.m_idx); }


    // Comparators
    bool operator== (const RingIterator& a) const { return (m_ptr == a.m_ptr && m_idx == a.m_idx); };
    bool operator!= (const RingIterator& a) const { return !(*this == a); };
    bool operator<  (const RingIterator& a) const { assert(m_ptr == a.m_ptr); return (m_idx < a.m_idx); }
    bool operator>  (const RingIterator& a) const { assert(m_ptr == a.m_ptr); return (m_idx > a.m_idx); }
    bool operator<= (const RingIterator& a) const { return (*this == a || *this < a); }
    bool operator>= (const RingIterator& a) const { return (*this == a || *this > a); }

    //friend bool operator== (const RingIterator& a, const RingIterator& b) { return a.m_ctr == b.m_ctr; };
    //friend bool operator!= (const RingIterator& a, const RingIterator& b) { return a.m_ctr != b.m_ctr; };

    protected:
        container *m_ptr;     // ringbuffer object pointer
        int m_idx;            // offset from head pointer

    private:

    inline pointer get(int idx) const {
        return m_ptr->at(std::abs(m_idx));    // offset from head
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
    int head = 0;
    std::unique_ptr<T[], decltype(free)*> data{nullptr, free};
    inline int tail() const { return (head + size)%capacity; }

    using Iterator = RingIterator<T, false>;
    using ConstIterator = RingIterator<T, true>;

    friend Iterator;
    friend ConstIterator;

    // safety check for ConstIterator
    static_assert(std::is_trivially_copy_constructible<ConstIterator>::value, "ConstIterator<> failed is_trivially_copy_constructible<> check");

protected:
    int size = 0;   // current buffer size

public:
    const size_t capacity;          // max buffer capacity

    explicit RingBuff (size_t _s) :
        capacity(_s) {
            auto p = static_cast<T*>(heap_caps_malloc(_s*sizeof(T), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));     // try to alloc SPI ram first

            if (!p)
                p = static_cast<T*>(malloc(_s*sizeof(T)));      // try any available RAM otherwise

            if (p){ // OK, we were able to allocate mem
                data.reset(p);
            }
        }

    // D-tor
    virtual ~RingBuff(){};


    T *at(int offset) const {
        if (!size)
            return nullptr;

        offset %= size;
        if (offset < 0)
            offset += size; 

        return &data[(head + offset) % capacity];    // offset from head
    }

    /**
     * @brief reset buffer to initial state
     * no data changed actually, only iterators and pointers are invalided
     */
    void clear(){ head = 0; size = 0; };

    int getSize() const { return this->size; }

    void push_back(T const &val);

    //T* pop_front(){};

	// Const iterator methods
    auto cbegin() const { return ConstIterator(this, 0); }
	auto cend()   const { return ConstIterator(this, size); }

    auto crbegin() const { return ConstIterator(this, 1-size); }
    auto crend()   const { return ConstIterator(this, 1); }

	// Mutable iterator methods
	auto begin() { return Iterator(this, 0); }
	auto end()   { return Iterator(this, size); }

	auto rbegin() { return Iterator(this, 1-size); }
	auto rend()   { return Iterator(this, 1); }

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
    std::unique_ptr<char[]> descr;      // Mnemonic name for the instance
    uint32_t tstamp;                    // last update timestamp mark
    uint32_t interval;                  // time interval between series

public:
    const uint8_t id;                   // TimeSeries unique ID

    /**
     * Class constructor
     */
    TimeSeries (uint8_t _id, size_t s, uint32_t start_time, uint32_t _inverval = 1, const char *_descr=nullptr) : RingBuff<T>(s), tstamp(start_time), interval(_inverval), id(_id) {
        if (!_descr || !*_descr){
            descr.reset(new char[9]);   // i.e. "Hourly"/"60sec", etc...
            sprintf(descr.get(), "TS-%d-%d", id, _inverval);
        }  else
            descr.reset(strcpy(new char[strlen(_descr) + 1], _descr));
    }
    //virtual ~TimeSeries(){};


    /**
     * @brief reset buffer to empty state and set last update time mark
     * 
     * @param t - current starting point (timestamp) 
     */
    void clear(uint32_t t);

    /**
     * @brief put a new element into buffer
     * time mark is checked to validate insertion period
     * @param val - object of stored type T
     * @param time - timestamp for current value
     * timestamp rollover for uint32_t is handled properly as long as interval between timestamps is consistent
     */
    void push(const T &val, uint32_t time);

    uint32_t getTstamp() const { return tstamp; }

    uint32_t getInterval() const { return interval; }

    const char* getDescr() const { return descr.get(); }

    // Setters
    void setInterval(uint32_t _interval, uint32_t newtime);
};

//
template <typename T>
class TSContainer {

public:
    TSContainer<T>(){};
    //~TSContainer();

    const TimeSeries<T>* getTS(uint8_t id) const;
    uint8_t addTS(size_t s, uint32_t start_time, uint32_t period = 1, const char *descr = nullptr, uint8_t id = 0);

    /**
     * @brief remove specific TS object
     * 
     * @param id TimeSeries ID
     */
    void removeTS(uint8_t id);

    /**
     * @brief destroy ALL TS's in chain and release memory
     * 
     */
    void purge(){ tschain.clear(); };

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
    void push(const T &val, uint32_t time);

    bool setTSinterval(uint8_t id, uint32_t _interval, uint32_t newtime);

    /**
     * @brief get TS size by id
     * return current number of elements in TimeSeries object.
     * it could be less than maximum capacity size depending on amount of samples being stored 
     * @param id - TS object id
     * @return int number of elements
     */
    int getTSsize(uint8_t id) const;

    /**
     * @brief get total TS containner size
     * return current number of elements in all TimeSeries objects.
     * it could be less than maximum capacity size depending on amount of samples being stored 
     * @return int number of elements
     */
    int getTSsize() const;

    /**
     * @brief get TS capacity by id
     * return max number of elements in TimeSeries object
     * @param id - TS object id
     * @return int number of elements
     */
    int getTScap(uint8_t id) const;

    /**
     * @brief get total TS container capacity
     * return total max number of elements in TSContainer object
     * @return int number of elements
     */
    int getTScap() const;

    int getTScnt() const { return tschain.size(); };

protected:
    LList<std::shared_ptr<TimeSeries<T>>> tschain;  // time-series chain

};
//

//  ===== Implementation follows below =====
template <typename T>
void RingBuff<T>::push_back(const T &val){
    if (!data)
        return;

    data[tail()] = val;
    if (size != capacity)
        ++size;
    else if (++head == capacity)
        head=0;
}

template <typename T>
void TimeSeries<T>::clear(uint32_t t){
    tstamp = t;
    RingBuff<T>::clear();
}

template <typename T>
void TimeSeries<T>::push(const T &val, uint32_t time){
    uint32_t _t = time;

    time -= tstamp;                 // разница времени с прошлой выборкой

    if (time < interval)            // промежуточные выборки отбрасываем
        return;                     // (здесь должна быть возможность аггрегации данных за интервал сторонними функциями)

    if (time >= 2*interval){        // пропущено несколько выборок
        if (time/interval > RingBuff<T>::capacity){  // пропустили выборок больше чем весь текущий буфер - сбрасываем всё
            clear(_t);
        } else {
            T def;                  // заполняем пропуски объектами по умолчанию
            do {
                RingBuff<T>::push_back(val);
                time -= interval;
            } while(time>interval);
        }
    }

    RingBuff<T>::push_back(val);

    tstamp = _t;                    // обновляем метку времени
}

template <typename T>
void TimeSeries<T>::setInterval(uint32_t _interval, uint32_t newtime){
    if (interval > 0){
        interval = _interval;
        clear(newtime);
    }
}

template <typename T>
const TimeSeries<T>* TSContainer<T>::getTS(uint8_t id) const {
    if (!tschain.size())
        return nullptr;

    for (auto _i = tschain.cbegin(); _i != tschain.cend(); ++_i){
        if (_i->get()->id == id)
            return _i->get();
    }
    return nullptr;
}

template <typename T>
uint8_t TSContainer<T>::addTS(size_t s, uint32_t start_time, uint32_t period, const char *descr, uint8_t id){

    if (id){                    // check if provided id is already exist
        auto n = getTS(id);
        if (n)
            return 0;
    }

    if (!id){                   // if provided id is 0 - than find next free one
        auto n = getTS(id);
        do {
            n = getTS(++id);
        } while (n && id);
        if (!id) return 0;
    }

    auto node = std::make_shared<TimeSeries<T>>(id, s, start_time, period, descr);
    if (tschain.add(node))
        return id;

    return 0;
}

template <typename T>
void TSContainer<T>::removeTS(uint8_t id){
    for (int idx = 0; idx != tschain.size(); ++idx){
        if (tschain.get(idx)->id == id){
            tschain.remove(idx);
            return;
        }
    }
}

template <typename T>
bool TSContainer<T>::setTSinterval(uint8_t id, uint32_t _interval, uint32_t newtime){
    for (auto _i = tschain.begin(); _i != tschain.end(); ++_i){
        if (_i->get()->id == id){
            _i->get()->setInterval(_interval, newtime);
            return true;
        }
    }
    return false;
}


template <typename T>
void TSContainer<T>::clear(){
    for (auto i = tschain.begin(); i != tschain.end(); ++i){
        i->get()->clear();
    }
}

template <typename T>
void TSContainer<T>::push(const T &val, uint32_t time){
    for (auto i = tschain.begin(); i != tschain.end(); ++i)
        i->get()->push(val, time);
}

template <typename T>
int TSContainer<T>::getTSsize(uint8_t id) const {
    auto ts = getTS(id);
    return ts ? ts->getSize() : 0;
}

template <typename T>
int TSContainer<T>::getTScap(uint8_t id) const {
    auto ts = getTS(id);
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
