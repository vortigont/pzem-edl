/*
PZEM EDL - PZEM Event Driven Library

This code implements communication and data exchange with PZEM004T V3.0 module using MODBUS proto
and provides an API for energy metrics monitoring and data processing.

This file is part of the 'PZEM event-driven library' project.

Copyright (C) Emil Muratov, 2021
GitHub: https://github.com/vortigont/pzem-edl
*/

#pragma once

#include <cstdlib>
#include <memory>
//#include "LList.h"
//#include <string.h>
//#include "psalloc.hpp"

#include <esp_heap_caps.h>
#include <esp_spiram.h>

//#include <cstddef>
//#include <cstdint>
//#include <type_traits>
//#include <iterator>

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
    using iterator_category = std::forward_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    using value_type        = T;
    using pointer           = typename std::conditional_t<Const, T const *, T *>;
    using reference         = typename std::conditional_t<Const, T const &, T &>;

    using container         = typename std::conditional_t<Const, RingBuff<T> const, RingBuff<T>>;

    // c-tors
    RingIterator(const RingIterator&) = default;

    template<bool c = Const, class = std::enable_if_t<c>>
    RingIterator(const RingIterator<T, false>& rhs) : m_ptr(rhs.m_ptr), m_idx(rhs.m_idx) {}

    RingIterator(container *ptr, int idx) : m_ptr(ptr), m_idx(idx) {}


    // const
    template< bool c = Const >
    std::enable_if_t< c, reference > operator*()  const { return  m_ptr->data[(m_ptr->head + m_idx) % m_ptr->size]; }

    template< bool c = Const >
    std::enable_if_t< c, pointer >   operator->() const { return &m_ptr->data[(m_ptr->head + m_idx) % m_ptr->size]; }


    // non-const
    template< bool c = Const >
    std::enable_if_t< !c, reference > operator*()  const { return  m_ptr->data[(m_ptr->head + m_idx) % m_ptr->size]; }

    template< bool c = Const >
    std::enable_if_t< !c, pointer >   operator->() const { return &m_ptr->data[(m_ptr->head + m_idx) % m_ptr->size]; }

    // Prefix increment
    RingIterator& operator++() { ++m_idx; return *this; }

    // Postfix increment
    RingIterator operator++(int) { RingIterator tmp = *this; ++(*this); return tmp; }

    // Comparators
    bool operator== (const RingIterator& a) const { return (m_ptr == a.m_ptr && m_idx == a.m_idx); };
    bool operator!= (const RingIterator& a) const { return (m_ptr != a.m_ptr || m_idx != a.m_idx); };
    //friend bool operator== (const RingIterator& a, const RingIterator& b) { return a.m_ctr == b.m_ctr; };
    //friend bool operator!= (const RingIterator& a, const RingIterator& b) { return a.m_ctr != b.m_ctr; };

    protected:
        container *m_ptr;     // ringbuffer object pointer
        int m_idx;           // offset from head
};


/**
 * @brief ring buffer container for arbitrary data
 * It ries to allocate memory from PSRAM if available, then falls back to malloc() if PSRAM fails
 * 
 * @tparam T type of stored data 
 * @param _s - container size (number of elemens stored)
 */
template <typename T>
class RingBuff {
    int head = 0, size = 0;
    std::unique_ptr<T[], decltype(free)*> data{nullptr, free};
    //std::vector<T, PSallocator<T>> data;
    int tail(){return (head + size)%capacity; }

    using Iterator = RingIterator<T, false>;
    using ConstIterator = RingIterator<T, true>;

    // safety check for ConstIterator
    static_assert(std::is_trivially_copy_constructible<ConstIterator>::value, "ConstIterator<> failed is_trivially_copy_constructible<> check");

    friend Iterator;
    friend ConstIterator;

public:
    const size_t capacity;          // max buffer capacity

    explicit RingBuff (size_t _s) :
        capacity(_s) {
            auto p = static_cast<T*>(heap_caps_malloc(_s*sizeof(T), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));     // try to alloc SPI ram first

            if (!p)
                p = static_cast<T*>(malloc(_s*sizeof(T)));      // try any available RAM otherwise

            if (p){ // OK, we were able to allocate mem
                //auto unique = std::unique_ptr<T[], decltype(free)*>{ p, free };
                data.reset(p);
            }
        }
        
    /**
     * @brief reset buffer to initial state
     * no data changed actually, only iterators and pointers are invalided
     */
    void reset(){ head = 0; size = 0; };

    void push_back(const T &val);

    //T* pop_front(){};

	// iterator methods
    auto cbegin() const { return ConstIterator(this, 0); }
	auto cend()   const { return ConstIterator(this, size); }

protected:
    // I do not want anyone to mess with data in a buff, so hiding mutable iterator for ancestors only
	auto begin() { return Iterator(this, 0); }
	auto end()   { return Iterator(this, size); }

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
    uint32_t tstamp;            // last update timestamp, atomic units

public:
    const uint32_t interval;    // time interval between series, atomic units

    explicit TimeSeries (size_t _s, uint32_t start_time, uint32_t period = 1) : RingBuff<T>(_s), tstamp(start_time), interval(period) {}
        
    /**
     * @brief reset buffer to empty state and set last update time mark
     * 
     * @param t - current starting point (timestamp) 
     */
    void reset(uint32_t t);

    /**
     * @brief put a new element into buffer
     * time mark is checked to validate insertion period
     * @param val - object of stored type T
     * @param time - timestamp for current value
     * timestamp rollover for uint32_t is handled properly as long as interval between timestamps is consistent
     */
    void put(const T &val, uint32_t time);

    /**
     * @brief obtain a const reference to the data
     * 
     * @return const T* 
     */
/*
    const T* get() const {
        return data.get();
    }
*/

};

/*
template <typename T>
class TimeSeriesContainer {

    LList<std::shared_ptr<TSNode>> ts;  // time-series containers

public:
    TimeSeriesContainer(){}
    ~TimeSeriesContainer();

}
*/

//  ===== Implementation follows below =====
template <typename T>
void RingBuff<T>::push_back(const T &val){

    data[tail()] = val;
    if (size != capacity)
        ++size;
    else if (++head == capacity)
        head=0;
}

template <typename T>
void TimeSeries<T>::reset(uint32_t t){
    tstamp = t;
    RingBuff<T>::reset();
}

template <typename T>
void TimeSeries<T>::put(const T &val, uint32_t time){
    uint32_t _t = time;

    time -= tstamp;                 // разница времени с прошлой выборкой

    if (time < interval)            // промежуточные выборки отбрасываем
        return;                     // (здесь должна быть возможность аггрегации данных за интервал сторонними функциями)

    if (time >= 2*interval){        // пропущено несколько выборок
        if (time/interval > RingBuff<T>::capacity){  // пропустили выборок больше чем весь текущий буфер - сбрасываем всё
            reset(_t);
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

