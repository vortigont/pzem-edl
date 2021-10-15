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

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <iterator>

// forward declarations
//template <typename T>
//struct ConstRingIterator;

//template <typename T, bool IsConst>
//struct RingIterator;

template <typename T> class RingBuff;


template <typename T, bool Const = false>
struct RingIterator {
    using iterator_category = std::forward_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    using value_type        = T;
    using pointer           = typename std::conditional<Const, T const *, T *>::type;
    using reference         = typename std::conditional<Const, T const &, T &>::type;
    //using pointer           = const T*;
    //using reference         = const T&;

    using obj           = typename std::conditional<Const, RingBuff<T> const *, RingBuff<T> *>::type;


    RingIterator(const RingIterator&) = default;

    template<bool c = Const, class = std::enable_if_t<c>>
    RingIterator(const RingIterator<T, false>& rhs) : m_ptr(rhs.m_ptr), m_idx(rhs.m_idx) {}

    //template< bool c = Const >
//    RingIterator(RingBuff<T> *ptr, int idx) : m_ptr{ reinterpret_cast<pointer>(ptr) }, m_idx(idx) {}
    //RingIterator(obj ptr, int idx) : m_ptr{ reinterpret_cast<obj>(ptr) }, m_idx(idx) {}
    RingIterator(obj ptr, int idx) : m_ptr(ptr), m_idx(idx) {}


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

    bool operator== (const RingIterator& a) const { return (m_ptr == a.m_ptr && m_idx == a.m_idx); };
    bool operator!= (const RingIterator& a) const { return (m_ptr != a.m_ptr || m_idx != a.m_idx); };
    //friend bool operator== (const ConstIterator& a, const ConstIterator& b) { return a.m_ctr == b.m_ctr; };
    //friend bool operator!= (const ConstIterator& a, const ConstIterator& b) { return a.m_ctr != b.m_ctr; };

    protected:
        obj m_ptr;     // ringbuffer object pointer
        int m_idx;              // offset from head
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

//    static_assert(std::is_trivially_copy_constructible<ConstIterator>::value);

    friend Iterator;
    friend ConstIterator;

public:
    const size_t capacity;          // capacity

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
    virtual void reset(){ head = 0; size = 0; };

    void push_back(const T &val);

    //T* pop_front(){};



	// iterator methods
    auto cbegin() const { return ConstIterator(this, 0); }
	auto cend()   const { return ConstIterator(this, size); }

    //ConstIterator cbegin() const { return ConstIterator(*this, 0); }
	//ConstIterator cend()   const { return ConstIterator(*this, size); }

protected:
	Iterator begin() { return Iterator(this, 0); }
	Iterator end()   { return Iterator(this, size); }

};




/*
template <typename T>
struct ConstRingIterator {
    using iterator_category = std::forward_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    using value_type        = T;
    using pointer           = const T*;
    using reference         = const T&;

    using Iterator = MyIterator<false>;
    using ConstIterator = MyIterator<true>;

    ConstRingIterator(RingBuff<T> *ptr, int idx) : m_ptr(ptr), m_idx(idx) {}

    const reference operator*()  const { return  m_ptr->data[(m_ptr->head + m_idx) % m_ptr->size]; }
    const pointer   operator->() const { return &m_ptr->data[(m_ptr->head + m_idx) % m_ptr->size]; }

    // Prefix increment
    ConstRingIterator& operator++() { ++m_idx; return *this; }

    // Postfix increment
    ConstRingIterator operator++(int) { ConstRingIterator tmp = *this; ++(*this); return tmp; }

    bool operator== (const ConstRingIterator& a) const { return (m_ptr == a.m_ptr && m_idx == a.m_idx); };
    bool operator!= (const ConstRingIterator& a) const { return (m_ptr != a.m_ptr || m_idx != a.m_idx); };
    //friend bool operator== (const ConstIterator& a, const ConstIterator& b) { return a.m_ctr == b.m_ctr; };
    //friend bool operator!= (const ConstIterator& a, const ConstIterator& b) { return a.m_ctr != b.m_ctr; };

    protected:
        RingBuff<T> *m_ptr;     // ringbuffer object pointer
        int m_idx;              // offset from head
};

template <typename T>
struct RingIterator : ConstRingIterator<T> {
    using iterator_category = std::forward_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    using value_type        = T;
    using pointer           = T*;
    using reference         = T&;

    RingIterator(RingBuff<T> *ptr, int idx) : ConstRingIterator<T>(ptr, idx) {}

    reference operator*()   { return  m_ptr->data[(m_ptr->head + m_idx) % m_ptr->size]; }
    pointer   operator->()  { return &m_ptr->data[(m_ptr->head + m_idx) % m_ptr->size]; }
};
*/


/**
 * @brief ring buffer container for time series data
 * 
 * 
 * @tparam T type of stored data 
 * @param _s - container size (number of elemens stored)
 * @param start_time - atomic timestamps (an increasing counter)
 *                      used to track missed data, could be provided as any timedata, i.e. millis(), micros(), epoch timestamp, etc...
 * @param period     - time series period, in units of time.
 *                      any intermediate samples (less than period time after previous) are skipped. (should be averaged, but not implemented yet)
 */
template <typename T>
class TimeSeries : public RingBuff<T> {
    uint32_t tstamp;                                                // last update timestamp, atomic units

public:
    const uint32_t interval;    // time interval between series, atomic units

    explicit TimeSeries (size_t _s, uint32_t start_time, uint32_t period = 1) : RingBuff<T>(_s), tstamp(start_time), interval(period) {}
        
    /**
     * @brief reset buffer to initial state
     * set all elements of the buffet to default contructed T type,
     * resets index to 0
     * 
     * @param t - current starting point (timestamp) 
     */
    void reset(uint32_t t);

    /**
     * @brief put a new element into buffer
     * 
     * @param val - object of stored type T
     * @param time - timestamp for current value
     * timestamp rollover for uint32_t is handled properly al long as interval between timestamps is consistent
     */
    void put(const T &val, uint32_t time);

    /**
     * @brief obtain a const reference to the data
     * (should be made private and replaced with iterators)
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

    if (time >= 2*interval){           // пропущено несколько выборок
        if (time/interval > RingBuff<T>::capacity){  // пропустили выборок больше чем весь буфер - сбрасываем всё
            reset(_t);
        } else {
            T def;
            do {                    // заполняем пропуски объектами по умолчанию
                RingBuff<T>::push_back(val);
                time -= interval;
            } while(time>interval);
        }
    }

    RingBuff<T>::push_back(val);

    tstamp = _t;                    // обновляем метку времени
}

