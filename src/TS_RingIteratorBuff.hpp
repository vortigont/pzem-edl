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
    RingIterator& operator-=(const difference_type& d) { m_idx -= d; return *this; }
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

    /**
     * @brief return current size of the buffer
     * i.e. a number of elements stored, could be less or equal to capacity
     * 
     * @return int 
     */
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



// Unary predicate for ID match
template <class T>
class MatchID {
    uint8_t _id;
public:
    explicit MatchID(uint8_t id) : _id(id) {}
    bool operator() ( T val ){
        // T is a shared_ptr
        return val->id == _id;
    }
};




