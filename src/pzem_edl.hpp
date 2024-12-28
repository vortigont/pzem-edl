/*
PZEM EDL - PZEM Event Driven Library

This code implements communication and data exchange with PZEM004T V3.0 module using MODBUS proto
and provides an API for energy metrics monitoring and data processing.

This file is part of the 'PZEM event-driven library' project.

Copyright (C) Emil Muratov, 2021
GitHub: https://github.com/vortigont/pzem-edl
*/

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "pzem_modbus.hpp"
#include <list>

#define POLLER_PERIOD       PZEM_REFRESH_PERIOD         // auto update period in ms
#define POLLER_MIN_PERIOD   2*PZEM_UART_TIMEOUT         // minimal poller period


typedef std::function<void (uint8_t id, const RX_msg*)> rx_callback_t;

/**
 * @brief - PowerMeter abstract instance class
 * Helds an object of one PZEM instance along with it's properties
 */
class PZEM {
    std::unique_ptr<char[]> descr;      // Mnemonic name for the instance
    bool sink_lock = false;             // flag marking rx_sink as an active call-back when attached

protected:
    MsgQ *q = nullptr;                  // UartQ sink for TX messages
    rx_callback_t rx_callback = nullptr;          // external callback to trigger on RX data


public:
    const uint8_t id;                   // device unique ID

    /**
     * Class constructor
     */
    explicit PZEM(uint8_t _id, const char *_descr = nullptr) : id(_id) {
        if (!_descr || !*_descr){
            descr.reset(new char[9]);   // i.e. PZEM-123
            sprintf(descr.get(), "PZEM-%d", id);
        }  else
            descr.reset(strcpy(new char[strlen(_descr) + 1], _descr));
    }
    virtual ~PZEM();

    // Copy semantics : not (yet) implemented
    PZEM(const PZEM&) = delete;
    PZEM& operator=(const PZEM&) = delete;

    bool active = true;     // Active/disabled state

    /**
     * @brief return configured MODBUS address
     * 
     */
    virtual uint8_t getaddr() const { return ADDR_ANY; }

    /**
     * @brief attach to UartQ object which provides queues for message exchange
     * UartQ object could be used in exclusive or shared mode.
     * TX sink is always shared among multiple PZEM objects, while RX callback
     * could be used only for one object instance
     * 
     * @param qport - UartQ object reference
     * @param tx_only - if 'true' than UartQ is used in "shared mode" w/o RX handler
     */
    void attachMsgQ(MsgQ *mq, bool tx_only = false);

    /**
     * @brief A sink for RX messages
     * should be set as a callback for UartQ or fed with messages in any other way
     * 
     * @param msg 
     */
    virtual void rx_sink(const RX_msg *msg) = 0;      // pure virtual method, must be redefined in derived classes

    /**
     * @brief detach IO queues
     * this is mandatory on destruct if RX callback has been set
     */
    void detachMsgQ();

    /**
     * @brief external callback function
     * it is fed with a ref to every incoming message along with instance ID
     * 
     * @param f callback function prototype: std::function<void (uint8_t id, const RX_msg*)>
     */
    void attach_rx_callback(rx_callback_t f);

    /**
     * @brief detach external callback
     */
    inline void detach_rx_callback(){rx_callback = nullptr;};

    /**
     * @brief poll PZEM for metrics
     * on call a mesage with metrics request is send to PZEM device
     * should be overriden in a derived class
     */
    virtual void updateMetrics() = 0;                 // pure virtual method, must be redefined in derived classes

    /**
     * @brief return description string as 'const char*'
     * 
     * @return const char* 
     */
    const char *getDescr() const { return descr.get(); };

    /**
     * @brief get auto-poll timer state - active/disabled
     * 
     */
    bool autopoll() const;

    /**
     * @brief set auto-poll timer state
     * 
     * @param newstate - active/disabled
     */
    bool autopoll(bool newstate);

    /**
     * @brief Get pollrate in ms
     * 
     * @return size_t poll period in ms
     */
    size_t getPollrate() const;
    
    /**
     * @brief (Re)Set pollrate in ms
     * NOTE: seems that PZEM has internal averaging period somewhat about ~1 second
     * so there is no reason to poll it at any higher rate - it just returns the same results each time
     * also there is no reason to reduce the default rate, polling is pretty cheap with this lib
     * and you'll always have fresh data. dataStale() method consider PZEM_REFRESH_PERIOD constant as 1 second.
     * The only case this could be required to reduce pollrate is to have more than a dozen PZEM's sharing same bus
     * that could take over 1 sec to poll all of them one after another (need a feedback for such setups)
     * 
     * @param t rate in ms
     * @return true if change successfull
     * @return false otherwise
     */
    bool setPollrate(size_t t);

    /**
     * @brief Get the PZEM State object reference
     * it contains all parameters and metrics for PZEM device
     * 
     * @return const pointer to pzmbus::state 
     */
    virtual const pzmbus::state* getState() const = 0;// const { return &pz; }

    /**
     * @brief Get the Metrics data structure
     * 
     * @return const pzmbus::metrics* 
     */
    virtual const pzmbus::metrics* getMetrics() const = 0; // const { return &pz.data; }

    /**
     * @brief send a command to PZEM device to reset it's internal energy counter
     * 
     */
    virtual void resetEnergyCounter() = 0;

private:
    TimerHandle_t t_poller = nullptr;
    size_t poll_period = POLLER_PERIOD;           // auto poll period in ms

    static void timerRunner(TimerHandle_t xTimer){
        if (!xTimer) return;

        PZEM* p = reinterpret_cast<PZEM*>(pvTimerGetTimerID(xTimer));
        if (p) p->updateMetrics();
    }

};



/**
 * @brief PZEM004v3.0 device class (same as PZEM-014/PZEM-016)
 *
 */
class PZ004 : public PZEM {

protected:
    pz004::state pz;                        // structure with PZEM004 state

public:
    // Derrived constructor
    explicit PZ004(uint8_t _id,  uint8_t modbus_addr = ADDR_ANY, const char *_descr = nullptr) :
        PZEM(_id, _descr) {
        pz.addr = modbus_addr;
    }
    virtual ~PZ004(){};

    // Copy semantics : not (yet) implemented
    PZ004(const PZ004&) = delete;
    PZ004& operator=(const PZ004&) = delete;

    /**
     * @brief return configured MODBUS address
     * 
     */
    uint8_t getaddr() const override { return pz.addr; }

    /**
     * @brief poll PZEM for metrics
     * on call a mesage with metrics request is send to PZEM device
     */
    void updateMetrics() override;

    /**
     * @brief Get the PZEM State object reference
     * it contains all parameters and metrics for PZEM device
     * 
     * @return const pzmbus::state& 
     */
    const pzmbus::state* getState() const override { return &pz; }
    const pz004::state*  getStatePZ004() const { return &pz; }

    /**
     * @brief Get the PZEM Metrics object
     * it contains all electric metrics for PZEM device
     * @return const pzmbus::metrics&
     */
    const pzmbus::metrics* getMetrics() const override { return &pz.data; }
    const pz004::metrics*  getMetricsPZ004() const { return &pz.data; }

    /**
     * @brief A sink for RX messages
     * should be set as a callback for UartQ or fed with messages in any other way
     * 
     * @param msg 
     */
    void rx_sink(const RX_msg *msg) override;

    /**
     * @brief send a command to PZEM device to reset it's internal energy counter
     * 
     */
    void resetEnergyCounter() override;

};


/**
 * @brief PZEM003 device class (same as PZEM-017)
 *
 */
class PZ003 : public PZEM {
protected:  
    pz003::state pz;              // structure with PZEM004 state

public:
    // Derrived constructor
    PZ003(const uint8_t _id,  uint8_t modbus_addr = ADDR_ANY, const char *_descr = nullptr) :
        PZEM(_id, _descr) {
        pz.addr = modbus_addr;
    }
    virtual ~PZ003(){};

    // Copy semantics : not (yet) implemented
    PZ003(const PZ003&) = delete;
    PZ003& operator=(const PZ003&) = delete;

    /**
     * @brief return configured MODBUS address
     * 
     */
    uint8_t getaddr() const override { return pz.addr; }

    /**
     * @brief poll PZEM for metrics
     * on call a mesage with metrics request is send to PZEM device
     */
    void updateMetrics() override;

    /**
     * @brief Set current shunt type
     * 
     * @param shunt - predefined shunt types
     */
    void setShunt(pz003::shunt_t shunt);

    /**
     * @brief Get the PZEM State object reference
     * it contains all parameters and metrics for PZEM device
     * 
     * @return const pzmbus::state*
     */
    const pzmbus::state* getState() const override { return &pz; }
    const pz003::state*  getStatePZ003() const { return &pz; }

    /**
     * @brief Get the PZEM Metrics object
     * it contains all electric metrics for PZEM device
     * @return const pzmbus::metrics*
     */
    const pzmbus::metrics* getMetrics() const override { return &pz.data; }
    const pz003::metrics*  getMetricsPZ003() const { return &pz.data; }

    /**
     * @brief A sink for RX messages
     * should be set as a callback for UartQ or fed with messages in any other way
     * 
     * @param msg 
     */
    void rx_sink(const RX_msg *msg) override;

    /**
     * @brief send a command to PZEM device to reset it's internal energy counter
     * 
     */
    void resetEnergyCounter() override;
};

/**
 * @brief a pool object that incorporates PZEM devices, UART ports and it's mapping
 * 
 * only PZEM004's are supported as pool members (so far)
 */
class PZPool {

    /**
     * @brief A PZEM node is PZEM instance linked to specific port instance
     * 
     */
    struct PZNode {
        std::shared_ptr<PZPort> port;
        std::unique_ptr<PZEM> pzem;
    };

protected:
    std::list< std::shared_ptr<PZPort> > ports;                           // list of registered ports
    std::list< std::shared_ptr<PZNode> > meters;                          // list of registered PZEM nodes                          // list of registered PZEM nodes
    std::shared_ptr<PZPort> port_by_id(uint8_t id);
    const PZEM* pzem_by_id(uint8_t id) const;


public:
    PZPool() = default;
    // Copy semantics : not implemented
    PZPool(const PZPool&) = delete;
    PZPool& operator=(const PZPool&) = delete;



    /**
     * @brief create and register UART port to the Pool
     * makes new port and attaches to it's queues
     * 
     * @param portcfg - a structure with configuration for a new port
     * @return true - on success
     * @return false - on any error
     */
    bool addPort(uint8_t _id, UART_cfg &portcfg, const char *descr = nullptr);

    /**
     * @brief attach an existing UART port object to the Pool
     * port will be detached from it's existing RX handler and redirected to Pool's dispatcher
     * 
     * @param port - shared pointer to the existing port
     * @return true - on success
     * @return false - on any error
     */
    bool addPort(std::shared_ptr<PZPort> port);

    /**
     * @brief Create and register new PZEM object
     * an already existing port id must be defined to which PZEM object will be attached to
     * 
     * @param port_id - an existing port id
     * @param pzem_id - id for the new PZEM object
     * @param modbus_addr - unique modbus address MUST be already set for PZEM device, catch-all address is not allowed in a pool
     * @param descr - mnemonic description
     * @return true   - on success
     * @return false  - on any error
     */
    bool addPZEM(const uint8_t port_id, const uint8_t pzem_id, uint8_t modbus_addr, pzmbus::pzmodel_t model, const char *descr = nullptr);
    bool addPZEM(const uint8_t port_id, PZEM *pz);

    /**
     * @brief check if the port with spefied id exist in a pool
     * 
     * @param id - port id
     * @return true if Port with this ID exist
     * @return false otherwise
     */
    bool existPort(uint8_t id){return port_by_id(id) == nullptr;}

    /**
     * @brief check if the PZEM with spefied id exist in a pool
     * 
     * @param id 
     * @return true 
     * @return false 
     */
    bool existPZEM(uint8_t id){return pzem_by_id(id)== nullptr;}

    /**
     * @brief delete PZEM object from the pool
     * it the object has been created within the poolm than it will be destructed
     * if it was added from external shared_pointer, than it may continue to live outside the pool
     */
    bool removePZEM(const uint8_t pzem_id);

    /**
     * @brief external callback function
     * it is fed with a ref to every incoming message along with instance ID
     * 
     * @param f callback function prototype: std::function<void (uint8_t id, const RX_msg*)>
     */
    void attach_rx_callback(rx_callback_t f);

    /**
     * @brief detach external callback
     */
    inline void detach_rx_callback(){rx_callback = nullptr;}

    /**
     * @brief get auto-poll timer state - active/disabled
     * 
     */
    bool autopoll() const;

    /**
     * @brief set auto-poll timer state
     * 
     * @param newstate - active/disabled
     */
    bool autopoll(bool newstate);

    /**
     * @brief Get pollrate in ms
     * 
     * @return size_t poll period in ms
     */
    size_t getPollrate() const;

    /**
     * @brief (Re)Set pollrate in ms
     * NOTE: seems that PZEM has internal averaging period somewhat about ~1 second.
     * So there is no reason to poll it at any higher rate - it just returns the same results each time.
     * also there is no reason to reduce the default rate, polling is pretty cheap with this lib
     * and you'll alway have fresh data.
     * The only case this could be required to reduce pollrate is to have more than a dozen PZEM's sharing same bus
     * that could take over 1 sec to poll all of them one after another
     * 
     * @param t rate in ms
     * @return true if change successfull
     * @return false otherwise
     */
    bool setPollrate(size_t t);

    /**
     * @brief update metrics for all PZEM Nodes in a pool
     * 
     */
    void updateMetrics();


    /**
     * @brief send a command to PZEM device in a pool with specific id to reset it's internal energy counter
     * NOTE: pzem_id is NOT a MODBUS address, it's PZEM id in a pool. If no PZEM device with this ID
     * in a pool, than this call does nothing
     * 
     */
    void resetEnergyCounter(uint8_t pzem_id);


    /**
     * @brief Get the PZEM State object reference for PZEM with specific id
     * it contains all parameters and metrics for PZEM device
     * NOTE: It is an undefined behavior to call this method on a non-existing id!!!
     * use existPZEM() method to check if unsure
     * 
     * @return const pzmbus::state&, nullptr if PZEM with specified id does not exist
     */
    const pzmbus::state* getState(uint8_t id) const;

    /**
     * @brief Get the PZEM Metrics object reference for PZEM with specific id
     * it contains all electric metrics for PZEM device
     * WARN: It is an undefined behavior to call this method on a non-existing id!!!    (to be fixed)
     * use existPZEM() method to check if unsure
     * 
     * @return const pzmbus::metrics&, nullptr if PZEM with specified id does not exist
     */
    const pzmbus::metrics* getMetrics(uint8_t id) const;

    /**
     * @brief return description string as 'const char*'
     * 
     * @return const char* 
     */
    const char* getDescr(uint8_t id) const;


private:
    TimerHandle_t t_poller = nullptr;
    size_t poll_period = POLLER_PERIOD;           // auto poll period in ms
    rx_callback_t rx_callback = nullptr;          // external callback to trigger on RX dat

    static void timerRunner(TimerHandle_t xTimer){
        if (!xTimer) return;

        PZPool* p = reinterpret_cast<PZPool*>(pvTimerGetTimerID(xTimer));
        if (p) p->updateMetrics();
    }

    void rx_dispatcher(const RX_msg *msg, const uint8_t port_id);

};


/*
  DummyPZEM uses long random() func from Arduono,
  it breaks compat with ESP-IDF, so let's guard it
  TODO: reimplement it for ESP-IDF
*/
#ifdef ARDUINO

struct var_t {
    uint8_t voltage;
    uint8_t current;
    uint8_t freq;
    uint8_t pf;
    var_t(uint8_t v, uint8_t c, uint8_t f, uint8_t pf) : voltage(v), current(c), freq(f), pf(pf) {};
};

/**
 * @brief metrics randomizer
 * it grabs a struct with PZ004 metrics data
 * and tosses it's values, while keep counting for proper pwr/energy per time
 * 
 */
class FakeMeterPZ004 {
public:
    // variances
    pz004::metrics mt;
    // randomise deviation thresholds (percents)
    var_t deviate = var_t(8, 30, 3, 20);
    // probability of randomizing a value on each poll, in 1/x
    var_t prob = var_t(10, 5, 15, 10);

    /**
     * @brief reset all values to defaults
     * 
     */
    void reset();

    /**
     * @brief toss metering variables
     * using deviation and probability settings 
     * 
     */
    void randomize(pz004::metrics& m);

    /**
     * @brief recalculate power/energy values based on time elapsed and current metrics
     * 
     */
    void updnrg(pz004::metrics& m);

protected:
    uint64_t timecount = 0;     // in ms
    uint32_t _nrg = 0;          // energy

};


/**
 * @brief A fake PZEM004 device class
 * it pretends to be PZEM004 class but does not talk via serial to the real device,
 * it just provides some random meterings. Could be used for PZEM software prototyping
 * without the need for real device.
 * Not all functions supported (yet), i.e. alarms.
 * 
 */
class DummyPZ004 : public PZ004 {

public:
    FakeMeterPZ004 fm;

    // Derrived constructor
    DummyPZ004(const uint8_t _id,  uint8_t modbus_addr = ADDR_ANY, const char *_descr = nullptr) : PZ004(_id, modbus_addr, _descr) { fm.reset(); pz.data = fm.mt; }

    //virtual d-tor
    virtual ~DummyPZ004(){};

    // Override methods
    void resetEnergyCounter() override { pz.data.energy = 0; fm.reset(); };

    void updateMetrics() override;

    /**
     * @brief a "do nothing" sink
     * I do not expect any real data to be fed to dummy device
     */
    void rx_sink(const RX_msg *msg) override {};

    /* ***  *** */
    // Own methods

    // reset energy counter to some specific value
    void resetEnergyCounter(uint32_t e){ pz.data.energy = e; fm.mt.energy = e; };

    const pzmbus::metrics* getMetrics() const override { return &pz.data; }
    const pz004::metrics*  getMetricsPZ004() const { return &pz.data; }

};
//////////
class FakeMeterPZ003 {
public:
    // variances
    pz003::metrics mt;
    // randomise deviation thresholds (percents)
    var_t deviate = var_t(8, 30, 3, 20);
    // probability of randomizing a value on each poll, in 1/x
    var_t prob = var_t(10, 5, 15, 10);

    /**
     * @brief reset all values to defaults
     * 
     */
    void reset();

    /**
     * @brief toss metering variables
     * using deviation and probability settings 
     * 
     */
    void randomize(pz003::metrics& m);

    /**
     * @brief recalculate power/energy values based on time elapsed and current metrics
     * 
     */
    void updnrg(pz003::metrics& m);

protected:
    uint64_t timecount = 0;     // in ms
    uint32_t _nrg = 0;          // energy

};


/**
 * @brief A fake PZEM004 device class
 * it pretends to be PZEM004 class but does not talk via serial to the real device,
 * it just provides some random meterings. Could be used for PZEM software prototyping
 * without the need for real device.
 * Not all functions supported (yet), i.e. alarms.
 * 
 */
class DummyPZ003 : public PZ003 {

public:
    FakeMeterPZ003 fm;

    // Derrived constructor
    DummyPZ003(const uint8_t _id,  uint8_t modbus_addr = ADDR_ANY, const char *_descr = nullptr) : PZ003(_id, modbus_addr, _descr) {
		fm.reset(); 
		pz.data = fm.mt; 
	}

    //virtual d-tor
    virtual ~DummyPZ003(){};

    // Override methods
    void resetEnergyCounter() override {
		pz.data.energy = 0; 
		fm.reset(); 
	};

    void updateMetrics() override;

    /**
     * @brief a "do nothing" sink
     * I do not expect any real data to be fed to dummy device
     */
    void rx_sink(const RX_msg *msg) override {};

    /* ***  *** */
    // Own methods

    // reset energy counter to some specific value
    void resetEnergyCounter(uint32_t e){ 
		pz.data.energy = e; 
		fm.mt.energy = e; 
	};
    const pzmbus::metrics* getMetrics() const override { return &pz.data; }
    const pz003::metrics*  getMetricsPZ003() const { return &pz.data; }

};
// #endif // ARDUINO

///////////



#endif // ARDUINO
