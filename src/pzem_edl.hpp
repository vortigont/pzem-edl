/*
PZEM EDL - PZEM Event Driven Library

This code implements communication and data exchange with PZEM004T V3.0 module using MODBUS proto
and provides an API for energy metrics monitoring and data processing.

This file is part of the 'PZEM event-driven library' project.

Copyright (C) Emil Muratov, 2021
GitHub: https://github.com/vortigont/pzem-edl
*/

#pragma once

#include "uartq.hpp"
#include "LList.h"

#define POLLER_PERIOD       PZEM_REFRESH_PERIOD         // auto update period in ms
#define POLLER_MIN_PERIOD   2*PZEM_UART_TIMEOUT         // minimal poller period


typedef std::function<void (uint8_t id, const pzmbus::RX_msg*)> rx_callback_t;

/**
 * @brief - PowerMeter instance class
 * Helds an object of one PZEM instance along with it's properties
 */
class PZEM {
    std::unique_ptr<char[]> descr;      // Mnemonic name for the instance
    pzmbus::pzem_state pz;              // structure with PZEM state
    UartQ *q = nullptr;                 // UartQ sink for TX messages
    bool sink_lock = false;             // flag marking rx_sink as an active call-back when attached

public:
    const uint8_t id;                   // device unique ID

    /**
     * Class constructor
     */
    PZEM(const uint8_t _id,  uint8_t modbus_addr=ADDR_ANY, const char *_descr=nullptr) : id(_id) {
        pz.addr = modbus_addr;
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
    inline uint8_t getaddr(){ return pz.addr; }

    /**
     * @brief attach to UartQ object which provides queues for message exchange
     * UartQ object could be used in exclusive or shared mode.
     * TX sink is always shared among multiple PZEM objects, while RX callback
     * could be used only for one object instance
     * 
     * @param qport - UartQ object reference
     * @param tx_only - if 'true' than UartQ is used in "shared mode" w/o RX handler
     */
    void attachUartQ(UartQ *qport, bool tx_only = false);

    /**
     * @brief detach IO queues
     * this is mandatory on destruct if RX callback has been set
     */
    void detachUartQ();

    /**
     * @brief A sink for RX messages
     * should be set as a callback for UartQ or fed with messages in any other way
     * 
     * @param msg 
     */
    void rx_sink(const pzmbus::RX_msg *msg);

    /**
     * @brief external callback function
     * it is fed with a ref to every incoming message along with instance ID
     * 
     * @param f callback function prototype: std::function<void (uint8_t id, const pzmbus::RX_msg*)>
     */
    void attach_rx_callback(rx_callback_t f);

    /**
     * @brief detach external callback
     */
    inline void detach_rx_callback(){rx_callback = nullptr;};

    /**
     * @brief poll PZEM for metrics
     * on call a mesage with metrics request is send to PZEM device
     */
    void updateMetrics();

    /**
     * @brief Get the PZEM State object reference
     * it contains all parameters and metrics for PZEM device
     * 
     * @return const pzmbus::pzem_state& 
     */
    const pzmbus::pzem_state &getState() const { return pz; }

    /**
     * @brief Get the PZEM Metrics object
     * it contains all electric metrics for PZEM device
     * @return const pzmbus::metrics&
     */
    const pzmbus::metrics &getMetrics() const { return pz.data; }

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
    bool autopoll();

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
    size_t get_pollrate();
    
    /**
     * @brief (Re)Set pollrate in ms
     * NOTE: seems that PZEM has internal averaging period somewhat about ~1 second
     * so there is no reason to poll it at any higher rate - it just returns the same results each time
     * also there is no reason to reduce the default rate, polling is pretty cheap with this lib
     * and you'll alway have fresh data
     * The only case this could be required to reduce pollrate is to have more than a dozen PZEM's sharing same bus
     * that could take over 1 sec to poll all of them one after another
     * 
     * @param t rate in ms
     * @return true if change successfull
     * @return false otherwise
     */
    bool set_pollrate(size_t t);


private:
    TimerHandle_t t_poller=nullptr;
    size_t poll_period = POLLER_PERIOD;           // auto poll period in ms

    rx_callback_t rx_callback = nullptr;          // external callback to trigger on RX data

    static void timerRunner(TimerHandle_t xTimer){
        if (!xTimer) return;

        PZEM* p = (PZEM*) pvTimerGetTimerID(xTimer);
        if (p) p->updateMetrics();
    }

};


/**
 * @brief a pool object that incorporates PZEM devices, UART ports and it's mapping
 * 
 * 
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
    LList<std::shared_ptr<PZPort>> ports;                           // list of registered ports
    LList<std::shared_ptr<PZNode>> meters;                          // list of registered PZEM nodes
    std::shared_ptr<PZPort> port_by_id(uint8_t id);
    //const std::shared_ptr<PZPort> port_by_id(uint8_t id) const;     // const overload
    PZEM* pzem_by_id(uint8_t id);


public:
    PZPool(){}
    ~PZPool();
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
    bool addPort(PZPort_cfg &portcfg);

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
    bool addPZEM(const uint8_t port_id, const uint8_t pzem_id, uint8_t modbus_addr, const char *descr=nullptr);
    bool addPZEM(const uint8_t port_id, PZEM *pz);

    /**
     * @brief check if the port with spefied id exist in a pool
     * 
     * @param id - port id
     * @return true if Port with this ID exist
     * @return false otherwise
     */
    bool existPort(uint8_t id){return (bool)port_by_id(id);}

    /**
     * @brief check if the PZEM with spefied id exist in a pool
     * 
     * @param id 
     * @return true 
     * @return false 
     */
    bool existPZEM(uint8_t id){return (bool)pzem_by_id(id);}

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
     * @param f callback function prototype: std::function<void (uint8_t id, const pzmbus::RX_msg*)>
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
    bool autopoll();

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
    size_t get_pollrate();

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
    bool set_pollrate(size_t t);

    /**
     * @brief update metrics for all PZEM Nodes in a pool
     * 
     */
    void updateMetrics();

    /**
     * @brief Get the PZEM State object reference for PZEM with specific id
     * it contains all parameters and metrics for PZEM device
     * NOTE: It is an undefined behavior to call this method on a non-existing id!!!
     * use existPZEM() method to check if unsure
     * 
     * @return const pzmbus::pzem_state&, nullptr if PZEM with specified id does not exist
     */
    const pzmbus::pzem_state &getState(uint8_t id){ return pzem_by_id(id)->getState(); };

    /**
     * @brief Get the PZEM Metrics object reference for PZEM with specific id
     * it contains all electric metrics for PZEM device
     * WARN: It is an undefined behavior to call this method on a non-existing id!!!    (to be fixed)
     * use existPZEM() method to check if unsure
     * 
     * @return const pzmbus::metrics&, nullptr if PZEM with specified id does not exist
     */
    const pzmbus::metrics &getMetrics(uint8_t id){ return pzem_by_id(id)->getMetrics(); };

    /**
     * @brief return description string as 'const char*'
     * 
     * @return const char* 
     */
    const char* getDescr(uint8_t id);


private:
    TimerHandle_t t_poller=nullptr;
    size_t poll_period = POLLER_PERIOD;           // auto poll period in ms
    rx_callback_t rx_callback = nullptr;          // external callback to trigger on RX dat

    static void timerRunner(TimerHandle_t xTimer){
        if (!xTimer) return;

        PZPool* p = (PZPool*) pvTimerGetTimerID(xTimer);
        if (p) p->updateMetrics();
    }

    void rx_dispatcher(const pzmbus::RX_msg *msg, const uint8_t port_id);

};