/*
PZEM EDL - PZEM Event Driven Library

This code implements communication and data exchange with PZEM004T V3.0 module using MODBUS proto
and provides an API for energy metrics monitoring and data processing.

This file is part of the 'PZEM event-driven library' project.

Copyright (C) Emil Muratov, 2021
GitHub: https://github.com/vortigont/pzem-edl
*/

#include "pzem_edl.hpp"
#include "esp32-hal-log.h"

using namespace pz004;

#define POLLER_NAME         "PZ_poll"
#define POOL_POLLER_NAME    "PZP_Poll"
#define TIMER_CMD_TIMEOUT    10     // block up to x ticks trying to change timer params

#ifndef pdTICKS_TO_MS
#define pdTICKS_TO_MS(xTicks)  (((TickType_t)(xTicks) * 1000u) / configTICK_RATE_HZ)
#endif


/**
 * @brief Destroy the PZEM::PZEM object
 * 
 */
PZEM::~PZEM(){
    #ifdef PZEM_EDL_DEBUG
        ESP_LOGD(TAG, "PZEM deconstruct, id: %d", id);
    #endif
    if (sink_lock)
        detachUartQ();
}

void PZEM::attachUartQ(UartQ *qport, bool tx_only){
    if (!qport || q)    // check if either new port is empty or there is already exist an attachment
        return;

    q = qport;

    if (tx_only)
        return;

    q->attach_RX_hndlr( [this](RX_msg *msg){ 
            rx_sink(msg);
            delete msg;     // must delete the message once processed, otherwise it will leak mem
        });
    sink_lock = true;
}

void PZEM::detachUartQ(){
    if (!q)
        return;
    if (sink_lock)
        q->detach_RX_hndlr();

    q = nullptr;
    sink_lock = false;
}

void PZEM::attach_rx_callback(rx_callback_t f){
    if (!f)
        return;
    rx_callback = std::move(f);
}

bool PZEM::autopoll(){
    if (t_poller && xTimerIsTimerActive(t_poller) != pdFALSE)
        return true;

    return false;
}

bool PZEM::autopoll(bool newstate){

    if (newstate){
        if (!t_poller){ // create new timer if absent
            t_poller = xTimerCreate(POLLER_NAME, pdMS_TO_TICKS(poll_period), pdTRUE, (void *)this, PZEM::timerRunner);
            if (!t_poller)
                return false;
        }

        // try to (re)start timer if not active
        if( xTimerIsTimerActive( t_poller ) == pdFALSE )
            return xTimerStart(t_poller, TIMER_CMD_TIMEOUT)==pdPASS;

        return true;    // seems it's already up and running, quit
    }

    // disable timer otherwise
    if (t_poller)
        return xTimerDelete(t_poller, TIMER_CMD_TIMEOUT)==pdPASS;

    return false;   // last resort state
}

size_t PZEM::get_pollrate(){
    if (t_poller)
        return pdTICKS_TO_MS( xTimerGetPeriod( t_poller ));

    return 0;
}

bool PZEM::set_pollrate(size_t t){
    if (t<POLLER_MIN_PERIOD)
        return false;

    if( xTimerChangePeriod( t_poller, t / portTICK_PERIOD_MS, TIMER_CMD_TIMEOUT ) == pdPASS )
        return true;

return false;
}



// ****  PZEM004 Implementation  **** //
void PZ004::updateMetrics(){
    if (!q)
        return;

    TX_msg* cmd = pz004::cmd_get_metrics(pz.addr);

    pz.reset_poll_us();
    q->txenqueue(cmd);
}

void PZ004::rx_sink(const RX_msg *msg){
    if (pz.parse_rx_mgs(msg)){          // update meter state with new packet data (if valid)
        if (rx_callback)
            rx_callback(id, msg);       // run external call-back function
    }
};


// ****  PZEM003 Implementation  **** //
void PZ003::updateMetrics(){
    if (!q)
        return;

    TX_msg* cmd = pz003::cmd_get_metrics(pz.addr);

    pz.reset_poll_us();
    q->txenqueue(cmd);
}

void PZ003::rx_sink(const RX_msg *msg){
    if (pz.parse_rx_mgs(msg)){          // update meter state with new packet data (if valid)
        if (rx_callback)
            rx_callback(id, msg);       // run external call-back function
    }
};



/*   === PZPool immplementation ===   */

/**
 * @brief Destroy the PZPool::PZPool object
 * All registered devices and ports are destructed
 */
PZPool::~PZPool(){
    meters.clear();
    ports.clear();
}

bool PZPool::addPort(PZPort_cfg &portcfg){
    if (port_by_id(portcfg.id))
        return false;       // port with such id already exist

    auto p = std::make_shared<PZPort>(portcfg);
    return addPort(p);
}

bool PZPool::addPort(std::shared_ptr<PZPort> port){
    if (port_by_id(port->id))
        return false;       // port with such id already exist

    if (!ports.add(port))
        return false;       // some LinkedList error

    uint8_t portid = port->id;

    // RX handler lambda catches port-id here and suppies this id to the handler function
    port->attach_RX_hndlr([this, portid](RX_msg *msg){
            if (!msg)
                return;

            rx_dispatcher(msg, portid);
            delete msg;     // must delete the message once processed, otherwise it will leak mem
      });

    return true;
}

// TODO: возвращать код ошибки
bool PZPool::addPZEM(const uint8_t port_id, const uint8_t pzem_id, uint8_t modbus_addr, const char *descr){
    if(!port_by_id(port_id) || pzem_by_id(pzem_id))
        return false;       // either port is missing or pzem with this id already exist

    if (modbus_addr < ADDR_MIN || modbus_addr > ADDR_MAX)   // we do not want any broadcasters or wrong addresses in our pool
        return false;

    auto p = port_by_id(port_id);

    auto node = std::make_shared<PZNode>();
    node->port = p;

    auto pz = std::unique_ptr<PZ004>(new PZ004(pzem_id, modbus_addr, descr));     // create new PZEM object
    pz->attachUartQ(node->port.get(), true);                                    // attach it to the specified PortQ (TX-only!)

    node->pzem = std::move(pz);

    return meters.add(node);
};

// TODO: возвращать код ошибки
bool PZPool::addPZEM(const uint8_t port_id, PZ004 *pz){

    // reject objects with catch-all or invalid address
    if (pz->getaddr() < ADDR_MIN || pz->getaddr() > ADDR_MAX)
        return false;

    auto p = port_by_id(port_id);
    if (!p)             // reject non-existing ports
        return false;

    auto node = std::make_shared<PZNode>();
    node->port = p;

    // detach existing port (if any)
    pz->detachUartQ();

    // and attach our port  (TX-only!)
    pz->attachUartQ(node->port.get(), true);

    node->pzem.reset(std::move(pz));

    return meters.add(node);
}

bool PZPool::removePZEM(const uint8_t pzem_id){
    for (int i = 0; i != meters.size(); ++i) {
        if (meters[i]->pzem->id == pzem_id){
            meters.remove(i);
            return true;
        }
    }
    return false;
}

void PZPool::rx_dispatcher(const RX_msg *msg, const uint8_t port_id){
    // битые пакеты отбрасываем сразу
    if (!msg->valid){
        #ifdef PZEM_EDL_DEBUG
        ESP_LOGW(TAG, "RX packet CRC err");
        #endif
        return;
    }
    
    //  ищем объект совпадающий по паре порт/modbus_addr
    for (int i = 0; i != meters.size(); ++i) {
        if (meters[i]->pzem->getaddr() == msg->addr && meters[i]->port->id == port_id){
            #ifdef PZEM_EDL_DEBUG
            ESP_LOGD(TAG, "Got match PZEM Node for port:%d , addr:%d\n", port_id, msg->addr);
            #endif
            meters[i]->pzem->rx_sink(msg);

            if (rx_callback)
                rx_callback(meters[i]->pzem->id, msg);       // run external call-back function (if set)
            return;
        }
    }
#ifdef PZEM_EDL_DEBUG
    ESP_LOGD(TAG, "Stray packet, no matching PZEM found");
#endif
}

void PZPool::updateMetrics(){
   for (int i = 0; i != meters.size(); ++i) {
       meters[i]->pzem->updateMetrics();
   }
}

// todo: дописать итераторы для класса LList
std::shared_ptr<PZPort> PZPool::port_by_id(uint8_t id){
   for (int i = 0; i != ports.size(); ++i) {
        if (ports[i]->id == id)
            return ports[i];
   }

    return nullptr;
}

// todo: дописать итераторы для класса LList
PZ004* PZPool::pzem_by_id(uint8_t id){
   for (int i = 0; i != meters.size(); ++i) {
        if (meters[i]->pzem->id == id)
            return meters[i]->pzem.get();
   }

    return nullptr;
}


bool PZPool::autopoll(){
    if (t_poller && xTimerIsTimerActive(t_poller) != pdFALSE)
        return true;

    return false;
}

bool PZPool::autopoll(bool newstate){

    if (newstate){
        if (!t_poller){ // create new timer if absent
            t_poller = xTimerCreate(POOL_POLLER_NAME, pdMS_TO_TICKS(poll_period), pdTRUE, (void *)this, PZPool::timerRunner);
            if (!t_poller)
                return false;
        }

        // try to (re)start timer if not active
        if( xTimerIsTimerActive( t_poller ) == pdFALSE )
            return xTimerStart(t_poller, TIMER_CMD_TIMEOUT)==pdPASS;

        return true;    // seems it's already up and running, quit
    }

    // disable timer otherwise
    if (t_poller)
        return xTimerDelete(t_poller, TIMER_CMD_TIMEOUT)==pdPASS;

    return false;   // last resort state
}

size_t PZPool::get_pollrate(){
    if (t_poller)
        return pdTICKS_TO_MS( xTimerGetPeriod( t_poller ));

    return 0;
}

bool PZPool::set_pollrate(size_t t){
    if (t<POLLER_MIN_PERIOD)
        return false;

    if( xTimerChangePeriod( t_poller, t / portTICK_PERIOD_MS, TIMER_CMD_TIMEOUT ) == pdPASS )
        return true;

return false;
}

void PZPool::attach_rx_callback(rx_callback_t f){
    if (!f)
        return;
    rx_callback = std::move(f);
}

const char* PZPool::getDescr(uint8_t id){
    const PZEM* p = pzem_by_id(id);
    if (p){
        return p->getDescr();
    }
    
    return nullptr;
};
