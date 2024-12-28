/*
PZEM EDL - PZEM Event Driven Library

This code implements communication and data exchange with PZEM004T V3.0 module using MODBUS proto
and provides an API for energy metrics monitoring and data processing.

This file is part of the 'PZEM event-driven library' project.

Copyright (C) Emil Muratov, 2021
GitHub: https://github.com/vortigont/pzem-edl
*/

#include "pzem_edl.hpp"
#ifdef ARDUINO
#include "esp32-hal-log.h"
#include <Arduino.h>            // it is required for random() func
#else
#include "esp_log.h"
#endif

#define POLLER_NAME         "PZ_poll"
#define POOL_POLLER_NAME    "PZP_Poll"
#define TIMER_CMD_TIMEOUT    10     // block up to x ticks trying to change timer params

#ifndef pdTICKS_TO_MS
#define pdTICKS_TO_MS(xTicks)  (((TickType_t)(xTicks) * 1000u) / configTICK_RATE_HZ)
#endif


// defaults for FakeMeter
#define DEF_U 2200;
#define DEF_I 500;
#define DEF_FREQ 500;
#define DEF_PF 80;


/**
 * @brief Destroy the PZEM::PZEM object
 * 
 */
PZEM::~PZEM(){
    #ifdef PZEM_EDL_DEBUG
        ESP_LOGD(TAG, "PZEM deconstruct, id: %d", id);
    #endif
    if (sink_lock)
        detachMsgQ();
}

void PZEM::attachMsgQ(MsgQ *mq, bool tx_only){
    if (!mq || q)    // check if either new port is empty or there is already exist an attachment
        return;

    q = mq;

    if (tx_only)
        return;

    q->attach_RX_hndlr( [this](RX_msg *msg){ 
            rx_sink(msg);
            delete msg;     // must delete the message once processed, otherwise it will leak mem
        });
    sink_lock = true;
}

void PZEM::detachMsgQ(){
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

bool PZEM::autopoll() const {
    if (t_poller && xTimerIsTimerActive(t_poller) != pdFALSE)
        return true;

    return false;
}

bool PZEM::autopoll(bool newstate){

    if (newstate){
        if (!t_poller){ // create new timer if absent
            t_poller = xTimerCreate(POLLER_NAME, pdMS_TO_TICKS(poll_period), pdTRUE, reinterpret_cast<void *>(this), PZEM::timerRunner);
            if (!t_poller)
                return false;
        }

        // try to (re)start timer if not active
        if( xTimerIsTimerActive( t_poller ) == pdFALSE )
            return xTimerStart(t_poller, TIMER_CMD_TIMEOUT) == pdPASS;

        return true;    // seems it's already up and running, quit
    }

    // disable timer otherwise
    if (t_poller)
        return xTimerDelete(t_poller, TIMER_CMD_TIMEOUT) == pdPASS;

    return false;   // last resort state
}

size_t PZEM::getPollrate() const {
    if (t_poller)
        return pdTICKS_TO_MS( xTimerGetPeriod( t_poller ));

    return 0;
}

bool PZEM::setPollrate(size_t t){
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

void PZ004::resetEnergyCounter(){
    TX_msg* cmd = pz004::cmd_energy_reset(pz.addr);
    q->txenqueue(cmd);                    // there is no error handling by default, just need to check E-counter on a next cycle
}


// ****  PZEM003 Implementation  **** //
void PZ003::updateMetrics(){
    if (!q)
        return;

    TX_msg* cmd = pz003::cmd_get_metrics(pz.addr);

    pz.reset_poll_us();
    q->txenqueue(cmd);
}

void PZ003::setShunt(pz003::shunt_t shunt){
    if (!q)
        return;

    TX_msg* cmd = pz003::cmd_set_shunt(shunt, pz.addr);

    q->txenqueue(cmd);
}

void PZ003::rx_sink(const RX_msg *msg){
    if (pz.parse_rx_mgs(msg)){          // update meter state with new packet data (if valid)
        if (rx_callback)
            rx_callback(id, msg);       // run external call-back function
    }
};

void PZ003::resetEnergyCounter(){
    TX_msg* cmd = pz003::cmd_energy_reset(pz.addr);
    q->txenqueue(cmd);                    // there is no error handling by default, just need to check E-counter on a next cycle
}



/*   === PZPool immplementation ===   */

bool PZPool::addPort(uint8_t _id, UART_cfg &portcfg, const char *descr){
    if (port_by_id(_id))
        return false;       // port with such id already exist

    auto p = std::make_shared<PZPort>(_id, portcfg, descr);
    return addPort(p);
}

bool PZPool::addPort(std::shared_ptr<PZPort> port){
    if (port_by_id(port->id))
        return false;       // port with such id already exist

    uint8_t portid = port->id;
    ports.emplace_back(port);

    // RX handler lambda catches port-id here and suppies this id to the handler function
    port->q->attach_RX_hndlr([this, portid](RX_msg *msg){
            if (!msg)
                return;

            rx_dispatcher(msg, portid);
            delete msg;     // must delete the message once processed, otherwise it will leak mem
      });

    return true;
}

// TODO: возвращать код ошибки
bool PZPool::addPZEM(const uint8_t port_id, const uint8_t pzem_id, uint8_t modbus_addr, pzmbus::pzmodel_t model, const char *descr){
    if (modbus_addr < ADDR_MIN || modbus_addr > ADDR_MAX)   // we do not want any broadcasters or wrong addresses in our pool
        return false;

    if(!port_by_id(port_id) || pzem_by_id(pzem_id))
        return false;       // either port is missing or pzem with this id already exist

    PZEM *pz;

    switch (model){
        case pzmbus::pzmodel_t::pzem004v3 : {
            pz = new PZ004(pzem_id, modbus_addr, descr);     // create new PZEM004 object
            break;
        }
        case pzmbus::pzmodel_t::pzem003 : {
            pz = new PZ003(pzem_id, modbus_addr, descr);     // create new PZEM003 object
            break;
        }
        default:
            return false;
    }

    if (addPZEM(port_id, pz))
        return true;
    else {
        delete pz;
        return false;
    }

};

// TODO: возвращать код ошибки
bool PZPool::addPZEM(const uint8_t port_id, PZEM *pz){

    // reject objects with catch-all or invalid address
    if (pz->getaddr() < ADDR_MIN || pz->getaddr() > ADDR_MAX)
        return false;

    auto p = port_by_id(port_id);
    if (!p)             // reject non-existing ports
        return false;

    auto node = std::make_shared<PZNode>();
    node->port = p;

    // detach existing rx call-back (if any)
    pz->detach_rx_callback();

    // detach existing port (if any)
    pz->detachMsgQ();

    // and attach our port  (TX-only!)
    pz->attachMsgQ(node->port.get()->q.get(), true);

    node->pzem.reset(std::move(pz));

    meters.emplace_back(std::move(node));
    return true;
}

bool PZPool::removePZEM(const uint8_t pzem_id){
    for (auto i = meters.begin(); i != meters.end(); ++i ){
        if ((*i)->pzem->id == pzem_id){
            meters.erase(i);
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
    for (const auto& i : meters){
        if (i->pzem->getaddr() == msg->addr && i->port->id == port_id){
            #ifdef PZEM_EDL_DEBUG
            ESP_LOGD(TAG, "Got match PZEM Node for port:%d , addr:%d\n", port_id, msg->addr);
            #endif
            i->pzem->rx_sink(msg);

            if (rx_callback)
                rx_callback(i->pzem->id, msg);       // run external call-back function (if set)
            return;
        }
    }
#ifdef PZEM_EDL_DEBUG
    ESP_LOGD(TAG, "Stray packet, no matching PZEM found");
#endif
}

void PZPool::updateMetrics(){
    for (const auto& i : meters)
       i->pzem->updateMetrics();
}

std::shared_ptr<PZPort> PZPool::port_by_id(uint8_t id){
    for (auto i : ports){
        if (i->id == id)
            return i;
    }

    return nullptr;
}

const PZEM* PZPool::pzem_by_id(uint8_t id) const {
    for (auto i = meters.cbegin(); i != meters.cend(); ++i){
        if (i->get()->pzem->id == id)
            return i->get()->pzem.get();
    }

/*
    for (const auto &i : std::as_const(meters)){        // std::as_const req c++17
        if (i->pzem->id == id)
            return i->pzem.get();
    }
*/
    return nullptr;
}


bool PZPool::autopoll() const {
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
            return xTimerStart(t_poller, TIMER_CMD_TIMEOUT) == pdPASS;

        return true;    // seems it's already up and running, quit
    }

    // disable timer otherwise
    if (t_poller)
        return xTimerDelete(t_poller, TIMER_CMD_TIMEOUT) == pdPASS;

    return false;   // last resort state
}

size_t PZPool::getPollrate() const {
    if (t_poller)
        return pdTICKS_TO_MS( xTimerGetPeriod( t_poller ));

    return 0;
}

bool PZPool::setPollrate(size_t t){
    if (t < POLLER_MIN_PERIOD)
        return false;

    return ( xTimerChangePeriod( t_poller, t / portTICK_PERIOD_MS, TIMER_CMD_TIMEOUT ) == pdPASS );
}

void PZPool::attach_rx_callback(rx_callback_t f){
    if (!f)
        return;
    rx_callback = std::move(f);
}

const char* PZPool::getDescr(uint8_t id) const {
    const PZEM* p = pzem_by_id(id);
    if (p){
        return p->getDescr();
    }
    
    return nullptr;
};

const pzmbus::state* PZPool::getState(uint8_t id) const {
    const auto *pz = pzem_by_id(id);

    if (pz)
        return pz->getState();

    return nullptr;
};

const pzmbus::metrics* PZPool::getMetrics(uint8_t id) const {
    const auto *pz = pzem_by_id(id);

    if (pz)
        return pz->getMetrics();

    return nullptr;
}

void PZPool::resetEnergyCounter(uint8_t pzem_id){
    for (auto &i : meters){
        if (i->pzem->id == pzem_id){
            i->pzem->resetEnergyCounter();
            return;
        }
    }
}


#ifdef ARDUINO
void FakeMeterPZ004::reset(){
    mt.voltage = DEF_U;
    mt.current = DEF_I;
    mt.freq = DEF_FREQ;
    mt.pf = DEF_PF;

    mt.power = mt.voltage * mt.current * mt.pf / 100000;
    timecount = esp_timer_get_time() >> 10;
    _nrg = 0;
}

void FakeMeterPZ004::randomize(pz004::metrics& m){
    // voltage
    if (random(prob.voltage) == prob.voltage-1){
        int deviation = mt.voltage * deviate.voltage / 100;
        m.voltage = mt.voltage + random(-1 * deviation, deviation);
    }

    // current
    if (random(prob.current) == prob.current-1){
        int deviation = mt.current * deviate.current / 100;
        m.current = mt.current + random(-1 * deviation, deviation);
    }

    // freq
    if (random(prob.freq) == prob.freq-1){
        int deviation = mt.freq * deviate.freq / 100;
        m.freq = mt.freq + random(-1 * deviation, deviation);
    }

    // pf
    if (random(prob.pf) == prob.pf-1){
        int deviation = mt.pf * deviate.pf / 100;
        m.pf = mt.pf + random(-1 * deviation, deviation);
        if (m.pf > 100) m.pf = 100;
    }
}

void FakeMeterPZ004::updnrg(pz004::metrics& m){
    int64_t t = esp_timer_get_time() >> 10;
    _nrg += mt.power * (t - timecount) / 10;      // find energy for the last time interval in W*ms
    timecount = t;
    mt.power = m.voltage * m.current * m.pf / 100000;     // 100000 = 100 is for pf, 1000 is for decivolts*ma (dw)
    mt.energy += _nrg / 3600000;            // increment W*h counter if we have enough energy consumed so far
    _nrg %= 3600000;
}

// ****  Dummy PZEM004 Implementation  **** //

void DummyPZ004::updateMetrics(){
    pz.update_us = esp_timer_get_time();
    fm.randomize(pz.data);
    fm.updnrg(pz.data);

    pz.data.power = fm.mt.power;
    pz.data.energy = fm.mt.energy;

    if (rx_callback)
        rx_callback(id, nullptr);           // run external call-back function with null data,
                                            // still possible to retrieve metrics from the PZEM obj
}

void FakeMeterPZ003::reset(){
    mt.voltage = DEF_U;
    mt.current = DEF_I;
    //mt.freq = DEF_FREQ;
    //mt.pf = DEF_PF;

    mt.power = mt.voltage * mt.current  / 100000;
    timecount = esp_timer_get_time() >> 10;
    _nrg = 0;
}

void FakeMeterPZ003::randomize(pz003::metrics& m){
    // voltage
    if (random(prob.voltage) == prob.voltage-1){
        int deviation = mt.voltage * deviate.voltage / 100;
        m.voltage = mt.voltage + random(-1 * deviation, deviation);
    }

    // current
    if (random(prob.current) == prob.current-1){
        int deviation = mt.current * deviate.current / 100;
        m.current = mt.current + random(-1 * deviation, deviation);
    }

    // freq
    //if (random(prob.freq) == prob.freq-1){
    //    int deviation = mt.freq * deviate.freq / 100;
    //    m.freq = mt.freq + random(-1 * deviation, deviation);
    //}

    // pf
    //if (random(prob.pf) == prob.pf-1){
    //    int deviation = mt.pf * deviate.pf / 100;
    //    m.pf = mt.pf + random(-1 * deviation, deviation);
    //    if (m.pf > 100) m.pf = 100;
    //}
}

void FakeMeterPZ003::updnrg(pz003::metrics& m){
    int64_t t = esp_timer_get_time() >> 10;
    _nrg += mt.power * (t - timecount) / 10;      // find energy for the last time interval in W*ms
    timecount = t;
    mt.power = m.voltage * m.current  / 100000;     // 100000 = 100 is for pf, 1000 is for decivolts*ma (dw)
    mt.energy += _nrg / 3600000;            // increment W*h counter if we have enough energy consumed so far
    _nrg %= 3600000;
}

// ****  Dummy PZEM004 Implementation  **** //

void DummyPZ003::updateMetrics(){
    pz.update_us = esp_timer_get_time();
    fm.randomize(pz.data);
    fm.updnrg(pz.data);

    pz.data.power = fm.mt.power;
    pz.data.energy = fm.mt.energy;

    if (rx_callback)
        rx_callback(id, nullptr);           // run external call-back function with null data,
                                            // still possible to retrieve metrics from the PZEM obj
}


#endif // ARDUINO
