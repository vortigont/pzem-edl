/*
PZEM EDL - PZEM Event Driven Library

This code implements communication and data exchange with PZEM004T V3.0 module using MODBUS proto
and provides an API for energy metrics monitoring and data processing.

This file is part of the 'PZEM event-driven library' project.

Copyright (C) Emil Muratov, 2021
GitHub: https://github.com/vortigont/pzem-edl
*/

#include "msgq.hpp"


void MsgQ::attach_RX_hndlr(rxdatahandler_t f){
    if (!f)
        return;
    rx_callback = std::move(f);
}

void MsgQ::detach_RX_hndlr(){
    rx_callback = nullptr;
}

//#define PZEM_EDL_DEBUG
#ifdef PZEM_EDL_DEBUG
void MsgQ::rx_msg_debug(const RX_msg *m){
    if (!m->len){
        ESP_LOGE(TAG, "Zero len RX packet");
        return;
    }
    char *buff = new char[m->len * 4];
    char *ptr = buff;
    for(uint8_t i=0; i < m->len; ++i){
        ptr += sprintf(ptr, "%.2x ", m->rawdata[i]);
    }
    ptr=0;
    // выводим с ERROR severity, т.к. по умолчанию CORE_DEBUG_LEVEL глушит дебаг
    ESP_LOGE(TAG, "RX packet, len:%d, CRC: %s, HEX: %s", m->len, m->valid ? "OK":"BAD", buff);
    delete[] buff;
}

void MsgQ::tx_msg_debug(const TX_msg *m){
    if (!m->len){
        ESP_LOGE(TAG, "Zero len TX packet");
        return;
    }
    char *buff = new char[m->len * 4];
    char *ptr = buff;
    for(uint8_t i=0; i < m->len; ++i){
        ptr += sprintf(ptr, "%.2x ", m->data[i]);
    }
    ptr=0;
    // print with ERROR severity, so no need to redefine CORE_DEBUG_LEVEL
    ESP_LOGE(TAG, "TX packet, len:%d, HEX: %s", m->len, buff);
    delete[] buff;
}

#else
void MsgQ::rx_msg_debug(const RX_msg *m){}
void MsgQ::tx_msg_debug(const TX_msg *m){}
#endif




UartQ::~UartQ(){
    rx_callback = nullptr;
    stopQueues();
    uart_driver_delete(port);
    vSemaphoreDelete(rts_sem);
}

void UartQ::init(const uart_config_t &uartcfg, int gpio_rx, int gpio_tx){
    // TODO: catch port init errors
    uart_param_config(port, &uartcfg);
    uart_set_pin(port, gpio_tx, gpio_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(port, RX_BUF_SIZE, TX_BUF_SIZE, rx_msg_q_DEPTH, &rx_msg_q, 0);
    rts_sem = xSemaphoreCreateBinary();     // Ready-To-Send-next semaphore
}


void UartQ::stopQueues(){
    stop_tx_msg_q();
    stop_rx_msg_q();
}

void UartQ::stop_tx_msg_q(){
    if (t_txq){
        vTaskDelete(t_txq);
        t_txq = nullptr;
    }

    if (!tx_msg_q)
        return;

    QueueHandle_t _t = tx_msg_q;
    tx_msg_q = nullptr;

    // очищаем все сообщения из очереди
    TX_msg* msg = nullptr;
    while (xQueueReceive(_t, &(msg), (portTickType)0) == pdPASS ){
        delete msg;
    }

    vQueueDelete(_t);
}

void UartQ::stop_rx_msg_q(){
    if (t_rxq){
        vTaskDelete(t_rxq);
        t_rxq = nullptr;
    }
}

bool UartQ::txenqueue(TX_msg *msg){
    if (!msg)
        return false;

    // check if q is present
    if (!tx_msg_q){
        delete msg;     // пакет надо удалять сразу, иначе, не попав в очередь, он останется потерян в памяти
        return false;
    }

    #ifdef PZEM_EDL_DEBUG
        ESP_LOGD(TAG, "TX packet enque, t: %ld", esp_timer_get_time()/1000);
    #endif

    if (xQueueSendToBack(tx_msg_q, (void *) &msg, (TickType_t)0) == pdTRUE)
        return true;
    else {
        delete msg;     // пакет надо удалять сразу, иначе, не попав в очередь, он останется потерян в памяти
        return false;
    }
}

void UartQ::attach_RX_hndlr(rxdatahandler_t f){
    if (!f)
        return;
    rx_callback = std::move(f);
    start_rx_msg_q();
}

void UartQ::detach_RX_hndlr(){
    rx_callback = nullptr;
    stop_rx_msg_q();
}

bool UartQ::start_rx_msg_q(){
    if (!rx_msg_q)      // avoid working on non-allocated queue
        return false;

    //Create a task to handle UART event from ISR
    if (!t_rxq)
        return xTaskCreate(UartQ::rxTask, EVT_TASK_NAME, EVT_TASK_STACK, (void *)this, EVT_TASK_PRIO, &t_rxq) == pdPASS;
    else
        return true;
}

bool UartQ::start_TX_msg_queue(){
    if (tx_msg_q)           // queue already exist
        return true;

    tx_msg_q = xQueueCreate( tx_msg_q_DEPTH, sizeof(TX_msg*) ); // make q for MSG struct pointers

    if (!tx_msg_q)
        return false;

    //Create a task to handle UART event from ISR
    if (!t_txq) return true;

    return xTaskCreate(UartQ::txTask, TXQ_TASK_NAME, TXQ_TASK_STACK, (void *)this, TXQ_TASK_PRIO, &t_txq) == pdPASS;
}

void UartQ::rxqueuehndlr(){
    uart_event_t event;

    // Task runs inside Infinite loop
    for (;;){
        xSemaphoreGive(rts_sem);                // сигналим что можно отправлять следующий пакет и мы готовы ловить ответ

        // 'xQueueReceive' will "sleep" untill an event messages arrives from the RX event queue
        if(xQueueReceive(rx_msg_q, (void*)&event, (portTickType)portMAX_DELAY)) {

            //Handle received event
            switch(event.type) {
                case UART_DATA: {
                    if (!rx_callback){              // if there is no RX handler, than discard all RX
                        uart_flush_input(port);
                        xQueueReset(rx_msg_q);
                        break;
                    }

                    size_t datalen = 0;
                    ESP_ERROR_CHECK(uart_get_buffered_data_len(port, &datalen));
                    if (0 == datalen){
                        ESP_LOGD(TAG, "can't retreive RX data from buffer, t: %lld", esp_timer_get_time()/1000);
                        uart_flush_input(port);
                        xQueueReset(rx_msg_q);
                        break;
                    }

                    ESP_LOGD(TAG, "RX buff has %u bytes data msg, t: %lld", datalen, esp_timer_get_time()/1000);

                    uint8_t* buff = (uint8_t*) malloc(datalen);
                    if (buff){
                        datalen = uart_read_bytes(port, buff, datalen, PZEM_UART_RX_READ_TICKS);
                        if (!datalen){
                            ESP_LOGD(TAG, "unable to read data from RX buff");
                            delete[] buff;
                            uart_flush_input(port);
                            xQueueReset(rx_msg_q);
                            break;
                        }

                        RX_msg *msg = new RX_msg(buff, datalen);

                        #ifdef PZEM_EDL_DEBUG
                            ESP_LOGD(TAG, "got RX data packet from buff, len: %d, t: %ld", datalen, esp_timer_get_time()/1000);
                            rx_msg_debug(msg);
                        #endif

                        rx_callback(msg);                   // call external function to process PZEM message
                    } else
                        uart_flush_input(port);             // если маллок не выдал память - очищаем весь инпут

                    break;
                }
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "UART RX fifo overflow!");
                    xQueueReset(rx_msg_q);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART RX ringbuff full");
                    uart_flush_input(port);
                    xQueueReset(rx_msg_q);
                    break;
                case UART_BREAK:
                case UART_FRAME_ERR:
                    ESP_LOGW(TAG, "UART RX err");
                    break;
                default:
                    break;
            }
        }

    }
    // Task needs to self-terminate before returning (but we should not ever reach this point anyway)
    vTaskDelete(NULL);
}

void UartQ::txqueuehndlr(){

    TX_msg* msg = nullptr;

    // Task runs inside Infinite loop
    for (;;){
        // 'xQueueReceive' will "sleep" untill some message arrives from the msg queue
        if(xQueueReceive(tx_msg_q, &(msg), (portTickType)portMAX_DELAY)) {

            // if smg would expect a reply than I need to grab a semaphore from the RX queue task
            if (msg->w4rx){
                ESP_LOGD(TAG, "Wait for tx semaphore, t: %lld", esp_timer_get_time()/1000);
                xSemaphoreTake(rts_sem, pdMS_TO_TICKS(PZEM_UART_TIMEOUT));
                // an old reply migh be still in the rx queue while I'm handling this one
                //uart_flush_input(port);     // input should be cleared from any leftovers if I expect a reply (in case of a timeout only)
                //xQueueReset(rx_msg_q);
            }

            // Send message data to the UART TX FIFO
            uart_write_bytes(port, (const char*)msg->data, msg->len);

            #ifdef PZEM_EDL_DEBUG
                ESP_LOGD(TAG, "TX - packet sent to uart FIFO, t: %ld", esp_timer_get_time()/1000);
                tx_msg_debug(msg);
            #endif

            // destroy message
            delete msg;
            msg = nullptr;
        }
    }
    // Task needs to self-terminate before returning (but we should not ever reach this point anyway)
    vTaskDelete(NULL);
}



// PZPort Implementation
PZPort::PZPort(uint8_t _id, MsgQ *mq, const char *_name) : id(_id) {
    //std::move(mq);
    q.reset(mq);
    setdescr(_name);
    qrun = q->startQueues();
}

// Construct a new UART port
PZPort::PZPort(uint8_t _id, UART_cfg &cfg, const char *_name) : id(_id) {
    UartQ *_q = new UartQ(cfg.p, cfg.uartcfg, cfg.gpio_rx, cfg.gpio_tx);
    q.reset(_q);
    setdescr(_name);
    qrun = q->startQueues();
}

const char* PZPort::getDescr() const {
    return descr.get();
}

bool PZPort::active(bool newstate){
    if (newstate)
        qrun = q->startQueues();
    else { q->stopQueues(); qrun = false;}

    return qrun;
}

void PZPort::setdescr(const char *_name){
    if (!_name || !*_name){
        descr = std::unique_ptr<char[]>(new char[9]);
        sprintf(descr.get(), "Port-%d", id);
    } else
        descr = std::unique_ptr<char[]>(strcpy(new char[strlen(_name) + 1], _name));
}


NullQ::~NullQ(){
    tx_callback = nullptr;
    rx_callback = nullptr;
}

// NullQ implementation methods
void NullQ::attach_TX_hndlr(txdatahandler_t f){
    if (f) tx_callback = std::move(f);
}

void NullQ::detach_TX_hndlr(){
    tx_callback = nullptr;
}

bool NullQ::txenqueue(TX_msg *msg){
    bool status = false;
    if (tx_callback){
        tx_callback(msg);
        status = true;
    }

    delete msg;
    return status;
}

bool NullQ::rxenqueue(RX_msg *msg){
    if (rx_callback){
        rx_callback(msg);
        return true;
    }

    delete msg;
    return false;
}


NullCable::NullCable(){
    portA.attach_TX_hndlr(std::bind(&NullCable::tx_rx, this, std::placeholders::_1, true));
    portB.attach_TX_hndlr(std::bind(&NullCable::tx_rx, this, std::placeholders::_1, false));
}

void NullCable::tx_rx(TX_msg *tm, bool atob){
    auto *rmsg = new RX_msg(tm->data, tm->len);
    atob ? portB.rxenqueue(rmsg) : portA.rxenqueue(rmsg);
    // receiver call will destroy dynamically allocated object
}