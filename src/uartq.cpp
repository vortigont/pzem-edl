/*
PZEM EDL - PZEM Event Driven Library

This code implements communication and data exchange with PZEM004T V3.0 module using MODBUS proto
and provides an API for energy metrics monitoring and data processing.

This file is part of the 'PZEM event-driven library' project.

Copyright (C) Emil Muratov, 2021
GitHub: https://github.com/vortigont/pzem-edl
*/

#include "uartq.hpp"

UartQ::~UartQ(){
    stopQueues();
    uart_driver_delete(port);
    vQueueDelete(rx_evt_queue);
    rx_evt_queue = nullptr;
    rx_callback = nullptr;
}

void UartQ::init(const uart_config_t &uartcfg, int gpio_rx, int gpio_tx){
    // TODO: catch port init errors
    uart_param_config(port, &uartcfg);
    uart_set_pin(port, gpio_tx, gpio_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(port, RX_BUF_SIZE, TX_BUF_SIZE, rx_evt_queue_DEPTH, &rx_evt_queue, 0);
    rts_sem = xSemaphoreCreateBinary();     // Ready-To-Send-next semaphore
}


void UartQ::stopQueues(){
    stop_TX_msg_queue();
    stop_RX_evt_queue();
}

void UartQ::stop_TX_msg_queue(){
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

void UartQ::stop_RX_evt_queue(){
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
        ESP_LOGD(TAG, "TX packet enque, t: %ld", micros());
    #endif

    if (xQueueSendToBack(tx_msg_q, (void *) &msg, (TickType_t)0) == pdTRUE)
        return true;
    else {
        delete msg;     // пакет надо удалять сразу, иначе, не попав в очередь, он останется потерян в памяти
        return false;
    }
}

void UartQ::attach_RX_hndlr(datahandler_t f){
    if (!f)
        return;
    rx_callback = std::move(f);
    start_RX_evt_queue();
}

void UartQ::detach_RX_hndlr(){
    rx_callback = nullptr;
    stop_RX_evt_queue();
}


const char* PZPort::getDescr() const {
    return descr.get();
}

bool PZPort::active(bool newstate){
    if (newstate)
        qrun = startQueues();
    else { stopQueues(); qrun = false;}

    return qrun;
}
