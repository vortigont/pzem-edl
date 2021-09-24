/*
PZEM EDL - PZEM Event Driven Library

This code implements communication and data exchange with PZEM004T V3.0 module using MODBUS proto
and provides an API for energy metrics monitoring and data processing.

This file is part of the 'PZEM event-driven library' project.

Copyright (C) Emil Muratov, 2021
GitHub: https://github.com/vortigont/pzem-edl
*/

#pragma once
#include "pzem_modbus.hpp"
#include "driver/uart.h"
#include <functional>
#include <memory>

#define PZEM_BAUD_RATE          9600
#define PZEM_UART               UART_NUM_1      // HW Serial Port 2 on ESP32
#define PZEM_UART_TIMEOUT       100             // ms to wait for PZEM RX/TX messaging
#define PZEM_UART_RX_READ_TICKS 10              // ticks to wait for RX byte read from buffer

#define RX_BUF_SIZE (UART_FIFO_LEN * 2)         // 2xUART_FIFO_LEN is enough to fit 10 PZEM msg's
#define TX_BUF_SIZE (0)                         // should be eq 0 or greater than UART_FIFO_LEN, I set it 0 'cause I have my own TX queue

// RX
#define rx_evt_queue_DEPTH      10
#define EVT_TASK_PRIO           4
#define EVT_TASK_STACK          3072
#define EVT_TASK_NAME           "UART_EVQ"

// TX
#define tx_msg_q_DEPTH          8
#define TXQ_TASK_PRIO           2
#define TXQ_TASK_STACK          2048
#define TXQ_TASK_NAME           "UART_TXQ"




/*
Ref links:
Queues: https://www.freertos.org/a00118.html
ring buffs: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos_additions.html

*/

/*
Is that a real default pins mapping for UART???
https://electropeak.com/learn/full-guide-to-esp32-pinout-reference-what-gpio-pins-should-we-use/

 UART 	RXD 	    TXD 	    RTS 	    CST
UART0 	GPIO 1 	    GPIO 3 	    GPIO 22 	GPIO 19
UART1 	GPIO 9 	    GPIO 10 	GPIO 11 	GPIO 6
UART2 	GPIO 16 	GPIO 17 	GPIO 7 	    GPIO 8 
*/

/**
 * @brief PZEM UART port instance configuration structure
 * used to spawn new PZPort instances for MODBUS devices
 * other than PZEMv30
 */
struct PZPort_cfg {
    uart_port_t p;
    int gpio_rx;
    int gpio_tx;
    uint8_t id;
    std::unique_ptr<char[]> descr;
    uart_config_t uartcfg;              // could be used to change uart properties for other modbus devices

    PZPort_cfg (uart_port_t _p=PZEM_UART, int _rx=UART_PIN_NO_CHANGE, int _tx=UART_PIN_NO_CHANGE, uint8_t _id = 0, const char *_name=nullptr) :
        p(_p), gpio_rx(_rx), gpio_tx(_tx), id(_id) {
            uartcfg = {     // default values for PZEMv30
                .baud_rate = PZEM_BAUD_RATE,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
            };
            if (!_name || !*_name){
                descr = std::unique_ptr<char[]>(new char[9]);
                sprintf(descr.get(), "Port-%d", _id);
            } else
                descr = std::unique_ptr<char[]>(strcpy(new char[strlen(_name) + 1], _name));
    }
};


/**
 * @brief ESP32 UART port with message queuesm used to service one-to-many PZEM messaging
 * RX and TX queues are synchronized with semaphore to prevent transmitting messages faster than
 * each single PZEM instance sends replies. Otherwise RX line will get collisions with 2 or more PZEM's
 * transmitting data
 * 
 * UART config
 * - Port: UART_X
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */
class UartQ {
    typedef std::function<void (pzmbus::RX_msg*)> datahandler_t;

    void init(const uart_config_t &uartcfg, int gpio_rx, int gpio_tx);

public:
    UartQ(const uart_port_t p, const uart_config_t cfg, int gpio_rx=UART_PIN_NO_CHANGE, int gpio_tx=UART_PIN_NO_CHANGE) : port(p){ init(cfg, gpio_rx, gpio_tx); }

    UartQ(const uart_port_t p, int gpio_rx=UART_PIN_NO_CHANGE, int gpio_tx=UART_PIN_NO_CHANGE) : port(p){
        uart_config_t uartcfg = {
            .baud_rate = PZEM_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
        };

        init(uartcfg, gpio_rx, gpio_tx);
    }

    // Class dtor
    virtual ~UartQ();

    // Copy semantics : forbidden
    UartQ(const UartQ&) = delete;
    UartQ& operator=(const UartQ&) = delete;

    // Device UART port number
    const uart_port_t port;

    /**
     * @brief start RX/TX queues Task handlers
     * 
     * @return true if success
     * @return false on any error or if Q's/tasks already running
     */
    bool startQueues(){ return start_RX_evt_queue() && start_TX_msg_queue(); };

    /**
     * @brief stop RX/TX queues Task handlers
     * 
     */
    void stopQueues();

    /**
     * @brief enqueue PZEM message and transmit once TX line is free to go
     * this method will take ownership on pzmbus::TX_msg object and 'delete' it
     * after sending to FIFO. It is an error to access/delete/change this object once passed here
     * 
     * @param msg PZEM command message object
     * @return true - if mesage has been enqueue's successfully
     * @return false - if enqueue failed due to Q is full or any other issue
     */
    bool txenqueue(pzmbus::TX_msg *msg);

    /**
     * @brief attach call-back function to feed it with arriving messages from RX line
     * if there is no call-back attached, incoming messages are discarded
     * @param f functional call-back 'std::function<void (pzmbus::RX_msg*)>'
     */
    virtual void attach_RX_hndlr(datahandler_t f);

    /**
     * @brief remove call-back function
     * if there is no call-back attached, incoming messages are discarded
     */
    void detach_RX_hndlr();

private:

    TaskHandle_t    t_rxq=nullptr;          // RX Q servicing task
    TaskHandle_t    t_txq=nullptr;          // TX Q servicing task
    QueueHandle_t   rx_evt_queue;           // UART RX event queue
    QueueHandle_t   tx_msg_q=nullptr;       // UART TX message queue
    SemaphoreHandle_t rts_sem;              // 'ready to send next' Semaphore
    datahandler_t   rx_callback = nullptr;  // RX data callback

    /**
     * @brief start task handling UART RX queue events
     * 
     */
    bool start_RX_evt_queue(){
        if (!rx_evt_queue)      // avoid working on non-allocated queue
            return false;

        //Create a task to handle UART event from ISR
        if (!t_rxq)
            return xTaskCreate(UartQ::rxTask, EVT_TASK_NAME, EVT_TASK_STACK, (void *)this, EVT_TASK_PRIO, &t_rxq) == pdPASS;
        else
            return true;
    }

    /**
     * @brief stop task handling UART RX queue events
     * 
     */
    void stop_RX_evt_queue();

    /**
     * @brief start task handling TX msg queue events
     * 
     */
    bool start_TX_msg_queue(){
        if (tx_msg_q)           // queue already exist
            return true;

        tx_msg_q = xQueueCreate( tx_msg_q_DEPTH, sizeof(pzmbus::TX_msg*) ); // make q for MSG struct pointers

        if (!tx_msg_q)
            return false;

        //Create a task to handle UART event from ISR
        if (!t_txq)
            return xTaskCreate(UartQ::txTask, TXQ_TASK_NAME, TXQ_TASK_STACK, (void *)this, TXQ_TASK_PRIO, &t_txq) == pdPASS;
        else
            return true;
    }

    /**
     * @brief stop task handling TX msg queue events
     * 
     */
    void stop_TX_msg_queue();

    // static wrapper for Task to call RX handler class member
    static void rxTask(void* pvParams){
        ((UartQ*)pvParams)->rxqueuehndlr();
    }

    // static wrapper for Task to call TX handler class member
    static void txTask(void* pvParams){
        ((UartQ*)pvParams)->txqueuehndlr();
    }


    /**
     * @brief RX Queue event handler function
     * NOTE: On RX event, handler creates new pzmbus::RX_msg object with received data
     * once this object is passed to the call-back function - it is up to the calee
     * to maintaint life-time of the object. Once utilised it MUST be 'delete'ed to prevent mem leaks
     */
    void rxqueuehndlr(){
        uart_event_t event;

        // Task runs inside Infinite loop
        for (;;){
            xSemaphoreGive(rts_sem);                // сигналим что можно отправлять следующий пакет и мы готовы ловить ответ

            // 'xQueueReceive' will "sleep" untill an event messages arrives from the RX event queue
            if(xQueueReceive(rx_evt_queue, (void*)&event, (portTickType)portMAX_DELAY)) {

                //Handle received event
                switch(event.type) {
                    case UART_DATA: {
                        if (!rx_callback){              // if there is no RX handler, than discard all RX
                            uart_flush_input(port);
                            xQueueReset(rx_evt_queue);
                            break;
                        }

                        size_t datalen = 0;
                        ESP_ERROR_CHECK(uart_get_buffered_data_len(port, &datalen));
                        if (0 == datalen){
                            ESP_LOGD(TAG, "can't retreive RX data from buffer, t: %ld", micros());
                            uart_flush_input(port);
                            xQueueReset(rx_evt_queue);
                            break;
                        }

                        ESP_LOGD(TAG, "RX buff has %d bytes data msg, t: %d", datalen);

                        uint8_t* buff = (uint8_t*) malloc(datalen);
                        if (buff){
                            datalen = uart_read_bytes(port, buff, datalen, PZEM_UART_RX_READ_TICKS);
                            if (!datalen){
                                ESP_LOGD(TAG, "unable to read data from RX buff");
                                delete[] buff;
                                uart_flush_input(port);
                                xQueueReset(rx_evt_queue);
                                break;
                            }

                            pzmbus::RX_msg *msg = new pzmbus::RX_msg(buff, datalen);

                            #ifdef PZEM_EDL_DEBUG
                                ESP_LOGD(TAG, "got RX data packet from buff, len: %d, t: %ld", datalen, micros());
                                rx_msg_debug(msg);
                            #endif

                            rx_callback(msg);                   // call external function to process PZEM message
                        } else
                            uart_flush_input(port);             // если маллок не выдал память - очищаем весь инпут

                        break;
                    }
                    case UART_FIFO_OVF:
                        ESP_LOGW(TAG, "UART RX fifo overflow!");
                        xQueueReset(rx_evt_queue);
                        break;
                    case UART_BUFFER_FULL:
                        ESP_LOGW(TAG, "UART RX ringbuff full");
                        uart_flush_input(port);
                        xQueueReset(rx_evt_queue);
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


    /**
     * @brief TX message Queue handler function
     * 
     */
    void txqueuehndlr(){

        pzmbus::TX_msg* msg = nullptr;

        // Task runs inside Infinite loop
        for (;;){
            // 'xQueueReceive' will "sleep" untill some message arrives from the msg queue
            if(xQueueReceive(tx_msg_q, &(msg), (portTickType)portMAX_DELAY)) {

                // if smg would expect a reply than I need to grab a semaphore from the RX queue task
                if (msg->w4rx){
                    xSemaphoreTake(rts_sem, pdMS_TO_TICKS(PZEM_UART_TIMEOUT));
                    uart_flush_input(port);     // input should be cleared from any leftovers if I expect a reply
                    //xQueueReset(rx_evt_queue);
                }

                // Send message data to the UART TX FIFO
                uart_write_bytes(port, (const char*)msg->data, msg->len);

                #ifdef PZEM_EDL_DEBUG
                    ESP_LOGD(TAG, "PZEM TX packet sent to uart FIFO, t: %ld", micros());
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


};


/**
 * @brief derrived from UartQ object
 * 
 */
class PZPort : public UartQ {
    bool qrun;
    std::unique_ptr<char[]> descr;

public:
    const uint8_t id;
    const char *getDescr() const;
    bool active() const {return qrun;}
    bool active(bool newstate);

    PZPort (PZPort_cfg &cfg) :
        UartQ(cfg.p, cfg.uartcfg, cfg.gpio_rx, cfg.gpio_tx), id(cfg.id) {
            descr = std::move(cfg.descr);
            qrun = startQueues();
    }
};
