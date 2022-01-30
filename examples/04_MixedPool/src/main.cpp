/*
PZEM EDL - PZEM Event Driven Library

This code implements communication and data exchange with PZEM004T V3.0 module using MODBUS proto
and provides an API for energy metrics monitoring and data processing.

This file is part of the 'PZEM event-driven library' project.

Copyright (C) Emil Muratov, 2021
GitHub: https://github.com/vortigont/pzem-edl
*/


#include "main.h"
using namespace pzmbus;     // use general pzem abstractions

/*
    This is a small sketch that shows how to run multiple PZEM instances over two serial ports:

    A setup consists of:
     - 3 PZEM004 devices attached to UART_NUM_1 and monitoring AC lines
     - 2 PZEM003 devices attached to UART_NUM_2 and monitoring DC lines

    Pre Steps
        - each PZEM device must be configured with a unique MODBUS address prior to attaching it to the shared serial lines.
        - check 'pzem_cli' example for an easy way to read/modify PZEM address
        - check 01_SinglePZEM example to get the idea of basic operations

     - create PZPool instance
     - allocate UART port to the pool
     - create PZEM objects
     - mannualy poll for data
     - enable auto-polling
     - use call-backs


    1. Connect several PZEM devices to ESP32's UART port(s) using default or any other custom pins
    2. Build the sketch and use some terminal programm like platformio's devmon, putty or Arduino IDE to check for sketch output

 */

#define PZEM_UART_PORT_1   UART_NUM_1     // port attached to pzem (UART_NUM_1 is 2-nd uart port on ESP32, 1-st one is usually busy with USB-to-serial chip)
#define PZEM_UART_PORT_2   UART_NUM_2     // assume we use 2 ports

#define PORT_1_ID   10      // an ID for port 1
#define PORT_2_ID   20      // an ID for port 2


#define RX_PIN 22           // custom RX pin number for port 1
#define TX_PIN 19           // custom TX pin number for port 1

// those are IDs for AC lines
#define PZEM_ID_1 42        // this is a unique PZEM ID just for reference, it is NOT slave MODBUS address, just some random id
#define PZEM_ID_2 43        // (we all know why '42' is THE number, right? :) )
#define PZEM_ID_3 44


// those are IDs for DC lines
#define PZEM_ID_4 50
#define PZEM_ID_5 51

// those are PZEM004 modbus addresses, must be programmed into PZEM in advance! (use pzem_cli tool to do this)
#define PZEM_1_ADDR 10
#define PZEM_2_ADDR 11
#define PZEM_3_ADDR 12

// those are PZEM003 modbus addresses, must be programmed into PZEM in advance! (use pzem_cli tool to do this)
#define PZEM_4_ADDR 24
#define PZEM_5_ADDR 25


/*
  Let's set a pool of two ports and 5 PZEM devices

  Port one has 3 PZEM's for AC lines
  Port two has 2 PZEM's for DC lines

  We will also give it some fancy names
*/



// We'll need a placeholder for PZPool object
PZPool *meters;

void setup(){
    Serial.begin(115200);       // just an ordinary Serial console to interact with

    Serial.printf("\n\n\n\tPZEM multiple instance example\n\n");

    // create a new PZPool object
    meters = new PZPool();

    // now we must set UART ports

    // for port object we need a config struct
    // this describes port_1 for AC PZEMs
    auto port1_cfg = UART_cfg(PZEM_UART_PORT_1,   // uart number
                                RX_PIN,             // rx pin remapped
                                TX_PIN);             // tx pin remapped

    // Ask PZPool object to create a PortQ object based on config provided
    // it will automatically start event queues for the port and makes it available for PZEM assignment
    if (meters->addPort(PORT_1_ID, port1_cfg, "Phase_lines")){
        Serial.printf("Added port id:%d\n", PORT_1_ID);
    } else {
        Serial.printf("ERR: Can't add port id:%d\n", PORT_1_ID);
    }


    // and another port for attaching to PZEM003 DC lines
    auto port2_cfg = UART_cfg(PZEM_UART_PORT_2,
                                UART_PIN_NO_CHANGE,   // using default pins, no remapping
                                UART_PIN_NO_CHANGE);   // using default pins, no remapping

    // PZEM003 requires custom config for serial port
    port2_cfg.uartcfg.stop_bits  = UART_STOP_BITS_2;          // PZEM003 needs 2 stop bits


    // Ask PZPool object to create a PortQ object based on config provided
    // it will automatically start event queues for the port and makes it available for PZEM assignment
    if (meters->addPort(PORT_2_ID, port2_cfg, "DC_lines")){
        Serial.printf("Added port id:%d\n", PORT_2_ID);
    } else {
        Serial.printf("ERR: Can't add port id:%d\n", PORT_2_ID);
    }

    /* Now, we create PZEM instances one by one attaching it to the corresponding Port IDs
        each PZEM instance must have:
        - unique ID within a pool
        - unique modbus address per port, different ports are allowed to have PZEM's with same address
        - an existing port id to attach to
    */
    // port_1 devs - AC lines
    if (meters->addPZEM(PORT_1_ID, PZEM_ID_1, PZEM_1_ADDR, pzmodel_t::pzem004v3, "Phase_1"))
        Serial.printf("Added PZEM id:%d addr:%d, port id:%d\n", PZEM_ID_1, PZEM_1_ADDR, PZEM_UART_PORT_1);

    if (meters->addPZEM(PORT_1_ID, PZEM_ID_2, PZEM_2_ADDR, pzmodel_t::pzem004v3, "Phase_2"))
        Serial.printf("Added PZEM id:%d addr:%d, port id:%d\n", PZEM_ID_2, PZEM_2_ADDR, PZEM_UART_PORT_1);

    if (meters->addPZEM(PORT_1_ID, PZEM_ID_3, PZEM_3_ADDR, pzmodel_t::pzem004v3, "Phase_3"))
        Serial.printf("Added PZEM id:%d addr:%d, port id:%d\n", PZEM_ID_3, PZEM_3_ADDR, PZEM_UART_PORT_1);

    // port_2 devs - DC lines
    if (meters->addPZEM(PORT_2_ID, PZEM_ID_4, PZEM_4_ADDR, pzmodel_t::pzem003, "Solar Panel"))
        Serial.printf("Added PZEM id:%d addr:%d, port id:%d\n", PZEM_ID_4, PZEM_4_ADDR, PZEM_UART_PORT_2);

    if (meters->addPZEM(PORT_2_ID, PZEM_ID_5, PZEM_5_ADDR, pzmodel_t::pzem003, "Accu's"))
        Serial.printf("Added PZEM id:%d addr:%d, port id:%d\n", PZEM_ID_5, PZEM_5_ADDR, PZEM_UART_PORT_2);

    // now it is all ready to exchange data with PZEMs
    // check 'Single PZEM' example for detailed description

    // let's update metrics for all devs
    meters->updateMetrics();

    // take some sleep while all devs are polled
    delay(200);     // for 200 ms

    // Let's check our 'Phase_1's power

    // obtain a reference to Metrics structure of a specific PZEM instance,
    // it is reuired to cast it to structure for the specific model
    const auto *m1 =(const pz004::metrics*)meters->getMetrics(PZEM_ID_1);

    if (m1){ // sanity check - make sure that a requested PZEM ID is valid and we have a real struct reference
        Serial.printf("Power value for '%s' is %f watts\n", meters->getDescr(PZEM_ID_1), m1->asFloat(meter_t::pwr));
    }


    // Let's check our solar panels voltage

    // obtain a reference to Metrics structure of a specific PZEM instance,
    // it is reuired to cast it to structure for the specific model
    const auto *m4 =(const pz003::metrics*)meters->getMetrics(PZEM_ID_4);

    if (m4){ // sanity check - make sure that a requested PZEM ID is valid and we have a real struct reference
        Serial.printf("Voltage for '%s' is %d volts\n", meters->getDescr(PZEM_ID_1), m4->voltage);
    }


    //    Run autopollig in background for all devs in pool
    if (meters->autopoll(true)){
        Serial.println("Autopolling enabled");
    } else {
        Serial.println("Sorry, can't autopoll somehow :(");
    }

    // let's assign our callback to the PZPool instance.
    // I'm using lambda here to provide functional callback
    meters->attach_rx_callback([](uint8_t pzid, const RX_msg* m){

        // I can do all the required things here, but to keep it sepparate -
        // let's just call our function
        mycallback(pzid, m);
    });

    /*
        just as an example so not to flood console let's change poll period to a lesser rate
        Normally you should not do this, better to always have fresh data and access it on demand
    */
    meters->setPollrate(5000);    // 5 sec

    // now I do not need to do anything
    // I can just halt here in an endless loop, but every second with a new message
    // console will print metrics data from PZEM
    for (;;){
        delay(1000);
    }
}



void loop(){
    // we do not need this loop at all :)
    // might even kill it's task
    vTaskDelete(NULL);
}


/**
 * @brief this is a custom callback for newly arrived data from PZEM
 * 
 * @param id - this will be the ID of PZEM object, receiving the data
 * @param m - this will be the struct with PZEM data (not only metrics, but any one)
 */
void mycallback(uint8_t id, const RX_msg* m){
    Serial.printf("\nTime: %ld / Heap: %d - Callback triggered for PZEM ID: %d, name: %s\n", millis(), ESP.getFreeHeap(), id,  meters->getDescr(id));

/*
    Since we have mexed pool of PZEM devies, we need to find out wich device in particular we've got this message from,
    than we can either use the packet data directly or access metrics struct for the specific PZEM instance 
*/
    switch(meters->getState(id)->model) {
        case pzmodel_t::pzem004v3 : {
            pz004::rx_msg_prettyp(m);       // parse incoming message and print some nice info

            // or we can access struct data for the updated object (an example)
            auto *s = (const pz004::state*)meters->getState(id);
            Serial.printf("===\nPower alarm: %s\n", s->alarm ? "present" : "absent");
            Serial.printf("Power factor: %d\n", s->data.pf);
            Serial.printf("Current value: %f\n", s->data.asFloat(meter_t::cur));
            break;
        }
        case pzmodel_t::pzem003 : {
            pz003::rx_msg_prettyp(m);       // parse incoming message and print some nice info

            // or we can access struct data for the updated object
            auto *s = (const pz003::state*)meters->getState(id);
            Serial.printf("===\nPower high alarm: %s\n", s->alarmh ? "present" : "absent");
            Serial.printf("Power low alarm: %s\n", s->alarml ? "present" : "absent");
            Serial.printf("Energy: %d\n", s->data.energy);
            Serial.printf("Current value: %f\n", s->data.asFloat(meter_t::cur));
            break;
        }
        default:
            break;
    }

    // that's the end of a callback
}
