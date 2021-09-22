/*
PZEM EDL - PZEM Event Driven Library

This code implements communication and data exchange with PZEM004T V3.0 module using MODBUS proto
and provides an API for energy metrics monitoring and data processing.

This file is part of the 'PZEM event-driven library' project.

Copyright (C) Emil Muratov, 2021
GitHub: https://github.com/vortigont/pzem-edl
*/


#include "main.h"
using namespace pzmbus;

#define PZEM_UART_PORT   UART_NUM_1       // port attached to pzem (UART_NUM_1 is 2-nd uart port on ESP32, 1-st one is usually busy with USB-to-serial chip)

#define RX_PIN 22                         // custom RX pin number
#define TX_PIN 19                         // custom TX pin number


/*
    This is a small sketch that shows how to work with a low level pzem-edl functions - craft PZEM messages and parse replies.
    Also it could be used to change PZEM's MODBUS slave address so that several of such meters could share same UART port

    1. Connect only ONE single device to ESP32 board UART port using default or any custom pins

    2. Build the sketch and use some terminal programm like putty or Arduino IDE to interact with PZEM and check/set it's MODBUS address

 */


// first we need a UartQ object to handle RX/TX message queues
UartQ *qport;   // portQ object ref


void setup(){
    Serial.begin(115200);       // just an ordinary Serial console to interact with

    // Create a new PortQ object using default UART pins for the port specified
    //qport = new UartQ(PZEM_UART_PORT);

    qport = new UartQ(PZEM_UART_PORT, RX_PIN, TX_PIN);      // or use custom pins
    qport->startQueues();                                   // start queues tasks
    qport->attach_RX_hndlr([](pzmbus::RX_msg *msg){ rx_handler(msg); delete msg; });        // attach call-back function to process pzem replies

    // now we are ready to exchange messages
}



void loop(){

    menu();     // print menu in a loop

}


/**
 * @brief this is our menu message
 * 
 * if offers to check or set MODBUS address for PZEM
 * 
 */
void menu(){

    Serial.println();
    Serial.println("==========================");
    Serial.println("PZEM modbus address setter (be sure to connect only ONE pzem at a time)");
    Serial.println();
    Serial.println("Enter command:");
    Serial.println("1 - run broadcast 'What's your modbus addr?' command");
    Serial.println("2 - Change device MODBUS address");
    Serial.println("3 - Poll for metrics data");
    Serial.println();

    while (Serial.available() == 0); // this is just good-old blocking loop method :)
    int cmd = Serial.parseInt();

    if (cmd ==1)
        get_addr_bcast();           // check address call

    if (cmd ==2)
        set_mbus_addr();            // set address call

    if (cmd ==3)
        get_metrics();            // set address call

}

void get_addr_bcast(){
    TX_msg* m = cmd_get_modbus_addr();      // create message 'report your mod_bus addr' with catch-all destination address
    qport->txenqueue(m);                    // send message to queue
    delay(500);                             // let the reply to be printed before returning to menu (prevent terminal garbage)
}

void set_mbus_addr(){
    Serial.println("Enter new modbus addr in range 1-247");
    while (Serial.available() == 0);        // this is just good-old blocking loop method :)
    int val = Serial.parseInt();

    Serial.printf("Please confirm that you want to set a new addr to '%d'\n", val);
    Serial.println("1 - to 'YES', 0 - to cancel");

    while (Serial.available() == 0);        // this is just good-old blocking loop method :)
    int v = Serial.parseInt();

    if (1 == v){
        TX_msg* m = cmd_set_modbus_addr(val);       // create message 'set MODBUS addr' and send it to the catch-all destination address
                                                    // this will effecively forces any PZEM device to change it's slave device address to the specified value 

        qport->txenqueue(m);
        delay(500);                         // let the reply to be printed before returning to menu (prevent terminal garbage)
    }
}


void get_metrics(){
    TX_msg* m = cmd_get_metrics();          // get data metrics
    qport->txenqueue(m);
    delay(500);                             // let the reply to be printed before returning to menu (prevent terminal garbage)
}

/**
 * @brief this is our call-back routine
 * every packet that PZEM sends pack is processed ehere
 * 
 * @param m 
 */
void rx_handler(pzmbus::RX_msg* m){

    // check if received packet is valid
    if(!m->valid){
        Serial.println("Bad reply packet!");
        Serial.println();
        delay(1000);
        return;
    }

    // we have a nice function that parses the packet and pretty prints it's internals to the terminal
    // let's use it and pass our packet there
    rx_msg_prettyp(m);
 
    delay(2000);
}