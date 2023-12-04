/*
PZEM EDL - PZEM Event Driven Library

This code implements communication and data exchange with PZEM004T V3.0 module using MODBUS proto
and provides an API for energy metrics monitoring and data processing.

This file is part of the 'PZEM event-driven library' project.

Copyright (C) Emil Muratov, 2021
GitHub: https://github.com/vortigont/pzem-edl
*/

#include "main.h"           // #include "pzem_edl.hpp"   library header

using namespace pz004;     // we will need this namespace for PZEM004v3.0 device

/*
    This is a small sketch that shows how to run a single PZEM004 instance:
    
     - create an event-driven UART Queue object
     - create PZ004 instance
     - attach UART Queue to the PZEM instance
     - mannualy poll for data
     - enable auto-polling
     - use call-backs

    1. Connect only ONE device to ESP32's UART port using default or any other custom pins
    2. Build the sketch and use terminal programm like platformio's devmon, putty or Arduino IDE to check for sketch output

 */

#define PZEM_UART_PORT   UART_NUM_1     // port attached to pzem (UART_NUM_1 is 2-nd uart port on ESP32, 1-st one is usually busy with USB-to-serial chip)

#define RX_PIN 22                       // custom RX pin number
#define TX_PIN 19                       // custom TX pin number

#define PZEM_ID 42                      // this is a unique PZEM ID for reference, it is NOT slave MODBUS address, just some random id
                                        // (we all know why '42' is THE number, right? :) )


// first, we need a UartQ object to handle RX/TX message queues
UartQ *qport;

// Also we need a placeholder for PZEM object
PZ004 *pz;

void setup(){
    Serial.begin(115200);       // just an ordinary Serial console to interact with

    Serial.printf("\n\n\n\tPZEM004 single instance example\n\n");


    // Create a new PortQ object using default UART pins for the port specified
    // qport = new UartQ(PZEM_UART_PORT);

    // OR we can map port to any custom pins
    qport = new UartQ(PZEM_UART_PORT, RX_PIN, TX_PIN);      // or use custom pins

    // Now let's create a PZEM004 object
    pz = new PZ004(PZEM_ID);

    // and link our port with PZEM object
    pz->attachMsgQ(qport);

    // one last step - we must start PortQ tasks to handle messages
    qport->startQueues();

    // now it is all ready to exchange data with PZEM
    // let's update it's metrics
    pz->updateMetrics();

    // and try to check the voltage value
    auto *m = pz->getMetricsPZ004();    // obtain a pointer to objects metrics

    Serial.printf("PZEM voltage: %d (decivolts)\n", m->voltage);

    /*
       But, hey! Wait a minute! Why the voltage value printed in console is ZERO?
        That's the trick! We are currently enqueued request message to PZEM and it will respond back
        some time in the future, somewhere in about ~100 ms... Almost an eternity in CPU cycles :)
        When we checked our metrics for data and printed the result - request might be still on it's
        way to PZEM over serial line.

        Let's not waste the time we have while PZEM is making a reply and do something usefull meanwhile
    */ 
    Serial.println("We can do some funny stuff while PZEM is preparing a reply");

    // OK, 100 ms is too long to print garbage, let's just take some sleep
    delay(200);     // for 200 ms

    // now we should have some response with updated values, let's check again
    Serial.printf("PZEM voltage: %d (decivolts)\n", m->voltage);

    // and find how long ago we had a reply came back while we were sleeping
    Serial.printf("PZEM data has been updated %lld ms ago\n", pz->getState()->dataAge());

    // let's check other metrics
    Serial.printf("PZEM current: %u (mA)\n", m->current);
    Serial.printf("PZEM power: %u (dW)\n", m->power);
    Serial.printf("PZEM energy: %u (Wh)\n", m->energy);
    Serial.printf("PZEM frequency: %u (dHz)\n", m->freq);
    Serial.printf("PZEM Power Factor: %u (<1)\n", m->pf);


    // Or if someone likes floats instead? (I don't)
    Serial.printf("PZEM current as float: %.3f (Amps)\n", m->asFloat(pzmbus::meter_t::cur));      // allowed arguments are in enum meter_t:uint8_t { vol, cur, pwr, enrg, frq, pf, alrm }

    /*
        OK, you may not like this updateMetrics(), than wait, than get values approach in the code, right?
        If u want just some simple 'gimme values' style? Here is the proper way - run autopollig in background!
        Let's enable it!
    */
    if (pz->autopoll(true)){
        Serial.println("Autopolling enabled");
    } else {
        Serial.println("Sorry, can't autopoll somehow :(");
    }

    /*
        Now we can do all kind of other stuff and anytime we ask for values from PZEM object it will give us some fresh data
        (default poll time is 1 second)

        Let's make a simple task - we will sleep for random time from 0 to 5 seconds and on wake we will check PZEM for new data
    */

   int times = 5;
   do {
       long t = random(5000);
        Serial.printf("Going to sleep for %ld ms...\n", t);
        delay(t);

        Serial.println("Wake up!");

        Serial.printf("PZEM voltage: %d (decivolts), last update time %lld ms ago\n\n", m->voltage, pz->getState()->dataAge());
   } while(--times);

    // Cool, huh? :)

    /*
        And the last trick - assume I don't want this sleep/check also, I want to call some action once new data from PZEM arrives,
        i.e. send this data over websocket to a nice GUI (something like im my ESPEM project https://github.com/vortigont/espem).
        I have my own call-back routine and I want PZEM object to pull the string once it gets a fresh reply.
        Check the function mycallback() below for now....
    */

    // let's assign our callback to the PZEM instance.
    // I'm using lambda here to provide functional callback
    pz->attach_rx_callback([](uint8_t pzid, const RX_msg* m){

        // I can do all the required things here, but to keep it sepparate -
        // let's just call our function
        mycallback(pzid, m);
    });

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

    // I can get the id of PZEM (might get handy if have more than one attached)
    Serial.printf("\nCallback triggered for PZEM ID: %d\n", id);

/*
    //It is possible to obtain a fresh new data same way as before

    Serial.printf("PZEM data has been updated %lld ms ago\n", pz->getState()->dataAge());

    auto metrics = pz->getMetricsPZ004();

    Serial.printf("Voltage:\t%d dV\t~ %.1f volts\n", metrics->voltage, metrics->asFloat(pzmbus::meter_t::vol));
    Serial.printf("Current:\t%u mA\t~ %.3f amperes\n", metrics->current, metrics->asFloat(pzmbus::meter_t::cur));
    Serial.printf("Power:\t\t%u dW\t~ %.1f watts\n", metrics->power, metrics->asFloat(pzmbus::meter_t::pwr));
    Serial.printf("Energy:\t\t%u Wh\t~ %.3f kWatt*hours\n", metrics->energy, metrics->asFloat(pzmbus::meter_t::enrg)/1000 );
    Serial.printf("Frequency:\t%d dHz\t~ %.1f Herz\n", metrics->freq, metrics->asFloat(pzmbus::meter_t::frq));
    Serial.printf("Power factor:\t%d/100\t~ %.2f\n", metrics->pf, metrics->asFloat(pzmbus::meter_t::pf));
    Serial.printf("Power Alarm:\t%s\n", metrics->alarm ? "Yes":"No");
*/

/*
    It is also possible to work directly on a raw data from PZEM
    let's call for a little help here and use a pretty_printer() function
    that parses and prints RX_msg to stdout
*/
    rx_msg_prettyp(m);

    // that's the end of a callback
}
