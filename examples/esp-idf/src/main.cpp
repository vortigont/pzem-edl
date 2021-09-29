/*
PZEM EDL - PZEM Event Driven Library

This code implements communication and data exchange with PZEM004T V3.0 module using MODBUS proto
and provides an API for energy metrics monitoring and data processing.

This file is part of the 'PZEM event-driven library' project.

Copyright (C) Emil Muratov, 2021
GitHub: https://github.com/vortigont/pzem-edl
*/
#include "main.h"



/*
    Single PZEM004 device poller running under ESP-IDF,
    could be build without Arduino framework

    see example '01_SinglePZEM004' for more details
*/


using namespace pz004;     // we will need this namespace for PZEM004v3.0 device


extern "C" {
    void app_main();
}

#define PZEM_UART_PORT   UART_NUM_1     // port attached to pzem (UART_NUM_1 is 2-nd uart port on ESP32, 1-st one is usually busy with USB-to-serial chip)

#define RX_PIN 22                       // custom RX pin number
#define TX_PIN 19                       // custom TX pin number

#define PZEM_ID 42                      // this is a unique PZEM ID for reference, it is NOT slave MODBUS address, just some random id
                                        // (we all know why '42' is THE number, right? :) )
// first, we need a UartQ object to handle RX/TX message queues
UartQ *qport;

// Also we need a placeholder for PZEM object
PZ004 *pz;


void app_main() {
    printf("\n\n\n\tPZEM004 single instance example\n\n");

    // Create a new PortQ object
    qport = new UartQ(PZEM_UART_PORT, RX_PIN, TX_PIN);      // or use custom pins

    // Now let's create a PZEM object
    pz = new PZ004(PZEM_ID);

    // and link our port with PZEM object
    pz->attachUartQ(qport);

    // one last step - we must start PortQ tasks to handle messages
    qport->startQueues();

    // let's update it's metrics
    pz->updateMetrics();

    if (pz->autopoll(true)){
        printf("Autopolling enabled\n");
    } else {
        printf("Sorry, can't autopoll somehow :(\n");
    }

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
    for (;;) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief this is a custom callback for newly arrived data from PZEM
 * 
 * @param id - this will be the ID of PZEM object, receiving the data
 * @param m - this will be the struct with PZEM data (not only metrics, but any one)
 */
void mycallback(uint8_t id, const RX_msg* m){

// Here I can get the id of PZEM (might get handy if have more than one attached)
    printf("\nCallback triggered for PZEM ID: %d\n", id);

/*
    It is also possible to work directly on a raw data from PZEM
    let's call for a little help here and use a pretty_printer() function
    that parses and prints RX_msg to the stdout
*/
    rx_msg_prettyp(m);

    // that's the end of a callback
}
