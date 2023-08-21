/*
PZEM EDL - PZEM Event Driven Library

This code implements communication and data exchange with PZEM004T V3.0 module using MODBUS proto
and provides an API for energy metrics monitoring and data processing.

This file is part of the 'PZEM event-driven library' project.

Copyright (C) Emil Muratov, 2021
GitHub: https://github.com/vortigont/pzem-edl
*/
#include "main.h"
//#include "pzem_edl.hpp"

#include <esp_spiram.h>
#include <esp_himem.h>

/*
    This example shows how to collect TimeSeries data for PZEM metrics
    Pls, check previous examples for basic operations

*/


using namespace pz004;     // we will need this namespace for PZEM004v3.0 device
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

    Serial.printf("\n\n\n\tPZEM004 TimeSeries example\n\n");

/*
    //printf("\nspiram size %u\n", esp_spiram_get_size());
    printf("himem free %u\n", esp_himem_get_free_size());
    printf("himem phys %u\n", esp_himem_get_phys_size());
    printf("himem reserved %u\n", esp_himem_reserved_area_size());
*/

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


    Serial.println("===");

    // print memory stat
    Serial.printf("SRAM Heap total: %d, free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
    Serial.printf("SPI-RAM heap total: %d, SPI-RAM free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
    Serial.printf("ChipRevision %d, Cpu Freq %d, SDK Version %s\n",ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
    Serial.printf("Flash Size %d, Flash Speed %d\n",ESP.getFlashChipSize(), ESP.getFlashChipSpeed());


    Serial.println("\nAllocate sampler buffer");

    // Create Container object for TimeSeries buffers
    TSContainer<pz004::metrics> tsc;

    /**
     * this will create TS object that holds per-second metrics data
     * total 60 samples. Each sample takes about 20 bytes of (SPI)-RAM,
     * It not a problem to stora thouthands if you have SPI-RAM
     * 
     * (esp_timer_get_time() >> 20) is a timestamp marking starting point
     * actually it is just ~seconds from boot
     * 
     */
    uint8_t sec = tsc.addTS(60, esp_timer_get_time() >> 20);        // each 
    Serial.printf("Add per-second TimeSeries, id: %d\n", sec);

    /**
     * the same for 5-seconds interval, 60 samples totals
     * no averaging is done, just saving probes every 5 seconds
     * 
     */
    uint8_t sec5 = tsc.addTS(60, esp_timer_get_time() >> 20, 5);
    Serial.printf("Add per-second TimeSeries, id: %d\n", sec5);

    /**
     * the same for 30-seconds interval, 100 samples totals
     * no averaging is done, just saving probes every 5 seconds
     */
    uint8_t sec30 = tsc.addTS(100, esp_timer_get_time() >> 20, 30);
    Serial.printf("Add per-second TimeSeries, id: %d\n", sec30);

    // check memry usage, if SPI-RAM is available on board, than TS will allocate buffer in SPI-RAM
    Serial.println();
    Serial.printf("SRAM Heap total: %d, free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
    Serial.printf("SPI-RAM heap total: %d, SPI-RAM free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());


    /**
     * Now I need to hookup to the PZEM object, it autopolls every second so I can use it's callback
     * to collect metrics data to TSContainer
     * 
     */
    auto ref = &tsc;    // a ref of our Container to feed it the labda function
    pz->attach_rx_callback([ref](uint8_t pzid, const RX_msg* m){
        auto *data = pz->getMetricsPZ004();             // obtain a pointer to objects metrics
        ref->push(*data, esp_timer_get_time() >> 20);   // push data to TS container and mark it with "seconds' timer
    });


    // OK, 1000 ms is too long to print garbage, let's just take some sleep
    delay(2000);     // for 200 ms

    // and try to check the voltage value
    auto *m = pz->getMetricsPZ004();    // obtain a pointer to objects metrics

    // now we should have some response with updated values, let's check again
    Serial.printf("PZEM voltage: %d (decivolts)\n", m->voltage);

    // and find how long ago we had a reply came back while we were sleeping
    Serial.printf("PZEM data has been updated %lld ms ago\n", pz->getState()->dataAge());

    // let's check other metrics
    Serial.printf("PZEM current: %u (mA)\n", m->current);

    // Or if someone likes floats instead? (I don't)
    Serial.printf("PZEM current as float: %.3f (Amps)\n", m->asFloat(pzmbus::meter_t::cur));      // allowed arguments are in enum meter_t:uint8_t { vol, cur, pwr, enrg, frq, pf, alrm }

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

/*
   int times = 5;
   do {
       long t = random(5000);
        Serial.printf("Going to sleep for %ld ms\n", t);
        delay(t);

        Serial.println("Wake up!");

        Serial.printf("PZEM voltage: %d (decivolts), last update time %lld ms ago\n\n", m->voltage, pz->getState()->dataAge());
   } while(--times);
*/

/*
  Serial.println("Release sampler");
    //pz->ts = new TSNode<pz004::metrics>(512);
    auto t = pz->ts;
    pz->ts = nullptr;
    delete t;

  Serial.printf("Internal Total heap %d, internal Free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
  Serial.printf("SPIRam Total heap %d, SPIRam Free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
*/



/*
    TimeSeries<pz004::metrics>ts(1, 30, 0);
    pz004::metrics mt;

        int x = 10;
        Serial.println("Fill ring buff");

        for (int i = 1; i != 31; ++i){
            mt.voltage = x++;
            ts.push(mt, i);
        }
*/

/*
    auto chain = tsc.getTSchain();
    const auto ts = chain->get(2);
//
        Serial.println("mod ring buff");
        for (auto j = ts->begin(); j != ts->end(); ++j){
            j->voltage += 10;
        } 
//
*/


    // now I do not need to do anything
    // I can just halt here in an endless loop, but every second with a new message
    // console will print metrics data from PZEM
    for (;;){
        delay(5000);

// Print timeseries for 1 2 30 sec
        Serial.println("Print fwd ring buff");

        auto ts = tsc.getTS(sec);

        for (auto _i = ts->cbegin(); _i != ts->cend(); ++_i){
            const auto &d = _i;
            Serial.printf("PZEM voltage, cur, pwr: %d\t%d\t%d\t%d\t%d\n", d->voltage, d->current, d->power, _i->energy, 0);
        }

        Serial.println("Print back ring buff");
        ts = tsc.getTS(sec5);

        for (auto _i = ts->crbegin(); _i != ts->crend(); ++_i){
            const auto &d = _i;
            //d->power += 1;
            Serial.printf("PZEM voltage, cur, pwr: %d\t%d\t%d\t%d\t%d\n", d->voltage, d->current, d->power, _i->energy, 0);
        }


        //Serial.printf("Data count: %d\n", pz->ts->data.size());

/*
        //for (const auto& d : pz->ts){
        for (auto _i = pz->ts->cbegin(); _i != pz->ts->cend(); ++_i){
        //for (const auto& _i = pz->ts->cbegin(); _i != pz->ts->cend(); ++_i){
            const auto &d = _i;
            auto x = *_i;
        //for (int i = 0; i != pz->ts->size; ++i){
            //auto d = pz->ts->get()[i];
            //d->voltage = 100;
            Serial.printf("PZEM voltage, cur, pwr: %d\t%d\t%d\t%d\t%d\n", d->voltage, d->current, d->power, _i->energy, x.freq);
            //Serial.printf("PZEM voltage, cur, pwr: %d\t%d\t%d\n", d[i].voltage, d[i].current, d[i].power);
        }
*/

    // test iterators



/*
        for (auto _i = pz->ts->cbegin(); _i != pz->ts->cend(); ++_i){
            const auto &d = _i;
            //const auto x = _i->;
            Serial.printf("PZEM voltage, cur, pwr: %d\t%d\t%d\t%d\n", d->voltage, d->current, d->power, _i->energy);
            //auto x = &_i;
            
        }
*/
/*
// No iterator
        for (int i = 0; i != pz->ts->size; ++i){
            auto d = pz->ts->get()[i];
            //d->voltage = 100;
            Serial.printf("PZEM voltage, cur, pwr: %d\t%d\t%d\n", d.voltage, d.current, d.power);
            //Serial.printf("PZEM voltage, cur, pwr: %d\t%d\t%d\n", d[i].voltage, d[i].current, d[i].power);
        }
*/
    }
}



void loop(){
    // we do not need this loop at all :)
    for (;;){
        delay(1000);
    }
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
    It is possible to obtain a fresh new data same way as before

    Serial.printf("PZEM current as float: %.3f (Amps)\n", pz->getMetricsPZ004()->asFloat(pzmbus::meter_t::cur));
*/

/*
    It is also possible to work directly on a raw data from PZEM
    let's call for a little help here and use a pretty_printer() function
    that parses and prints RX_msg to the stdout
*/
    rx_msg_prettyp(m);

    // that's the end of a callback
}
