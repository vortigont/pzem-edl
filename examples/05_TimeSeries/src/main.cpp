/*
PZEM EDL - PZEM Event Driven Library

This code implements communication and data exchange with PZEM004T V3.0 module using MODBUS proto
and provides an API for energy metrics monitoring and data processing.

This file is part of the 'PZEM event-driven library' project.

Copyright (C) Emil Muratov, 2023
GitHub: https://github.com/vortigont/pzem-edl
*/
#include <Arduino.h>
#include "pzem_edl.hpp"
#include "timeseries.hpp"
#include <WiFi.h>
#include <ctime>
#include "esp_sntp.h"

/*
    This example shows how to collect TimeSeries data for PZEM metrics
    Pls, check previous examples for basic operations

*/



// For this example we need an internet connection to get time from NTP server

const char* ssid     = "your-ssid";
const char* password = "your-password";
const char* ntpServer1 = "pool.ntp.org";
const char* time_zone = "MSK-3";

using namespace pz004;                  // we will need this namespace for PZEM004v3.0 device

#define PZEM_UART_PORT   UART_NUM_1     // port attached to pzem (UART_NUM_1 is 2-nd uart port on ESP32, 1-st one is usually busy with USB-to-serial chip)

#define RX_PIN 22                       // custom RX pin number
#define TX_PIN 19                       // custom TX pin number

#define PZEM_ID 42                      // this is a unique PZEM ID for reference, it is NOT slave MODBUS address, just some random id
                                        // (we all know why '42' is THE number, right? :) )


// first, we need a placeholder for UartQ object to handle RX/TX message queues
UartQ *qport;

// Also we need a placeholder for PZEM004 object
PZ004 *pz;

// Container object for TimeSeries data
TSContainer<pz004::metrics> tsc;

// IDs for out time series buffers
uint8_t sec1, sec30, sec300;

// I need a timer to do console printing
TimerHandle_t t_5sec;

// forward declarations
void print_wait4data(void*);

// a call-back function that will configure TimeSeries buffers
void setup_timeseries(struct timeval *t);


void setup(){
    Serial.begin(115200);       // just an ordinary Serial console to interact with

    Serial.printf("\n\n\n\tPZEM004 TimeSeries example\n\n");

    // Create new serial Q object attached to specified gpios 
    qport = new UartQ(PZEM_UART_PORT, RX_PIN, TX_PIN);

    // Now let's create a PZEM004 object
    pz = new PZ004(PZEM_ID);

    // and link our port with PZEM object
    pz->attachMsgQ(qport);

    // one last step - we must start PortQ tasks to handle messages
    qport->startQueues();
    // enable polling
    if (pz->autopoll(true))
        Serial.println("Autopolling enabled");
    else
        Serial.println("Sorry, can't autopoll somehow :(");


    Serial.println("===");

    // print memory stat
    Serial.printf("SRAM Heap total: %d, free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
    Serial.printf("SPI-RAM heap total: %d, SPI-RAM free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
    Serial.printf("ChipRevision %d, Cpu Freq %d, SDK Version %s\n",ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
    Serial.printf("Flash Size %d, Flash Speed %d\n",ESP.getFlashChipSize(), ESP.getFlashChipSpeed());

    /*
        TimeSeries data needs a valid time source to function properly
        so we will set an NTP server to sync with first
        and a call-back function to get notification when time will be synchronized with NTP
    */

    /*
        set notification call-back function
        initializing TimeSeries buffer requires a timestamp sequence to start with.
        We use unixtime AKA "epoch time" - the number of seconds that have elapsed since 00:00:00 UTC on 1 January 1970, the Unix epoch.
        For this we need synchronize with NTP before we can set a starting point for TimeSeries.
        So we postpone the configuration to the call-back function which will trigger once time synchronization from NTP is complete
    */
    sntp_set_time_sync_notification_cb( setup_timeseries );

    // configure TimeZone and ntp server
    configTzTime(time_zone, ntpServer1);

    Serial.println();
    Serial.print("[WiFi] Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    // Will try for about 10 seconds (20x 500ms)
    int tryDelay = 500;
    int numberOfTries = 20;        

    // Wait for the WiFi connection event
    while (true) {

        switch(WiFi.status()) {
          case WL_CONNECTED:
            Serial.println("[WiFi] WiFi is connected!");
            Serial.print("[WiFi] IP address: ");
            Serial.println(WiFi.localIP());
            return;
            break;
          default:
            Serial.print("[WiFi] WiFi Status: ");
            Serial.println(WiFi.status());
            break;
        }
        delay(tryDelay);
        
        if(numberOfTries <= 0){
          Serial.print("[WiFi] Failed to connect to WiFi!");
          // Use disconnect function to force stop trying to connect
          WiFi.disconnect();
          return;
        } else {
          numberOfTries--;
        }
    }

}



void loop(){
    // we do not need this loop at all :)
    for (;;){
        delay(1000);
    }
}


// Callback function (get's called when time adjusts via NTP)
void setup_timeseries(struct timeval *t)
{
    Serial.println("Got time adjustment from NTP!");
    Serial.println("\nAllocate sampler buffer");

    /**
     * this will create TS object that holds per-second metrics data
     * total 300 samples will keep per-second data for the duration of 5 minute, then it will roll-over.
     * Each sample takes about 28 bytes of (SPI)-RAM, it's not a problem to store thouthands if you have SPI-RAM
     * 
     */
    sec1 = tsc.addTS(300, time(nullptr) /* current timestamp*/, 1 /* second interval*/, "TimeSeries 1 Second" /* Mnemonic descr*/ );
    Serial.printf("Add per-second TimeSeries, id: %d\n", sec1);

    /**
     * the same for 30-seconds interval, 240 samples totals, will keep data for 120 min
     */
    sec30 = tsc.addTS(240, time(nullptr) /* current timestamp*/, 10 /* second interval*/, "TimeSeries 30 Seconds" /* Mnemonic descr*/ );
    Serial.printf("Add 30 second TimeSeries, id: %d\n", sec30);

    /**
     * the same for 300-seconds interval, 288 samples totals, will keep data for 24 hours
     */
    sec300 = tsc.addTS(288, time(nullptr) /* current timestamp*/, 300 /* second interval*/, "TimeSeries 5 Min" /* Mnemonic descr*/ );
    Serial.printf("Add 5 min TimeSeries, id: %d\n", sec300);

    // check memory usage, if SPI-RAM is available on board, than TS will allocate buffer in SPI-RAM
    Serial.println();
    Serial.printf("SRAM Heap total: %d, free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
    Serial.printf("SPI-RAM heap total: %d, SPI-RAM free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());


    /**
     * Now I need to hookup to the PZEM object, it autopolls every second so I can use it's callback
     * to collect metrics data and push it to TSContainer
     * 
     */
    auto ref = &tsc;    // a ref of our Container to feed it to lambda function
    pz->attach_rx_callback([ref](uint8_t pzid, const RX_msg* m){
        // obtain a pointer to objects metrics and push data to TS container marking it with current timestamp
        ref->push(*(pz->getMetricsPZ004()), time(nullptr));
    });

    // here I will set a timer to do printing task to serial
    t_5sec = xTimerCreate("5sec", pdMS_TO_TICKS(5000), pdTRUE, nullptr, print_wait4data);
    xTimerStart(t_5sec, 100);
};

// function is triggered by a timer each minute
void print_timeseries(){
    // Print timeseries for 1 30 300 sec
    Serial.println();
    Serial.println("=== Print TimeSeries data ===");

    // Process 1 sec data

    // get a ptr to TimeSeries buffer
    auto ts = tsc.getTS(sec1);

    // get const iterator pointing to the begining of the buffer, i.e. the oldest data sample
    auto iter = ts->cbegin();

    // for the sake of simplicity so no to clotter terminal with printin all 300 samples from buffer let's print only 10 most recent samples
    // it will also show how to manipulate with iterators

    size_t cnt = 10;    // we need only 10 samples

    // now we need to shift the iterator from the beginning of the buffer to 'end'-10, i.e. 10 most recent items from the end
    iter += ts->getSize() - cnt;

    Serial.printf("TimeSeries buffer %s has %d items, will only get last %d\n", ts->getDescr(), ts->getSize(), cnt);

    // let's run the iterator and print sampled data 
    for (iter; iter != ts->cend(); ++iter){
        /*
            now I need to get the timestamp for each sample
            TS buffer only stores timestamp for the last sample, not for the each item,
            so I need to calculate the time based on last timestamp, interval and counter 
        */
        auto t = ts->getTstamp() - (ts->cend() - iter) * ts->getInterval();
        Serial.println("TimeStamp\tdV\tmA\tW\tWh\tdHz\tpf");
        Serial.print(t);            Serial.print("\t");
        Serial.print(iter->voltage);   Serial.print("\t");
        Serial.print(iter->current);   Serial.print("\t");
        Serial.print(iter->power);     Serial.print("\t");
        Serial.print(iter->energy);    Serial.print("\t");
        Serial.print(iter->freq);      Serial.print("\t");
        Serial.print(iter->pf);        Serial.println();
    }


    // Let's do same for 30 sec TimeSeries
    // just to make it a bit differetnt let't print human-readable date/time instead of time-stamp

    ts = tsc.getTS(sec30);
    iter = ts->cbegin();
    cnt = 10;    // we need only 10 samples
    iter += ts->getSize() - cnt;
    Serial.println();
    Serial.printf("TimeSeries buffer %s has %d items, will only get last %d\n", ts->getDescr(), ts->getSize(), cnt);
    for (iter; iter != ts->cend(); ++iter){
        std::time_t t = ts->getTstamp() - (ts->cend() - iter) * ts->getInterval();
        char* dtime = std::ctime(&t);
        Serial.println("Date/time\tdV\tmA\tW\tWh\tdHz\tpf");
        Serial.print(dtime);        Serial.print("\t");
        Serial.print(iter->voltage);   Serial.print("\t");
        Serial.print(iter->current);   Serial.print("\t");
        Serial.print(iter->power);     Serial.print("\t");
        Serial.print(iter->energy);    Serial.print("\t");
        Serial.print(iter->freq);      Serial.print("\t");
        Serial.print(iter->pf);        Serial.println();
    }

    // Same for 5 min TimeSeries
    ts = tsc.getTS(sec30);
    iter = ts->cbegin();
    cnt = 10;    // we need only 10 samples
    iter += ts->getSize() - cnt;
    Serial.println();
    Serial.printf("TimeSeries buffer %s has %d items, will only get last %d\n", ts->getDescr(), ts->getSize(), cnt);
    for (iter; iter != ts->cend(); ++iter){
        std::time_t t = ts->getTstamp() - (ts->cend() - iter) * ts->getInterval();
        char* dtime = std::ctime(&t);
        Serial.println("Date/time\tdV\tmA\tW\tWh\tdHz\tpf");
        Serial.print(dtime);        Serial.print("\t");
        Serial.print(iter->voltage);   Serial.print("\t");
        Serial.print(iter->current);   Serial.print("\t");
        Serial.print(iter->power);     Serial.print("\t");
        Serial.print(iter->energy);    Serial.print("\t");
        Serial.print(iter->freq);      Serial.print("\t");
        Serial.print(iter->pf);        Serial.println();
    }


    // An example on how to export TS data in json format via WebServer pls see ESPEM Project https://github.com/vortigont/espem

};

// function is triggered by a timer each 5 sec
void print_wait4data(void*){
    static unsigned cnt = 60;
    Serial.print("Pls, wait, collecting data for ");
    Serial.print(cnt);
    Serial.println(" seconds more...");
    cnt -= 5;
    if (!cnt) {
        cnt = 60;
        // print our collected data
        print_timeseries();
    }
};
