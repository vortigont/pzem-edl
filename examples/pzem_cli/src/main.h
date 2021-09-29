/*
PZEM EDL - PZEM Event Driven Library

This code implements communication and data exchange with PZEM004T V3.0 module using MODBUS proto
and provides an API for energy metrics monitoring and data processing.

This file is part of the 'PZEM event-driven library' project.

Copyright (C) Emil Muratov, 2021
GitHub: https://github.com/vortigont/pzem-edl
*/

#include <Arduino.h>
#include "pzem_modbus.hpp"

void menu();
void get_addr_bcast();
void rx_handler(RX_msg* m);
void set_mbus_addr();
void get_metrics();
void reset_nrg();
void get_alrm_thr();
void set_alrm_thr();
