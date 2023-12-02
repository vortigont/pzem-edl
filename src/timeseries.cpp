/*
PZEM EDL - PZEM Event Driven Library

This code implements communication and data exchange with PZEM004T V3.0 module using MODBUS proto
and provides an API for energy metrics monitoring and data processing.

This file is part of the 'PZEM event-driven library' project.

Copyright (C) Emil Muratov, 2021
GitHub: https://github.com/vortigont/pzem-edl
*/

#include "timeseries.hpp"

void MeanAveragePZ004::push(const pz004::metrics& m){
    v += m.voltage;
    c += m.current;
    p += m.power;
    e = m.energy;
    f += m.freq;
    pf += m.pf;
    ++_cnt;
}

pz004::metrics MeanAveragePZ004::get(){
    pz004::metrics _m;
    _m.voltage = v / _cnt;
    _m.current = c / _cnt;
    _m.power = p / _cnt;
    _m.energy = e;
    _m.freq = f / _cnt;
    _m.pf = pf / _cnt;
    return _m;
}

void MeanAveragePZ004::reset(){
    v=c=p=e=f=pf=_cnt = 0;
}
