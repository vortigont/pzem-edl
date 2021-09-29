/*
PZEM EDL - PZEM Event Driven Library

This code implements communication and data exchange with PZEM004T V3.0 module using MODBUS proto
and provides an API for energy metrics monitoring and data processing.

This file is part of the 'PZEM event-driven library' project.

Copyright (C) Emil Muratov, 2021
GitHub: https://github.com/vortigont/pzem-edl
*/

#include <pzem_modbus.hpp>
#ifdef ARDUINO
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
#endif

namespace pzmbus {

TX_msg* create_msg(uint8_t cmd, uint16_t reg_addr, uint16_t value, uint8_t slave_addr, bool w4r){

    TX_msg *msg = new TX_msg(GENERIC_MSG_SIZE);
    if (!msg)
        return nullptr;

    msg->data[0] = slave_addr;
    msg->data[1] = cmd;

    *(uint16_t*)&msg->data[2] = __builtin_bswap16(reg_addr);
    *(uint16_t*)&msg->data[4] = __builtin_bswap16(value);

    msg->w4rx = w4r;

    modbus::setcrc16(msg->data, GENERIC_MSG_SIZE);
    return msg;
}

TX_msg* cmd_set_modbus_addr(uint8_t new_addr, const uint8_t current_addr){
    if(new_addr < ADDR_MIN || new_addr > ADDR_ANY)
        new_addr = current_addr;    // keep the old address if new one is out of allowed range

    return create_msg(static_cast<uint8_t>(pzemcmd_t::WSR), WREG_ADDR, new_addr, current_addr);
}

TX_msg* cmd_energy_reset(const uint8_t addr){

    TX_msg *msg = new TX_msg(ENERGY_RST_MSG_SIZE);

    msg->data[0] = addr;
    msg->data[1] = CMD_RST_ENRG;

    modbus::setcrc16(msg->data, ENERGY_RST_MSG_SIZE);

    return msg;
}


} // end of 'namespace pzmbus'

namespace pz004 {

using namespace pzmbus;

TX_msg* cmd_get_metrics(uint8_t addr){
    return pzmbus::create_msg(static_cast<uint8_t>(pzemcmd_t::RIR), REG_METER_DATA_START, REG_METER_DATA_LEN, addr);
}

TX_msg* cmd_get_opts(const uint8_t addr){
    return pzmbus::create_msg(static_cast<uint8_t>(pzemcmd_t::RHR), WREG_BEGIN, WREG_LEN, addr);
};

TX_msg* cmd_set_modbus_addr(uint8_t new_addr, const uint8_t current_addr){return pzmbus::cmd_set_modbus_addr(new_addr, current_addr);}

TX_msg* cmd_get_modbus_addr(const uint8_t addr){ return cmd_get_opts(addr); }

TX_msg* cmd_get_alarm_thr(const uint8_t addr){ return cmd_get_opts(addr); }

TX_msg* cmd_set_alarm_thr(uint16_t w, const uint8_t addr){
    return pzmbus::create_msg(static_cast<uint8_t>(pzemcmd_t::WSR), WREG_ALARM_THR, w, addr);
}

TX_msg* cmd_energy_reset(const uint8_t addr){return pzmbus::cmd_energy_reset(addr);}

void rx_msg_prettyp(const RX_msg *m){
    pzem_state pz;

    pz.parse_rx_mgs(m, false);

    printf("=== PZEM DATA ===\n");

    switch (static_cast<pzemcmd_t>(m->cmd)){
        case pzemcmd_t::RIR : {
            printf("Packet with metrics data\n");
            printf("Voltage:\t%d dV\t~ %.1f volts\n", pz.data.voltage, pz.data.asFloat(meter_t::vol));
            printf("Current:\t%u mA\t~ %.3f amperes\n", pz.data.current, pz.data.asFloat(meter_t::cur));
            printf("Power:\t\t%u dW\t~ %.1f watts\n", pz.data.power, pz.data.asFloat(meter_t::pwr));
            printf("Energy:\t\t%u Wh\t~ %.3f kWatt*hours\n", pz.data.energy, pz.data.asFloat(meter_t::enrg)/1000 );
            printf("Frequency:\t%d dHz\t~ %.1f Herz\n", pz.data.freq, pz.data.asFloat(meter_t::frq));
            printf("Power factor:\t%d/100\t~ %.2f\n", pz.data.pf, pz.data.asFloat(meter_t::pf));
            printf("Power Alarm:\t%s\n", pz.data.alarm ? "Yes":"No");
            break;
        }
        case pzemcmd_t::RHR : {
            printf("Configured MODBUS address:\t%d\n", pz.addr);
            printf("Configured Alarm threshold:\t%d\n", pz.alrm_thrsh);
            break;
        }
        case pzemcmd_t::WSR : {
            if (m->rawdata[3] == WREG_ADDR){
                printf("Device MODBUS address changed to:\t%d\n", pz.addr);
            } else if (m->rawdata[3] == WREG_ALARM_THR){
                printf("Alarm threshold value changed to:\t%d\n", pz.alrm_thrsh);
            } else {
                printf("Unknown WSR value\n");
            }
            break;
        }
        case pzemcmd_t::reset_energy :
            printf("Energy counter reset!\n");
            break;
        default:
            printf("Other data (to be done)...\n");
            // To be DONE....
            break;
    }
}



}  // end of 'namespace pz004'


// implementation for PZEM003 device
namespace pz003 {

using namespace pzmbus;

TX_msg* cmd_get_metrics(uint8_t addr){
    return pzmbus::create_msg(static_cast<uint8_t>(pzemcmd_t::RIR), PZ003_RIR_DATA_BEGIN, PZ003_RIR_DATA_LEN, addr);
}

TX_msg* cmd_get_opts(const uint8_t addr){
    return pzmbus::create_msg(static_cast<uint8_t>(pzemcmd_t::RHR), PZ003_RHR_BEGIN, PZ003_RHR_CNT, addr);
};

TX_msg* cmd_set_modbus_addr(uint8_t new_addr, const uint8_t current_addr){return pzmbus::cmd_set_modbus_addr(new_addr, current_addr);}

TX_msg* cmd_get_modbus_addr(const uint8_t addr){ return cmd_get_opts(addr); }

TX_msg* cmd_get_alarm_thr(const uint8_t addr){ return cmd_get_opts(addr); }

TX_msg* cmd_set_alarmh_thr(uint16_t w, const uint8_t addr){
    return pzmbus::create_msg(static_cast<uint8_t>(pzemcmd_t::WSR), PZ003_RHR_ALARM_H, w, addr);
}

TX_msg* cmd_set_alarml_thr(uint16_t w, const uint8_t addr){
    return pzmbus::create_msg(static_cast<uint8_t>(pzemcmd_t::WSR), PZ003_RHR_ALARM_L, w, addr);
}

TX_msg* cmd_energy_reset(const uint8_t addr){return pzmbus::cmd_energy_reset(addr);}

void rx_msg_prettyp(const RX_msg *m){
    pzem_state pz;

    pz.parse_rx_mgs(m, false);

    printf("=== PZEM DATA ===\n");

    switch (static_cast<pzemcmd_t>(m->cmd)){
        case pzemcmd_t::RIR : {
            printf("Packet with metrics data\n");
            printf("Voltage:\t%d dV\t~ %.1f volts\n", pz.data.voltage, pz.data.asFloat(meter_t::vol));
            printf("Current:\t%u mA\t~ %.3f amperes\n", pz.data.current, pz.data.asFloat(meter_t::cur));
            printf("Power:\t\t%u dW\t~ %.1f watts\n", pz.data.power, pz.data.asFloat(meter_t::pwr));
            printf("Energy:\t\t%u Wh\t~ %.3f kWatt*hours\n", pz.data.energy, pz.data.asFloat(meter_t::enrg)/1000 );
            printf("Power Alarm H:\t%s\n", pz.data.alarmh ? "Yes":"No");
            printf("Power Alarm L:\t%s\n", pz.data.alarml ? "Yes":"No");
            break;
        }
        case pzemcmd_t::RHR : {
            printf("Configured MODBUS address:\t%d\n", pz.addr);
            printf("Configured Alarm High threshold:\t%d\n", pz.alrmh_thrsh);
            printf("Configured Alarm Low threshold:\t%d\n", pz.alrml_thrsh);
            printf("Configured current range:\t%d\n", pz.irange);
            break;
        }
        case pzemcmd_t::WSR : {
            switch (m->rawdata[3]){
                case PZ003_RHR_ALARM_H :
                    printf("Alarm High threshold value changed to:\t%d\n", pz.alrmh_thrsh);
                    break;
                case PZ003_RHR_ALARM_L :
                    printf("Alarm Low threshold value changed to:\t%d\n", pz.alrml_thrsh);
                    break;
                case PZ003_RHR_ADDR :
                    printf("Device MODBUS address changed to:\t%d\n", pz.addr);
                    break;
                case PZ003_RHR_CURRENT_RANGE :
                    printf("Current range changed to:\t%d\n", pz.irange);
                    break;
                default:
                    printf("Unknown WSR value\n");
                    break;
            }
        }
        case pzemcmd_t::reset_energy :
            printf("Energy counter reset!\n");
            break;
        default:
            printf("Other data (to be done)...\n");
            // To be DONE....
            break;
    }
}

}  // end of 'namespace pz003'

