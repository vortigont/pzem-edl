/*
PZEM EDL - PZEM Event Driven Library

This code implements communication and data exchange with PZEM004T V3.0 module using MODBUS proto
and provides an API for energy metrics monitoring and data processing.

This file is part of the 'PZEM event-driven library' project.

Copyright (C) Emil Muratov, 2021
GitHub: https://github.com/vortigont/pzem-edl
*/

#pragma once
#include <stdint.h>
#include <sys/types.h>

namespace modbus {

/**
 * @brief calculate crc16 over *data array using CRC16_MODBUS precomputed table
 * 
 * @param data - byte array, must be 2 bytes at least
 * @param size  - array size
 * @return uint16_t CRC16
 */
uint16_t crc16(const uint8_t *data, uint16_t size);

// Check MODBUS CRC16 over provided data vector
bool checkcrc16(const uint8_t *buf, uint16_t len);

/**
 * @brief set CRC16 to the byte array at position (len-2)
 * CRC16 calculated up to *data[len-2] bytes
 * @param data - source array fer
 * @param len - array size
 */
void setcrc16(uint8_t *data, uint16_t len);

}   // end of 'namespace modbus'