#pragma once

#include <stdint.h>
#include "trame.h"

void startBitrateMeasurement();
void recordtrame(uint32_t payload_bytes, uint32_t trame_bytes);
void printBitrate();

const char* typeToString(TypeCommunication type);
void printBytes(const uint8_t* data, size_t length);
void printTrame(const char* tag, const Trame& trame);

