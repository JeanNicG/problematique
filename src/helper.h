#pragma once

#include <stdint.h>
#include "trame.h"

void startBitrateMeasurement();
void recordtrame(uint32_t payload_bytes, uint32_t trame_bytes);
void printBitrate();

const char* typeToString(TypeCommunication type);
void printTrame(const char* tag, const Trame& trame);

void logTxTrame(const Trame& trame, bool is_emetteur);
void logRxTrame(const Trame& trame, bool is_emetteur);
