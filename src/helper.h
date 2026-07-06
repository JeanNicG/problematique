/*
Auteur: Jean-Nicolas Gosselin et Anahì Michelle Mongelos Toledo
CIP: gosj2008 et mona3503
Date: 7 Juillet 2026
*/

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
