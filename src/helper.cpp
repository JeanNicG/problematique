/*
Auteur: Jean-Nicolas Gosselin et Anahì Michelle Mongelos Toledo
CIP: gosj2008 et mona3503
Date: 7 Juillet 2026
*/

#include "helper.h"
#include <Arduino.h>

static uint32_t session_start_time = 0;
static uint32_t total_payload_bytes = 0;
static uint32_t total_trame_bytes = 0;

void startBitrateMeasurement() {
    session_start_time = esp_timer_get_time();
    total_payload_bytes = 0;
    total_trame_bytes = 0;
}

void recordtrame(uint32_t payload_bytes, uint32_t trame_bytes) {
    total_payload_bytes += payload_bytes;
    total_trame_bytes += trame_bytes;
}

void printBitrate() {
    float duration_s = (esp_timer_get_time() - session_start_time) / 1000000.0f;
    Serial.printf("Duration: %.2f s\n", duration_s);
    Serial.printf("Payload Bitrate: %.2f bps\n", (total_payload_bytes * 8) / duration_s);
    Serial.printf("Raw Bitrate: %.2f bps\n\n", (total_trame_bytes * 8) / duration_s);
}

const char* typeToString(TypeCommunication type) {
    switch (type) {
        case TypeCommunication::Debut:
            return "Debut";
        case TypeCommunication::Data:
            return "Data";
        case TypeCommunication::Fin:
            return "Fin";
        case TypeCommunication::Nack:
            return "Nack";
        default:
            return "Unknown";
    }
}

void printTrame(const char* tag, const Trame& trame) {
    Serial.printf(
        "%s type=%s seq=%u volume=%u payload_len=%u crc=0x%04X",
        tag,
        typeToString(trame.entete.type),
        trame.entete.numero_sequence,
        trame.entete.volume,
        trame.entete.longueur_payload,
        trame.crc16
    );
    if (trame.entete.type == TypeCommunication::Data){
        Serial.print("\n  payload text: ");
        for (size_t i = 0; i < sizeof(trame.payload); ++i) {
            if (trame.payload[i] == 0) {
                break;
            }
            Serial.write(trame.payload[i]);
        }
    }
    Serial.println();
    if (trame.entete.type == TypeCommunication::Fin) {
        Serial.println();
    }
}

void logTxTrame(const Trame& trame, bool is_emetteur) {
    if (is_emetteur) {
        if (trame.entete.type != TypeCommunication::Nack) {
            printTrame("[TX]", trame);
        }
    } else {
        if (trame.entete.type == TypeCommunication::Nack) {
            printTrame("[TX]", trame);
        }
    }
}

void logRxTrame(const Trame& trame, bool is_emetteur) {
    if (is_emetteur) {
        if (trame.entete.type == TypeCommunication::Nack) {
            printTrame("[RX]", trame);
        }
    } else {
        if (trame.entete.type == TypeCommunication::Data || 
            trame.entete.type == TypeCommunication::Debut || 
            trame.entete.type == TypeCommunication::Fin) {
            printTrame("[RX]", trame);
        }
    }
}