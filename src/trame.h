#pragma once
#include <cstddef>
#include <cstdint>


enum TypeCommunication: uint8_t {
    Debut = 0x01,
    Data = 0x02,
    Fin = 0x03,
    Nack = 0x04
};

class __attribute__((packed)) Entete{
    public:
        TypeCommunication type;
        uint8_t numero_sequence;
        uint8_t longueur_payload;
        uint8_t volume;
};

class __attribute__((packed)) Trame{
    public:
        uint8_t preambule  = 0x55;
        uint8_t start = 0x7E;
        Entete entete;
        uint8_t payload[80];
        uint16_t crc16;
        uint8_t end = 0x7E;
        Trame(TypeCommunication type, uint8_t numero_sequence, uint8_t volume, uint8_t payload[80]);
};

const char* typeToString(TypeCommunication type);
void printBytes(const uint8_t* data, size_t length);
void printTrame(const char* tag, const Trame& trame);