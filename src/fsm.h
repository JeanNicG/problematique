#pragma once
#include <cstdint>

// Emetteur FSM
enum class EtatEmetteur {Initialisation, FluxContinu, Fermeture};
enum class EventTypeEmetteur {DemarrerSession, NackIntercepte, TousPaquetsValides, FermetureSession};
struct EventEmetteur {
    EventTypeEmetteur type;
    union {
        uint8_t sequence_manquante;
    } data;
};

class Emetteur {
    EtatEmetteur etat = EtatEmetteur::Initialisation;
    uint8_t numero_sequence = 0;
    uint8_t volume = 0;
    const char* const* packets = nullptr;
public:
    Emetteur(uint8_t volume, const char* const* packets);
    void handleEvent(EventEmetteur e);
    EtatEmetteur getEtat() const { return etat; }
    uint8_t getNumeroSequence() const { return numero_sequence; }
    void setNumeroSequence(uint8_t seq) { numero_sequence = seq; }
};

// Recepteur FSM
enum class EtatRecepteur { Initialisation, ReceptionContinue, AttenteDeCorrection};
enum class EventTypeRecepteur {ReceptionTrameDebut,TrameHorsSequence, ErreurCorrigee,ReceptionTrameFin};
struct EventRecepteur {
    EventTypeRecepteur type;
    union {
        uint8_t volume;
    } data;
};

class Recepteur {
    EtatRecepteur etat = EtatRecepteur::Initialisation;
    uint8_t numero_sequence_attendu = 1;
    uint8_t volume = 0;
public:
    Recepteur();
    void handleEvent(EventRecepteur e);
    EtatRecepteur getEtat() const { return etat; }
    uint8_t getNumeroSequenceAttendu() const { return numero_sequence_attendu; }
    void setNumeroSequenceAttendu(uint8_t seq) { numero_sequence_attendu = seq; }
};