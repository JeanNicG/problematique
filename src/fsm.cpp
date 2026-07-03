#include "fsm.h"

Emetteur::Emetteur(uint8_t volume, const char* const* packets) : volume(volume), packets(packets) {}

void Emetteur::handleEvent(EventEmetteur e) {
    switch (etat) {
        case EtatEmetteur::Initialisation:
            if (e.type == EventTypeEmetteur::DemarrerSession) {
                etat = EtatEmetteur::FluxContinu;
            }
            break;
        case EtatEmetteur::FluxContinu:
            if (e.type == EventTypeEmetteur::NackIntercepte) {
                numero_sequence = e.data.sequence_manquante;
            }
            if (e.type == EventTypeEmetteur::TousPaquetsValides) {
                etat = EtatEmetteur::Fermeture;
            }
            break;
        case EtatEmetteur::Fermeture:
            if (e.type == EventTypeEmetteur::FermetureSession) {
                etat = EtatEmetteur::Initialisation;
            }
            break;
    }
}

Recepteur::Recepteur() = default;

void Recepteur::handleEvent(EventRecepteur e) {
    switch (etat) {
        case EtatRecepteur::Initialisation:
            if (e.type == EventTypeRecepteur::ReceptionTrameDebut) {
                etat = EtatRecepteur::ReceptionContinue;
                volume = e.data.volume;
                numero_sequence_attendu = 1;
            }
            break;
        case EtatRecepteur::ReceptionContinue:
            if (e.type == EventTypeRecepteur::TrameHorsSequence) {
                etat = EtatRecepteur::AttenteDeCorrection;
            }
            if (e.type == EventTypeRecepteur::ReceptionTrameFin) {
                etat = EtatRecepteur::Initialisation;
                numero_sequence_attendu = 0;
            }
            break;
        case EtatRecepteur::AttenteDeCorrection:
            if (e.type == EventTypeRecepteur::ErreurCorrigee) {
                etat = EtatRecepteur::ReceptionContinue;
            }
            break;
    }
}