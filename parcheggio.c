#include <xc.h>
#include "pic_config.h"
#include "CANlib.h"
#include "delay.c"
#include "delay.h"
#include "pwm.h"
#include "timers.h"
#include "idCan.h"

CANmessage msg;
bit activation = 0;
bit request_sent = 0;
unsigned int distance_dx = 0;
unsigned int distance_sx = 0;
unsigned int sensor_distance[] = 0;
void configurazione(void);
BYTE data [8] = 0;

__interrupt(low_priority) void ISR_Bassa(void) {
    //INTERRUPT CANBUS
    if ((PIR3bits.RXB0IF == 1) || (PIR3bits.RXB1IF == 1)) {
        if (CANisRxReady()) {
            CANreceiveMessage(&msg);
            if (msg.identifier == PARK_ASSIST_ENABLE) {
                if (msg.data[0] == 1) {
                    activation = 1;
                } else {
                    activation = 0;
                }
            }
            if (msg.identifier == COUNT_STOP) {
                distance_dx = msg.data[1];
                distance_dx = ((distance_dx << 8) | msg.data[0]);
                distance_sx = msg.data[3];
                distance_sx = ((distance_dx << 8) | msg.data[2]);
            }

        }
        PIR3bits.RXB0IF == 0;
        PIR3bits.RXB1IF == 0;
    }
}

void main(void) {
    configurazione();
    while (1) {
        while (activation == 1) {
            if (sensor_distance[0] > 150) { //VALORE RANDOM!!!!
                if (request_sent == 0) { //DA AZZERARE A FINE ROUTINE
                    while (!CANisTxReady()) {
                        CANsendMessage(COUNT_START, data, 8, CAN_CONFIG_STD_MSG & CAN_REMOTE_TX_FRAME & CAN_TX_PRIORITY_0);
                        request_sent = 1;
                    }
                }
            }
        }
    }
}

void configurazione(void) {
    LATA = 0x00;
    TRISA = 0b11111101; //ALL IN

    LATB = 0x00;
    TRISB = 0b11111111; //RB1 e RB2 INPUT

    LATC = 0x00;
    TRISC = 0b11111100; //RC0 OUTPUT

    CANInitialize(4, 6, 5, 1, 3, CAN_CONFIG_LINE_FILTER_OFF & CAN_CONFIG_SAMPLE_ONCE & CAN_CONFIG_ALL_VALID_MSG & CAN_CONFIG_DBL_BUFFER_ON);

    PIR3bits.RXB1IF = 0; //azzera flag interrupt can bus buffer1
    PIR3bits.RXB0IF = 0; //azzera flag interrupt can bus buffer0
    IPR3bits.RXB1IP = 0; //interrupt bassa priorità per can
    IPR3bits.RXB0IP = 0; //interrupt bassa priorità per can
    PIE3bits.RXB1IE = 1; //abilita interrupt ricezione can bus buffer1
    PIE3bits.RXB0IE = 1; //abilita interrupt ricezione can bus buffer0
    INTCONbits.GIEH = 1; //abilita interrupt alta priorità
    INTCONbits.GIEL = 1; //abilita interrupt bassa priorità periferiche
}