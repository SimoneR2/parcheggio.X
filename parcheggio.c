//RB0 output multiplexer
//RA0-1-2-3 canali multiplexer
//TEMPO TRA INVIO E RICEZIONE(fronte di salita): 800uS
// ======PARK_ASSIST_STATE=====
// ===1: found park space======
// ==2: starting the movement==
// ============================
#include <xc.h>
#include "pic_config.h"
#include "CANlib.h"
#include "delay.c"
#include "delay.h"
#include "idCan.h"
#define _XTAL_FREQ 16000000
#define spazio_parcheggio 1
void configurazione(void);
void park_search(void);

CANmessage msg;
bit activation = 0;
bit request_sent = 0;
unsigned int distance_dx = 0; //distanza percorsa (da abs)
unsigned int distance_sx = 0; //distanza percorsa (da abs)
unsigned int distance_average = 0; //media distanza
volatile unsigned int sensor_distance[8] = 0;
BYTE data [8] = 0;
volatile unsigned char MUX_index = 0;
unsigned char MUX_select[8] = 0;
volatile unsigned int timerValue = 0;
volatile unsigned int timerValue2 = 0;
unsigned int pulse_time = 0;
unsigned int distance = 0;
volatile unsigned char gianni = 0;
volatile unsigned char asus = 0;

__interrupt(high_priority) void ISR_Alta(void) {
    if (INTCON2bits.INTEDG0 == 1) {

        //        gianni = TMR0H;
        //        asus = TMR0L;
        //        timerValue = gianni;
        //        timerValue = ((timerValue << 8) | (asus));
        INTCON2bits.INTEDG0 = 0; //interrupt sul fronte di discesa
        TMR3H = 0;
        TMR3L = 0;

    } else {

        gianni = TMR3H;
        asus = TMR3L;
        timerValue2 = gianni;
        timerValue2 = ((timerValue2 << 8) | (asus));
        pulse_time = timerValue2 / 2; //500nani->uS
        sensor_distance[MUX_index] = pulse_time / 58; //cm
        INTCON2bits.INTEDG0 = 1;

    }
    INTCONbits.INT0IF = 0;
}

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
                distance_average = (distance_dx + distance_sx) / 2;
            }

        }
        PIR3bits.RXB0IF == 0;
        PIR3bits.RXB1IF == 0;
    }

    if (INTCONbits.TMR0IF == 1) {
        INTCONbits.INT0IE = 0;
        TMR0H = 0x34; //26mS
        TMR0L = 0xE0;
        MUX_index++;
        if (MUX_index == 8) {
            MUX_index = 0;
        }
        unsigned char gigi = 0;
        gigi = MUX_select[MUX_index];
        PORTA = gigi;
        //        PORTAbits.RA0 = (MUX_select[MUX_index])&(0b00000001);
        //        PORTAbits.RA1 = ((MUX_select[MUX_index])&(0b00000010)) >> 1;
        //        PORTAbits.RA2 = ((MUX_select[MUX_index])&(0b00000100)) >> 2;
        //        PORTAbits.RA3 = ((MUX_select[MUX_index])&(0b00001000)) >> 3;
        TRISBbits.RB0 = 0;
        LATBbits.LATB0 = 1; //attiva sensore
        __delay_us(15);
        LATBbits.LATB0 = 0; //azzera LATB
        TRISBbits.RB0 = 1;
        INTCONbits.INT0IF = 0;
        INTCONbits.INT0IE = 1;
        INTCONbits.TMR0IF = 0;
    }
}

void main(void) {
    configurazione();
    MUX_select[0] = 0b00000000;
    MUX_select[1] = 0b00000001;
    MUX_select[2] = 0b00000010;
    MUX_select[3] = 0b00000011;
    MUX_select[4] = 0b00000100;
    MUX_select[5] = 0b00000101;
    MUX_select[6] = 0b00000110;
    MUX_select[7] = 0b00000111;

    PORTBbits.RB4 = 0;
    PORTBbits.RB5 = 0;
    PORTBbits.RB6 = 0;

    request_sent = 0;

    while (1) {
        activation = 1; //DEBUG
        park_search();
    }
}

void park_search(void) {
    while ((PORTBbits.RB6 == 1)&&(activation == 1)) {
        if (sensor_distance[0] > 50) { //VALORE RANDOM!!!!
            PORTBbits.RB4 = 1;
            if (request_sent == 0) {
                while (!CANisTxReady());
                CANsendMessage(COUNT_START, data, 8, CAN_CONFIG_STD_MSG & CAN_REMOTE_TX_FRAME & CAN_TX_PRIORITY_0);
                request_sent = 1;
                LATBbits.LATB4 = 1;
            }
        }
        if ((sensor_distance[0] < 50)&&(request_sent == 1)) { //VALORE RANDOM!!!!
            request_sent = 0;
            while (!CANisTxReady());
            CANsendMessage(COUNT_STOP, data, 8, CAN_CONFIG_STD_MSG & CAN_REMOTE_TX_FRAME & CAN_TX_PRIORITY_0);
        }
        if (distance_average > spazio_parcheggio) {
            PORTBbits.RB6 = 1;
            while (!CANisTxReady());
            data[0] = 1;
            CANsendMessage(PARK_ASSIST_STATE, data, 1, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
        }
    }
}

void configurazione(void) {
    LATA = 0x00;
    TRISA = 0b11110000; //ALL IN

    LATB = 0x00;
    TRISB = 0b10001010; //CAN+LED

    LATC = 0x00;
    TRISC = 0b11111111; //RC0 OUTPUT

    ADCON1 = 0xFF;
    CANInitialize(4, 6, 5, 1, 3, CAN_CONFIG_LINE_FILTER_OFF & CAN_CONFIG_SAMPLE_ONCE & CAN_CONFIG_ALL_VALID_MSG & CAN_CONFIG_DBL_BUFFER_ON);

    RCONbits.IPEN = 1;
    PIR3bits.RXB1IF = 0; //azzera flag interrupt can bus buffer1
    PIR3bits.RXB0IF = 0; //azzera flag interrupt can bus buffer0
    IPR3bits.RXB1IP = 0; //interrupt bassa priorità per can
    IPR3bits.RXB0IP = 0; //interrupt bassa priorità per can
    //PIE3bits.RXB1IE = 1; //abilita interrupt ricezione can bus buffer1
    //PIE3bits.RXB0IE = 1; //abilita interrupt ricezione can bus buffer0
    INTCON2bits.TMR0IP = 0; //interrupt bassa priorità timer0
    T0CON = 0x80; //configurazione timer0
    T3CON = 0x11;
    TMR0H = 0x34;
    TMR0L = 0xE0;
    INTCONbits.TMR0IF = 0; //azzera flag interrupt
    INTCONbits.TMR0IE = 1; //abilita interrupt
    INTCON2bits.INTEDG0 = 1;
    INTCONbits.INT0IF = 0;
    INTCONbits.INT0IE = 0;

    INTCONbits.GIEL = 1; //abilita interrupt alta priorità
    INTCONbits.GIEH = 1; //abilita interrupt bassa priorità periferiche
}