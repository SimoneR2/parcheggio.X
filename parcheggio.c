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
#include <stdio.h>
#include <math.h>
#define _XTAL_FREQ 16000000

void configurazione(void);
void park_search(void);
void park_routine(void);
void can_send(void);
void can_interpreter(void);

int spazio_parcheggio = 100;

//variabili per il can
CANmessage msg;
BYTE data [8] = 0;
BYTE data_steering [8] = 0;
BYTE data_speed [8] = 0;
BYTE data_brake [8] = 0;
BYTE data_speed_rx[8] = 0;
BYTE distance_set [8] = 0;
volatile bit distance_error = 0;
volatile bit distance_wait = 0;
volatile bit activation = 0;
volatile bit request_sent = 0;
volatile bit distance_recived = 0;
volatile bit start_operation = 0;
unsigned int set_speed = 0;
bit dir = 0;
//variabili da abs
volatile unsigned int distance_dx = 0; //distanza percorsa (da abs)
volatile unsigned int distance_sx = 0; //distanza percorsa (da abs)
volatile unsigned int distance_average = 0; //media distanza
unsigned int right_speed = 0;
unsigned int left_speed = 0;
unsigned int actual_speed = 0;

volatile unsigned int sensor_distance[8] = 0;

//variabili per ultrasuoni
volatile unsigned char MUX_index = 0;
unsigned char MUX_select[8] = 0;
volatile unsigned int pulse_time = 0;
volatile unsigned int distance = 0;
volatile unsigned char gianni = 0;
volatile unsigned char asus = 0;
volatile unsigned int timerValue2 = 0;

__interrupt(high_priority) void ISR_Alta(void) {

    if (INTCON2bits.INTEDG0 == 1) {
        INTCON2bits.INTEDG0 = 0; //interrupt sul fronte di discesa
        TMR3H = 0;
        TMR3L = 0;
        distance_error = 0;
    } else {
        gianni = TMR3H;
        asus = TMR3L;
        timerValue2 = gianni;
        timerValue2 = ((timerValue2 << 8) | (asus));
        pulse_time = timerValue2 / 2; //500nani->uS
        sensor_distance[MUX_index] = pulse_time / 58; //cm
        INTCON2bits.INTEDG0 = 1; //interrupt fronte di discesa
    }
    INTCONbits.INT0IF = 0;
}

__interrupt(low_priority) void ISR_Bassa(void) {
    //INTERRUPT CANBUS
    if ((PIR3bits.RXB0IF == 1) || (PIR3bits.RXB1IF == 1)) {
        CANreceiveMessage(&msg);

        if ((msg.identifier == COUNT_STOP)&&(msg.RTR != 1)) {
            distance_average = 0;
            distance_dx = msg.data[1];
            distance_dx = ((distance_dx << 8) | msg.data[0]);
            distance_sx = msg.data[3];
            distance_sx = ((distance_sx << 8) | msg.data[2]);
            distance_average = (distance_sx + distance_dx) / 2;
            distance_recived = 1;
        }
        if (msg.identifier == PARK_ASSIST_ENABLE) {
            if (msg.data[0] == 1) {
                activation = 1;
                PORTBbits.RB6 = 1;
            } else {
                activation = 0;
                PORTBbits.RB4 = 0;
                PORTBbits.RB5 = 0;
                PORTBbits.RB6 = 0;
            }
        }
        if (msg.identifier == PARK_ASSIST_BEGIN) {
            start_operation = 1;
        }
        if (msg.identifier == ACTUAL_SPEED) {
            for (unsigned char i = 0; i < 8; i++) {
                data_speed_rx[i] = msg.data[i];
            }
            if (msg.identifier == DISTANCE_SET) {
                distance_wait = 0;
            }

        }

        PIR3bits.RXB0IF = 0;
        PIR3bits.RXB1IF = 0;
    }

    if (INTCONbits.TMR0IF == 1) {
        INTCONbits.INT0IE = 0;
        TMR0H = 0x34; //26mS
        TMR0L = 0xE0;
        if (distance_error == 1) {
            sensor_distance[MUX_index] = 5000;
        }
        MUX_index++;
        if (MUX_index == 8) {
            MUX_index = 0;
        }

        unsigned char gigi = 0;
        gigi = MUX_select[MUX_index];
        PORTA = gigi;
        TRISBbits.RB0 = 0;
        LATBbits.LATB0 = 1; //attiva sensore
        __delay_us(15);
        distance_error = 1;
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
    while(1){
        park_search();
        can_interpreter();
        if ((actual_speed > 0)&&(activation == 1)){
            while (!CANisTxReady());
            CANsendMessage(COUNT_START, data, 8, CAN_CONFIG_STD_MSG & CAN_REMOTE_TX_FRAME & CAN_TX_PRIORITY_0);
            distance_recived = 0;
            while (actual_speed > 10) {
                can_interpreter();
            }
            while (!CANisTxReady());
            CANsendMessage(COUNT_STOP, data, 8, CAN_CONFIG_STD_MSG & CAN_REMOTE_TX_FRAME & CAN_TX_PRIORITY_0);
            while (distance_recived == 0); //aspetta di ricevere il dato
        } else {
            distance_average = 0;
        }
        park_routine();
    }
}

void park_search(void) {
    while ((activation == 1)&&(PORTBbits.RB5 == 0)) {
        if (sensor_distance[0] > 50) { //VALORE RANDOM!!!!
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
        if (distance_recived == 1) {
            if (distance_average > 100) {
                PORTBbits.RB5 = 1;

                data[0] = 1;
                CANsendMessage(PARK_ASSIST_STATE, data, 1, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
                distance_recived = 0;
            } else {
                distance_recived = 0;
                PORTBbits.RB5 = 0;
            }
        }
    }
}

void park_routine(void) {
    if ((distance_average > 0)&&(activation ==1)) {
        while (!CANisTxReady());
        data[0] = distance_average;
        data[1] = distance_average >> 8;
        //CANsendMessage(0xAA, data, 1, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
        data_steering[0] = 90; //ruote in posizione centrale
        set_speed = 100; //mm/s 
        dir = 0; //retromarcia
        distance_set [0] = distance_average;
        while (!CANisTxReady());
        CANsendMessage(DISTANCE_SET, distance_set, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
        distance_wait = 1;
        can_send();
        while (distance_wait == 1) {
            can_send();
            __delay_ms(10);
        }
        set_speed = 0;
        can_send();
    }
    while ((PORTBbits.RB5 == 1)&&(start_operation == 1)) {
        data_steering[0] = 90;
        set_speed = 200; //mm/s
        dir = 0; //retromarcia
    }
}

void can_send(void) {
    while (CANisTxReady() != 1);
    CANsendMessage(STEERING_CHANGE, data_steering, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
    data_speed[0] = set_speed;
    data_speed[1] = (set_speed >> 8);
    data_speed[2] = dir;
    while (CANisTxReady() != 1);
    CANsendMessage(SPEED_CHANGE, data_speed, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
    while (CANisTxReady() != 1);
    CANsendMessage(BRAKE_SIGNAL, data_brake, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_1);

}

void can_interpreter(void) {
    left_speed = data_speed_rx[1];
    left_speed = ((left_speed << 8) | (data_speed_rx[0]));
    right_speed = data_speed_rx[3];
    right_speed = ((right_speed << 8) | (data_speed_rx[2]));
    actual_speed = (right_speed + left_speed) / 2;
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
    PIE3bits.RXB1IE = 1; //abilita interrupt ricezione can bus buffer1
    PIE3bits.RXB0IE = 1; //abilita interrupt ricezione can bus buffer0
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