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
#include <stdlib.h>
#include <math.h>
#define _XTAL_FREQ 16000000

#define tolleranza 5
void configurazione(void);
void park_search(void);
void park_routine(void);
void can_send(void);
void can_interpreter(void);
void parallelo(void);
void matematica(void);

//variabili programma
int spazio_parcheggio = 100;
float alignment_gap = 0;
float old_alignment_gap = 0;
volatile bit old_data_correction = 0;
volatile bit asd = 0;

float x = 0; //DEBUG
float z = 0; //DEBUG
//variabili per il can
CANmessage msg;
BYTE data [8] = 0;
BYTE data_steering [8] = 0;
BYTE data_speed [8] = 0;
BYTE data_brake [8] = 0;
BYTE data_speed_rx[8] = 0;
BYTE distance_set [8] = 0;
BYTE data_correction [8] = 0;
volatile bit distance_error = 0;
volatile bit distance_wait = 0;
volatile bit activation = 0;
volatile bit request_sent = 0;
volatile bit request_sent1 = 0;
volatile bit distance_received = 0;
volatile bit distance_received1 = 0;
volatile bit start_operation = 0;
volatile bit steering_correction_dir = 0;
unsigned char data_test[8];
volatile bit first = 0; //invia solo una volta il dato

unsigned char steering_correction = 0;
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

//VARIABILI PARCHEGGIO?
float raggio = 51; //52
float larghezza = 32;
volatile float bordo = 0;
float alfa = 0;
float beta = 0;
float n = 0;
float prima_sterzata = 0;
float kkk = 0;
float jjj = 0;
float Pminimo = 0;

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

    if (INTCONbits.TMR0IF == 1) {
        INTCONbits.INT0IE = 0;
        T0CONbits.TMR0ON = 0;
        TMR0H = 0x34; //26mS
        TMR0L = 0xE0;
        T0CONbits.TMR0ON = 1;
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
        INTCON2bits.INTEDG0 = 1; //interrupt fronte di salita
    }
    //INTERRUPT CANBUS
    if ((PIR3bits.RXB0IF == 1) || (PIR3bits.RXB1IF == 1)) {
        CANreceiveMessage(&msg);
        if (msg.identifier == DISTANCE_SET) {
            asd = 0;
        }
        if ((msg.identifier == COUNT_STOP) && (msg.RTR != 1)) {
            distance_average = 0;
            distance_dx = msg.data[1];
            distance_dx = ((distance_dx << 8) | msg.data[0]);
            distance_sx = msg.data[3];
            distance_sx = ((distance_sx << 8) | msg.data[2]);
            distance_average = (distance_sx + distance_dx) / 2;
            distance_received = 1;
            distance_received1 = 1;
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
                request_sent = 0;
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

}

void main(void) {
    MUX_select[0] = 0b00000000;
    MUX_select[1] = 0b00000001;
    MUX_select[2] = 0b00000010;
    MUX_select[3] = 0b00000011;
    MUX_select[4] = 0b00000100;
    MUX_select[5] = 0b00000101;
    MUX_select[6] = 0b00000110;
    MUX_select[7] = 0b00000111;
    configurazione();
    PORTBbits.RB4 = 0;
    PORTBbits.RB5 = 0;
    PORTBbits.RB6 = 0;
    request_sent = 0;
    start_operation = 0;

    //        while (1) {
    //            if (sensor_distance[2] > 20) {
    //                PORTBbits.RB4 = 1;
    //            }
    //            if (sensor_distance[2] > 50) {
    //                PORTBbits.RB5 = 1;
    //            }
    //                    unsigned char data_correction1[];
    //            data_correction1[0] = sensor_distance[2];
    //            data_correction1[1] = (sensor_distance[2]>>8);
    //            CANsendMessage(0xAA, data_correction1, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
    //            delay_s(1);
    //        }
    while (1) {
        while (activation != 1);
        park_search();
        can_interpreter();
        park_routine();
    }
}

void park_search(void) {
    while ((activation == 1) && (PORTBbits.RB5 == 0)) {
        parallelo(); //HEY DUDE HAVE A LOOK HERE!
        if (sensor_distance[0] > 50) { //VALORE RANDOM!!!!
            if (request_sent == 0) {
                while (!CANisTxReady());
                CANsendMessage(COUNT_START, data, 8, CAN_CONFIG_STD_MSG & CAN_REMOTE_TX_FRAME & CAN_TX_PRIORITY_0);
                request_sent = 1;
                alignment_gap = 0; //<==
                LATBbits.LATB4 = 1;
            }
        } else {
            LATBbits.LATB4 = 0;
            alignment_gap = abs(sensor_distance[0] - sensor_distance[1]);
        }
        if ((sensor_distance[0] < 50) && (request_sent == 1)) { //VALORE RANDOM!!!!
            request_sent = 0;
            while (!CANisTxReady());
            CANsendMessage(COUNT_STOP, data, 8, CAN_CONFIG_STD_MSG & CAN_REMOTE_TX_FRAME & CAN_TX_PRIORITY_0);
        }
        if (distance_received == 1) {
            if (distance_average > 65) {
                PORTBbits.RB5 = 1;
                data[0] = 1;
                CANsendMessage(PARK_ASSIST_STATE, data, 1, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
                if ((request_sent1 == 0)&&(sensor_distance[0] < 40)) {
                    data_test[0] = 80;
                    asd = 1;
                    while (!CANisTxReady());
                    CANsendMessage(DISTANCE_SET, data_test, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
                    //CANsendMessage(COUNT_START, data, 8, CAN_CONFIG_STD_MSG & CAN_REMOTE_TX_FRAME & CAN_TX_PRIORITY_0);
                    distance_received1 = 0;
                    request_sent1 = 1;
                }
            } else {
                distance_received = 0;
                PORTBbits.RB5 = 0;
            }
        }
    }
}

void park_routine(void) {
    while ((asd == 1)&&(PORTBbits.RB5 == 1) && (activation == 1)) {
        PORTBbits.RB6 = ~PORTBbits.RB6;
        parallelo();
        while (!CANisTxReady());
        CANsendMessage(STEERING_CORRECTION, data_correction, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
        delay_ms(100);
    }
    if ((PORTBbits.RB5 == 1) && (activation == 1)) {
        PORTBbits.RB6 = 0;
        data[0] = 2;
        CANsendMessage(PARK_ASSIST_STATE, data, 1, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
    }
    while ((start_operation != 1)&&(activation == 1)) {
        if (first == 0) {
            set_speed = 0;
            data_steering[0] = 90;
            data_brake [0] = 0;
            data_brake [1] = 1;
            can_send();
            first = 1;
        }
    }
    while ((PORTBbits.RB5 == 1) && (activation == 1)&&(start_operation == 1)) {
        LATBbits.LATB4 = 1;
        //CANsendMessage(COUNT_STOP, data, 8, CAN_CONFIG_STD_MSG & CAN_REMOTE_TX_FRAME & CAN_TX_PRIORITY_0);
        bordo = (sensor_distance[0] + sensor_distance[1]) / 2;
        matematica();
        set_speed = 0;
        data_steering[0] = 90;
        data_brake [0] = 0;
        data_brake [1] = 1;
        can_send();
        delay_s(1);

        data_brake [0] = 3;
        data_brake [1] = 1;
        //while (distance_received1 == 0);
        set_speed = 1000;
        dir = 0; //indietro
        data_steering[0] = 90;
        data_test[0] = ((80 + Pminimo + tolleranza)-(n + 41));
        asd = 1;
        CANsendMessage(DISTANCE_SET, data_test, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
        can_send();
        while (asd == 1);

        set_speed = 0;
        data_steering[0] = 90;
        data_brake [0] = 0;
        data_brake [1] = 1;
        can_send();
        delay_s(1);

        data_brake [0] = 3;
        data_brake [1] = 1;
        set_speed = 1000;
        dir = 0;
        data_steering[0] = 180;
        data_test[0] = prima_sterzata;
        asd = 1;
        CANsendMessage(DISTANCE_SET, data_test, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
        can_send();
        while (asd == 1);
        set_speed = 1000;
        data_steering[0] = 0;
        asd = 1;
        data_test[0] = prima_sterzata + 13;
        CANsendMessage(DISTANCE_SET, data_test, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
        can_send();
        while (asd == 1);
        data_brake [0] = 0;
        data_brake [1] = 1;
        set_speed = 0;
        data_steering[0] = 90;
        can_send();
        delay_s(1);
        if (sensor_distance [2] > 20) {
            data_brake [0] = 3;
            data_brake [1] = 1;
            set_speed = 500;
            data_steering[0] = 90;
            dir = 1;
            can_send();
            while (sensor_distance [2] > 20);
        }
        data_brake [0] = 0;
        data_brake [1] = 1;
        set_speed = 0;
        data_steering[0] = 90;
        dir = 0;
        can_send();
        delay_s(1);
        activation = 0;
        PORTBbits.RB5 = 0;
        start_operation = 0;
        data[0] = 3;
        while (!CANisTxReady());
        CANsendMessage(PARK_ASSIST_STATE, data, 1, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
        RESET();
    }
}

void can_send(void) {
    if (PORTBbits.RB5 == 1) {
        while (CANisTxReady() != 1);
        CANsendMessage(STEERING_CHANGE, data_steering, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
        data_speed[0] = set_speed;
        data_speed[1] = (set_speed >> 8);
        data_speed[2] = dir;
        while (CANisTxReady() != 1);
        CANsendMessage(SPEED_CHANGE, data_speed, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
        while (CANisTxReady() != 1);
        data_brake[1] = 0;
        CANsendMessage(BRAKE_SIGNAL, data_brake, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
    } else {
        CANsendMessage(STEERING_CORRECTION, data_correction, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
        unsigned char data_correction1[];
        data_correction1[0] = data_correction[0];
        data_correction1[1] = 0;
        CANsendMessage(0xAA, data_correction1, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
    }
}

void parallelo(void) {
    alignment_gap = abs(sensor_distance[0] - sensor_distance[1]);
    if (alignment_gap < 30) {
        if ((sensor_distance[0] < 40) && (sensor_distance[1] < 40)) {
            //alignment_gap =10; //debug qua c'è il problema che anche aligment gap deve essere con valore float, che è diverso da quello int o simili.
            //se scrivo 8 infatti il simulatore mi da un valore tipo 10^-44.....devo scrivere 8.0 e allora lo prende.
            //bisogna vedere cosa fa il programma nelle altre parti e che non faccia casini con il float che gira (oppure bisogna fare una conversione...ma non so come...).
            x = ((100) + (alignment_gap * alignment_gap));
            x = sqrt(x);
            z = alignment_gap / x;
            z = asin(z);
            z = z / M_PI * 180; //trasformazione da radianti a gradi
            data_correction[0] = z;

            if (sensor_distance[0] < sensor_distance[1]) {
                data_correction[1] = 0;
            } else {
                data_correction[1] = 1;
            }
            if ((old_alignment_gap != alignment_gap) || (old_data_correction != data_correction[1])) {
                old_alignment_gap = alignment_gap;
                old_data_correction = data_correction[1];
                can_send();
            }
        } else if (data_correction[0] != old_data_correction) {
            data_correction[0] = 0;
            old_data_correction = 0;
            can_send();
        }
    } else {
        if (old_data_correction != 0) {
            data_correction[0] = 0;
            old_data_correction = 0;
            can_send();
        }
    }

}

void matematica(void) {
    alfa = asin(((2 * raggio)-(larghezza / 2) - (bordo + (larghezza / 2))) / (2 * raggio));
    alfa = alfa / M_PI * 180;
    beta = 90 - alfa;
    alfa = (alfa * M_PI) / 180;
    n = cos(alfa);
    n = 2 * raggio *n;
    prima_sterzata = 2 * M_PI * raggio * beta / 360;
    kkk = raggio + (2 * larghezza / 3);
    jjj = raggio - (2 * larghezza / 3);
    Pminimo = kkk * cos(asin((jjj / kkk)));
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