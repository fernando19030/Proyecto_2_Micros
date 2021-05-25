// Archivo:  Lab09.c
// Dispositivo:	PIC16F887
// Autor:    Fernando Arribas
// Compilador:	pic-as (v2.31), MPLABX V5.45
// 
// Programa: Conversion de analago a digital con potenciometros y control de
//           2 servos con PWM
// Hardware: 2 servos en PORTC y 2 potenciometros en PORTA 
//
// Creado: 27 abr, 2021
// Ultima modificacion: 30 abr, 2021

// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

//******************************************************************************
// Directivas del Compilador
//******************************************************************************

#define _XTAL_FREQ  8000000     //Definimos el valor del reloj para los delays


//******************************************************************************
// Variables
//******************************************************************************

//******************************************************************************
// Prototipos de funciones
//******************************************************************************
void setup(void);           //Definimos las funciones que vamos a utilizar 

//******************************************************************************
// Interupción
//******************************************************************************
void __interrupt() isr(void)
{   
    if (PIR1bits.ADIF == 1) {
        if  (ADCON0bits.CHS == 0) { //Verificamos el canal que se esta convirtiendo
            CCPR1L = (ADRESH>>1) + 125;         //Dependiendo el canal guardamos el resultado
            CCP1CONbits.DC1B1 = ADRESH & 0b01;
            CCP1CONbits.DC1B0 = (ADRESH >> 7);
        }
        
        else {
            CCPR2L = (ADRESH>>1) + 125;
            CCP2CONbits.DC2B1 = ADRESH & 0b01;
            CCP2CONbits.DC2B0 = (ADRESH >> 7);
        }
        PIR1bits.ADIF = 0;          //Reiniciamos la interupcion
    }
    
}

//******************************************************************************
// Ciclo Principal
//******************************************************************************

void main(void) {
    setup();                //Llamamos a la configuracion del PIC
    ADCON0bits.GO   = 1;    //Damos inicio a la conversion
    
    
//******************************************************************************
//Loop principal
//******************************************************************************
    
    while (1) 
    {
        if (ADCON0bits.GO == 0){        //Cuando termine la conversion
            if (ADCON0bits.CHS == 0) {  //Verificamos cual fue el ultimo canal convertido
                ADCON0bits.CHS = 1;     //Despues cambiamos al siguiente canal
            }
            else {
                ADCON0bits.CHS = 0;
            }
            
            __delay_us(200);            //Esperamos un tiempo para que la conversion
            ADCON0bits.GO = 1;          //termine correctamente
        } 
    }

    return;
}

//******************************************************************************
// Configuracion
//******************************************************************************

void setup(void) {
    //Configuracion de los puertos
    ANSEL   = 0X03;       //Colocamos RA0 y RA1 como entradas analogicas
    ANSELH  = 0X00;       
    
    TRISA   = 0X03;       //Colocamos RA0 y RA1 como entradas y el resto del

    PORTA   = 0x00;
    //Configuracion del Oscilador
    OSCCONbits.IRCF2 = 1;       //Reloj interno de 8MHz
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.SCS   = 1;
            
    //Configuracion Interupciones
    INTCONbits.GIE   = 1;       //Activamos las interupciones ADC 
    INTCONbits.PEIE  = 1;
    PIE1bits.ADIE    = 1;
    
    PIR1bits.ADIF    = 0;
    
    //Configuracion ADC
    ADCON1bits.ADFM     = 0;    //Justificado a la izquierda
    ADCON1bits.VCFG0    = 0;    //Colocamos los voltajes de ref como VSS y VDD
    ADCON1bits.VCFG1    = 0;
    
    ADCON0bits.ADCS     = 0b10;    //Reloj de conversion como FOSC/32
    ADCON0bits.CHS      = 0;    //Chanel 0
    __delay_us(200);
    ADCON0bits.ADON     = 1;    //Encendemos el ADC
    __delay_us(200);
    
    //Configuracion PWM
    TRISCbits.TRISC2    = 1;        //RC2/CCP1 como entrada
    TRISCbits.TRISC1    = 1;        //RC1/CCP2 como entrada
    PR2                 = 250;      //Config del periodo 2 ms
    CCP1CONbits.P1M     = 0;        //Config modo PWM
    CCP1CONbits.CCP1M   = 0b1100;   //Le indicamos el modo PWM
    CCP2CONbits.CCP2M   = 0b1100;
    
    CCPR1L              = 0x0f;     //Ciclo de trabajo inicial
    CCPR2L              = 0x0f;
    CCP1CONbits.DC1B    = 0;        //Bits memos significativos
    CCP2CONbits.DC2B0   = 0;
    CCP2CONbits.DC2B1   = 0;
    
    //Configuracion TMR2
    PIR1bits.TMR2IF     = 0;        //Limpiamos la bandera del TMR2
    T2CONbits.T2CKPS    = 0b11;     //Prescaler de 16
    T2CONbits.TMR2ON    = 1;        //Encendemos el TMR2
    
    while (PIR1bits.TMR2IF == 0);   //Esperamos a que se de una interupcion
    PIR1bits.TMR2IF     = 0;        //Limpiamos la bandera del TMR2
    
    TRISCbits.TRISC2    = 0;        //Colocamos RC1 y RC2 como salidas 
    TRISCbits.TRISC1    = 0;
    
    return;
}
