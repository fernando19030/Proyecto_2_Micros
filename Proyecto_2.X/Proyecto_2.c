// Archivo:  Proyecto_2.c
// Dispositivo:	PIC16F887
// Autor:    Fernando Arribas
// Compilador:	pic-as (v2.31), MPLABX V5.45
// 
// Programa: Conversion de analago a digital con potenciometros y control de
//           2 servos con PWM
// Hardware: 2 servos en PORTC y 2 potenciometros en PORTA 
//
// Creado: 25 may, 2021
// Ultima modificacion: 25 may, 2021

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
#pragma config LVP = OFF         // Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdio.h>

//******************************************************************************
// Directivas del Compilador
//******************************************************************************

#define _XTAL_FREQ  8000000     //Definimos el valor del reloj para los delays


//******************************************************************************
// Variables
//******************************************************************************
int speed;
int counter;
char servo1;
char servo2;
char lec1;
char lec2;
char dato;
char direccion;
char flag;

//******************************************************************************
// Prototipos de funciones
//******************************************************************************
void setup(void);           //Definimos las funciones que vamos a utilizar 
void escribir_EEPROM (char dato, char direccion);
char leer_EEPROM (char direccion);
void putch(char data);
void comunicacion (void);

//******************************************************************************
// Interupción
//******************************************************************************
void __interrupt() isr(void)
{   
    if (PIR1bits.ADIF == 1) {
    
        if  (ADCON0bits.CHS == 0) { //Verificamos el canal que se esta convirtiendo
            servo1 = ADRESH;                //Dependiendo el canal guardamos el resultado
            CCPR1L = (servo1 >> 1) + 125;         
            CCP1CONbits.DC1B1 = servo1 & 0b01;
            CCP1CONbits.DC1B0 = (servo1 >> 7);
        }
        
        else if (ADCON0bits.CHS == 1) {
            servo2 = ADRESH;                //Dependiendo el canal guardamos el resultado
            CCPR2L = (servo2 >> 1) + 125;         
            CCP2CONbits.DC2B1 = servo2 & 0b01;
            CCP2CONbits.DC2B0 = (servo2 >> 7);
            
        }
        
        else if (ADCON0bits.CHS == 2) {
            speed = ADRESH; 
            
        }
        PIR1bits.ADIF = 0;          //Reiniciamos la interupcion
    }
    
    if (INTCONbits.T0IF ==1) // timer 0 interrupt flag
    {
        counter++;
        INTCONbits.T0IF = 0;         // clear the flag
        TMR0 = 131;           // reset the timer preset count
        
        if (counter >= speed) {
            PORTDbits.RD0 = 0;
            PORTDbits.RD1 = 0;
        }
        
        else {
            PORTDbits.RD0 = 1;
            PORTDbits.RD1 = 1;
        }
        
        if (counter == 256) {
            counter = 0;
        }
        
    }
    
    if (INTCONbits.RBIF == 1) {
        
        if (PORTBbits.RB0 == 0) {
            PORTBbits.RB3 = 1;
            
            escribir_EEPROM(servo1, 0x10);
            escribir_EEPROM(servo2, 0x11);
            
            __delay_ms(1000);
        }
        
        else if (PORTBbits.RB1 == 0) {
            
            ADCON0bits.ADON = 0;
            PORTBbits.RB4 = 1;
            
            lec1 = leer_EEPROM(0x10);
            lec2 = leer_EEPROM(0x11);
            
            CCPR1L = (lec1 >> 1) + 125;
            CCPR2L = (lec2 >> 1) + 125;
            
            __delay_ms(2500);
            ADCON0bits.ADON = 1;
               
        }
        
        else if (PORTBbits.RB2 == 0) {
            
            if (flag == 0) {
                PORTBbits.RB5 = 1;
                flag = 1;
            }
            else {
                PORTBbits.RB5 = 0;
                flag = 0;
            }
        }
        
        else {
            PORTBbits.RB3 = 0;
            PORTBbits.RB4 = 0;
        }
        
        INTCONbits.RBIF = 0;
        
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
        
        if (flag == 1) {
            comunicacion(); 
        }
        else {
            if (ADCON0bits.GO == 0){        //Cuando termine la conversion
                if (ADCON0bits.CHS == 0) {  //Verificamos cual fue el ultimo canal convertido
                    ADCON0bits.CHS = 1;     //Despues cambiamos al siguiente canal
                }
                else if (ADCON0bits.CHS == 1) {
                    ADCON0bits.CHS = 2;
                }
                else if (ADCON0bits.CHS == 2) {
                    ADCON0bits.CHS = 0;
                }
            
                __delay_us(200);            //Esperamos un tiempo para que la conversion
                ADCON0bits.GO = 1;          //termine correctamente
            }
            
        }
        
    }

    return;
}

//******************************************************************************
// Configuracion
//******************************************************************************

void setup(void) {
    //Configuracion de los puertos
    ANSEL   = 0X07;       //Colocamos RA0 y RA1 como entradas analogicas
    ANSELH  = 0X00;       
    
    TRISA   = 0X07;       //Colocamos RA0 y RA1 como entradas y el resto del
    TRISB   = 0x07;
    TRISD   = 0X00;
    
    IOCB    = 0x07;
    OPTION_REGbits.nRBPU = 0;
    WPUB    = 0x07;
    
    PORTA   = 0x00;
    PORTB   = 0x00;
    PORTD   = 0x00;
    
    //Configuracion del Oscilador
    OSCCONbits.IRCF2 = 1;       //Reloj interno de 8MHz
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.SCS   = 1;
            
    //Configuracion Interupciones
    INTCONbits.GIE   = 1;       //Activamos las interupciones ADC 
    INTCONbits.PEIE  = 1;
    INTCONbits.T0IE  = 1;
    INTCONbits.RBIE  = 1;
    PIE1bits.ADIE    = 1;
    
    PIR1bits.ADIF    = 0;       //Limíamos banderas
    INTCONbits.RBIF  = 1;
    INTCONbits.T0IF  = 0;
    
    //Configuracion TMR0
    //Timer0 Registers Prescaler= 64 - TMR0 Preset = 131 - Freq = 250.00 Hz - Period = 0.004000 seconds
    OPTION_REGbits.T0CS = 0;  // bit 5  TMR0 Clock Source Select bit...0 = Internal Clock (CLKO) 1 = Transition on T0CKI pin
    OPTION_REGbits.T0SE = 0;  // bit 4 TMR0 Source Edge Select bit 0 = low/high 1 = high/low
    OPTION_REGbits.PSA = 0;   // bit 3  Prescaler Assignment bit...0 = Prescaler is assigned to the Timer0
    OPTION_REGbits.PS2 = 1;   // bits 2-0  PS2:PS0: Prescaler Rate Select bits
    OPTION_REGbits.PS1 = 0;
    OPTION_REGbits.PS0 = 1;
    TMR0 = 131;               // preset for timer register

    
    //Configuracion ADC
    ADCON1bits.ADFM     = 0;    //Justificado a la izquierda
    ADCON1bits.VCFG0    = 0;    //Colocamos los voltajes de ref como VSS y VDD
    ADCON1bits.VCFG1    = 0;
    
    ADCON0bits.ADCS     = 0b10;    //Reloj de conversion como FOSC/32
    ADCON0bits.CHS      = 0;       //Chanel 0
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
    
    //Configuracion de TX y RX
    TXSTAbits.SYNC  = 0;    //Modo asincrono
    TXSTAbits.BRGH  = 1;    //Activamos la alta velocidad del Baud rate
    
    BAUDCTLbits.BRG16   = 1;    //Utilizamos los 16 bits del Baud rate
    
    SPBRG   = 207;  //Elegimos el baud rate 9600
    SPBRGH  = 0;
    
    RCSTAbits.SPEN  = 1;    //Activamos los puertos seriales
    RCSTAbits.RX9   = 0;    //No utilizamos los nueve bits
    RCSTAbits.CREN  = 1;    //Activamos la recepción continua
    
    TXSTAbits.TXEN  = 1;    //Activamos la transmición
    
    return;
}

//******************************************************************************
//******************************Memoria EEPROM *********************************
//******************************************************************************

void escribir_EEPROM (char dato, char direccion) {
    EEADR = direccion;      //Le indicamos en que localidad se va a guardar 
    EEDAT = dato;           //Dato a guardar en la memoria
    
    INTCONbits.GIE = 0;     //Desactivamos las interupciones globales
    
    EECON1bits.EEPGD = 0;   //Apuntar hacia la data memory
    EECON1bits.WREN = 1;    //Habilitamos la escritura
    
    EECON2 = 0x55;
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;      //Iniciamos la escritura

    while(PIR2bits.EEIF == 0);  //Esperamos a que termine la escritura
    PIR2bits.EEIF = 0;      //Limpiamos la bandera
    
    EECON1bits.WREN = 0;    //Deshabilitamos la escritura
}

char leer_EEPROM (char direccion) {
    EEADR = direccion;      //Le indicamos en que localidad se va a leer
    
    EECON1bits.EEPGD = 0;   //Apuntamos hacia la data memory
    EECON1bits.RD = 1;      //Activamos la lectura 
    
    char dato = EEDATA;     //Guardamos lo que esta en la memoria en la variable
    return dato;            //La operación regresa con una variable
    
}

//******************************************************************************
//**********************************EUSART**************************************
//******************************************************************************
void putch(char data){
    while (TXIF == 0);      //Esperar a que se pueda enviar un nueva caracter
    TXREG = data;           //Transmitir un caracter
    return;
}

void comunicacion (void) {
    
    __delay_ms(100);    //Printf llama a la funcion Putch para enviar todos los
    printf("\rQue accion desea ejecutar?: \r"); //caracteres dentro de las comillas
    __delay_ms(100);    //y mostramos todas las opciones del menu
    printf("    (1) Controlar motores \r");
    __delay_ms(100);
    printf("    (2) EEPROM  \r");
    __delay_ms(100);
    printf("    (3) Terminar \r");
    
    while (RCIF == 0);  //Esperar a que se ingrese un dato de la computadora
    
    if (RCREG == '1') { //Si presionamos 1 mandamos un cadena de caracteres
        __delay_ms(00);
        printf("\r\rQue motor desea controlar:");
        __delay_ms(100);
        printf("\r\r (1)Servomotor 1");
        __delay_ms(100);
        printf("\r\r (2)Servomotor 2");
        __delay_ms(100);
        printf("\r\r (3)DC");
        
        while (RCIF == 0);  //Esperamos a que el usuario ingrese un dato
        
        if (RCREG == '1') {
            __delay_ms(00);
            printf("\r\rQue direccion:");
            __delay_ms(100);
            printf("\r\r (a)Derecha");
            __delay_ms(100);
            printf("\r\r (b)Izquierda");
            __delay_ms(100);
            printf("\r\r (c)Centro");
            
            while (RCIF == 0);  //Esperamos a que el usuario ingrese un dato
            
            if (RCREG == 'a') {
                CCPR1L = (250 >> 1) + 125;
            }
            
            else if (RCREG == 'b') {
                CCPR1L = (0 >> 1) + 125;
            }
            
            else if (RCREG == 'c') {
                CCPR1L = (127 >> 1) + 125;
            }
            
            else {
                NULL;
            }   
        }
        
        if (RCREG == '2') {
            __delay_ms(00);
            printf("\r\rQue altura del aleron:");
            __delay_ms(100);
            printf("\r\r (a)Arriba");
            __delay_ms(100);
            printf("\r\r (b)Abajo");
            __delay_ms(100);
            printf("\r\r (c)Centro");
            
            while (RCIF == 0);  //Esperamos a que el usuario ingrese un dato
            
            if (RCREG == 'a') {
                CCPR2L = (250 >> 1) + 125;
            }
            
            else if (RCREG == 'b') {
                CCPR2L = (0 >> 1) + 125;
            }
            
            else if (RCREG == 'c') {
                CCPR2L = (127 >> 1) + 125;
            }
            
            else {
                NULL;
            }   
        }
        
        if (RCREG == '3') {
            __delay_ms(00);
            printf("\r\rQue velocidad:");
            __delay_ms(100);
            printf("\r\r (a)Ilegal");
            __delay_ms(100);
            printf("\r\r (b)Parar");
            __delay_ms(100);
            printf("\r\r (c)Legal");
            
            while (RCIF == 0);  //Esperamos a que el usuario ingrese un dato
            
            if (RCREG == 'a') {
                speed = 255;
            }
            
            else if (RCREG == 'b') {
                speed = 0;
            }
            
            else if (RCREG == 'c') {
                speed = 127;
            }
            
            else {
                NULL;
            }   
        }
        
    }
    
    else if (RCREG == '2') {    //Si presionamos dos enviamos un caracter a PORTA
        __delay_ms(500);    //Preguntamos el caracter
        printf("\r\rQue desea hacer en la memoria EEPROM:");
        __delay_ms(100);
        printf("\r\r (a)Grabar");
        __delay_ms(100);
        printf("\r\r (b)Leer");
        
        
        while (RCIF == 0);  //Esperamos a que el usuario ingrese un dato
        
        if (RCREG == 'a') {
            PORTBbits.RB3 = 1;
            
            escribir_EEPROM(servo1, 0x10);
            escribir_EEPROM(servo2, 0x11);
            
            __delay_ms(1000);
            PORTBbits.RB3 = 0;
        }
            
        else if (RCREG == 'b') {
            ADCON0bits.ADON = 0;
            PORTBbits.RB4 = 1;
            
            lec1 = leer_EEPROM(0x10);
            lec2 = leer_EEPROM(0x11);
            
            CCPR1L = (lec1 >> 1) + 125;
            CCPR2L = (lec2 >> 1) + 125;
            
            __delay_ms(2500);
            ADCON0bits.ADON = 1;
            PORTBbits.RB4 = 0;
        }
            
        else {
            NULL;
        }
        
    }
    
    else if (RCREG == '3') {    //Si presionamos dos enviamos un caracter a PORTB
        __delay_ms(500);    //Preguntamos el caracter
        printf("\r\rAdios\r");
        flag = 0;
        PORTBbits.RB5 = 0;
    } 
    
    else {  //Si el usuario presiona cualquier otro caracter no sucede nada
        NULL; 
    }
    return;
}