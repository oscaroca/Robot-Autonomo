
#include <msp432p401r.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "lib_PAE.h"


/* ##################################################################### */
/* ################# INICIO DE DEFINICIÓN DE VARIABLES ################# */
/* ##################################################################### */


typedef uint8_t byte;           // Creamos el tipo de dato 'byte' para facilitarnos la lectura y escritura del codigo.


/* ######## JOYSTICKS ######## */

/*
 * Joystick Centro      P4.1
 * Joystick Derecho     P4.5
 * Joystick Izquierdo   P4.7
 * Joystick Arriba      P5.4
 * Joystick Abajo       P5.5
 */

 //Posicion en el puerto
#define JC_POS 1
#define JD_POS 5
#define JI_POS 7
#define JA_POS 4
#define JB_POS 5

//Posicion en el vector de interrupción
#define JC_INT 0x04
#define JD_INT 0x0C
#define JI_INT 0x10
#define JA_INT 0x0A
#define JB_INT 0x0C

//Bit en el puerto
#define JC_BIT BIT(JC_POS)
#define JD_BIT BIT(JD_POS)
#define JI_BIT BIT(JI_POS)
#define JA_BIT BIT(JA_POS)
#define JB_BIT BIT(JB_POS)

/* ########################## */





/* ### BOTONES DE BOOSTERPACK ### */

/*
 * Boton S1     P5.1
 * Boton S2     P3.5
 */

 //Posicion en el puerto
#define B_SW1_POS 1
#define B_SW2_POS 5

//Posicion en el vector de interrupción
#define B_SW1_INT 0x04
#define B_SW2_INT 0x0C

//Bit en el puerto
#define B_SW1_BIT BIT(B_SW1_POS)
#define B_SW2_BIT BIT(B_SW2_POS)

/* ########################## */





/* ### BOTONES PLACA BASE ### */

/*
 * Boton S1     P1.1
 * Boton S2     P1.4
 */

 //Posicion en el puerto
#define SW1_POS 1
#define SW2_POS 4

//Posicion en el vector de interrupción
#define SW1_INT 0x04            //0000 0100
#define SW2_INT 0x0A            //0000 1010

//Bit en el puerto
#define SW1_BIT BIT(SW1_POS)    //BIT(1)
#define SW2_BIT BIT(SW2_POS)    //BIT(100)

/* ########################## */





/* ### DEFINICIÓN ID PERIFERICOS ### */

//#define USB 1

#ifdef USB              // En caso de estar conectados por USB, definimos los ID que usaremos.
#define ID_motor_L 0x01
#define ID_motor_R 0x02
#define ID_sensor 0x03

#else                   // En caso de estar conectados al robot, definimos los ID que usaremos.
#define ID_motor_L 0x03
#define ID_motor_R 0x02
#define ID_sensor 100
#endif

/* ################################# */








/* ### VARIABLES PARA LA MUSICA ### */

#define La 0
#define La_ 1
#define Si 2
#define Dom 3
#define Dom_ 4
#define Re 5
#define Re_ 6
#define Mi 7
#define Fa 8
#define Fa_ 9
#define Sol 10
#define Sol_ 11


byte i;                         // Variable que usaremos para recorrer el array.
byte melody_length=27;          // Variable que define la longitud de notas de la melodia.
byte melody[27] = {La,Si,Re,Si,Fa_,Fa_,Mi,La,Si,Re,Si,Mi,Mi,Re,Dom_,Si,La,Si,Re,Si,Re,Mi,Dom_,La,La,Mi,Re};  // Array con la melodia en cuestión.
byte ready;                     // Este FLAG nos permitirá definir si estamos ready para tocar la siguiente nota.
byte counter;                   // Variable que nos ayudara a mantener un contador al tocar la melodia.

/* ################################ */




/* ### MAQUINA DE ESTADOS (Main y Autonomo) ### */

#define STATE_AUTONOMO          0x00
#define STATE_STOP              0x01
#define STATE_FORWARD           0x02
#define STATE_BACKWARD          0x03
#define STATE_RIGHT             0x04
#define STATE_LEFT              0x05
#define STATE_SPIN_RIGHT        0x06
#define STATE_SPIN_LEFT         0x07
#define STATE_SPIN_RIGHT_FAST   0x08
#define STATE_SPIN_LEFT_FAST    0x09
#define STATE_SET_L             0x0A
#define STATE_SET_R             0x0B
#define STATE_SET_C             0x0C
#define STATE_SET_DIRECTION     0x0D
#define STATE_MUSIC             0x0E
#define STATE_SHOW_DATA         0x0F

#define NO_WALL                 0x00
#define LEFT_WALL               0x01
#define RIGHT_WALL              0x02

#define ESTADO_DEFAULT 0xFF
byte estado = ESTADO_DEFAULT;             // La variable estado nos servira para definir que función usar desde el main.
byte estado_autonomo = ESTADO_DEFAULT;    // La variable estado_autonomo nos servira para definir que función usar en modo autonomo.

/* ############################################ */





/* ### INICIALIZACIÓN DE UART ### */

#define os16 1                  // Valor del OverSampling
#define UCBR 3                  // Parametro del BaudRate
#define UCBRF 0                 // Parametro del BaudRate
#define UCBRS 0                 // Parametro del BaudRate

/* ############################## */





/* ### VARIABLES PARA SISTEMA DE PALMADAS ### */

byte clap_times = 0;
byte soundLevel;

#define clapLimit 150

/* ########################################## */





/* ### VARIABLES Y FLAGS DEL MOVIMIENTO DEL ROBOT ### */

byte pared;                     // Esta variable nos ayudara a definir si estamos o no siguiendo una pared. Y en que lado. (0 = No. 1 = Left. 2 = Right)
byte direccion;                 // Este variable nos permite definir que dirección tomaremos al encontrarnos con un muro. (0 = Left Wall. 1 = Right Wall)

byte pared_R;                   // Este FLAG nos permite definir si estamos siguiendo una pared por la derecha.
byte pared_L;                   // Este FLAG nos permite definir si estamos siguiendo una pared por la izquierda.
byte girando;                   // Este FLAG nos permite definir si estamos girando.
byte esquina;                   // Este FLAG nos permite definir si nos hemos encontrado con una esquina.

/* ################################################## */





/* ### VARIABLES Y FLAGS DEL RECIBIMIENTO DE PAQUETES (RX) ### */

struct RxReturn {               // Esta estructura nos permitirá almacenar la respuesta Rx.
    byte StatusPacket[9];       // Podremos gestionar correctamente la respuesta si contiene maximo 9 bytes.
};

byte Rx_time_out = 0;           // Este FLAG nos permite definir si ha habido un timeout en Rx.
byte Byte_Recibido;             // Este FLAG nos permite definir si hemos leido un dato del buffer.
byte DatoLeido_UART;            // Esta variable contendrá el byte recibido del buffer.

struct RxReturn answer;         // Esta variable contendrá la respuesta de Rx.
struct RxReturn Pkt;            // Esta variable nos permitirá una posterior gestión de la respuesta Rx.

/* ########################################################### */





/* ### VARIABLES DE LOS SENSORES ### */

byte L_data, C_data, R_data;    // Las variables que usaremos para leer los tres sensores IR del sensor.
byte L_value, C_value, R_value; // Las variables que usaremos para definir los valores establecidos en la calibración.
byte L_max, L_min;              // Las variables que usaremos para definir un rango en el L_value.
byte R_max, R_min;              // Las variabels que usaremos para definir un rango en el R_value.

/* ################################# */





/* ### VARIABLES PARA IMPRIMIR COSAS POR PANTALLA ### */

char cadena[16];                // Variable que nos permitira almacenar la linea de caracteres que se mostrará posteriormente por pantalla.

/* ################################################## */





/* #################################################################### */
/* #################  FIN DE DEFINICIÓN DE VARIABLES  ################# */
/* #################################################################### */






/* ######################################################### */
/* #################  INICIO DEL PROGRAMA  ################# */
/* ######################################################### */



void init_interrupciones(){

    //PORT 1
    NVIC->ICPR[1] |= 1 << (PORT1_IRQn & 31);    // Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[1] |= 1 << (PORT1_IRQn & 31);    // y habilito las interrupciones del puerto

    //PORT 3
    NVIC->ICPR[1] |= 1 << (PORT3_IRQn & 31);    // Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[1] |= 1 << (PORT3_IRQn & 31);    // y habilito las interrupciones del puerto

    //PORT 4
    NVIC->ICPR[1] |= 1 << (PORT4_IRQn & 31);    // Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[1] |= 1 << (PORT4_IRQn & 31);    // y habilito las interrupciones del puerto

    //PORT 5
    NVIC->ICPR[1] |= 1 << (PORT5_IRQn & 31);    // Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[1] |= 1 << (PORT5_IRQn & 31);    // y habilito las interrupciones del puerto

    #ifdef USB
    //USCI_A0
    NVIC->ICPR[0] |= 1 << (EUSCIA0_IRQn);       // Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para Uart
    NVIC->ISER[0] |= 1 << (EUSCIA0_IRQn);       // y habilito las interrupciones del Uart

    #else
    //USCI_A2
    NVIC->ICPR[0] |= 1 << (EUSCIA2_IRQn);       // Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para Uart
    NVIC->ISER[0] |= 1 << (EUSCIA2_IRQn);       // y habilito las interrupciones del Uart
    #endif

    //TIMER A0
    NVIC->ICPR[0] |= 1 << (TA0_0_IRQn);         // Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este timer,
    NVIC->ISER[0] |= 1 << (TA0_0_IRQn);         // y habilito las interrupciones del timer

}



void init_botons(){

    //Configuramos los perifericos del puerto 1
    //Botón S1 y S2 de la placa base
    P1SEL0 &= ~(SW1_BIT + SW2_BIT);             //Els polsadors son GPIOs
    P1SEL1 &= ~(SW1_BIT + SW2_BIT);             //Els polsadors son GPIOs
    P1DIR &= ~(SW1_BIT + SW2_BIT );             //Un polsador es una entrada
    P1REN |= (SW1_BIT + SW2_BIT );              //Pull-up/pull-down pel pulsador
    P1OUT |= (SW1_BIT + SW2_BIT );              //Donat que l'altra costat es GND, volem una pull-up
    P1IE |= (SW1_BIT + SW2_BIT );               //Interrupcions activades
    P1IES &= ~(SW1_BIT + SW2_BIT );             //amb transicio L->H
    P1IFG = 0;                                  //Netegem les interrupcions anteriors

    //Configuramos los perifericos del puerto 3
    //Botón S2 de la Boosterpack
    P3SEL0 &= ~(B_SW2_BIT);                     //Els polsadors son GPIOs
    P3SEL1 &= ~(B_SW2_BIT);                     //Els polsadors son GPIOs
    P3DIR &= ~(B_SW2_BIT );                     //Un polsador es una entrada
    P3REN |= (B_SW2_BIT );                      //Pull-up/pull-down pel pulsador
    P3OUT |= (B_SW2_BIT );                      //Donat que l'altra costat es GND, volem una pull-up
    P3IE |= (B_SW2_BIT );                       //Interrupcions activades
    P3IES &= ~(B_SW2_BIT );                     //amb transicio L->H
    P3IFG = 0;                                  //Netegem les interrupcions anteriors

    //Configuramos los perifericos del puerto 4
    //Joystick Derecha y Joystick Izquierda de la Boosterpack
    P4SEL0 &= ~(JD_BIT + JI_BIT + JC_BIT);      //Els polsadors son GPIOs
    P4SEL1 &= ~(JD_BIT + JI_BIT + JC_BIT);      //Els polsadors son GPIOs
    P4DIR &= ~(JD_BIT + JI_BIT + JC_BIT);       //Un polsador es una entrada
    P4REN |= (JD_BIT + JI_BIT + JC_BIT);        //Pull-up/pull-down pel pulsador
    P4OUT |= (JD_BIT + JI_BIT + JC_BIT);        //Donat que l'altra costat es GND, volem una pull-up
    P4IE |= (JD_BIT + JI_BIT + JC_BIT);         //Interrupcions activades
    P4IES &= ~(JD_BIT + JI_BIT + JC_BIT);       //amb transicio L->H
    P4IFG = 0;                                  //Netegem les interrupcions anteriors

    //Configuramos los perifericos del puerto 5
    //Botón S1, Joystick Arriba y Joystick Abajo de la Boosterpack
    P5SEL0 &= ~(B_SW1_BIT + JA_BIT + JB_BIT);   //Els polsadors son GPIOs
    P5SEL1 &= ~(B_SW1_BIT + JA_BIT + JB_BIT);   //Els polsadors son GPIOs
    P5DIR &= ~(B_SW1_BIT + JA_BIT + JB_BIT);    //Un polsador es una entrada
    P5REN |= (B_SW1_BIT + JA_BIT + JB_BIT);     //Pull-up/pull-down pel pulsador
    P5OUT |= (B_SW1_BIT + JA_BIT + JB_BIT);     //Donat que l'altra costat es GND, volem una pull-up
    P5IE |= (B_SW1_BIT + JA_BIT + JB_BIT);      //Interrupcions activades
    P5IES &= ~(B_SW1_BIT + JA_BIT + JB_BIT);    //amb transicio L->H
    P5IFG = 0;                                  //Netegem les interrupcions anteriors
}



void init_UART(void) {

    /* Inicializamos el Direction Port (pin 3.0) */

    P3SEL0 &= ~(BIT0 );     //Establecemos el Direction_Port como GPIO
    P3SEL1 &= ~(BIT0 );

    //En caso que sea una entrada, es un RxD.
    //En caso que sea una salida, es un TxD.
    P3DIR |= (BIT0);        // Lo inicializamos como RxD.

    #ifdef USB                              // En caso de estar usando USB, usamos la UART 0.

    UCA0CTLW0 |= UCSWRST;                   // Fem un reset de la USCI, desactiva la USCI
    UCA0CTLW0 |= UCSSEL__SMCLK;             // UCSYNC=0 mode asíncron
                                            // UCMODEx=0 seleccionem mode UART
                                            // UCSPB=0 nomes 1 stop bit
                                            // UC7BIT=0 8 bits de dades
                                            // UCMSB=0 bit de menys pes primer
                                            // UCPAR=x ja que no es fa servir bit de paritat
                                            // UCPEN=0 sense bit de paritat
                                            // Triem SMCLK (24MHz) com a font del clock BRCLK

    UCA0MCTLW = UCOS16;                     // Simplemente seguimos los pasos del Technical reference
    UCA0BRW = UCBR;                         // La idea es un Baud Rate de 500kb.Usamos el smlk de 24MHz. N = 24Mhz/500kb = 48; activamos sobremostreo de 16,
                                            // y UCBR0=48/16=3   ,  UCBRFx=0 y UCBRSx=0
                                            // Necessitem sobre-mostreig => bit 0 = UCOS16 = 1
    UCA0MCTLW |= UCBRS;                     // UCBRSx, part fractional del baud rate

    /* Configurem els pins de la UART */
    P1SEL0 |= BIT2 | BIT3;                  // I/O funció: P1.3 = UART0TX, P1.2 = UART0RX
    P1SEL1 &= ~(BIT2 | BIT3 );
    UCA0CTLW0 &= ~UCSWRST;                  // Reactivem la línia de comunicacions sèrie
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A0 RX interrupt, nomes quan tinguem la recepcio


    #else                                   // En caso de estar conectados al robot, usamos la UART 2.

    UCA2CTLW0 |= UCSWRST;                   // Fem un reset de la USCI, desactiva la USCI

    UCA2CTLW0 |= UCSSEL__SMCLK;             // UCSYNC=0 mode asíncron
                                            // UCMODEx=0 seleccionem mode UART
                                            // UCSPB=0 nomes 1 stop bit
                                            // UC7BIT=0 8 bits de dades
                                            // UCMSB=0 bit de menys pes primer
                                            // UCPAR=x ja que no es fa servir bit de paritat
                                            // UCPEN=0 sense bit de paritat
                                            // Triem SMCLK (24MHz) com a font del clock BRCLK

    UCA0MCTLW = UCOS16;                     // Simplemente seguimos los pasos del Technical reference
    UCA0BRW = UCBR;                         // La idea es un Baud Rate de 500kb.Usamos el smlk de 24MHz. N = 24Mhz/500kb = 48; activamos sobremostreo de 16,
                                            // y UCBR0=48/16=3   ,  UCBRFx=0 y UCBRSx=0
                                            // Necessitem sobre-mostreig => bit 0 = UCOS16 = 1
    UCA0MCTLW |= UCBRS;                     // UCBRSx, part fractional del baud rate

    /* Configurem els pins de la UART */
    P3SEL0 |= BIT2 | BIT3;                  // I/O funció: P3.3 = UART2TX, P3.2 = UART2RX
    P3SEL1 &= ~(BIT2 | BIT3 );

    UCA2CTLW0 &= ~UCSWRST;                  // Reactivamos la comunicación en serie.

    EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A2 RX interrupt
    #endif
}


void init_timers(void) {        // Inicializamos los timers.

    /* TIMER A0 */
    // Uamos el Timer A0 para el time_out a la hora de recibir un paquete.
    TIMER_A0->CTL = TIMER_A_CTL_ID__1 | TIMER_A_CTL_SSEL__ACLK | TIMER_A_CTL_CLR | TIMER_A_CTL_MC__UP;  //Divisor entre 1, timer del aclk, clear, modo up, cuenta hasta TA0_CCR0
    TIMER_A0->CCR[0] = 328;     // 100 Hz
    /**
     * Assignamos al CCR el valor de 328 (327,68), ya que corresponde a hacer 328 tics en la frequencia 32768Hz
     * dando como resultado una interrupción cada 0,01s (10ms) lo qual corresponde a 100Hz.
     *
     * 100 Hz = 32768 Hz / 328
     *
     * 10ms = 1/100Hz
     */
}


void Config_UCS() {             // CONFIGURACION MCLK(CPU), SMCLK(PERI), ACLK(PERI)
    init_ucs_24MHz();           //  ACLK a 2^15 y SMCLK 24MHz
    init_UART();                // Inicializamos la UART.
}



void Sentit_Dades_Rx(void) {    // Configuració del Half Duplex dels motors: Recepció
    P3OUT &= ~BIT0;             // El pin P3.0 (DIRECTION_PORT) el posem a 0 (Rx)
}


void Sentit_Dades_Tx(void) {    // Configuració del Half Duplex dels motors: Transmissió
    P3OUT |= BIT0;              // El pin P3.0 (DIRECTION_PORT) el posem a 1 (Tx)
}


#ifdef USB                              // En caso de estar conectados por USB
#define TXD0_READY (UCA0IFG & UCTXIFG)  // Definimos la variable para saber si Tx esta preparado con la UART0.

void TxUAC0(uint8_t bTxdData) {         // Esta función envia un byte de datos por la UART 0.

    while (!TXD0_READY);                // Esta espera a que la UART0 esté preparada para enviar un byte de datos.
    UCA0TXBUF = bTxdData;               // Una vez esta preparada, lo envia.
}

#else                                   // En caso de estar conectados al robot
#define TXD2_READY (UCA2IFG & UCTXIFG)  // Definimos la variable para saber si Tx esta preparado con la UART2.

void TxUAC2(uint8_t bTxdData) {         // Esta función envia un byte de datos por la UART 2.

    while (!TXD2_READY);                // Esta espera a que la UART2 esté preparada para enviar un byte de datos.
    UCA2TXBUF = bTxdData;               // Una vez esta preparada, lo envia.
}
#endif




// RxPacket. Procesa el recibimiento de la respuesta del robot despues de haber enviado una instrucción.
struct RxReturn RxPacket(void){

    struct RxReturn respuesta;                      // Estructura RxReturn que almacenará la respuesta.
    byte bCount, bLenght, bChecksum, bTotal_L;      // Bytes complementarios que nos ayudaran durante el proceso de la función.


    TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE;        // Activamos las interrupciones del timer A0 (10ms)
    TIMER_A0->CTL |= TIMER_A_CTL_CLR;

    for(bCount = 0; bCount < 4; bCount++) {         // Recorremos los cuatro primeros bytes de la respuesta (0xFF, 0xFF, ID, Length).

        Byte_Recibido=0;                            // Establecemos que no se ha recibido ningun byte.
        Rx_time_out = 0;                            // Establecemos que no ha habido time_out.
        TIMER_A0->CTL |= TIMER_A_CTL_CLR;

        while (!Byte_Recibido) {                    // Esperamos a que se haya recibido un byte.
            if (Rx_time_out){                       // En caso que el tiempo de espera del byte supere el time_out (10ms),
                TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIE;   // Desactivamos las interrupciones del timer A0.
                break;                              // Y salimos del while.
            }
        }

        if (Rx_time_out)break;                      // En caso que se haya salido del while a causa del time_out, se sale del for, acabando con el recibimiento de datos.

        respuesta.StatusPacket[bCount] = DatoLeido_UART;    // En caso contrario, entramos el byte leido a la estructura RxReturn.
    }
    if (!Rx_time_out){                              // En caso que NO se haya hecho el time_out

        bLenght = respuesta.StatusPacket[3];        // Se extrae de los datos ya leidos el byte Length.

        for(bCount = 0; bCount < bLenght; bCount++) {   // Recorremos los bytes restantes.

            Byte_Recibido=0;                        // Establecemos que no se ha recibido ningun byte.
            Rx_time_out = 0;                        // Establecemos que no ha habido time_out.
            TIMER_A0->CTL |= TIMER_A_CTL_CLR;

            while (!Byte_Recibido) {                // Esperamos a que se haya recibido un byte.
                if (Rx_time_out){                   // En caso que el tiempo de espera del byte supere el time_out (10ms),
                    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIE;   // Desactivamos las interrupciones del timer A0.
                    break;                          // Y salimos del while.
                }
            }

            if (Rx_time_out)break;                  // En caso que se haya salido del while a causa del time_out, se sale del for, acabando con el recibimiento de datos.

            respuesta.StatusPacket[bCount+4] = DatoLeido_UART;  // En caso contrario, entramos el byte leido a la estructura RxReturn.
        }

        TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIE;   // Desactivamos la interrupción del Timer_A0.
        Rx_time_out = 0;                            // Reestablecemos el valor del time out en caso que se haya establecido mientras acababamos de recibir paquetes.

        // Comprobamos el CheckSum recibido.
        bChecksum = 0;
        bTotal_L = bLenght + 4;
        for (bCount = 2; bCount < bTotal_L-1; bCount++){
            bChecksum += respuesta.StatusPacket[bCount];
        }
        bChecksum = ~bChecksum;

        if(respuesta.StatusPacket[3+bLenght] != bChecksum){
            respuesta.StatusPacket[4] = (respuesta.StatusPacket[4] | BIT7); // Aprovechando que el bit 7 del byte de error esta inutilizado, lo usaremos para definir que ha habido un Checksum en el RxPacket.
        }

    }

    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIE;   // Desactivamos la interrupción del Timer_A0.
    return respuesta;                           // Y devolvemos el paquete RxReturn.
}


//TxPacket() 4 paràmetres: ID del Dynamixel, Mida dels paràmetres, Instruction byte, Parametres. Torna el RxPacket amb el missatge resposta Rx.
struct RxReturn TxPacket(byte bID, byte bParameterLength, byte bInstruction, byte Parametros[16]) {

    byte bCount, bCheckSum, bPacketLength;      // Generamos las variables internas.
    byte TxBuffer[32];                          // Array de bytes que nos permite estructurar todos los datos a enviar.

    Sentit_Dades_Tx();                          // Ponemos el UART en Tx para poder enviar datos.

    TxBuffer[0] = 0xff;                         // Los dos primeros bytes tienen que ser 0xFF, para indicar el inicio del envio.
    TxBuffer[1] = 0xff;
    TxBuffer[2] = bID;                          // ID del modulo al cual queremos enviar el mensaje.
    TxBuffer[3] = bParameterLength + 2;         // Cantidad de bytes que quedan para acabar la trama. (byte instrucción, parametros y checksum)
    TxBuffer[4] = bInstruction;                 // Instrucción a hacer en el modulo.

    for (bCount = 0; bCount < bParameterLength; bCount++) {    // Introducimos los parametros en el TxBuffer.
        TxBuffer[bCount + 5] = Parametros[bCount];
    }

    bCheckSum = 0;                              // Generamos el checksum.
    bPacketLength = bParameterLength + 4 + 2;   // Tamaño total del paquete.

    for (bCount = 2; bCount < bPacketLength - 1; bCount++) {    // Recorremos los bytes a partir del ID (incluido).
        bCheckSum += TxBuffer[bCount];          // Sumamos el valor de cada byte.
    }
    TxBuffer[bCount] = ~bCheckSum;              // Invertimos el valor resultante, correspondiendo al valor final del checksum.


    for (bCount = 0; bCount < bPacketLength; bCount++) {        // Enviamos el mensaje al robot.

        #ifdef USB                              // En caso que estemos conectados a USB
        TxUAC0(TxBuffer[bCount]);               // Enviamos el mensaje por la UART 0.

        #else                                   // En caso que estemos conectados al robot.
        TxUAC2(TxBuffer[bCount]);               // Enviamos el mensaje por la UART 2.
        #endif
    }

    #ifdef USB                                  // En caso que estemos conectados a USB
    while ((UCA0STATW & UCBUSY));               // Comprobamos que se han enviado todos los bytes por la UART 0.

    #else                                       // En caso que estemos conectados al robot.
    while ((UCA2STATW & UCBUSY));               // Comprobamos que se han enviado todos los bytes por la UART 2.
    #endif

    Sentit_Dades_Rx();                          // Ponemos la UART en Rx, para poder recibir datos.

    return RxPacket();                          // Seguido esperamos la respuesta del robot. Haciendo return de la misma.
}





//FUNCIONES DE TX


/*
 * BIT 7. 0
 * BIT 6. Instruction Error
 * BIT 5. Overload Error
 * BIT 4. Checksum Error
 * BIT 3. Range Error
 * BIT 2. Overheating Error
 * BIT 1. Angle Limit Error
 * BIT 0. Input Voltage Error
 */
int Check_error(byte error){                    // Comprueba el byte de error de los RxPacket de respuesta.
    //halLcdClearLine(1);
    if(error != 0x00){                          // En caso que el byte de error sea distinto a 0, sabremos que hay error.

        sprintf(cadena, "!!ERROR: %02x", error);// Se escribe el texto a imprimir
        halLcdPrintLine(cadena, 1, NORMAL_TEXT);// Y se imprime por pantalla el error en questión, para poder analizarlo
        return 1;                               // Devuelve 1 para poder gestionar el error de manera especifica en cada función si se quisiera.
    }
    else if (Rx_time_out){                      // En caso que el byte de time out sea 1, implica que ha habido un timeout.

        Rx_time_out = false;                    // Reestablecemos el timeout.
        halLcdPrintLine("Time out!!", 1, NORMAL_TEXT);// Y se imprime por pantalla el error en questión, para poder analizarlo
        return 2;                               // Devuelve 2 para poder gestionar el error de manera especifica en cada función si se quisiera.
    }

    return 0;                                   // En caso que no haya habido ningun error, devuelve 0.
}


void Set_Motor_LED(byte ID, byte OnOff){        // Envia una instrucción al motor con ID para encender o apagar la LED.

    byte Parametros[16];                        // Array de bytes que usaremos para definir los parametros.
    byte error;                                 // Byte que nos servira para saber si ha habido un error.

    Parametros[0] = 0x19;                       // El primer parametro sera la dirección inicial de memoria a escribir.
    Parametros[1] = OnOff;                      // Asignamos el valor del LED.

    answer = TxPacket(ID, 0x02, 0x03, Parametros);  // Llamamos a la fuinción TxPacket con los datos correspondientes.
                                                    // Esta nos devuelve un RxPacket con la respuesta del motor.

    error = answer.StatusPacket[4];             // Extraemos el byte de error de esta respuesta.
    Check_error(error);                         // Comprobamos que no haya error.
}


void Set_CW_CCW(){          // Envia una instrucción a los dos motores para establecer CW y CCW a 0 para que funcione en ENDLESS Turn(Que el motor gire hasta que le digamos lo contrario).

    byte Parametros[16];    // Array de bytes que usaremos para definir los parametros.
    byte error;             // Byte que nos servira para saber si ha habido un error.

    Parametros[0] = 0x06;   // El primer parametro sera la dirección inicial de memoria a escribir.
    Parametros[1] = 0x00;   // Asignamos el CW  Angle Limit Low  a 0
    Parametros[2] = 0x00;   // Asignamos el CW  Angle Limit High a 0
    Parametros[3] = 0x00;   // Asignamos el CCW Angle Limit Low  a 0
    Parametros[4] = 0x00;   // Asignamos el CCW Angle Limit High a 0

    answer = TxPacket(ID_motor_L, 0x05, 0x03, Parametros);  // Llamamos a la función TxPacket con los datos correspondientes para el motor derecho.
                                                            // Esta nos devuelve una RxPacket con la respuesta del motor.

    error = answer.StatusPacket[4];                         // Extraemos el byte de error de esta respuesta.
    Check_error(error);                                     // Comprobamos que no haya error.

    answer = TxPacket(ID_motor_L, 0x05, 0x03, Parametros);  // Llamamos a la función TxPacket con los datos correspondientes para el motor izquierdo.
                                                            // Esta nos devuelve una RxPacket con la respuesta del motor.

    error = answer.StatusPacket[4];                         // Extraemos el byte de error de esta respuesta.
    Check_error(error);                                     // Comprobamos que no haya error.
}


void Move(uint16_t speed, byte direction, byte ID){ // Envia una instrucción al motor con velocidad, dirección e ID por parametros para establecer el movimiento del mismo.

    byte Parametros[16];                            // Array de bytes que usaremos para definir los parametros.
    byte error;                                     // Byte que nos servira para saber si ha habido un error.

    // Asignamos los valores de los registros Low y High de Mov_speed
    byte Mov_speed_L_L = speed & 0xFF;
    byte Mov_speed_H_L = ((direction << 2) & 0x04) | ((speed >> 8) & 0x03);

    Parametros[0] = 0x20;                           // El primer parametro será la dirección de memoria a escribir.
    Parametros[1] = Mov_speed_L_L;                  // Asignamos el registro Low
    Parametros[2] = Mov_speed_H_L;                  // Asignamos el registro High

    answer = TxPacket(ID, 0x03, 0x03, Parametros);  // Llamamos a la función TxPacket con los datos correspondientes.
                                                    // Esta nos devuelve un RxPacket con la respuesta del motor.

    error = answer.StatusPacket[4];                 // Extraemos el byte de error de esta respuesta.
    Check_error(error);                             // Comprobamos que no haya error.
}


struct RxReturn Read_Sensors() {        // Envia una instrucción al sensor para recibir los datos de los sensores IR.

    byte Parametros[16];                // Array de bytes que usaresos para definir los parametros.
    byte adress = 0x1A;                 // Dirección de donde queremos leer los datos.
    byte amount = 0x03;                 // Cantidad de bytes que queremos leer.
    byte error;                         // Byte que nos servira para saber si ha habido un error.

    Parametros[0] = adress;             // El primer parametro será la dirección.
    Parametros[1] = amount;             // El segundo parametro será la cantidad de Bytes a leer.

    answer = TxPacket(ID_sensor, 0x02, 0x02, Parametros);   // Llamamos a la función TxPacket con los datos correspondientes.
                                                            // Esta nos devuelve un RxPacket con la respuesta del sensor.

    error = answer.StatusPacket[4];     // Extraemos el byte de error de esta respuesta.
    Check_error(error);                 // Comprobamos que no haya error.

    return answer;                      // Devolvemos los datos.
}


void sound(byte sound, byte duration){ // Envia una instruccion para que suene un tiempo determinado

    byte Parametros[16];                            // Array de bytes que usaremos para definir los parametros.
    byte error;                                     // Byte que nos servira para saber si ha habido un error.

    //sound no puede ser mayor de 51, ni duration de 254 (50 para 5 sec max. 254 para inf).
    // Si se utilizan valores mas grandes, es para hacer tonos especificos. Por ejemplo, 255 en duracion y un valor entre 0-26 en sound genera melodias (En teoria, no lo hemos testeado)

    Parametros[0] = 0x28;                       // El primer parametro será la dirección de memoria a escribir.
    Parametros[1] = sound;                      // Asignamos el registro
    Parametros[2] = duration;                   // Asignamos el registro

    answer = TxPacket(ID_sensor, 0x03, 0x03, Parametros);  // Llamamos a la función TxPacket con los datos correspondientes.
                                                    // Esta nos devuelve un RxPacket con la respuesta del sensor

    error = answer.StatusPacket[4];                 // Extraemos el byte de error de esta respuesta.
    Check_error(error);                             // Comprobamos que no haya error.
}


struct RxReturn Read_Time_Sound() {        // Envia una instrucción al sensor para recibir el dato de la duracion del sonido. Si es 0, es que no hay sonido.

    byte Parametros[16];                // Array de bytes que usaresos para definir los parametros.
    byte adress = 0x29;                 // Dirección de donde queremos leer los datos.
    byte amount = 0x01;                 // Cantidad de bytes que queremos leer.
    byte error;                         // Byte que nos servira para saber si ha habido un error.

    Parametros[0] = adress;             // El primer parametro será la dirección.
    Parametros[1] = amount;             // El segundo parametro será la cantidad de Bytes a leer.

    answer = TxPacket(ID_sensor, 0x02, 0x02, Parametros);   // Llamamos a la función TxPacket con los datos correspondientes.
                                                            // Esta nos devuelve un RxPacket con la respuesta del sensor.

    error = answer.StatusPacket[4];     // Extraemos el byte de error de esta respuesta.
    Check_error(error);                 // Comprobamos que no haya error.

    return answer;                      // Devolvemos los datos.
}


struct RxReturn Read_Sound() {        // Envia una instrucción al sensor para recibir la intensidad del sonido que detecta

    byte Parametros[16];                // Array de bytes que usaresos para definir los parametros.
    byte adress = 0x23;                 // Dirección de donde queremos leer los datos.
    byte amount = 0x01;                 // Cantidad de bytes que queremos leer.
    byte error;                         // Byte que nos servira para saber si ha habido un error.

    Parametros[0] = adress;             // El primer parametro será la dirección.
    Parametros[1] = amount;             // El segundo parametro será la cantidad de Bytes a leer.

    answer = TxPacket(ID_sensor, 0x02, 0x02, Parametros);   // Llamamos a la función TxPacket con los datos correspondientes.
                                                            // Esta nos devuelve un RxPacket con la respuesta del sensor.

    error = answer.StatusPacket[4];     // Extraemos el byte de error de esta respuesta.
    Check_error(error);                 // Comprobamos que no haya error.

    return answer;                      // Devolvemos los datos.
}




// MOVIMIENTOS DEL ROBOT

void Move_Forward() {                   // Efectua un movimiento hacia adelante
    Move(0x0FFF, 0X01, ID_motor_R);     // Activa el movimiento del motor Derecho   en sentido 1 y velocidad 0x0FF
    Move(0x0FFF, 0X00, ID_motor_L);     // Activa el movimiento del motor Izquierdo en sentido 0 y velocidad 0x0FF
}


void Move_Backward() {                  // Efectua un movimiento hacia atras
    Move(0X0CFF, 0X00, ID_motor_R);     // Activa el movimiento del motor Derecho   en sentido 0 y velocidad 0x0FF
    Move(0X0CFF, 0X01, ID_motor_L);     // Activa el movimiento del motor Izquierdo en sentido 1 y velocidad 0x0FF
} 

void Move_Right() {                     // Efectua un giro hacia la derecha
    Move(0X04FF, 0X01, ID_motor_R);     // Activa el movimiento del motor Derecho   en sentido 1 y velocidad 0xFFF
    Move(0X0FFF, 0X00, ID_motor_L);     // Activa el movimiento del motor Izquierdo en sentido 0 y velocidad 0x0FF
}

void Move_Left() {                      // Efectua un giro hacia la izquierda.
    Move(0X0FFF, 0X01, ID_motor_R);     // Activa el movimiento del motor Derecho   en sentido 1 y velocidad 0x0FF
    Move(0X04FF, 0X00, ID_motor_L);     // Activa el movimiento del motor Izquierdo en sentido 0 y velocidad 0xFFF
}

void Spin_L_Fast() {                    // Efectua un giro cerrado hacia la izquierda.
    Move(0X0FFF, 0x01, ID_motor_R);     // Activa el movimiento del motor Derecho   en sentido 1 y velocidad 0xFFF.
    Move(0X0FFF, 0x01, ID_motor_L);     // Activa el movimiento del motor Izquierdo en sentido 1 y velocidad 0xFFF.
} 

void Spin_L() {                         // Efectua un giro cerrado hacia la izquierda.
    Move(0X0AFF, 0x01, ID_motor_R);     // Activa el movimiento del motor Derecho   en sentido 1 y velocidad 0xFFF.
    Move(0X0AFF, 0x01, ID_motor_L);     // Activa el movimiento del motor Izquierdo en sentido 1 y velocidad 0xFFF.
}

void Spin_R_Fast() {                    // Efectua un giro cerrado hacia la derecha.
    Move(0X0FFF, 0x00, ID_motor_R);     // Activa el movimiento del motor Derecho   en sentido 1 y velocidad 0xFFF.
    Move(0X0FFF, 0x00, ID_motor_L);     // Activa el movimiento del motor Izquierdo en sentido 1 y velocidad 0xFFF.
}

void Spin_R() {                         // Efectua un giro cerrado hacia la derecha.
    Move(0X0AFF, 0x00, ID_motor_R);     // Activa el movimiento del motor Derecho   en sentido 1 y velocidad 0xFFF.
    Move(0X0AFF, 0x00, ID_motor_L);     // Activa el movimiento del motor Izquierdo en sentido 1 y velocidad 0xFFF.
}

void Stop() {                           // Para el movimiento del robot.
    Move(0x000, 0x00, ID_motor_R);      // Para el movimiento del motor Derecho.
    Move(0x000, 0x00, ID_motor_L);      // Para el movimiento del motor Izquierdo.
}





// FUNCIONES AUXILIARES

void escribir(char String[], uint8_t Linea){        // Este metodo permite escribir en apntalla de manera facil.
    halLcdPrintLine(String, Linea, NORMAL_TEXT);    // Enviamos la String al LCD, sobreescribiendo la Linea indicada.
}







// FUNCIONES MOVIMIENTO AUTONOMO

void Check_data() {

    struct RxReturn datos = Read_Sensors();         // Leemos los datos de los sensores.
                                                    // Extraemos los datos relevantes.

    L_data = datos.StatusPacket[5];                 // Datos sensor izquierdo.
    C_data = datos.StatusPacket[6];                 // Datos sensor central.
    R_data = datos.StatusPacket[7];                 // Datos sensor derecho.



    if (pared == NO_WALL){                          // Mientras no tenemos ninguna pared asignada, entramos aqui.

        // En caso que no detecte nada en ninguna de las direcciones, se moverá hacia adelante hasta que lo haga.
        if (L_data == 0 && C_data == 0 && R_data == 0) estado_autonomo = STATE_FORWARD;


        // En caso que encuentre algo suficientemente cerca por delante,
        else if (C_data > C_value || L_data > L_value || R_data > R_value){

            if (direccion == LEFT_WALL){            // si esta establecido que mantenga la pared a la izquierda,
                pared = LEFT_WALL;                  // se establecerá LEFT WALL,
                estado_autonomo = STATE_SPIN_RIGHT; // y girará hacia la derecha sobre si mismo.
            }

            else {                                  // En caso que esté establecido que mantenga la pared a la derecha,
                pared = RIGHT_WALL;                 // se establecerá RIGHT WALL,
                estado_autonomo = STATE_SPIN_LEFT;  // y girará hacia la izquierda sobre si mismo.
            }

        }

        else{                                       // En caso que no se cumpla ninguna de las condiciones previas,
            estado_autonomo = STATE_FORWARD;        // se establece movimiento recto.
        }
    }



    else if (pared == LEFT_WALL) {                  // Una vez asignado el movimiento por la pared izquierda entramos aqui.

        if (pared_L == false){                      // Esta flag nos permitirá definir que estamos siguiendo una pared.
            if (L_data > L_min && L_data < L_max) {
                pared_L = true;                     //Una vez nos aproximemos lo suficiente, estableceremos que estamos siguiendo la pared.
            }
        }

        else if (girando == true){                  // Este flag nos permitirá definir que estamos girando.
            if (C_data == 0) {
                girando = false;                    // En caso de estar girando, si dejamos de tener una pared delante, dejará de girar, permitiendo actuar en diferentes apartados.
            }
        }

        else if (esquina == true){                  // Este flag nos permite definir que nos hemos encontrado con una esquina muy cerrada.
            if (L_data < 60){                       // Una vez la esquina se ha alejado considerablemente,
                estado_autonomo = STATE_SPIN_RIGHT; // giramos alejandonos de ella,
                esquina = false;                    // y establecemos la normalidad otra vez.
            }
        }

        else {                                      // En caso que no haya ningun flag activo, entramos en el funcionamiento normal.

            if(C_data > C_value) {                   // En caso que tengamos una pared de cara muy cerca,
                estado_autonomo = STATE_SPIN_RIGHT; // giramos a la derecha para mantenerla a la izquierda,
                girando = true;                     // establecemos que estamos girando,
                pared_L = false;                    // y asignamos que no estamos siguiendo una pared.
            }

            else if (L_data > 190) {                // En caso que nos encontremos una esquina muy cerrada que nos bloquea la rueda,
                esquina = true;                     // establecemos que nos hemos encontraod una esquina,
                estado_autonomo = STATE_BACKWARD;   // y establecemos el robot en marcha atrás.
            }

            else if (L_data == 0) {
                estado_autonomo = STATE_LEFT;       // En caso que perdamos la pared izquierda de repente, implica que hemos encontrado una esquina, haciendo que gire hacia la izquierda lentamente.
            }

            else if(L_data > L_min && L_data < L_max) {
                estado_autonomo = STATE_FORWARD;    // En caso que estemos dentro del rango establecido al calibrar, el robot sigue recto.
            }

            else if (L_data < L_min && L_data > 0) {
                estado_autonomo = STATE_SPIN_LEFT;  // En caso que el robot se aleje mucho de la pared, hace un pequeño giro a la izquierda.
            }

            else if (L_data > L_max) {
                estado_autonomo = STATE_SPIN_RIGHT; // En caso que el robot se acerque mucho a la pared, hace un pequeño giro a la derecha.
            }
        }
    }


    else if (pared == RIGHT_WALL){                  // Una vez asignado el movimiento por la pared derecha entramos aqui.

        if (pared_R == false){                      // Esta flag nos permitirá definir que estamos siguiendo una pared.
            if (R_data > R_min && R_data < R_max) {
                pared_R = true;                     //Una vez nos aproximemos lo suficiente, estableceremos que estamos siguiendo la pared.
            }
        }

        else if (girando == true){                  // Este flag nos permitirá definir que estamos girando.
            if (C_data == 0) {
                girando = false;                    // En caso de estar girando, si dejamos de tener una pared delante, dejará de girar, permitiendo actuar en diferentes apartados.
            }
        }

        else if (esquina == true){                  // Este flag nos permite definir que nos hemos encontrado con una esquina muy cerrada.
            if (R_data < 60){                       // Una vez la esquina se ha alejado considerablemente,
                estado_autonomo = STATE_SPIN_LEFT;  // giramos alejandonos de ella,
                esquina = false;                    // y establecemos la normalidad otra vez.
            }
        }

        else {                                      // En caso que no haya ningun flag activo, entramos en el funcionamiento normal.

            if (C_data > C_value){                   // En caso que tengamos una pared de cara muy cerca,
                estado_autonomo = STATE_SPIN_LEFT;  // giramos a la izquierda para mantenerla a la derecha,
                girando = true;                     // establecemos que estamos girando,
                pared_R = false;                    // y asignamos que no estamos siguiendo una pared.
            }

            else if (R_data > 190){                 // En caso que nos encontremos una esquina muy cerrada que nos bloquea la rueda,
                esquina = true;                     // establecemos que nos hemos encontraod una esquina,
                estado_autonomo = STATE_BACKWARD;   // y establecemos el robot en marcha atrás.
            }

            else if (R_data == 0) {
                estado_autonomo = STATE_RIGHT;      // En caso que perdamos la pared derecha de repente, implica que hemos encontrado una esquina, haciendo que gire hacia la derecha lentamente.
            }
            else if (R_data > R_min && R_data < R_max) {
                estado_autonomo = STATE_FORWARD;    // En caso que estemos dentro del rango establecido al calibrar, el robot sigue recto.
            }
            else if (R_data < R_min && R_data > 0) {
                estado_autonomo = STATE_SPIN_RIGHT; // En caso que el robot se aleje mucho de la pared, hace un pequeño giro a la derecha.
            }
            else if (R_data > R_max) {
                estado_autonomo = STATE_SPIN_LEFT;  // En caso que el robot se acerque mucho a la pared, hace un pequeño giro a la izquierda.
            }

        }
    }
}



void init_checkValues(){
    pared = 0;
    pared_R = 0;
    pared_L = 0;
    girando = 0;
    direccion = 0;
    esquina = 0;
    clap_times = 0;
}





void Autonomo() {

    struct RxReturn datosSound;                         // Este paquete nos permite recibir datos del sonido.

    while (clap_times < 1){                             // Saldremos de este while cuando haya detectado que hemos hecho una palma.
        datosSound = Read_Sound();                      // Leemos el valor del sonido
        soundLevel = datosSound.StatusPacket[5];        // extraemos el valor del sonido.

        sprintf(cadena, "sounsVal = %3d", soundLevel);  // Escribimos el valor del sonido en pantalla para tener una referencia
        escribir(cadena, 0);                            // visual de que sonidos esta recibiendo.

        if (soundLevel > clapLimit){                    // En caso que el valor del sonido supere un umbral establecido,
            clap_times ++;                              // aumentamos en 1 el valor de clap.
        }
    }


    while(estado != STATE_STOP){                        // En caso que se establezca que el robot pare, se saldrá del movimiento autonomo.

        Check_data();                                   // Hacemos Check de los datos y reaccionamos a partir de estos.

        while(estado_autonomo != ESTADO_DEFAULT){       // Si el estado autonomo es distinto a una constante (0xFF), entramos.

            switch(estado_autonomo){                    // Entramos en el switch para ver que estado se ha establecido.

            case STATE_FORWARD:                         // MOVIMIENTO HACIA ADELANTE
                Move_Forward();
                break;

            case STATE_BACKWARD:                        // MOVIMIENTO HACIA ATRAS
                Move_Backward();
                break;

            case STATE_LEFT:                            // GIRO HACIA LA IZQUIERDA
                Move_Left();
                break;

            case STATE_RIGHT:                           // GIRO HACIA LA DERECHA
                Move_Right();
                break;

            case STATE_SPIN_LEFT:                       // GIRO HACIA LA IZQUIERDA SOBRE SI MISMO
                Spin_L();
                break;

            case STATE_SPIN_RIGHT:                      // GIRO HACIA LA DERECHA SOBRE SI MISMO
                Spin_R();
                break;
            }

        estado_autonomo = ESTADO_DEFAULT;
        }
    }
    init_checkValues();                                 // Reseteamos los values
}







//MAIN

int main() {

    WDTCTL = WDTPW + WDTHOLD;   // Stop watchdog timer.
    init_botons();              // Establecemos e inicializamos los interruptores.
    init_timers();              // Establecemos e inicializamos los Timers.
    Config_UCS();               // Configuración de los UARTS
    init_interrupciones();      // Inicializamos las interrupcones del vector NVIC
    halLcdInit();               // Inicializar y configurar la pantallita
    halLcdClearScreenBkg();     // Borrar la pantalla, rellenando con el color de fondo
    __enable_interrupts();      // Habilitamos las interrupciones


    init_checkValues();         // Inicializamos variables generales
    Set_CW_CCW();               // Establecemos CW y CCW a 0.


    escribir("Calibracion:", 2);
    escribir("L_Value = 000", 3);
    escribir("C_Value = 000", 4);
    escribir("R_Value = 000", 5);
    escribir("Direccion = ", 7);

    while(true) {                           // Entramos en el bucle sin fin.

        if(estado != 0xFF) {                // En caso que se haya establecido otro estado.


            switch(estado) {                // Accedemos al estado en cuestión.

                case STATE_STOP:
                    Stop();                                         // Establecemos que pare su movimiento.
                    break;

                case STATE_AUTONOMO:
                    Autonomo();                                     // Establecemos un movimiento rotatorio sobre si mismo.
                    break;

                case STATE_SET_L:                                   // Establecemos el estado Left para tenerlo com referencia
                    sound(7,2);                                     // Hacemos sonar un pitido para saber que esta funcionando.
                    Pkt = Read_Sensors();                           // Obtenemos el paquete de los sensores.

                    L_value= Pkt.StatusPacket[5];                   // Establecemos el valor en cuestión.
                    L_max = L_value + 10;                           // Así como su rango maximo,
                    L_min = L_value - 10;                           // y su rango minimo.

                    sprintf(cadena, "L_Value = %3d", L_value);      // Establecemos la cadena que mostraremos por pantalla.
                    escribir(cadena, 3);                            // Mostramos esta cadena por pantalla.
                    break;

                case STATE_SET_C:                                   // Establecemos el estado Central para tenerlo com referencia
                    sound(5,2);                                     // Hacemos sonar un pitido para saber que esta funcionando.
                    Pkt = Read_Sensors();                           // Obtenemos el paquete de los sensores.

                    C_value= Pkt.StatusPacket[6];                   // Establecemos el valor en cuestión.

                    sprintf(cadena, "C_Value = %3d", C_value);      // Establecemos la cadena que mostraremos por pantalla.
                    escribir(cadena, 4);                            // Mostramos esta cadena por pantalla.
                    break;

                case STATE_SET_R:                                   // Establecemos el estado Left para tenerlo com referencia
                    sound(10,2);                                    // Hacemos sonar un pitido para saber que esta funcionando.
                    Pkt = Read_Sensors();                           // Obtenemos el paquete de los sensores.

                    R_value= Pkt.StatusPacket[7];                   // Establecemos el valor en cuestión.
                    R_max = R_value + 10;                           // Así como su rango maximo,
                    R_min = R_value - 10;                           // y su rango minimo.

                    sprintf(cadena, "R_Value = %3d", R_value);      // Establecemos la cadena que mostraremos por pantalla.
                    escribir(cadena, 5);                            // Mostramos esta cadena por pantalla.
                    break;

                case STATE_SET_DIRECTION:                           // Establecemos la dirección que querremos que tome el robot posteriormente.

                    if(direccion == RIGHT_WALL){                    // En caso que la dirección previamente establecida sea RIGHT WALL,
                        direccion = LEFT_WALL;                      // establecemos la nueva dirección a LEFT WALL,
                        escribir("Pared Right", 8);                 // y escribimos por pantalla el resultado.
                    }
                    else{                                           // En caso contrario,
                        direccion = RIGHT_WALL;                     // establecemos la nueva dirección a RIGHT WALL.
                        escribir("Pared Left ", 8);                 // y escribimos por pantalla el resultado.
                    }
                    break;


                case STATE_MUSIC:                                   // Haremos sonar musica a la vez que haremos bailar al robot.

                    ready = false;                                  // Establecemos a false el FLAG ready.
                    counter = 0;                                    // Inicializamos el contador.

                    for(i = 0; i < melody_length; i++){             // Empieza la marcha!

                        sound(melody[i], 0.5);                      // Suena la nota [i]

                        if(counter == false){                       // En caso que el contador sea false
                            Spin_R();                               // Giramos hacia la derecha
                        }
                        else{                                       // En caso contrario
                            Spin_L();                               // Giramos hacia la izquierda
                        }

                        while(!ready){                              // Mientras no esté preparado, esperamos la respuesta del robot.

                            struct RxReturn datos = Read_Time_Sound();

                            if(datos.StatusPacket[5] == 0){         // Comprobamos que esta respuesta no contiene error
                                ready = true;                       // Establecemos que estamos ready para otra nota.
                            }
                        }

                        counter ^= true;                            // Alternamos el valor del counter, para alternar el paso de baile
                        ready = false;                              // Y reseteamos el FLAG de ready.
                    }

                    Stop();                                         // Una vez acabada la melodia, paramos el robot.
                    break;

                default:
                    break;
            }

            estado = ESTADO_DEFAULT;        // Establecemos el estado en un valor que nunca usaremos.
        }

    }
}





//GESTION DE INTERRUPCIONES

//TIMERS
void TA0_0_IRQHandler(void) {               // USAREMOS EL TIMER A0_0 PARA LA DETECCIÓN DEL TIMEOUT AL RECIBIR UN PAQUETE POR RX.

    TA0CCTL0 &= ~TIMER_A_CCTLN_CCIFG;       // Limpiamos el flag de interrupción.

    Rx_time_out = true;                     // Y establecemos el Rx_timeout a true.
}



//UART

#ifdef USB                                  // EN CASO QUE SE USE EL USB USAREMOS LA UART0

void EUSCIA0_IRQHandler(void) {             // Se interrumpe al recibir un paquete por la UART0.

    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    // Limpiamos el flag de interrupción.
    UCA0IE &= ~UCRXIE;                      // Interrupciones desactivadas en RX.

    DatoLeido_UART = UCA0RXBUF;             // Leemos el dato del buffer.
    Byte_Recibido = true;                      // Establecemos que hemos recibido un byte.

    UCA0IE |= UCRXIE;                       // Interrupciones reactivadas en RX.
}

#else                                       // SI NO USAMOS USB, USAMOS LA UART2

void EUSCIA2_IRQHandler(void) {             // Se interrumpe al recibir un paquete por la UART2.

    EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG;    // Limpiamos el flag de interrupción.
    UCA2IE &= ~UCRXIE;                      // Interrupciones desactivadas en RX

    DatoLeido_UART = UCA2RXBUF;             // Leemos el dato del buffer.
    Byte_Recibido = true;                      // Establecemos que hemos recibido un byte.

    UCA2IE |= UCRXIE;                       // Interrupciones reactivadas en RX.
}
#endif



//PUERTOS

#ifdef USB                                  // EN CASO QUE SE USE EL USB, USAREMOS UNICAMENTE LOS BOTONES DE LA PLACA BASE, NO DEL BOOSTERPACK

void PORT1_IRQHandler(void) {

    uint8_t flag = P1IV;                    // Guardamos el vector de interrupciones. De paso, al acceder a este vector, se limpia automaticamente.
    P1IE &= ~(SW1_BIT + SW2_BIT);           // Interrupciones del boton S1 y S2 desactivadas

    switch (flag)
    {
    case SW1_INT:                           // En caso que se haya pulsado el botón S1 de la Placa base
        estado = STATE_STOP;                // establecemos el estado stop.
        break;

    case SW2_INT:                           // En caso que se haya pulsado el botón S2 de la Placa base
        estado = STATE_AUTONOMO;            // establecemos el estado autonomo.
        break;

    default:
        break;
    }

    P1IE |= (SW1_BIT + SW2_BIT);            // Interrupciones del boton S1 y S2 reactivadas
}


#else                                       // EN CASO DE NO USAR USB USAREMOS LOS INTERRUPTORES DE LA BOOSTERPACK.
    
void PORT3_IRQHandler(void) {

    uint8_t flag = P3IV;                    // Guardamos el vector de interrupciones. De paso, al acceder a este vector, se limpia automaticamente.
    P3IE &= ~(B_SW2_BIT);                   // Interrupciones del boton S2 del BoosterPack desactivadas

    switch (flag)
    {
    case B_SW2_INT:                         // En caso que se haya pulsado el botón S2 de la BoosterPack
        estado = STATE_MUSIC;
        break;

    default:
        break;
    }

    P3IE |= (B_SW2_BIT);                    // Interrupciones del boton S2 del BoosterPack reactivadas
}


void PORT4_IRQHandler(void) {

    uint8_t flag = P4IV;                    // Guardamos el vector de interrupciones. De paso, al acceder a este vector, se limpia automaticamente.
    P4IE &= ~(JD_BIT + JI_BIT + JC_BIT);    // Interrupciones del Joystick Derecha, Izquierda y Centro del BoosterPack desactivadas

    switch (flag)
    {
    case JD_INT:                            // En caso que se haya pulsado el Joystick Derecha de la BoosterPack
        estado = STATE_SET_R;               // se establece el estado SET RIGHT.
        break;

    case JI_INT:                            // En caso que se haya pulsado el Joystick Izquierda de la BoosterPack
        estado = STATE_SET_L;               // se establece el estado SET LEFT.
        break;

    case JC_INT:                            // En caso que se haya pulsado el Joystick Centro de la BoosterPack
        estado = STATE_STOP;                // se establece el estado STOP.
        pared = false;
        break;

    default:
        break;
    }

    P4IE |= (JD_BIT + JI_BIT + JC_BIT);     // Interrupciones del Joystick Derecha, Izquierda y Centro del BoosterPack reactivadas
}


void PORT5_IRQHandler(void) {

    uint8_t flag = P5IV;                    // Guardamos el vector de interrupciones. De paso, al acceder a este vector, se limpia automaticamente.
    P5IE &= ~(B_SW1_BIT + JA_BIT + JB_BIT); // Interrupciones del boton S1, Joystick Arriba y Abajo del BoosterPack desactivadas

    switch (flag)
    {
    case B_SW1_INT:                         // En caso que se haya pulsado el botón S1 de la BoosterPack
        estado = STATE_AUTONOMO;            // establecemos el estado autonomo.
        break;

    case JA_INT:                            // En caso que se haya pulsado el Joystick Arriba de la BoosterPack
        estado = STATE_SET_C;               // establecemos el estado SET CENTRAL.
        break;

    case JB_INT:                            // En caso que se haya pulsado el Joystick Abajo de la BoosterPack
        estado = STATE_SET_DIRECTION;       // establecemos el estado SET DIRECTION.
        break;

    default:
        break;
    }

    P5IE |= (B_SW1_BIT + JA_BIT + JB_BIT);  // Interrupciones del boton S1, Joystick Arriba y Abajo del BoosterPack reactivadas
}

#endif
