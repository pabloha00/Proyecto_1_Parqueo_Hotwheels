/* 
 * File:   uart_config.h
 * Author: Andy Bonilla
 *
 * Created on 17 de julio de 2021, 11:32 PM
 */

/*-----------------------------------------------------------------------------
 ----------------------------LIBRERIAS-----------------------------------------
 -----------------------------------------------------------------------------*/
#include <xc.h>
#include <pic16f887.h>
#include <pic16f887.h>                 //se incluye libreria del compilador
#include "UART_CONFIG.h"

/*-----------------------------------------------------------------------------
-------------------------- FUNCION DE LIBRERIA --------------------------------
-----------------------------------------------------------------------------*/
//--------configuracion de bits para comunicacion uart
void uart_config(void)
{
    //CONFIGURACION DE UART
    TXSTAbits.SYNC = 0;     //comunicacion asincrona
	TXSTAbits.BRGH = 1;     //comunicacion de alta velocidad
	TXSTAbits.TX9 = 0;      //comunicacion de 8bits
	BAUDCTLbits.BRG16 = 0;  //
	SPBRGH = 0;             //configuracion de braudeaje segun Freq osc
	SPBRG = 51;
	//SETUP RECIBIR
	RCSTAbits.SPEN = 1;     //se habilita modulo
	RCSTAbits.RX9 = 0;      //comunicacion de 8bits
	RCSTAbits.CREN = 1;     //
	TXSTAbits.TXEN = 1;     //
}