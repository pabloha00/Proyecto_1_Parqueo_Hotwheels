/* 
 * File:   multiplexada.h
 * Author: Andy Bonilla
 *
 * Created on 17 de julio de 2021, 11:32 PM
 */

/*-----------------------------------------------------------------------------
 ----------------------------LIBRERIAS-----------------------------------------
 -----------------------------------------------------------------------------*/
#include <xc.h>                 //se incluye libreria del compilador
#include <stdint.h>             //se incluye libreria
#include <pic16f887.h>          //se incluye libreria del pic
#include "ADC_CONFIG.h"
#define _XTAL_FREQ 8000000

/*-----------------------------------------------------------------------------
-------------------------- FUNCION DE LIBRERIA --------------------------------
-----------------------------------------------------------------------------*/

//--------FUNCION DE CONFIGURACION DEL ADC
void ADC_config()
{
    //CONFIGURACION DE ADC
    ADCON1bits.ADFM = 1 ;       // se justifica a la derecha
    ADCON1bits.VCFG0 = 0 ;      // voltajes de referencia
    ADCON1bits.VCFG1 = 0 ;      // voltaje de referencia
    
    ADCON0bits.ADCS = 0b00;     // se usa division de 4us con F/8
    ADCON0bits.CHS = 0;         // seleccion de canal 0
    __delay_us(100);             //delay de 50us para que cargue capacitor
    ADCON0bits.ADON = 1 ;       // se prende modulo ADC
}