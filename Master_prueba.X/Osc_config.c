
/* 
 * File:   Osc_config.h
 * Author: Andy Bonilla
 *
 * Created on 30 de julio de 2021, 11:32 PM
 */
/*-----------------------------------------------------------------------------
 ----------------------------LIBRERIAS-----------------------------------------
 -----------------------------------------------------------------------------*/
#include <xc.h>                 //se incluye libreria del compilador
#include <stdint.h>             //se incluye libreria
#include <pic16f887.h>          //se incluye libreria del pic

#include "Osc_config.h"

void osc_config(uint8_t freq)       //oscilador en MHz
{
    switch(freq)
    {
        case(1):
            OSCCONbits.IRCF=0b100;      //Freq a 1MHz
            OSCCONbits.SCS=1;           //Oscilador interno
            break;
            
        case(2):
            OSCCONbits.IRCF=0b101;      //Freq a 2MHz
            OSCCONbits.SCS=1;           //Oscilador interno
            break;
            
        case(4):
            OSCCONbits.IRCF=0b110;      //Freq a 4MHz
            OSCCONbits.SCS=1;           //Oscilador interno
            break;
            
        case(8):
            OSCCONbits.IRCF=0b111;      //Freq a 8MHz
            OSCCONbits.SCS=1;           //Oscilador interno
            break;
            
    }
    
}
