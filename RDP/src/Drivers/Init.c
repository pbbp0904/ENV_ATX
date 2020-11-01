
#include <avr\io.h>
#include <avr\interrupt.h>
#include "Init.h"

void Init_all(void)
{
//PortInit();
UART_Init();


}
///////////////////////////////////////
////  PWM_Init	Initializing PWM system
//////////////////
void PortInit(void)
{
DDRD=(0<<PIND0)|(1<<PIND1)|(0<<PIND2)|(0<<PIND3)|(0<<PIND4)|(1<<PIND5)|(1<<PIND6)|(1<<PIND7); 



}


