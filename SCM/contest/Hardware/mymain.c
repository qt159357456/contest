#include "mymain.h"
 
void Data_Handle1(void)
{
    if ((handle_Buff1[0] == 'O') && (handle_Buff1[1] == 'K'))
    {
        debug_printf("RIGHT\r\n");
    }
    if ((handle_Buff1[0] == 'N') && (handle_Buff1[1] == 'O'))
    {
        debug_printf("ERROR\r\n");
    }    
}

