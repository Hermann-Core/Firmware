#include "features.h"

const u16 a = 3;

int main (void)
{
    SysTick_Config(72000);
    
    while (true)
    {
        RTT_WriteString(0, "\n\nFirst test\n");
        RTT_printf(0, "\nLe double de 3 est : %02u", a*2);
    }
}
