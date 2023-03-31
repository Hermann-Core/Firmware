#include "features.h"

const u16 var = 2;

int main (void)
{
    u16 a = var;

    while (true)
    {
        RTT_WriteString(0, "\n\nFirst test\n");
        RTT_printf(0, "\nLe double de 3 est : %02u", a*2);
    }
}
