#include "i2c.h"

#ifndef BH_1750
#define BH_1750
//---------variables-------------//
#define BH_1750_Adress (0x23 << 1)
#define BH_1750_On 0x01
#define BH_1750_Cont_H_Res_Mode 0x10

//---------functions-------------//
void BH_1750_Init();
float BH_1750_Read();
#endif
