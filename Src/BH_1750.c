#include "BH_1750.h"

void BH_1750_Init(){
	uint8_t command;
	command = BH_1750_On;
	HAL_I2C_Master_Transmit(&hi2c1, BH_1750_Adress, &command, 1, 0xffff);

	command = BH_1750_Cont_H_Res_Mode;
	HAL_I2C_Master_Transmit(&hi2c1, BH_1750_Adress, &command, 1, 0xffff);
}
float BH_1750_Read(){
	float light;
	uint8_t buff[2];

	HAL_I2C_Master_Receive(&hi2c1, BH_1750_Adress, buff, 2, 0xffff);
	light = ((buff[0] << 8)|buff[1])/1.2;

	return light;
}
