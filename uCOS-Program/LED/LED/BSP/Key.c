#include"Key.h"

void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_Initure.Pin=GPIO_PIN_0;
	GPIO_Initure.Mode=GPIO_MODE_INPUT;
	GPIO_Initure.Pull=GPIO_PULLDOWN;
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);
}

unsigned char KEY_Scan(void)
{
	
if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0))
{
	
	return 0xFF;
	
}
else
   return 0x00;
	
	
}