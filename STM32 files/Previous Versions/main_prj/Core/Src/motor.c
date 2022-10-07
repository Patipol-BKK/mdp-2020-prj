/*
 * motor.c
 *
 *  Created on: Aug 31, 2022
 *      Author: patipol001
 */
/*
 * Returns pwmValues depending on the progress (from 0 to 1)
 */
//void PWM_func(float progress)
//{
//	return 1500 * (1 - (progress >= 1));
//}
///*
// * Rotates back wheel motors
// * 	- motor_code : Specify which motor to move
// * 		L -> Left
// * 		R -> Right
// * 		B -> Both
// * 	- duration : length of rotation in ms
// */
//void RotateBackMotor(int motor_code, void (*PWM_func)(), int duration)
//{
//	uint8_t pwmVal = 1500;
//	//forward and backward
//	htim1.Instance ->CCR4 = 145;
//	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal);
//	osDelay(5000);
//	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal);
//	osDelay(5000);
//	switch(motor_code)
//	{
//
//	}
//}

