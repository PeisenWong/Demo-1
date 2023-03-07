/* Includes ------------------------------------------------------------------*/
#include "main.h"

/**
 * @brief  The application entry point.
 * @retval int
 */

int main(void)
{
	set();

	const osThreadAttr_t MainTask_attributes = {
			.name = "MainTask",
			.stack_size = 1024 * 4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osThreadAttr_t EmergencyTask_attributes = {
			.name = "EmergencyTask",
			.stack_size = 256 * 4,
			.priority = (osPriority_t) osPriorityNormal,
	};

//	const osThreadAttr_t TuneTask_attributes = {
//			.name = "TuneTask",
//			.stack_size = 256 * 4,
//			.priority = (osPriority_t) osPriorityAboveNormal,
//	};

	const osThreadAttr_t Calculation_Task_attributes = {
			.name = "Calculation_Task",
			.stack_size = 512 *  4,
			.priority = (osPriority_t) osPriorityAboveNormal,
	};

	const osSemaphoreAttr_t CalcSemaphore_attributes = {
				.name = "InitSemaphore"
		};

//	const osSemaphoreAttr_t TuneSemaphore_attributes = {
//			.name = "TuneSemaphore"
//	};

	ILI9341_Init();
//	ILI9341_Set_Rotation(1);
	ILI9341_Fill_Screen(BLACK);
	ILI9341_Init_List();

	osKernelInitialize();

	MainTaskHandle = osThreadNew(MainTask, NULL, &MainTask_attributes);
	EmergencyTaskHandle = osThreadNew(EmergencyTask, NULL, &EmergencyTask_attributes);
	Calculation_Task_Handle = osThreadNew(Calculation, NULL, &Calculation_Task_attributes);
//	TuneTaskHandle = osThreadNew(TuneTask, NULL, &TuneTask_attributes);
//	TuneSemaphore = osSemaphoreNew(1, 0, &TuneSemaphore_attributes);
	CalcSemaphore = osSemaphoreNew(1, 0, &CalcSemaphore_attributes);
	led2 = 1;
	led3 = 0;
	osKernelStart();

	while(1)
	{

	}
}

void TIM6_DAC_IRQHandler(void)
{
	static int counter = 0;
	osSemaphoreRelease(CalcSemaphore);

	if(counter % 4 == 0)
	{
		led1 = !led1;
		switch(mode)
		{
			case NORMAL:
//				sprintf((char*)debug, "S: %.2f, 1: %.5f 2: %.5f 3: %.2f 4: %.2f X: %.2f Y: %.2f Z: %.2f\n", vesc_move_speed, VESCNav.a_vel, VESCNav.b_vel,
//					VESCNav.c_vel, VESCNav.d_vel, pp.real_x, pp.real_y, pp.real_z_rad);
//				sprintf((char*)debug, "1: %.2f 2: %.2f 3: %.2f 4: %.2f\n", fFLeftVelU, fFRightVelU,
//									fBLeftVelU, fBRightVelU);
//				sprintf((char*)debug, "1: %.2f 2: %.2f 3: %.2f 4: %.2f\n", VESCNav.a.info.current, VESCNav.b.info.current,
//										VESCNav.c.info.current, VESCNav.d.info.current);
				sprintf((char*)debug, "Z: %.2f Ex: %.2f Ux: %.2f Ey: %.2f Uy: %.2f\n", pp.real_z_rad, pp.error_x, pp.outx, pp.error_y, pp.outy);
			break;

			case INITIALIZE:
				sprintf((char*)debug, "Pitch: %ld Yaw: %ld L: %ld P: %ld Vesc: %.2f\n", pitch.currentPulse, yaw_servo.currentPulse,
								load.currentPulse, pick.currentPulse, vesc_pdc);
				break;

			case TUNE_PID:
				switch(wheel)
				{
					case 1:
						sprintf((char*)debug, "P: %.2f I: %.2f D: %.4f Vel: %.2f VelR: %.2f W: %d\n", AP, AI, AD, VESCNav.a_vel, vesc_pulse, wheel);
						break;
					case 2:
						sprintf((char*)debug, "P: %.2f I: %.2f D: %.4f Vel: %.2f VelR: %.2f W: %d\n", BP, BI, BD, VESCNav.b_vel, vesc_pulse, wheel);
						break;
					case 3:
						sprintf((char*)debug, "P: %.2f I: %.2f D: %.4f Vel: %.2f VelR: %.2f W: %d\n", CP, CI, CD, VESCNav.c_vel, vesc_pulse, wheel);
						break;
					case 4:
						sprintf((char*)debug, "P: %.2f I: %.2f D: %.4f Vel: %.2f VelR: %.2f W: %d\n", DP, DI, DD, VESCNav.d_vel, vesc_pulse, wheel);
						break;
				}
			break;
		}
		//
		//	sprintf((char*)debug, "Err: %.2f Output: %.2f\n", fFLeftVelErr, fFLeftVelU);
		//	sprintf((char*)debug, "Yaw: %.2f\n", fyaw);
//		HAL_UART_Transmit(&huart3, debug, strlen((char*)debug), HAL_MAX_DELAY);
	}
	counter++;
	HAL_TIM_IRQHandler(&htim6);
}

void MainTask(void *argument)
{
	RobotStart();
	while(1)
	{
		switch(mode)
		{
			case NORMAL:
//				NormalControl();
//				ROSTune();
				if(!PB1)
				{
					while(!PB1);
					LidarSendIns(NEAR, &lidar);
				}

				if(!PB2)
				{
					while(!PB2);
					LidarSendIns(FAR, &lidar);
				}

				if(lidar.start == 1)
					led3 = 1;
			break;

			case INITIALIZE:
				Init();
			break;

			case TUNE_PID:
				PS4Tune();
			break;
		}

		if(ps4.button == SELECT)
		{
			while(ps4.button == SELECT);
			mode++;
			if(mode > TUNE_PID) mode = NORMAL;
		}
	}
}

void Calculation(void *argument)
{
	while(1)
	{
		osSemaphoreAcquire(CalcSemaphore,osWaitForever);

		if(!pp.pp_start)
			realMODN(&ps4, &rns);
		else
		{
			VESCPIDProcess(pp.v1, pp.v2, pp.v3, pp.v4);
			hb_count = HAL_GetTick();
		}

		VESCNav5ms();

		if(flywheel)
		{
			VESCPDC(vesc_pdc, &flywheel1);
			VESCPDC(vesc_pdc, &flywheel2);
		}
		else
		{
			VESCStop(&flywheel1);
			VESCStop(&flywheel2);
		}
	}
}

void EmergencyTask(void *argument){

	while(1){

		if(ps4.button == TOUCH){
//			RNSStop(&rns);
			VESCNavStop();
			osThreadSuspend(FlywheelPitchPIDTaskHandle);
			osThreadSuspend(FlywheelYawPIDTaskHandle);
			osThreadTerminate(MainTaskHandle);
			osDelay(5);

			const osThreadAttr_t MainTask_attributes = {
					.name = "MainTask",
					.stack_size = 256 * 4,
					.priority = (osPriority_t) osPriorityNormal,
			};

			MainTaskHandle = osThreadNew(MainTask, NULL, &MainTask_attributes);
		}

	}
}

void TuneTask(void *argument){
	tunePending();
	int page = -1;
	while(1){
		osSemaphoreAcquire(TuneSemaphore,osWaitForever);

		if(tuneCommand == '0'){
			page = 0;
			tuneInit(page);
		}
		if(tuneCommand == '1'){
			page = 1;
			tuneInit(page);
		}
		if(tuneCommand == '2'){
			page = 2;
			tuneInit(page);
		}
		if(tuneCommand == '3'){
			page = 3;
			tuneInit(page);
		}
		if(tuneCommand == '4'){
			page = 4;
			tuneInit(page);
		}

		if(tuneCommand == 'r'){
			tuneUpdate(page);
		}

		if(tuneCommand == 'w'){
			led3 = 0; led2 = 1;
			led2 = 0;
			uint32_t writeSuccess=0;
			writeSuccess += tuneWriteInt();
			writeSuccess += tuneWriteFloat();

			if( writeSuccess == tuneWriteSuccess){
				HAL_UART_Transmit(tuneUart, (uint8_t *)((char *)"EEPROM WRITTEN\n"), 15, 10);
				led2 = 1;
			}else{
				HAL_UART_Transmit(tuneUart, (uint8_t *)((char *)"EEPROM WRITE FAIL\n"), 18, 10);
				led3 = 1;
			}
			tunePending();
		}

		if(tuneCommand == 'e'){
			tuneEdit(page);
		}
		if(tune_flag.editTerminated && page == 0){
			led3 = !led3;
			RNSSet(&rns, RNS_F_LEFT_VEL_PID,  AP, AI, AD);   //
			RNSSet(&rns, RNS_F_RIGHT_VEL_PID, BP, BI, BD);   //
			RNSSet(&rns, RNS_B_LEFT_VEL_PID,  CP, CI, CD);   //
			RNSSet(&rns, RNS_B_RIGHT_VEL_PID, DP, DI, DD);   //
			tune_flag.editTerminated = 0;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == tuneUart)
		osSemaphoreRelease(TuneSemaphore);

	if(huart == ROS_UART){
		ROS_Handler();
	}
//	if(huart == ROS_navi.Send_Pos)
//		ROS_Navi_Pos_Handler();
//	else if(huart == ROS_navi.Recv_Vel)
//		ROS_Navi_Vel_Handler();

	if(huart == lidar.lidar_UART)
	{
		ObstacleHandler(&lidar);
	}

	if(huart == imu.huartx)
	{
		R6091U_Handler(&imu);
	}

//		else if(huart == ps4.huartps4){
//		PSx_HandlerUART(&ps4);
//	}
}

/**
 * @brief  This function is executed in case of errorYaw occurrence.
 */
void Error_Handler(void)
{


}
#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
