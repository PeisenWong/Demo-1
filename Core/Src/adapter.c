

/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "adapter.h"


//char uartbuff[500];
uint8_t mailbox = 0, buf2_flag = 0, buf2_flagC2 = 0;

/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/
void Initialize(){
	/* MCU Configuration--------------------------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	GPIOPinsInit (LED1_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	GPIOPinsInit (LED2_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	GPIOPinsInit (LED3_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);

	GPIOPinsInit (PB1_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	GPIOPinsInit (PB2_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);

	/*Normal IOs*/
	GPIOPinsInit (IP1_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLDOWN);
	GPIOPinsInit (IP2_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLDOWN);
	GPIOPinsInit (IP3_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLDOWN);
	GPIOPinsInit (IP4_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP5_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP6_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLDOWN);
	GPIOPinsInit (IP7_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLDOWN);
	GPIOPinsInit (IP8_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLDOWN);
	GPIOPinsInit (IP9_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP10_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP11_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP12_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP13_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP14_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP15_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);

	GPIOPinsInit (IP16_Analog1_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP17_Analog2_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP18_Analog3_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP19_Analog4_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP20_Analog5_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP21_Analog6_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);

	GPIOPinsInit(GPIOB, GPIO_PIN_10, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit(GPIOB, GPIO_PIN_11, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);

	//Unused peripheral pins can be used as GPIO Input or Output
//	GPIOPinsInit (UART3_Rx, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);

//	MUXInit(&MUX, MUX1_INPUT_PIN, MUX1_S0_PIN, MUX1_S1_PIN, MUX1_S2_PIN);
	SHIFTREGInit (&SR, CASCADE_1, SR_SCK_PIN, SR_RCK_PIN, SR_SI_PIN);

	//https://stackoverflow.com/questions/50243996/what-are-valid-values-of-hal-nvic-setpriority-when-using-stm32-and-freertos
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);


	/*************** Set Interrupt Priorities in BIOS/priorities.h ***************/

//	I2CxInit (&hi2c1,main_board_1, CLOCK_SPEED_400KHz,ENABLE);
	I2CX_DMA_RX_Init(&hi2c1, &hi2c1_rx_dma, main_board_1, CLOCK_SPEED_400KHz);
//	I2CX_DMA_RX_Init(&hi2c2, &hi2c2_rx_dma, main_board_1, CLOCK_SPEED_400KHz);
//	I2CxInit(&hi2c2, main_board_1, CLOCK_SPEED_100KHz, ENABLE);

	//Servo Driver - recommended to use 100KHz I2C as 400KHz hang frequently
//	I2CxInit (&hi2c3,main_board_1, CLOCK_SPEED_100KHz,DISABLE);
//	ServoDriverInit(&srv_drv,&hi2c3,0x40);

//	UARTx_DMA_Rx_Init(&huart2, &hdma_usart2_rx, 115200);//Bluebee Tuning
//	UARTx_DMA_Rx_Init(&huart4, &hdma_uart4_rx, 115200); //Jetson
	UARTInit(&huart3, 115200, ENABLE); // Debug
	UARTInit(&huart4, 115200, ENABLE); // ROS UART
	UARTInit(&huart5, 115200, ENABLE); //
	UARTInit(&huart2, 115200, ENABLE); //

	//	QEIInit(&htim8);

	CANxInit(&hcan1,4,CAN_FILTER_FIFO0,0,0,0,CAN_500KHz);
	CANxInit(&hcan2,4,CAN_FILTER_FIFO0,0,0,0,CAN_500KHz);

	PWMTimeBaseInit(&htim3, 19999, 83);
	PWMChannelConfig(&htim3, TIM_CHANNEL_3, TIM3_CHANNEL3_PIN);
	PWMChannelConfig(&htim3, TIM_CHANNEL_4 , TIM3_CHANNEL4_PIN);

	PWMTimeBaseInit(&htim5, 19999, 83);
	PWMChannelConfig(&htim5, TIM_CHANNEL_1, TIM5_CHANNEL1_PIN);
	PWMChannelConfig(&htim5, TIM_CHANNEL_2, TIM5_CHANNEL2_PIN);
	PWMChannelConfig(&htim5, TIM_CHANNEL_3, TIM5_CHANNEL3_PIN);
	PWMChannelConfig(&htim5, TIM_CHANNEL_4, TIM5_CHANNEL4_PIN);

	PWMTimeBaseInit(&htim9, 19999, 167);
	PWMChannelConfig(&htim9, TIM_CHANNEL_1, TIM9_CHANNEL1_PIN);
	PWMChannelConfig(&htim9, TIM_CHANNEL_2, TIM9_CHANNEL2_PIN);


	BDCInit(&BDC1, &htim3, TIM_CHANNEL_4, SHIFTREG, &(SR.cast[1]), Bit6, Bit7); // yaw_servo
	BDCInit(&BDC2, &htim3, TIM_CHANNEL_3, SHIFTREG, &(SR.cast[1]), Bit4, Bit5); // pitch servo
	BDCInit(&BDC3, &htim9, TIM_CHANNEL_1, SHIFTREG, &(SR.cast[1]), Bit2, Bit3); // pick servo
	BDCInit(&BDC4, &htim9, TIM_CHANNEL_2, SHIFTREG, &(SR.cast[1]), Bit0, Bit1); // load servo
	BDCInit(&BDC5, &htim5, TIM_CHANNEL_4, SHIFTREG, &(SR.cast[0]), Bit6, Bit7); // shot valve
	BDCInit(&BDC6, &htim5, TIM_CHANNEL_3, SHIFTREG, &(SR.cast[0]), Bit4, Bit5); // enabling valve
	BDCInit(&BDC7, &htim5, TIM_CHANNEL_2, SHIFTREG, &(SR.cast[0]), Bit2, Bit3); // pick valve
	BDCInit(&BDC8, &htim5, TIM_CHANNEL_1, SHIFTREG, &(SR.cast[0]), Bit0, Bit1);

	// Servo
	ServoxInit(&yaw_servo, &htim9, IP5_PIN, TIM_CHANNEL_1); // BDC3
	ServoInitPulseLimit(&yaw_servo, 500, 2500);
//	ServoSetPulse(&yaw_servo, 1500);

	ServoxInit(&pitch, &htim3, IP6_PIN, TIM_CHANNEL_4); // BDC1
	ServoInitPulseLimit(&pitch, 500, 2500);
//	ServoSetPulse(&pitch, 1500);

	ServoxInit(&pick, &htim9, IP2_PIN, TIM_CHANNEL_2); // BDC4
	ServoInitPulseLimit(&pick, 500, 2500);
//	ServoSetPulse(&pick, 500);

	ServoxInit(&load, &htim3, IP3_PIN, TIM_CHANNEL_3); // BDC2
	ServoInitPulseLimit(&load, 500, 2500);
//	ServoSetPulse(&load, 500);

	//Laser
//	ADC_DMAxInit(&adc,&hadc2,&hdma_adc1,2);
//	ADC_Channel_Config(&adc,ADC_CHANNEL_10,IP16_Analog1_PIN);
//	ADC_Channel_Config(&adc,ADC_CHANNEL_11,IP17_Analog2_PIN);

	SPIx_DMA_TX_Init(&hspi1, &hdma_spi1_tx, SPI1_NSS_PIN, SPI_MODE_MASTER);
}



void CAN1_RX0_IRQHandler()
{
	HAL_CAN_IRQHandler(&hcan1);

}

void CAN2_RX1_IRQHandler()
{

	HAL_CAN_IRQHandler(&hcan2);

}


void CAN_PROCESS(PACKET_t packet_src){


	switch(packet_src){

	case VESC_PACKET:
		decode_VESC();
		break;

	case RNS_PACKET:

		if(insData_receive[0] == 1){
			rns.RNS_data.common_instruction = insData_receive[1];
			insData_receive[0]=2;
		}
		if(insData_receive[0] == 17){
			if(buf2_flag == 1){
				rns.RNS_data.common_instruction = insData_receive[1];
				rns.RNS_data.common_buffer[0].data = buf1_receive[0].data;
				rns.RNS_data.common_buffer[1].data = buf1_receive[1].data;
				rns.RNS_data.common_buffer[2].data = buf2_receive[0].data;
				rns.RNS_data.common_buffer[3].data = buf2_receive[1].data;
				insData_receive[0]=3;
			}
		}

		break;

	}

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)

{
	uint8_t aData[8];
	PACKET_t source;

	if(hcan == &hcan1){
//		led2 = !led2;
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0 , &CAN1RxMessage, aData);
		//	sprintf(data,"%d %d %d %d %d %d %d %d \r\n",aData[0],aData[1],aData[2],aData[3],aData[4],aData[5],aData[6],aData[7]);

		if(CAN1RxMessage.IDE == CAN_ID_EXT)
		{
			vescmsg.Rxmsg = CAN1RxMessage;
			memcpy(vescmsg.Data, aData,8);
			set_rx_frames(&vescmsg);
			source = VESC_PACKET;
		}else{
			source = RNS_PACKET;
			switch(CAN1RxMessage.StdId){
			case RNS_TO_mainboard:
				memcpy(&insData_receive, &aData, CAN1RxMessage.DLC);
				buf2_flag = 0;

				break;
			case RNS_TO_mainboard_buf1:
				memcpy(&buf1_receive, &aData, CAN1RxMessage.DLC);

				break;
			case RNS_TO_mainboard_buf2:
				memcpy(&buf2_receive, &aData, CAN1RxMessage.DLC);
				buf2_flag = 1;
				break;
			case XY_feedback_state :
				break;
			case 1872:
				if(faul_counter >= 2000){
					led2 = !led2;
					faul_counter = 0;
				}else{
					faul_counter ++;
				}
			default:
				break;
			}
		}
		CAN_PROCESS(source);
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	}else{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0 , &CAN2RxMessage, aData);
		//sprintf(data,"%d %d %d %d %d %d %d %d \r\n",aData[0],aData[1],aData[2],aData[3],aData[4],aData[5],aData[6],aData[7]);

		if(CAN2RxMessage.IDE == CAN_ID_EXT)
		{
			vescmsg.Rxmsg = CAN2RxMessage;
			memcpy(vescmsg.Data, aData,8);
			set_rx_frames(&vescmsg);
			source = VESC_PACKET;
		}else{
			source = RNS_PACKET;
			switch(CAN1RxMessage.StdId){
			case RNS_TO_mainboard:
				memcpy(&insData_receive, &aData, CAN2RxMessage.DLC);
				buf2_flag = 0;

				break;
			case RNS_TO_mainboard_buf1:
				memcpy(&buf1_receive, &aData, CAN2RxMessage.DLC);

				break;
			case RNS_TO_mainboard_buf2:
				memcpy(&buf2_receive, &aData, CAN2RxMessage.DLC);
				buf2_flag = 1;
				break;
			case XY_feedback_state :
				break;
			default:
				break;
			}
		}
		CAN_PROCESS(source);
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)

{
	uint8_t aData[8];
	PACKET_t source;

	if(hcan == &hcan1){

		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1 , &CAN1RxMessage, aData);
		//sprintf(data,"%d %d %d %d %d %d %d %d \r\n",aData[0],aData[1],aData[2],aData[3],aData[4],aData[5],aData[6],aData[7]);
		if(CAN1RxMessage.IDE == CAN_ID_EXT)
		{
			vescmsg.Rxmsg = CAN1RxMessage;
			memcpy(vescmsg.Data, aData,8);
			set_rx_frames(&vescmsg);
			source = VESC_PACKET;

		}else{
			source = RNS_PACKET;
			switch(CAN2RxMessage.StdId){
			case RNS_TO_mainboard:
				memcpy(&insData_receive, &aData, CAN1RxMessage.DLC);
				buf2_flag = 0;

				break;
			case RNS_TO_mainboard_buf1:
				memcpy(&buf1_receive, &aData, CAN1RxMessage.DLC);

				break;
			case RNS_TO_mainboard_buf2:
				memcpy(&buf2_receive, &aData, CAN1RxMessage.DLC);
				buf2_flag = 1;
				break;
			case XY_feedback_state :
				break;
			default:
				break;
			}
		}
		CAN_PROCESS(source);
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	}else{

		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1 , &CAN2RxMessage, aData);
		//sprintf(data,"%d %d %d %d %d %d %d %d \r\n",aData[0],aData[1],aData[2],aData[3],aData[4],aData[5],aData[6],aData[7]);
		if(CAN2RxMessage.IDE == CAN_ID_EXT)
		{
			vescmsg.Rxmsg = CAN2RxMessage;
			memcpy(vescmsg.Data, aData,8);
			set_rx_frames(&vescmsg);
			source = VESC_PACKET;
		}else{
			source = RNS_PACKET;
			switch(CAN2RxMessage.StdId){
			case RNS_TO_mainboard:
				memcpy(&insData_receive, &aData, CAN2RxMessage.DLC);
				buf2_flag = 0;

				break;
			case RNS_TO_mainboard_buf1:
				memcpy(&buf1_receive, &aData, CAN2RxMessage.DLC);

				break;
			case RNS_TO_mainboard_buf2:
				memcpy(&buf2_receive, &aData, CAN2RxMessage.DLC);
				buf2_flag = 1;
				break;
			case XY_feedback_state :
				break;
			default:
				break;
			}
		}
		CAN_PROCESS(source);
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
	}

}

//Developing

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//
//{
//	//	led2 = !led2;
//	uint8_t aData[8];
//	PACKET_t source;
//	if(hcan == &hcan1){
//		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0 , &CAN1RxMessage, aData);
//		//	sprintf(data,"%d %d %d %d %d %d %d %d \r\n",aData[0],aData[1],aData[2],aData[3],aData[4],aData[5],aData[6],aData[7]);
//
//		if(CAN1RxMessage.IDE == CAN_ID_STD)
//		{
//			switch(CAN1RxMessage.StdId){
//			case 1871:
//				ILI9341_PrintLine("FaulHaber Node 79 Boot Up", &ILI9341row);
//				break;
//			case 1487:{
//				Faul_t.can = 1;
//				//SDO Tx
//				Faul_t.ODindex = *((uint16_t *)&aData[1]);
//				Faul_t.ODsubindex = aData[3];
//
//				if(aData[0] == 0x60){
//					//Faulhaber Response on Successful write to OD
//					char temp[50];
//					sprintf(temp, "%lu: SDO(tx)-WR 0x%x%x.%x", CAN1RxMessage.FilterMatchIndex, aData[2], aData[1], aData[3]);
//					ILI9341_PrintLine(temp, &ILI9341row);
//				}else{
//					char temp[50];
//					sprintf(temp, "%lu: SDO(tx)-RR 0x%x%x.%x", CAN1RxMessage.FilterMatchIndex, aData[2], aData[1], aData[3]);
//					ILI9341_PrintLine(temp, &ILI9341row);
//
//					if(aData[0] == 0x4F){
//						Faul_t.num_valid = 1;
//						Faul_t.buffer[0] = aData[4];
//					}else if(aData[0] == 0x4B){
//						Faul_t.num_valid = 2;
//						memcpy(Faul_t.buffer, &aData[4], 2);
//					}else if(aData[0] == 0x47){
//						Faul_t.num_valid = 3;
//						memcpy(Faul_t.buffer, &aData[4], 3);
//					}else if(aData[0] == 0x43){
//						Faul_t.num_valid = 4;
//						memcpy(Faul_t.buffer, &aData[4], 4);
//					}
//
//					switch(Faul_t.ODindex){
//					case 0x6064:
//						Faul_t.pos_act = *((int *)&Faul_t.buffer);
//						sprintf(temp, "%s = %d", varName(Faul_t.pos_act), Faul_t.pos_act);
//						ILI9341_PrintLine(temp, &ILI9341row);
//						break;
//					case 0x607C:
//						Faul_t.HomeOffset = *((int *)Faul_t.buffer);
//						sprintf(temp, "%s = %d", varName(Faul_t.HomeOffset), Faul_t.HomeOffset);
//						ILI9341_PrintLine(temp, &ILI9341row);
//						break;
//					case 0x2344:{
//						switch(Faul_t.ODsubindex){
//						case 0x00:
//							break;
//						case 0x01:
//							Faul_t.velKp = *((uint32_t *)Faul_t.buffer);
//							sprintf(temp, "%s = %lu", varName(Faul_t.velKp), Faul_t.velKp);
//							ILI9341_PrintLine(temp, &ILI9341row);
//						}
//						break;
//					}
//					break;
//					case 0x6060:
//						Faul_t.modeOperation = *((int8_t *)Faul_t.buffer);
//						sprintf(temp, "%s = %d", varName(Faul_t.modeOperation), Faul_t.modeOperation);
//						ILI9341_PrintLine(temp, &ILI9341row);
//						break;
//					}
//
//
//				}
//			}
//			break;
//
//			case 463:{
//				//TxPDO1
//				Faul_t.can = 1;
//				Faul_t.statusword = *((uint16_t *)aData);
//				ILI9341_PrintLine("TxPDO1", &ILI9341row);
//			}
//			break;
//
//			case 719:{
//				//TxPDO2
//				Faul_t.can = 1;
//				Faul_t.statusword = *((uint16_t *)aData);
//				Faul_t.pos_act = *((int *)&aData[2]);
//				ILI9341_PrintLine("TxPDO2", &ILI9341row);
//			}
//			break;
//
//			case 975:{
//				//TxPDO3
//				Faul_t.can = 1;
//				Faul_t.statusword = *((uint16_t *)aData);
//				Faul_t.vel_act = *((int *)&aData[2]);
//				ILI9341_PrintLine("TxPDO3", &ILI9341row);
//			}
//			break;
//
//			case 1231:{
//				//TxPDO4
//				Faul_t.can = 1;
//				Faul_t.statusword = *((uint16_t *)aData);
//				Faul_t.tor_act = *((int *)&aData[2]);
//				ILI9341_PrintLine("TxPDO4", &ILI9341row);
//			}
//			break;
//			}
//		}
//		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
//	}else{
//		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0 , &CAN2RxMessage, aData);
//		//sprintf(data,"%d %d %d %d %d %d %d %d \r\n",aData[0],aData[1],aData[2],aData[3],aData[4],aData[5],aData[6],aData[7]);
//
//		if(CAN2RxMessage.IDE == CAN_ID_EXT)
//		{
//			vescmsg.Rxmsg = CAN2RxMessage;
//			memcpy(vescmsg.Data, aData,8);
//			set_rx_frames(&vescmsg);
//			source = VESC_PACKET;
//		}else{
//			source = RNS_PACKET;
//			switch(CAN1RxMessage.StdId){
//			case RNS_TO_mainboard:
//				memcpy(&insData_receive, &aData, CAN2RxMessage.DLC);
//				buf2_flag = 0;
//
//				break;
//			case RNS_TO_mainboard_buf1:
//				memcpy(&buf1_receive, &aData, CAN2RxMessage.DLC);
//
//				break;
//			case RNS_TO_mainboard_buf2:
//				memcpy(&buf2_receive, &aData, CAN2RxMessage.DLC);
//				buf2_flag = 1;
//				break;
//			case XY_feedback_state :
//				break;
//			default:
//				break;
//			}
//		}
//		CAN_PROCESS(source);
//		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
//	}
//}
