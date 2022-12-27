/*******************************************************************************
 * Title   : common.h
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 12/10
 *******************************************************************************
 * Description: Sensor and function definitions
 *
 * Version History:
 *  1.0 - converted to hal library
 *
 * Bugs:
 *
 ******************************************************************************/

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "adapter.h"
#include "cmsis_os.h"

// Blinking Period
#define InitMode 925
#define TuneDelayMode 500
#define TunePIDMode 125

// Pulse for servo
#define pick_open 750
#define pick_close 500
#define pick_hold 600
#define pitch_shot 1200
#define pitch_reload 1550
#define load_up10 600
#define load_up9 680
#define load_up8 860
#define load_up7 1040
#define load_up6 1220
#define load_up5 1440
#define load_up4 1620
#define load_up3 1800
#define load_up2 1980
#define load_up1 2120
#define load_retract 750
#define load_fix 900
#define load_down 2500
#define yaw_max 2500
#define yaw_min 500
#define pitch_max 2500
#define pitch_min 500

// BDC5 for 3 cylinders
#define shot SR.cast[0].bit0 = 1;
#define retract SR.cast[0].bit0 = 0;

#define enable_shot SR.cast[0].bit1 = 0
#define disable_shot SR.cast[0].bit1 = 1

#define start_pick __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 19999);
#define stop_pick __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);

// Servo Initialization
#define yaw_ori ServoSetPulse(&yaw_servo, 900);
#define pitch_ori ServoSetPulse(&pitch, pitch_reload); // Must Initial 1500, angle modify later
#define pick_ori ServoSetPulse(&pick, pick_open); // Initial 500, open 700
#define load_ori ServoSetPulse(&load, load_up10); // Initial 500

// Servo Functioning
#define servo_pick_open ServoSetPulse(&pick, pick_open);
#define servo_pick_hold ServoSetPulse(&pick, pick_hold);
#define servo_pick_close ServoSetPulse(&pick, pick_close);
#define servo_load_up1 ServoSetPulse(&load, load_up1);
#define servo_load_up2 ServoSetPulse(&load, load_up2);
#define servo_load_up3 ServoSetPulse(&load, load_up3);
#define servo_load_up4 ServoSetPulse(&load, load_up4);
#define servo_load_up5 ServoSetPulse(&load, load_up5);
#define servo_load_up6 ServoSetPulse(&load, load_up6);
#define servo_load_up7 ServoSetPulse(&load, load_up7);
#define servo_load_up8 ServoSetPulse(&load, load_up8);
#define servo_load_up9 ServoSetPulse(&load, load_up9);
#define servo_load_up10 ServoSetPulse(&load, load_up10);
#define servo_load_retract ServoSetPulse(&load, load_retract);
#define servo_load_fix ServoSetPulse(&load, load_fix);
#define servo_load_down ServoSetPulse(&load, load_down);
#define servo_pitch_shot ServoSetPulse(&pitch, pitch_shot);
#define servo_pitch_reload ServoSetPulse(&pitch, pitch_reload);

// Mode
enum
{
	NORMAL,
	INITIALIZE,
	TUNE_PID
};

typedef struct
{
	float distance[6];
	float angle[6];
}ENCPole_t;

ENCPole_t EncPole;

//#define printPressure

#define IP1  		HAL_GPIO_ReadPin(IP1_PIN)
#define IP2  		HAL_GPIO_ReadPin(IP2_PIN)
#define IP3  		HAL_GPIO_ReadPin(IP3_PIN)
#define IP4			HAL_GPIO_ReadPin(IP4_PIN)
#define IP5 		HAL_GPIO_ReadPin(IP5_PIN)
#define IP6 		HAL_GPIO_ReadPin(IP6_PIN)
#define IP7		    HAL_GPIO_ReadPin(IP7_PIN)
#define IP8	 		HAL_GPIO_ReadPin(IP8_PIN)
#define IP9  		HAL_GPIO_ReadPin(IP9_PIN)
#define IP10    	HAL_GPIO_ReadPin(IP10_PIN)
#define IP11  		HAL_GPIO_ReadPin(IP11_PIN)
#define IP12 		HAL_GPIO_ReadPin(IP12_PIN)
#define IP13  		HAL_GPIO_ReadPin(IP13_PIN)
#define IP14 		HAL_GPIO_ReadPin(IP14_PIN)
#define IP15		HAL_GPIO_ReadPin(IP15_PIN)

//ANALOG PIN//
#define IP16	HAL_GPIO_ReadPin(IP16_Analog1_PIN)
#define IP17	HAL_GPIO_ReadPin(IP17_Analog2_PIN)
#define	IP18   HAL_GPIO_ReadPin(IP18_Analog3_PIN)
#define IP19	HAL_GPIO_ReadPin(IP19_Analog4_PIN)
#define IP20	HAL_GPIO_ReadPin(IP20_Analog5_PIN)
#define IP21	HAL_GPIO_ReadPin(IP21_Analog6_PIN)


#define Mux1		 MUX.mux_data.bit0
#define Mux2		 MUX.mux_data.bit1
#define Mux3		 MUX.mux_data.bit2
#define Mux4		 MUX.mux_data.bit3
#define Mux5		 MUX.mux_data.bit4
#define Mux6		 MUX.mux_data.bit5
#define Mux7		 MUX.mux_data.bit6
#define Mux8		 MUX.mux_data.bit7

osThreadId_t MainTaskHandle;
osThreadId_t LaserNavigateTaskHandle;
osThreadId_t EmergencyTaskHandle;
osThreadId_t SecondaryTaskHandle;
osThreadId_t TuneTaskHandle;
osThreadId_t FlywheelPitchPIDTaskHandle;
osThreadId_t FlywheelYawPIDTaskHandle;
osThreadId_t TestTaskHandle;

osSemaphoreId_t TuneSemaphore;

typedef union{
	uint16_t flags;
	struct{
		//Least significant 16 bits can be cleared all at once by
		//sys.flags = 0 for example during emergency
		unsigned manual       :1;
		unsigned tunePid      :1;
		unsigned pp_ready	  :1;
		unsigned pp_start     :1;
		unsigned rns_busy     :1;
		unsigned ros_test_start  :1;
		unsigned ros_path_start  :1;
		unsigned ros_stop		  :1;
		unsigned vel_ready        :1;
		unsigned stop        :1;
		unsigned navi_vel	      :1;
		unsigned flag11		  :1;
		unsigned flag12		  :1;
		unsigned flag13       :1;
		unsigned flag14       :1;
		unsigned flag15       :1;

		//Most significant 16 bits are not clear
		unsigned flag16       :1;
		unsigned flag17       :1;
		unsigned flag18	      :1;
		unsigned flag19       :1;
		unsigned flag20       :1;
		unsigned flag21       :1;
		unsigned flag22       :1;
		unsigned flag23		  :1;
		unsigned flag24       :1;
		unsigned flag25       :1;
		unsigned flag26	      :1;
		unsigned flag27		  :1;
		unsigned flag28		  :1;
		unsigned flag29       :1;
		unsigned flag30       :1;
		unsigned flag31       :1;

		//A flag can use more than 1 bit
		//Example : Combine flag30 and flag31
		//unsigned flag29     :1;
		//unsigned flag30     :2;
		//the value of sys.flag30 range from 0 to 3 then overflow to 0 again and vice versa
		//the value of flag29 is not affected when flag30 overflow
	};
}sys_t;

sys_t sys;

void RNS_config(CAN_HandleTypeDef* hcanx);
void set(void);
void manual_mode(void);
void PS4Tune(void);
void RobotStart(void);
void DelayTune(void);
void NormalControl(void);
void CalcPoleDist(ENCPole_t* pole);
void Init(void);
void enq(void);
void Await(uint32_t ticks);

#define NUM_INT_UPDATE		1
#define NUM_FLOAT_UPDATE	1

void ILI9341_Init_List(void);
void ILI9341_Update_List(void);
void calcSpeed(uint32_t);
void registerPath(float path[][7], int numPoint, ...);

struct{
	UART_HandleTypeDef* huart;
	uint8_t buffer[10];
	char lagoriColor;
	uint16_t lagoriHeight;
	uint16_t lagoriWidth;
	uint32_t lagoriArea;
}esp32;

#endif /* INC_COMMON_H_ */
