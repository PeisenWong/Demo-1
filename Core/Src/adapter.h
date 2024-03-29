/*******************************************************************************
 * Title   : adapter.h
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 12/10
 *******************************************************************************
 * Description: includes all the important includes and pin definitions
 *
 * Version History:
 *  1.0 - converted to HAL library
 *
 * Bugs:
 *
 ******************************************************************************/

#ifndef SRC_ADAPTER_H_
#define SRC_ADAPTER_H_


/* Private variables ---------------------------------------------------------*/


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "BIOS/bios.h"
#include <math.h>
#include "CAN/can.h"
#include "MODN/MODN.h"
#include "PID/PID.h"
#include "ABT/ABT.h"
#include "I2C/i2c.h"
#include "SPI/SPI.h"
#include "PSx_Interface/PSx_Interface.h"
#include "RNS_interface/RNS_interface.h"
#include "ADC/adc.h"
#include "SERVO/servo.h"
#include "KF/KF.h"
#include "LASER/laser.h"
#include "STEPPER/stepper.h"
#include "SERVO_DRIVER/servo_driver.h"
#include "Moving_Average/mov_ave.h"
#include "VESC_CAN/vesc_interface.h"
#include "Eeprom/eeprom.h"
#include "ILI9341/ILI9341_Driver.h"
#include "Dongbei/dongbei.h"
#include "RGB/rgb.h"
#include "TuningInterface/TuningInterface.h"
#include "Tune.h"
#include "ROS_Interface/ROS_Interface.h"
#include "ROS_navi/ROS_navi.h"
#include "R6091U/r6091u.h"
#include "obstacle_detect/obstacle_detect.h"
/* Exported functions prototypes ---------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define LED1_PIN			GPIOC, GPIO_PIN_13
#define LED2_PIN			GPIOC, GPIO_PIN_14
#define LED3_PIN			GPIOC, GPIO_PIN_15

#define PB1_PIN				GPIOB, GPIO_PIN_7
#define PB2_PIN				GPIOE, GPIO_PIN_0

#define IP1_PIN				GPIOE, GPIO_PIN_12              /* TIM1_CH3N                                              */
#define IP2_PIN				GPIOE, GPIO_PIN_13              /* TIM1_CH3                                               */
#define IP3_PIN				GPIOE, GPIO_PIN_14              /* TIM1_CH4                                               */
#define IP4_PIN				GPIOE, GPIO_PIN_15              /* TIM1_BKN                                               */
#define IP5_PIN				GPIOB, GPIO_PIN_14              /* TIM12_CH1                                              */
#define IP6_PIN				GPIOB, GPIO_PIN_15              /* TIM12_CH2                                              */
#define IP7_PIN				GPIOD, GPIO_PIN_10              /*                                                        */
#define IP8_PIN				GPIOD, GPIO_PIN_11              /*                                                        */
#define IP9_PIN				GPIOC, GPIO_PIN_8               /* TIM8_CH3                                               */
#define IP10_PIN			GPIOA, GPIO_PIN_10              /* TIM1_CH3                                               */
#define IP11_PIN			GPIOD, GPIO_PIN_3               /*                                                        */
#define IP12_PIN			GPIOD, GPIO_PIN_4               /*                                                        */
#define IP13_PIN			GPIOD, GPIO_PIN_7               /*                                                        */
#define IP14_PIN			GPIOB, GPIO_PIN_5               /*                                                        */
#define IP15_PIN			GPIOB, GPIO_PIN_6               /* TIM4_CH1                                               */

#define IP16_Analog1_PIN	GPIOC, GPIO_PIN_0
#define IP17_Analog2_PIN	GPIOC, GPIO_PIN_1
#define IP18_Analog3_PIN	GPIOC, GPIO_PIN_2
#define IP19_Analog4_PIN	GPIOC, GPIO_PIN_3
#define IP20_Analog5_PIN	GPIOC, GPIO_PIN_4
#define IP21_Analog6_PIN	GPIOC, GPIO_PIN_5

#define QEI1_PLUSEA_PIN		GPIOE , GPIO_PIN_9
#define QEI1_PLUSEB_PIN		GPIOE , GPIO_PIN_11

#define QEI4_PLUSEA_PIN		GPIOD , GPIO_PIN_12
#define QEI4_PLUSEB_PIN		GPIOD , GPIO_PIN_13

#define QEI6_PLUSEA_PIN		GPIOC , GPIO_PIN_6
#define QEI6_PLUSEB_PIN		GPIOC , GPIO_PIN_7

#define TIM5_CHANNEL1_PIN	GPIOA, GPIO_PIN_0
#define TIM5_CHANNEL2_PIN	GPIOA, GPIO_PIN_1
#define TIM5_CHANNEL3_PIN	GPIOA, GPIO_PIN_2
#define TIM5_CHANNEL4_PIN	GPIOA, GPIO_PIN_3

#define TIM9_CHANNEL1_PIN	GPIOE, GPIO_PIN_5
#define TIM9_CHANNEL2_PIN	GPIOE, GPIO_PIN_6

#define TIM3_CHANNEL1_PIN   GPIOA, GPIO_PIN_6
#define TIM3_CHANNEL3_PIN	GPIOB, GPIO_PIN_0
#define TIM3_CHANNEL4_PIN	GPIOB, GPIO_PIN_1

#define MUX1_INPUT_PIN 		GPIOE , GPIO_PIN_1
#define MUX1_S0_PIN 		GPIOE , GPIO_PIN_2
#define MUX1_S1_PIN 		GPIOE , GPIO_PIN_3
#define MUX1_S2_PIN 		GPIOE , GPIO_PIN_4

#define SR_SCK_PIN			GPIOE , GPIO_PIN_7
#define SR_RCK_PIN			GPIOE , GPIO_PIN_8
#define SR_SI_PIN			GPIOE , GPIO_PIN_10

#define SPI1_NSS_PIN		GPIOA, GPIO_PIN_4
#define SPI1_SCK_PIN		GPIOA, GPIO_PIN_5
#define SPI1_MISO_PIN		GPIOA, GPIO_PIN_6
#define SPI1_MOSI_PIN		GPIOA, GPIO_PIN_7

#define UART2_Tx			GPIOD , GPIO_PIN_5
#define UART2_Rx			GPIOD , GPIO_PIN_6

#define UART3_Tx			GPIOD , GPIO_PIN_9
#define UART3_Rx			GPIOD , GPIO_PIN_8

#define UART4_Tx			GPIOC , GPIO_PIN_10
#define UART4_Rx			GPIOC , GPIO_PIN_11

#define UART5_Tx			GPIOC , GPIO_PIN_12
#define UART5_Rx			GPIOD , GPIO_PIN_2

#define CAN1_Tx				GPIOD , GPIO_PIN_1
#define CAN1_Rx				GPIOD , GPIO_PIN_0

#define CAN2_Tx				GPIOB , GPIO_PIN_13
#define CAN2_Rx				GPIOB , GPIO_PIN_12


MUX_t MUX;
shiftreg_t SR;
RNS_interface_t rns;
BDC_t BDC1, BDC2, BDC3, BDC4, BDC5, BDC6, BDC7, BDC8;
uint8_t insData_receive[2];
PSxBT_t ps4;
ABT_t filter;
ADC_t adc;
LASER_t r_laser,l_laser;
KALMANFILTER_t kf_adc_r,kf_adc_l,kf_pres;
PID_t pid_laser_R,pid_laser_L,pid_pres,pid_z;
Srv_Drv_t srv_drv;
Mov_Ave_t mov_l_r,mov_l_l;
//VESC_t vesc1, vesc2, vesc3, vesc4, vesc5;
SERVO_t vesc_ppm[3], yaw_servo, load, pick, pitch;
R6091U_t imu;

#define PB1 		GPIOB_IN->bit7
#define PB2 		GPIOE_IN->bit0

#define led1		GPIOC_OUT->bit13
#define led2		GPIOC_OUT->bit14
#define led3		GPIOC_OUT->bit15

//Global Declarations
float v1, v2, v3, v4, wa, xr, yr;                 //MODN variables
float xpos, ypos, z;                              //Encoder Values
float a, b, c, d, pa, pb, pc, pd;  				  //Enquiry Values
volatile uint16_t  adc1_buf[7];
volatile uint16_t  adc2_buf[7];
volatile uint16_t  adc3_buf[7];

int counter;                                      //global timer variable
float speed;                                      //for manual control

float FLEnc, FREnc, BLEnc, BREnc, XEnc, YEnc, yaw;
float stop_time;
uint32_t servo_pulse;
uint8_t position[12], debug[200];
int acks, flywheel;

uint8_t I2CIMU[80];

// Testing VESC navigation
float vesc_input_x, vesc_input_y, vesc_input_w, vesc_move_speed, vesc_spin_speed, vesc_pulse, vesc_pdc;
float retract_delay, enable_delay, pick_delay, load_delay, pick_cylinder_delay;
float ps4_yaw, ps4_pitch;
int hb_count, run, inc;
uint8_t imu_data[15];

/* SAT, KE, KU, KP, KI, KD, KN */
extern float fFLeftVG[7];
extern float fFRightVG[7];
extern float fBLeftVG[7];
extern float fBRightVG[7];

extern float fXPosGain[3];
extern float fYPosGain[3];

// Calculate Pole Distance
extern float pole_dist_x[6];
extern float pole_dist_y[6];

ABT_t x_data, y_data;
float fXEncData, fYEncData;
float fXPos, fYPos;			/* Position, ABT output, pos PID feedback */
float fXVel, fYVel;			/* Velocity, ABT output */
float fXAcc, fYAcc;			/* Acceleration, ABT output*/

/* PID input/output */
float fFLeftVelR, fFRightVelR, fBLeftVelR, fBRightVelR;						/* Velocity PID reference */
float fFLeftVelU, fFRightVelU, fBLeftVelU, fBRightVelU;						/* Velocity PID output */
float fFLeftVelErr, fFRightVelErr, fBLeftVelErr, fBRightVelErr;				/* Velocity PID error */
PID_t fleft_vel, fright_vel, bleft_vel, bright_vel, IMU_PID;

// tune flag
int tune_p, tune_i, tune_d, wheel;

int mode, before;

VESC_t flywheel1, flywheel2, vesc1;

union{
	float data;
	struct{
		char byte1;
		char byte2;
		char byte3;
		char byte4;
	};
}buf1_receive[2];
union{
	float data;
	struct{
		char byte1;
		char byte2;
		char byte3;
		char byte4;
	};
}buf2_receive[2];

typedef enum{
	RNS_PACKET,
	VESC_PACKET,
}PACKET_t;

uint32_t faul_counter;
struct{
	uint16_t statusword;
	int pos_act;
	int vel_act;
	int16_t tor_act;
	uint16_t ODindex;
	uint8_t ODsubindex;
	uint8_t num_valid;
	uint8_t buffer[4];
	int HomeOffset;
	uint32_t velKp;
	int8_t modeOperation;
	union{
		uint16_t flags;
		struct{
			unsigned can    	    :1;
			unsigned flag1          :1;
			unsigned flag2    		:1;
			unsigned flag3          :1;
			unsigned flag4          :1;
			unsigned flag5          :1;
			unsigned flag6          :1;
			unsigned flag7          :1;
			unsigned flag8          :1;
			unsigned flag9          :1;
			unsigned flag10         :1;
			unsigned flag11         :1;
			unsigned flag12         :1;
			unsigned flag13 	    :1;
			unsigned flag14         :1;
			unsigned flag15        	:1;
		};
	};
}Faul_t;
void CAN_PROCESS(PACKET_t packet_src);
void Initialize (void);

#ifdef __cplusplus
}
#endif
#endif /* SRC_ADAPTER_H_ */
