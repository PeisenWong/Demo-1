#ifndef VESC_CAN_VESC_INTERFACE_H_
#define VESC_CAN_VESC_INTERFACE_H_

#include "vesc_can.h"
#include "motor_test.h"
#include "../PP/PP.h"

#define MIN_POSCNT 500

typedef struct{
	float max_rpm;
	float pole_pairs;
	float wheel_diameter;
	uint16_t Instance;
	INFO info;

	union{
		uint8_t flags;
		struct{
			unsigned current_flag	: 1;
			unsigned rpm_flag		: 1;
			unsigned pdc_flag		: 1;
			unsigned flag4			: 1;
			unsigned flag5			: 1;
			unsigned flag6			: 1;
			unsigned flag7			: 1;
			unsigned flag8			: 1;
		};

	};

}VESC_t;

struct
{
	VESC_t a, b, c, d;
	float a_vel, b_vel, c_vel, d_vel;
	float xPtd, yPtd;
}VESCNav;

void VESCInit(float max_rpm, float pole_pairs, float wheel_diameter,uint16_t Instance, VESC_t* vesc);
void VESCVelocity(float Velocity, VESC_t* vesc);
void VESCRPM(float RPM, VESC_t* vesc);
void VESCPDC(float Duty, VESC_t* vesc);
void VESCStop(VESC_t* vesc);
void VESCHandBrake(VESC_t* vesc, float current);

void VESCNavInit(float max_rpm, float pole_pairs, float xPtd, float yPtd, float wheel_diameter, uint16_t Instance1, uint16_t Instance2, uint16_t Instance3, uint16_t Instance4);
void VESCNavVel(float a, float b, float c, float d);
void VESCNavPDC(float a, float b, float c, float d);
void VESCPDCFlywheel(float Duty, VESC_t* vesc);
void VESCNavRPM(float a, float b, float c, float d);
void VESCNavStop();
void VESCNavBrake(float a, float b, float c, float d);
void VESCNavCalVel(float rpmA, float rpmB, float rpmC, float rpmD);
float VESC_calVel(float rpm);
void VESCPIDProcess(float a, float b, float c, float d);
void VESCPIDAct(void);
void VESCNav5ms();

mc_values VESCEnquire(VESC_t* vesc, COMM_GET_VALUE_SELECTIVE_t parameter);

#endif /* VESC_CAN_VESC_INTERFACE_H_ */
