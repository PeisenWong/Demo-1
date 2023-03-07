#include "../adapter.h"
#include <math.h>

#define HandBrake 100

static void bldc_send_packet(uint8_t controller_id, uint8_t *data, unsigned int len);

/*\
 * Assume same type of motor used in navigation
 * Init all motor at once with different CAN_ID
 */
void VESCNavInit(float max_rpm, float pole_pairs, float xPtd, float yPtd, float wheel_diameter, uint16_t Instance1, uint16_t Instance2, uint16_t Instance3, uint16_t Instance4)
{
	VESCInit(max_rpm, pole_pairs, wheel_diameter, Instance1, &VESCNav.a);
	VESCInit(max_rpm, pole_pairs, wheel_diameter, Instance2, &VESCNav.b);
	VESCInit(max_rpm, pole_pairs, wheel_diameter, Instance3, &VESCNav.c);
	VESCInit(max_rpm, pole_pairs, wheel_diameter, Instance4, &VESCNav.d);
	VESCNav.xPtd = xPtd;
	VESCNav.yPtd = yPtd;
}

/*
 * Send Velocity command to all motor at once
 */
void VESCNavVel(float a, float b, float c, float d)
{
	fFLeftVelR = a;
	fFRightVelR = b;
	fBLeftVelR = c;
	fBRightVelR = d;
}

/*
 * Send Velocity command to all motor at once
 */
void VESCNavPDC(float a, float b, float c, float d)
{
	VESCPDC(a, &VESCNav.a);
	VESCPDC(b, &VESCNav.b);
	VESCPDC(c, &VESCNav.c);
	VESCPDC(d, &VESCNav.d);
}

/*
 * Send Velocity command to all motor at once
 */
void VESCNavRPM(float a, float b, float c, float d)
{
	VESCRPM(a, &VESCNav.a);
	VESCRPM(b, &VESCNav.b);
	VESCRPM(c, &VESCNav.c);
	VESCRPM(d, &VESCNav.d);
}

/*
 * Stop all motor at once
 */
void VESCNavStop()
{
	if((VESCNav.a_vel || VESCNav.b_vel || VESCNav.c_vel || VESCNav.d_vel) && HAL_GetTick() - hb_count > HandBrake)
	{
		VESCPIDProcess(-pow(VESCNav.a_vel, 2), -pow(VESCNav.b_vel, 2), -pow(VESCNav.c_vel, 2), -pow(VESCNav.d_vel, 2));
	}
	else
	{
		sys.navi_vel = 0;

		PIDDelayInit(&fright_vel);
		PIDDelayInit(&fleft_vel);
		PIDDelayInit(&bleft_vel);
		PIDDelayInit(&bright_vel);

		fFLeftVelU = 0.0;
		fFRightVelU = 0.0;
		fBLeftVelU = 0.0;
		fBRightVelU = 0.0;

		VESCStop(&VESCNav.a);
		VESCStop(&VESCNav.b);
		VESCStop(&VESCNav.c);
		VESCStop(&VESCNav.d);
	}
}

/*
 * Brake all motor at once
 */
void VESCNavBrake(float a, float b, float c, float d)
{
	VESCHandBrake(&VESCNav.a, a);
	VESCHandBrake(&VESCNav.b, b);
	VESCHandBrake(&VESCNav.c, c);
	VESCHandBrake(&VESCNav.d, d);
}

void VESCNavCalVel(float rpmA, float rpmB, float rpmC, float rpmD)
{
//	rpmA *= (MCCONF_SI_MOTOR_POLES / 2);
//	rpmB *= (MCCONF_SI_MOTOR_POLES / 2);
//	rpmC *= (MCCONF_SI_MOTOR_POLES / 2);
//	rpmD *= (MCCONF_SI_MOTOR_POLES / 2);

	VESCNav.a_vel = (((rpmA / (MCCONF_SI_MOTOR_POLES / 2)) / MCCONF_SI_GEAR_RATIO) * (M_PI * MCCONF_SI_WHEEL_DIAMETER)) / 60;
	VESCNav.b_vel = (((rpmB / (MCCONF_SI_MOTOR_POLES / 2)) / MCCONF_SI_GEAR_RATIO) * (M_PI * MCCONF_SI_WHEEL_DIAMETER)) / 60;
	VESCNav.c_vel = (((rpmC / (MCCONF_SI_MOTOR_POLES / 2)) / MCCONF_SI_GEAR_RATIO) * (M_PI * MCCONF_SI_WHEEL_DIAMETER)) / 60;
	VESCNav.d_vel = (((rpmD / (MCCONF_SI_MOTOR_POLES / 2)) / MCCONF_SI_GEAR_RATIO) * (M_PI * MCCONF_SI_WHEEL_DIAMETER)) / 60;
}

/*
 * Calculate vel from rpm
 */
float VESC_calVel(float rpm)
{
	float vel;
//	rpm /= (MCCONF_SI_MOTOR_POLES / 2);
	vel = (((rpm / (MCCONF_SI_MOTOR_POLES / 2.0)) * MCCONF_SI_GEAR_RATIO) * (M_PI * 0.127)) / 60;
	return vel;
}

/*
 * Keep calculating error from feedback of motor
 * Should be called in Calculation task
 * Recommend to use this function for robot navigation rather than VESCVelocity
 */
void VESCPIDProcess(float a, float b, float c, float d)
{
	fFLeftVelErr = a - VESCNav.a_vel;
	fFRightVelErr = b - VESCNav.b_vel;
	fBLeftVelErr = c - VESCNav.c_vel;
	fBRightVelErr = d - VESCNav.d_vel;

	sys.navi_vel = 1;
}

/*
 * Implement the PID output for robot navigation
 */
void VESCPIDAct(void)
{
	if(sys.navi_vel)
	{
		PID(&fleft_vel);
		PID(&fright_vel);
		PID(&bleft_vel);
		PID(&bright_vel);
	}
	VESCPDC(fFLeftVelU / 19999.00, &VESCNav.a);
	VESCPDC(fFRightVelU / 19999.00, &VESCNav.b);
	VESCPDC(fBLeftVelU / 19999.00, &VESCNav.c);
	VESCPDC(fBRightVelU / 19999.00, &VESCNav.d);
}

/*
 * System 5ms
 */
void VESCNav5ms()
{
	fXEncData = 0.05 / 4000.0 * 3.142 * (QEIRead(QEI1) - 500);
	fYEncData = 0.05 / 4000.0 * 3.142 * (QEIRead(QEI4) - 500);

	ABT(&x_data);
	ABT(&y_data);
	fyaw = (fyaw >= 0.0) ? fyaw : 360.0 + fyaw;

	PathPlan(&pp);
	VESCNavCalVel(VESCNav.a.info.rpm, VESCNav.b.info.rpm, VESCNav.c.info.rpm, VESCNav.d.info.rpm);
	CalcPoleDist(&EncPole);
	VESCPIDAct();
}

/*
 * Function Name		: VESCInit
 * Function Description : Initialize the bldc parameters
 * Function Remarks		: NONE
 * Function Arguments	: max_rpm			maximum rpm of the bldc
 * 						  pole_pairs		number of pole pairs of the bldc
 * 						  wheel_diameter 	wheel diameter
 * 						  vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCInit(40000,1,0.0037,&csev);
 */
void VESCInit(float max_rpm, float pole_pairs, float wheel_diameter,uint16_t Instance, VESC_t* vesc){
	bldc_interface_init(bldc_send_packet);
//	bldc_interface_set_rx_value_selective_func(bldc_val_selective_received_cb);

	vesc->max_rpm = max_rpm;
	vesc->pole_pairs = pole_pairs;
	vesc->wheel_diameter = wheel_diameter;
	vesc->Instance = Instance;
}

/*
 * Function Name		: VESCVelocity
 * Function Description : Command the VESC to move with specified velocity without any position control.
 * Function Remarks		: NONE
 * Function Arguments	: FLeftVel		speed of front left motor in meter per second
 * 						  FRightVel		speed of front right motor in meter per second
 * 						  BLeftVel 		speed of back left motor in meter per second
 * 						  BRightVel		speed of back right motor in meter per second
 * 						  vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCVelocity(1.0, 1.5 , 1.5 , 1.0, &csev);
 */
void VESCVelocity(float Velocity, VESC_t* vesc) {

	float vel, rpm;
//	vel = ((Velocity * 60) / (M_PI * vesc->wheel_diameter)) * vesc->pole_pairs;
	vel = ((Velocity * 60) / (M_PI * vesc->wheel_diameter)) * MCCONF_SI_GEAR_RATIO;
	rpm = vel * vesc->pole_pairs;

	VESCRPM(rpm, vesc);
}

/*
 * Function Name		: VESCRPM
 * Function Description : Command the VESC to move with specified RPM without any position control.
 * Function Remarks		: NONE
 * Function Arguments	: FLeftRPM		speed of front left motor in revolution per minute
 * 						  FRightRPM		speed of front right motor in revolution per minute
 * 						  BLeftRPM 		speed of back left motor in revolution per minute
 * 						  BRightRPM		speed of back right motor in revolution per minute
 * 						  vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCRPM(2000, 2000, 2000, 2000, &csev);
 */
void VESCRPM(float RPM, VESC_t* vesc) {

	if(RPM > vesc->max_rpm){
		RPM = vesc->max_rpm;
	}

//	RPM = RPM * vesc->pole_pairs;

	comm_can_set_rpm(vesc->Instance, RPM);

	vesc->rpm_flag = 1;
}

/*
 * Function Name		: VESCPDC
 * Function Description : Command the VESC to move with specified duty cycle without any position control.
 * Function Remarks		: The range of duty cycle : 0.0-0.9
 * Function Arguments	: FLeftPDC		duty cycle of front left motor
 * 						  FRightPDC		duty cycle of front right motor
 * 						  BLeftPDC 		duty cycle of back left motor
 * 						  BRightPDC		duty cycle of back right motor
 * 						  vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCPDC(0.5, 0.5, 0.5, 0.5);
 */
void VESCPDC(float Duty, VESC_t* vesc) {

	if(Duty > 0.9){
		Duty = 0.9;
	}

	comm_can_set_duty(vesc->Instance, Duty);

	vesc->pdc_flag = 1;
}

void VESCPDCFlywheel(float Duty, VESC_t* vesc) {

	if(Duty > 0.9){
		Duty = 0.9;
	}

	comm_can_set_duty_flywheel(vesc->Instance, Duty);

	vesc->pdc_flag = 1;
}

/*
 * Function Name		: VESCStop
 * Function Description : Command the VESC to stop
 * Function Remarks		: NONE
 * Function Arguments	: vesc 			pointer to a VESC data structure with VESC_t type
 * Function Return		: None
 * Function Example		: VESCStop(&csev);
 */
void VESCStop(VESC_t* vesc) {

	if(vesc->pdc_flag) {
		comm_can_set_duty(vesc->Instance, 0.0);
	}

	if(vesc->rpm_flag) {
		comm_can_set_rpm(vesc->Instance, 0.0);
	}

	if(vesc->current_flag){
		comm_can_set_current(vesc->Instance, 0.0);
	}

	vesc->flags = 0;
}

void VESCHandBrake(VESC_t* vesc, float current)
{
	comm_can_set_handbrake(vesc->Instance, current);
}

/*
 * Function Name		: VESCEnquire
 * Function Description : Enquire the VESC board and the motor parameters
 * Function Remarks		: NONE
 * Function Arguments	: controller_id		CAN id of the target VESC board
 * 						  parameter			Enumeration to the enquired parameters
 * 						  print				True to print the result
 * Function Return		: None
 * Function Example		: VESCEnquire(VESC1, AVG_ID, true);
 */
mc_values VESCEnquire(VESC_t* vesc, COMM_GET_VALUE_SELECTIVE_t parameter) {
//	rx_value_complete = false;
	rx_value_selective_print = false;
	if(parameter < ALL)
		bldc_interface_get_values_selective(vesc->Instance, parameter);
	else
		bldc_interface_get_values(vesc->Instance);
//	while(!rx_value_complete);
//	rx_value_complete = false;
	return rx_value_buf;
}

/*	Private Function	*/
void bldc_send_packet(uint8_t controller_id, uint8_t *data, unsigned int len) {
	comm_can_send_buffer(controller_id, data, len, 0);
}

