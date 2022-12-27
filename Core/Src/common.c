
/*********************************************/
/*          Include Header                   */
/*********************************************/

#include "common.h"
#include "math.h"

/* SAT, KE, KU, KP, KI, KD, KN */
float fFLeftVG[7] = {1.0, 1.0 / 22.44, 3999, 1.42, 1.19, 0.00, 60.0};
float fFRightVG[7] = {1.0, 1.0 / 22.44, 3999, 1.36, 1.27, 0.0028, 60.0};
float fBLeftVG[7] = {1.0, 1.0 / 22.44, 3999, 1.58, 1.23, 0.00, 60.0};
float fBRightVG[7] = {1.0, 1.0 / 22.44, 3999, 1.55, 1.34, 0.00, 60.0};

// For ABT of x, y
float fXPosGain[3] = {0.8, 0.4, 0.2};
float fYPosGain[3] = {0.8, 0.4, 0.2};

// start point = (5.975, 0.5) Center of starting zone
// Pole1 (2.775, 2.75) Pole2 (5.975, 2.75) Pole3(9.175, 2.75)
// Pole4 (4.675, 4.675) Pole5 (7.275, 4.675)
// Pole6 (5.975, 6)
float pole_dist_x[6] = {-3.2, 0.0, 3.2, -1.3, 1.3, 0.0};
float pole_dist_y[6] = {2.25, 2.25, 2.25, 4.175, 4.175, 5.5};

/*
 * Demo 1
 * 4 servo used, 2 for ring picking, 2 to control shooting angle
 * 6 VESC used, 4 for navigation, 2 for flywheel
 * 3 cylinder used to push ring out
 */

void set(void) {

	Initialize();
//	ROS_Navi_Init(&huart4, &huart5);
	ROS_Init(&huart4);
	ROS_Read_Flash();
	PSxInitDMA(&ps4,&hi2c1);
//	PSxInitUART(&ps4, &huart4);
//	LidarInit(&huart5);
	TIMxInit(&htim6, 5000, 84); // 5ms
//	tuneRegister(2, &huart2);
//	tuneRead();
//	RNS_config(&hcan1);
	MODNInit(MODN_FWD_OMNI, BRUSHLESS, 2.0 , 0.5, 2.0, 0.1);
	setOrientationMODN(OPERATOR_TURNED_90_DEGREES_ANTICLOCKWISE);
	VESCNavInit(35000, MCCONF_SI_MOTOR_POLES / 2, 0.05 / 4000.0 * 3.142, 0.05 / 4000.0 * 3.142, 0.1, 111, 112, 113, 114);
	VESCInit(35000, MCCONF_SI_MOTOR_POLES / 2, 0.127, 115, &flywheel1);
	VESCInit(35000, MCCONF_SI_MOTOR_POLES / 2, 0.127, 116, &flywheel2);

	//ROBOCONPID
	/* Left velocity PID*/
	PIDSourceInit(&fFLeftVelErr, &fFLeftVelU, &fleft_vel);
	PIDDelayInit(&fleft_vel);
	PIDGainInit(SAMPLE_TIME, fFLeftVG[0], fFLeftVG[1], fFLeftVG[2], fFLeftVG[3], fFLeftVG[4], fFLeftVG[5], fFLeftVG[6], &fleft_vel);

	PIDSourceInit(&fBLeftVelErr, &fBLeftVelU, &bleft_vel);
	PIDDelayInit(&bleft_vel);
	PIDGainInit(SAMPLE_TIME, fBLeftVG[0], fBLeftVG[1], fBLeftVG[2], fBLeftVG[3], fBLeftVG[4], fBLeftVG[5], fBLeftVG[6], &bleft_vel);

	/* Right velocity PID*/
	PIDSourceInit(&fFRightVelErr, &fFRightVelU, &fright_vel);
	PIDDelayInit(&fright_vel);
	PIDGainInit(SAMPLE_TIME, fFRightVG[0], fFRightVG[1], fFRightVG[2], fFRightVG[3], fFRightVG[4], fFRightVG[5], fFRightVG[6], &fright_vel);

	PIDSourceInit(&fBRightVelErr, &fBRightVelU, &bright_vel);
	PIDDelayInit(&bright_vel);
	PIDGainInit(SAMPLE_TIME, fBRightVG[0], fBRightVG[1], fBRightVG[2], fBRightVG[3], fBRightVG[4], fBRightVG[5], fBRightVG[6], &bright_vel);

	/* X & Y position ABT */
	ABTInit(SAMPLE_TIME, fXPosGain[0], fXPosGain[1], fXPosGain[2], &fXEncData ,&fXPos, &fXVel, &fXAcc, &x_data);
	ABTEstimateInit(&x_data);

	ABTInit(SAMPLE_TIME, fYPosGain[0], fYPosGain[1], fYPosGain[2], &fYEncData, &fYPos, &fYVel, &fYAcc, &y_data);
	ABTEstimateInit(&y_data);

	// X, Y Encoder
	QEIInit(&htim1);
	QEIInit(&htim4);
	QEISwap(QEI4, QEI_Swap);

	QEIWrite(QEI1, 500);
	QEIWrite(QEI4, 500);

	PPInit(0, &fXPos, &fYPos, &fyaw, &pp);
	PP_PIDPathSet(1.0, 0.5, 0.5, &pp);
	PP_PIDEndSet(0.5, 0.1, 0.7, &pp);
	PP_PIDZSet(0.5, 0.05, 0.2, 5.5, &pp);
	PP_SetCrv_Points(10, &pp);

	R6091U_Init(&imu, &huart5);
}

void CalcPoleDist(ENCPole_t* pole)
{
	for(int i = 0; i < 6; i++)
	{
		pole_dist_x[i] -= pp.real_x;
		pole_dist_y[i] -= pp.real_y;

		pole->distance[i] = sqrt((pow(pole_dist_x[i], 2)) + (pow(pole_dist_y[i], 2)));
		pole->angle[i] = atan(pole_dist_y[i] / pole_dist_x[i]) * 180 / M_PI;
	}
}

void RobotStart()
{
	// Tune PID
	tune_p = 0;
	tune_i = 0;
	tune_d = 0;

	AP = 1.5;
	AI = 1.5;
	AD = 0.0;

	BP = 1.0;
	BI = 1.5;
	BD = 0.0;

	CP = 1.0;
	CI = 1.5;
	CD = 0.0;

	DP = 1.0;
	DI = 1.5;
	DD = 0.0;

	wheel = 1;
	vesc_pulse = 1.4;
	vesc_move_speed = 2;
	vesc_pdc = 0.5;

	mode = NORMAL;

	// Servo Init
	pick_ori;
	load_ori;
	yaw_ori;
	pitch_ori;

	// Cylinder Init
	retract;
	enable_shot;
	stop_pick;

	pick_delay = 500;
	load_delay = 1000;
	retract_delay = 500;
	enable_delay = 500;

	ps4_yaw = 750;
	ps4_pitch = 1550;

}

void NormalControl()
{
	led3 = 0;

	if(ps4.button == L1)
	{
		while(ps4.button == L1);
		vesc_move_speed++;
		if(vesc_move_speed >= 5) vesc_move_speed = 5;
		setSpeedMODN(vesc_move_speed);
	}

	if(ps4.button == R1)
	{
		while(ps4.button == R1);
		vesc_move_speed--;
		if(vesc_move_speed <= 0) vesc_move_speed = 0;
		setSpeedMODN(vesc_move_speed);
	}

	// Pick
	if(ps4.button == CIRCLE)
	{
		while(ps4.button == CIRCLE);

		servo_pitch_reload;
		servo_load_down;

		servo_pick_open;

		start_pick;
	}

	// Load
	if(ps4.button == SQUARE)
	{
		while(ps4.button == SQUARE)
		{
			servo_pick_close;
		}

		stop_pick;
		osDelay(50);

		pitch_ori;

		servo_load_up1;
		osDelay(240);

		servo_load_up2;
		osDelay(240);

		servo_load_up3;
		osDelay(240);

		servo_load_up4;
		osDelay(240);

		servo_load_up5;
		osDelay(240);

		servo_load_up6;
		osDelay(240);

		servo_load_up7;
		osDelay(240);

		servo_load_up8;
		osDelay(240);

		servo_load_up9;
		osDelay(240);

		servo_load_up10;

		osDelay(200);
		servo_pitch_shot;

		osDelay(500);
		servo_pick_hold;

		osDelay(200);

		enable_shot;
		osDelay(500);

		disable_shot;
		flywheel = 1;

	}

	// Shot
	if(ps4.button == TRIANGLE)
	{
		while(ps4.button == TRIANGLE);
		shot;
		osDelay(500);

		servo_load_retract;
		osDelay(500);

		retract;
		osDelay(500);

		servo_load_up10;
		osDelay(150);

		enable_shot;
		osDelay(500);

		disable_shot;
	}

	// Try fix ring position
	if(ps4.button == OPTION)
	{
		while(ps4.button == OPTION);
		servo_load_retract;
		enable_shot;

		osDelay(500);

		servo_load_up10;
		osDelay(150);
		servo_pick_close;

		osDelay(500);
		servo_pick_hold;

		disable_shot;
	}

	// Enable / Disable fly wheel
	if(ps4.button == CROSS)
	{
		while(ps4.button == CROSS);

		if(!flywheel)
			flywheel = 1;
		else
			flywheel = 0;
	}

	if(ps4.button == LEFT)
	{
		while(ps4.button == LEFT);
		vesc_pdc += 0.1;
	}

	if(ps4.button == RIGHT)
	{
		while(ps4.button == RIGHT);
		vesc_pdc -= 0.1;
	}
}

void Init()
{
//	realMODN(&ps4, &rns);

	// Picking Cylinder
	if(ps4.button == CIRCLE)
	{
		while(ps4.button == CIRCLE);
		static int count = 0;
		if(count == 0)
		{
			start_pick;
			count++;
		}
		else
		{
			stop_pick;
			count = 0;
		}
	}

	// Shot Cylinder
	if(ps4.button == CROSS)
	{
		while(ps4.button == CROSS);
		static int count = 0;
		if(count == 0)
		{
			shot;
			count++;
		}
		else
		{
			retract;
			count = 0;
		}
	}

	// Enable Shot Cylinder
	if(ps4.button == TRIANGLE)
	{
		while(ps4.button == TRIANGLE);
		static int count = 0;
		if(count == 0)
		{
			disable_shot;
			count++;
		}
		else
		{
			enable_shot;
			count = 0;
		}
	}

	// Set Servo Range
	if(ps4.button == L1)
	{
		while(ps4.button == L1);
		ServoMovePulse(&pitch, 50);
	}

	if(ps4.button == R1)
	{
		while(ps4.button == R1);
		ServoMovePulse(&pitch, -50);
	}

	if(ps4.button == UP)
	{
		while(ps4.button == UP);
		ServoMovePulse(&pick, 50);
	}

	if(ps4.button == DOWN)
	{
		while(ps4.button == DOWN);
		ServoMovePulse(&pick, -50);
	}

	if(ps4.button == LEFT)
	{
		while(ps4.button == LEFT);
		ServoMovePulse(&load, 50);
	}

	if(ps4.button == RIGHT)
	{
		while(ps4.button == RIGHT);
		ServoMovePulse(&load, -50);
	}

	if(ps4.button == SQUARE)
	{
		while(ps4.button == SQUARE);
		ServoMovePulse(&yaw_servo, 50);
	}

	if(ps4.button == OPTION)
	{
		while(ps4.button == OPTION);
		ServoMovePulse(&yaw_servo, -50);
	}

	if(HAL_GetTick() - before >= InitMode)
	{
		led3 = !led3;
		before = HAL_GetTick();
	}
}

void PS4Tune()
{

	static int run = 0;
	if(ps4.button == CIRCLE)
	{
		while(ps4.button == CIRCLE);
		static int count = 0;
		if(count == 0)
		{
			run = 1;
			count++;
		}
		else
		{
			run = 0;
			count = 0;
		}
	}

	if(run)
	{
		sys.navi_vel = 1;
		VESCPIDProcess(vesc_pulse, vesc_pulse, vesc_pulse, vesc_pulse);
	}
//	else
//		realMODN(&ps4, &rns);

	if(ps4.button == L1)
	{
		while(ps4.button == L1);
		vesc_pulse++;
		if(vesc_pulse >= 5) vesc_pulse = 5;
	}

	if(ps4.button == R1)
	{
		while(ps4.button == R1);
		vesc_pulse--;
		if(vesc_pulse <= 0) vesc_pulse = 0;
	}

	if(ps4.button == UP)
	{
		while(ps4.button == UP);
		tune_p = 1;
		tune_i = 0;
		tune_d = 0;
	}

	if(ps4.button == DOWN)
	{
		while(ps4.button == DOWN);
		tune_p = 0;
		tune_i = 1;
		tune_d = 0;
	}

	if(ps4.button == LEFT)
	{
		while(ps4.button == LEFT);
		tune_p = 0;
		tune_i = 0;
		tune_d = 1;
	}

	if(ps4.button == RIGHT)
	{
		while(ps4.button == RIGHT);
		wheel++;
		if(wheel > 5)
			wheel = 1;
	}

	if(tune_p)
	{
		switch(wheel)
		{
		case 1:
			AP += ps4.joyR_y * 0.0001;
			break;
		case 2:
			BP += ps4.joyR_y * 0.0001;
			break;
		case 3:
			CP += ps4.joyR_y * 0.0001;
			break;
		case 4:
			DP += ps4.joyR_y * 0.0001;
			break;
		}
	}
	else if(tune_i)
	{
		switch(wheel)
		{
		case 1:
			AI += ps4.joyR_y * 0.0001;
			break;
		case 2:
			BI += ps4.joyR_y * 0.0001;
			break;
		case 3:
			CI += ps4.joyR_y * 0.0001;
			break;
		case 4:
			DI += ps4.joyR_y * 0.0001;
			break;
		}
	}
	else
	{
		switch(wheel)
		{
		case 1:
			AD += ps4.joyR_y * 0.00001;
			break;
		case 2:
			BD += ps4.joyR_y * 0.00001;
			break;
		case 3:
			CD += ps4.joyR_y * 0.00001;
			break;
		case 4:
			DD += ps4.joyR_y * 0.00001;
			break;
		}
	}

	if(ps4.button == OPTION)
	{
		while(ps4.button == OPTION);
		switch(wheel)
		{
		case 2:
			fFRightVG[3] = AP;
			fFRightVG[4] = AI;
			fFRightVG[5] = AD;
			PIDGainSet(KP, AP, &fright_vel);
			PIDGainSet(KI, AI, &fright_vel);
			PIDGainSet(KD, AD, &fright_vel);
			break;

		case 1:
			fFLeftVG[3] = BP;
			fFLeftVG[4] = BI;
			fFLeftVG[5] = BD;
			PIDGainSet(KP, BP, &fleft_vel);
			PIDGainSet(KI, BI, &fleft_vel);
			PIDGainSet(KD, BD, &fleft_vel);
			break;

		case 3:
			fBLeftVG[3] = CP;
			fBLeftVG[4] = CI;
			fBLeftVG[5] = CD;
			PIDGainSet(KP, CP, &bleft_vel);
			PIDGainSet(KI, CI, &bleft_vel);
			PIDGainSet(KD, CD, &bleft_vel);
			break;

		case 4:
			fBRightVG[3] = DP;
			fBRightVG[4] = DI;
			fBRightVG[5] = DD;
			PIDGainSet(KP, DP, &bright_vel);
			PIDGainSet(KI, DI, &bright_vel);
			PIDGainSet(KD, DD, &bright_vel);
			break;
		}
	}

	if(HAL_GetTick() - before >= TunePIDMode)
	{
		led3 = !led3;
		before = HAL_GetTick();
	}
}

void RNS_config(CAN_HandleTypeDef* hcanx) {
	RNSInit(hcanx, &rns);
	//0b11100101
	//Encoder dcba(0-swap, 1-keep)  BDC dcba(0-keep, 1-swap) //0x00 0x00 0x
	RNSSet(&rns, RNS_DEVICE_CONFIG, (float) 0b00000001, (float) fwd_omni, (float) roboconPID);
	RNSSet(&rns, RNS_X_Y_ENC_CONFIG, 0.05 / 4000.0 * 3.142, 2.0,
			0.05 / 4000.0 * 3.142, 1.0); //1.0 for nonswap , 2.0 for swap
//	RNSSet(&rns, RNS_F_KCD_PTD, 2442.0/ 2442.0,//remember to put floating point 5175/5174 wrong
//			(float)(0.15 * 3.142 / 244.0));
//	RNSSet(&rns, RNS_B_KCD_PTD, 2440.0/ 2445.0,
//			0.15 * 3.142 / 244.0);
	RNSSet(&rns, RNS_F_KCD_PTD, 5700.0/ 5695.0,
			0.125 * 3.142 / 570.0);
	RNSSet(&rns, RNS_B_KCD_PTD, 5665.0/ 5683.0,
			0.125 * 3.142 / 566.5);

	RNSSet(&rns, RNS_F_LEFT_VEL_SATEU, 1.0, 1.0 / 16.79, 19999.0);
	RNSSet(&rns, RNS_F_RIGHT_VEL_SATEU, 1.0, 1.0 / 17.27 , 19999.0);
	RNSSet(&rns, RNS_B_LEFT_VEL_SATEU, 1.0, 1.0 / 17.37, 19999.0);
	RNSSet(&rns, RNS_B_RIGHT_VEL_SATEU, 1.0, 1.0 / 17.58, 19999.0);

	RNSSet(&rns, RNS_F_LEFT_VEL_PID,  AP, AI, AD);
	RNSSet(&rns, RNS_F_RIGHT_VEL_PID, BP, BI, BD);
	RNSSet(&rns, RNS_B_LEFT_VEL_PID,  CP, CI, CD);
	RNSSet(&rns, RNS_B_RIGHT_VEL_PID, DP, DI, DD);

//	RNSSet(&rns, RNS_F_LEFT_VEL_PID,  1, 1.5, 0);
//	RNSSet(&rns, RNS_F_RIGHT_VEL_PID, 1, 1.5, 0);
//	RNSSet(&rns, RNS_B_LEFT_VEL_PID,  1, 1.5, 0);
//	RNSSet(&rns, RNS_B_RIGHT_VEL_PID, 1, 1.5, 0);

	RNSSet(&rns, RNS_F_LEFT_VEL_FUZZY_PID_BASE, 0.2, 0.2, 0.2);
	RNSSet(&rns, RNS_F_LEFT_VEL_FUZZY_PID_PARAM, 0.02, 0.02, 0.02);

	RNSSet(&rns, RNS_PPInit); //Path Planning
	RNSSet(&rns, RNS_PPPathPID, 1.0, 0.5, 0.5);
	RNSSet(&rns, RNS_PPEndPID, 0.5, 0.1, 0.7);
	RNSSet(&rns, RNS_PPZPID, 0.5, 0.05, 0.2, 5.5);
	RNSSet(&rns, RNS_PPSetCRV_PTS, 10.0);         // Change No. of Points in the Curved Path
}

void manual_mode(void){

	while(sys.manual){

		if(ps4.button == R1){
			setSpeedMODN(4.0);
		}else if(ps4.button == L1){
			setSpeedMODN(2.0);
		}else{
			setSpeedMODN(3.0);
		}
		realMODN(&ps4, &rns);
	}
}

void enq (void){

	//	RNSEnquire(RNS_VEL_BOTH, &rns);
	//	a = rns.enq.enq_buffer[0].data;
	//	b = rns.enq.enq_buffer[1].data;
	//	c = rns.enq.enq_buffer[2].data;
	//	d = rns.enq.enq_buffer[3].data;


//		RNSEnquire(RNS_POS_BOTH, &rns);
//		a = rns.enq.enq_buffer[0].data;
//		b = rns.enq.enq_buffer[1].data;
//		c = rns.enq.enq_buffer[2].data;
//		d = rns.enq.enq_buffer[3].data;

	RNSEnquire(RNS_COORDINATE_X_Y_Z_Zrad, &rns);
	xpos = rns.enq.enq_buffer[0].data;
	ypos = rns.enq.enq_buffer[1].data;
	z = rns.enq.enq_buffer[2].data;

//	sprintf((char*)debug, "X: %.2f Y: %.2f W: %.2f\n", xpos, ypos, z);
//	HAL_UART_Transmit(&huart5, debug, strlen((char*)debug), HAL_MAX_DELAY);

	//	RNSEnquire(RNS_X_Y_POS, &rns);
	//	xpos = rns.enq.enq_buffer[0].data;
	//	ypos = rns.enq.enq_buffer[1].data;
}

void Await(uint32_t ticks){
	counter = 0;
	while(counter < ticks);
}


Tune_Int_t lcdIntList[NUM_INT_UPDATE]={
		//Example: float a = 1.0;
//		varI(ps4.button, 0, 0),
};

Tune_Float_t lcdFloatList[NUM_FLOAT_UPDATE]={
//		varF(ps4.joyR_x, 0, 0),
};


void ILI9341_Init_List(void){
	ILI9341row = 5;

	for(int i=0; i<NUM_INT_UPDATE; i++){
		ILI9341_Draw_String(5, ILI9341row, WHITE, BLACK, lcdIntList[i].varName, 2);
		ILI9341row += 15;
	}

	for(int i=0; i<NUM_FLOAT_UPDATE; i++){
		ILI9341_Draw_String(5, ILI9341row, WHITE, BLACK, lcdFloatList[i].varName, 2);
		ILI9341row += 15;
	}
}

void ILI9341_Update_List(void){
	//Don't update freqeuntly as this function eat around 30ms even with spi dma transmit
	int updateRow=5;
	char lcdData[100];
	for(int i=0; i<NUM_INT_UPDATE; i++){
		sprintf(lcdData, "%8d", *(lcdIntList[i].ptr));
		ILI9341_Draw_String(110, updateRow, WHITE, BLACK, lcdData, 2);
		updateRow += 15;
	}

	for(int i=0; i<NUM_FLOAT_UPDATE; i++){
		sprintf(lcdData, "%8.3f", *(lcdFloatList[i].ptr));
		ILI9341_Draw_String(110, updateRow, WHITE, BLACK, lcdData, 2);
		updateRow += 15;
	}
}

void registerPath(float path[][7], int numPoint, ...){
	va_list pinconfig;
	va_start(pinconfig, numPoint);
	float* point;
	for(int i=0; i<numPoint; i++){
		point = va_arg(pinconfig, float *);
		for(int j=0; j<7; j++){
			path[i][j] = point[j];
		}
	}
	va_end(pinconfig);
}

void esp32Init(UART_HandleTypeDef* huart){
	esp32.huart = huart;
	HAL_UART_Receive_IT(huart, esp32.buffer, 1);
}

void esp32StartSend(void){
	HAL_UART_Transmit(esp32.huart, (uint8_t *)'c', 1, 1);
}

void esp32StopSend(void){
	HAL_UART_Transmit(esp32.huart, (uint8_t *)'s', 1, 1);
}

void esp32Handler(void){
	static uint8_t state=0;
	switch(state){
	case 0:
		if(esp32.buffer[0] == 0x03){
			state = 1;
			HAL_UART_Receive_IT(esp32.huart, esp32.buffer, 10);
		}else{
			HAL_UART_Receive_IT(esp32.huart, esp32.buffer, 1);
		}
		break;
	case 1:
		if(esp32.buffer[9] == 0x24){
			esp32.lagoriHeight = *((uint16_t *)&esp32.buffer[0]);
			esp32.lagoriWidth  = *((uint16_t *)&esp32.buffer[2]);
			esp32.lagoriArea   = *((uint32_t *)&esp32.buffer[4]);
			esp32.lagoriColor  = esp32.buffer[8];
		}
		state = 0;
		HAL_UART_Receive_IT(esp32.huart, esp32.buffer, 1);
		break;
	}
}
