//#include "../adapter.h"
#include "math.h"
#include "../BIOS/bios.h"
#include "../PID/PID.h"

typedef struct{

	float *qeix;
	float *qeiy;
	float *yaw;
	float prev_yaw;
	float real_z;
	float prev_real_z;
	float real_z_rad;
	float prev_real_z_rad;
	float setz;
	float yaw_offset;
	float yaw_constant;
	float pos_x;
	float prev_x;
	float real_x;
	float prev_real_x;
	float pos_y;
	float prev_y;
	float real_y;
	float prev_real_y;
	float del_pos_x;
	float del_pos_y;

	float dx;
	float dy;
	float heading;

	float vx;
	float vy;

	float target_x[15];
	float target_y[15];
	float target_z[15];
	float target_accurate[15];
	float target_angle[15];
	float target_angle1[15];
	float target_vel[15];
	float prev_tgt_x;
	float prev_tgt_y;
	float ku_x[15];
	float ku_y[15];
	float kp[2],ki[2],kd[2];

	float prev_target_x;
	float prev_target_y;
	float prev_target_z;

	float error_x;
	float error_y;
	float error_z;

	float outx;
	float outy;
	float outz;

	float rux;
	float ruy;
	float rvx;
	float rvy;

	float u1,u2,u3,u4;
	float v1,v2,v3,v4;

	float tol_xy,tol_z,f_tol_xy,f_tol_z,tol_xy_crv;

	uint8_t base_shape;

	PID_t x;
	PID_t y;
	PID_t z;

	int timcount;
	int point_count;
	int target_point;
	float pp_lck;
	float pp_lck_count;
	float pp_lck_final;
	float pp_lck_cal;
	int   pp_lck_enb;
	float pp_crv_radius[15];
	float pp_crv_x[10];
	float pp_crv_y[10];
	float pp_crv_z[10];
	float pp_crv_const[15];
	float pp_rad_ptx;
	float pp_rad_pty;
	int crnt_crv_pt;
	int pp_crv_calc;
	float pp_orgz_kp;
	int pp_no_crv_pts;

	float fWVel;

	union{
		uint16_t Halfword;
		struct{
			unsigned pp_start	:	1;
			unsigned point_start:	1;
			unsigned final_f	:	1;
			unsigned rotate		:	1;
			unsigned rotate_only:	1; // Avoid outx, outy be Nan when error_xy = 0
			unsigned bit5		:	1;
			unsigned bit6		:	1;
			unsigned bit7		:	1;
			unsigned bit8		:	1;
			unsigned bit9		:	1;
			unsigned bit10		:	1;
			unsigned bit11		:	1;
			unsigned bit12		:	1;
			unsigned bit13		:	1;
			unsigned bit14		:	1;
			unsigned bit15		:	1;
		};
	};

}PathPlan_t;

char uartbuff[100];
PathPlan_t pp;

void PPInit(uint8_t base,float *qeix, float *qeiy, float*imu,PathPlan_t *pp);
void PP_PIDPathSet(float kp, float ki, float kd, PathPlan_t *pp);
void ROS_PP_start(float** point,int no_point,PathPlan_t *pp);
void PP_PIDZSet(float kp, float ki, float kd, float ku, PathPlan_t *pp);
void PP_PIDEndSet(float kp, float ki, float kd, PathPlan_t *pp);
void PP_SetZ (float z,PathPlan_t *pp);
void PP_reset (PathPlan_t *pp);
void PP_start(float point[][7],int no_point,PathPlan_t *pp);
void PP_stop (PathPlan_t *pp);
void PathPlan (PathPlan_t *pp);
void PP_setXY (int x,int y,PathPlan_t *pp);
void PP_SetCrv_Points (int z,PathPlan_t *pp);
