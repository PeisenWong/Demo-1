#ifndef SRC_OBSTACLE_DETECT_H
#define SRC_OBSTACLE_DETECT_H

#include "../adapter.h"

typedef struct
{
	double x;
	double y;
	double distance;
	double angle;
}Pole;

typedef enum
{
	FAR,
	NEAR,
	ONE,
	TWO,
	THREE,
	FOUR,
	FIVE,
	SIX
}Instruction_t; // Give instruction to pi

typedef enum
{
	OK,
	NO
}Response_t; // Check response from pi

typedef enum
{
	CONTINUOUS,
	DISCONTINUOUS
}MODE_t;

typedef struct
{
	uint8_t obstacle_ack, obstacle_send[200], obstacle_receive[200];
	int obstacle_count;
	uint8_t inst, res;

	UART_HandleTypeDef* lidar_UART;
	MODE_t mode;
	Pole pole;
	Pole* Polelist;
	Response_t response;

	// Lidar offset from center of robot and pole(destination) offset
	float lidar_offset_x, lidar_offset_y, pole_offset_x, pole_offset_y;

	union{
		uint16_t flags;
		struct{
			unsigned new		:	1;
			unsigned reject		:	1;
			unsigned start		:	1;
			unsigned bit3		:	1;
			unsigned bit4		:	1;
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
}Lidar_t;

Lidar_t lidar;
void LidarInit(UART_HandleTypeDef* lidarUART, MODE_t mode, float lidar_offset_x, float lidar_offset_y,
		float pole_offset_x, float pole_offset_y, Lidar_t* lidar);
void LidarSendIns(Instruction_t ins, Lidar_t* lidar);
void ObstacleHandler();
void LidarGo(Lidar_t* lidar);

#endif
