#ifndef SRC_OBSTACLE_DETECT_H
#define SRC_OBSTACLE_DETECT_H

#include "../adapter.h"

uint8_t obstacle_ack, obstacle_send[200], obstacle_receive[200];
int obstacle_count, new;
float* obstacle;

UART_HandleTypeDef* lidar_UART;

typedef struct
{
	float x;
	float y;
	float distance;
}Pole;

Pole pole;
Pole* Polelist;

void LidarInit(UART_HandleTypeDef* lidarUART);
void ObstacleHandler();

#endif
