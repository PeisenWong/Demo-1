#include "obstacle_detect.h"

void LidarInit(UART_HandleTypeDef* lidarUART)
{
	lidar_UART = lidarUART;
	new = 0;
	HAL_UART_Receive_IT(&huart5, obstacle_receive, 5);
}

void ObstacleHandler()
{
	static int mode = 0;
	switch(mode)
	{
		case 0:
			if(obstacle_receive[0] == 0x01)
			{
				memcpy(&obstacle_count, &obstacle_receive[1], 4);
				if(new == 0)
				{
					obstacle = (float*)malloc(obstacle_count * 3 * sizeof(float));
					Polelist = (Pole*)malloc(obstacle_count * 3 * sizeof(float));
				}
				else
				{
					obstacle = (float*)realloc(obstacle, obstacle_count * 3 * sizeof(float));
					Polelist = (Pole*)realloc(Polelist, obstacle_count * 3 * sizeof(float));
				}

				HAL_UART_Transmit(lidar_UART, (uint8_t*)&obstacle_count, 4, HAL_MAX_DELAY);
				HAL_UART_Receive_IT(lidar_UART, obstacle_receive, obstacle_count * 3 * 4);
				mode = 1;
				new = 1;
			}
			break;

		case 1:
			for(int i = 0; i < obstacle_count * 3; i++)
			{
				memcpy(&obstacle[i], &obstacle_receive[i * 4], 4);
			}

			for(int i = 0; i < obstacle_count; i++)
			{
				pole.x = obstacle[i * 3];
				pole.y = obstacle[i* 3 + 1];
				pole.distance = obstacle[i * 3 + 2];

				Polelist[i] = pole;
			}

			for(int i = 0; i < obstacle_count; i++)
			{
				HAL_UART_Transmit(lidar_UART, (uint8_t*)&Polelist[i].x, 4, HAL_MAX_DELAY);
				HAL_UART_Transmit(lidar_UART, (uint8_t*)&Polelist[i].y, 4, HAL_MAX_DELAY);
				HAL_UART_Transmit(lidar_UART, (uint8_t*)&Polelist[i].distance, 4, HAL_MAX_DELAY);
			}

			mode = 0;
			HAL_UART_Receive_IT(lidar_UART, obstacle_receive, 5);
			break;
	}
}
