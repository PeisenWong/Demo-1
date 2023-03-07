#include "obstacle_detect.h"

void LidarInit(UART_HandleTypeDef* lidarUART, MODE_t mode, float lidar_offset_x, float lidar_offset_y,
		float pole_offset_x, float pole_offset_y, Lidar_t* lidar)
{
	lidar->lidar_UART = lidarUART;
	lidar->mode = mode;
	lidar->start = 0;
	lidar->lidar_offset_x = lidar_offset_x;
	lidar->lidar_offset_y = lidar_offset_y;
	lidar->pole_offset_x = pole_offset_x;
	lidar->pole_offset_y = pole_offset_y;

	if(lidar->mode == CONTINUOUS)
	{
		HAL_UART_Receive_IT(lidar->lidar_UART, lidar->obstacle_receive, 5);
		lidar->new = 0;
	}
	else
	{
		lidar->response = NO;
		HAL_UART_Receive_IT(lidar->lidar_UART, lidar->obstacle_receive, 2);
	}
}

// Send Instruction to pi/pc
void LidarSendIns(Instruction_t ins, Lidar_t* lidar)
{
	lidar->inst = ins;
	lidar->obstacle_send[0] = 0x01;
	memcpy(&lidar->obstacle_send[1], &lidar->inst, 1);

	HAL_UART_Transmit(lidar->lidar_UART, lidar->obstacle_send, 2, HAL_MAX_DELAY);
	HAL_UART_Receive_IT(lidar->lidar_UART, lidar->obstacle_receive, 25);
}

void LidarGo(Lidar_t* lidar)
{
	float target_x, target_y;

	if(lidar->response == OK)
	{
		// Need to know shooting which pole

		// Assume shooting front pole
		// Adjust lidar offset
		lidar->pole.x -= lidar->lidar_offset_x;
		lidar->pole.y -= lidar->lidar_offset_y;

		// Adjust pole offset (Shot 45 deg or straight?)
		// Assume shooting straight
		target_x = pp.real_x + lidar->pole.x + lidar->pole_offset_x;
		target_y = pp.real_y + lidar->pole.y + lidar->pole_offset_y;

		float point[1][7] = {{1.0, target_x, target_y, 0.0, 0.0, 0.0, 0.0}};
		PP_start(point, 1, &pp);
	}
}

// Call this in UART Callback
void ObstacleHandler(Lidar_t* lidar)
{
	if(lidar->mode == CONTINUOUS)
	{
		static int mode = 0;
		switch(mode)
		{
			case 0:
				if(lidar->obstacle_receive[0] == 0x01)
				{
					memcpy(&lidar->obstacle_count, &lidar->obstacle_receive[1], 4);
					if(lidar->new == 0)
					{
							lidar->Polelist = (Pole*)malloc(lidar->obstacle_count * 3 * sizeof(float));
					}
					else
					{
							lidar->Polelist = (Pole*)realloc(lidar->Polelist, lidar->obstacle_count * 3 * sizeof(float));
					}

					HAL_UART_Transmit(lidar->lidar_UART, (uint8_t*)&lidar->obstacle_count, 4, HAL_MAX_DELAY);
					HAL_UART_Receive_IT(lidar->lidar_UART, lidar->obstacle_receive, lidar->obstacle_count * 3 * 4);
					mode = 1;
					lidar->new = 1;
				}
				break;

			case 1:

				for(int i = 0; i < lidar->obstacle_count; i++)
				{
					memcpy(&lidar->pole.x, &lidar->obstacle_receive[i * 12], 4);
					memcpy(&lidar->pole.y, &lidar->obstacle_receive[i * 12 + 4], 4);
					memcpy(&lidar->pole.distance, &lidar->obstacle_receive[i * 12 + 8], 4);

					lidar->pole.angle = atanf(lidar->pole.y / lidar->pole.x);
					lidar->Polelist[i] = lidar->pole;
				}

				// For checking purpose
				for(int i = 0; i < lidar->obstacle_count; i++)
				{
					HAL_UART_Transmit(lidar->lidar_UART, (uint8_t*)&lidar->Polelist[i].x, 4, HAL_MAX_DELAY);
					HAL_UART_Transmit(lidar->lidar_UART, (uint8_t*)&lidar->Polelist[i].y, 4, HAL_MAX_DELAY);
					HAL_UART_Transmit(lidar->lidar_UART, (uint8_t*)&lidar->Polelist[i].distance, 4, HAL_MAX_DELAY);
				}

				mode = 0;
				HAL_UART_Receive_IT(lidar->lidar_UART, lidar->obstacle_receive, 5);
				break;
		}
	}
	else
	{
		if(lidar->start == 0)
		{
			if(lidar->obstacle_receive[0] == 0x01)
			{
				memcpy(&lidar->res, &lidar->obstacle_receive[1], 1);
				lidar->response = lidar->res;

				if(lidar->response == OK)
				{
					HAL_UART_Transmit(lidar->lidar_UART, &lidar->res, 1, HAL_MAX_DELAY);
					lidar->start = 1;
					lidar->response = NO;
					HAL_UART_Receive_IT(lidar->lidar_UART, lidar->obstacle_receive, 25);
				}
			}
		}
		else
		{
			memcpy(&lidar->res, &lidar->obstacle_receive[0], 1);
			lidar->response = lidar->res;

			if(lidar->response == OK)
			{
//				lidar->response == NO;
				// Ready to use this data
				memcpy(&lidar->pole.x, &lidar->obstacle_receive[1], 8);
				memcpy(&lidar->pole.y, &lidar->obstacle_receive[9], 8);
				memcpy(&lidar->pole.distance, &lidar->obstacle_receive[17], 8);

				lidar->pole.angle = atanf(lidar->pole.y / lidar->pole.x);
			}
			else
			{
				lidar->reject = 1;
				// Error Handler here
				LidarSendIns(lidar->inst, lidar);
			}

			// Checking Purpose on pc side
			HAL_UART_Transmit(lidar->lidar_UART, (uint8_t*)&lidar->res, 1, HAL_MAX_DELAY);
			HAL_UART_Transmit(lidar->lidar_UART, (uint8_t*)&lidar->pole.x, 8, HAL_MAX_DELAY);
			HAL_UART_Transmit(lidar->lidar_UART, (uint8_t*)&lidar->pole.y, 8, HAL_MAX_DELAY);
			HAL_UART_Transmit(lidar->lidar_UART, (uint8_t*)&lidar->pole.distance, 8, HAL_MAX_DELAY);

		}
	}
}
