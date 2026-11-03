#include "bsp_fdcan.h"

#include <string.h>

#include "fdcan.h"

#include <stdbool.h>

#include "joint_manager.h"

#include "tim.h"
/**
************************************************************************
* @brief:      	bsp_can_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN 使能
************************************************************************
**/
void bsp_can_init(void)
{
	can_filter_init();
	HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
	fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
	fdcan_filter.FilterID1 = 0x00;                               
	fdcan_filter.FilterID2 = 0x00;

	HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter); 		 				  //接收ID2
	//拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
//	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 1);
//	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0);
}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier=id;
    pTxHeader.IdType=FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;
	
	if (len == 4)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_4;
	if (len == 8)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_8;
	if(len==12)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
	if(len==16)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
	if(len==20)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
	if(len==24)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
	if(len==32)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
	if(len==48)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
	if(len==64)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_64;
	
    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_OFF;
    pTxHeader.FDFormat=FDCAN_CLASSIC_CAN;
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;
 
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data)!=HAL_OK) 
		return 1;//发送
	return 0;	
}
/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t fdcanx_receive(
	hcan_t *hfdcan,
	uint16_t *rec_id,
	uint8_t *buf
)
{
	FDCAN_RxHeaderTypeDef rx_header;

	if (HAL_FDCAN_GetRxMessage(
			hfdcan,
			FDCAN_RX_FIFO0,
			&rx_header,
			buf) != HAL_OK)
	{
		return 0;
	}

	if (rx_header.IdType != FDCAN_STANDARD_ID ||
		rx_header.RxFrameType != FDCAN_DATA_FRAME ||
		rx_header.DataLength != FDCAN_DLC_BYTES_8)
	{
		return 0;
	}

	*rec_id = (uint16_t)rx_header.Identifier;

	return 8;
}

void canx_rx_parse(DM4310_Feedback *motor, const uint8_t *data)
{
	motor->motor_id = data[0] & 0x0F;
	motor->state = (data[0] >> 4) & 0x0F;

	motor->position_raw =
	   ((uint16_t)data[1] << 8) |
	   data[2];

	motor->velocity_raw =
		((uint16_t)data[3] << 4) |
		(data[4] >> 4);

	motor->torque_raw =
		(((uint16_t)data[4] & 0x0F) << 8) |
		data[5];

	motor->mos_temp = data[6];
	motor->rotor_temp = data[7];
}

uint8_t rx_data1[8] = {0};
uint16_t rec_id1;
void fdcan1_rx_callback(void)
{
	uint16_t rx_id;
	uint8_t data[8];
	DM4310_Feedback feedback;


	while (HAL_FDCAN_GetRxFifoFillLevel(
			   &hfdcan1, FDCAN_RX_FIFO0) > 0U)
	{
		if (fdcanx_receive(&hfdcan1, &rx_id, data) != 8U)
		{
			continue;
		}

		const uint32_t rx_us = __HAL_TIM_GET_COUNTER(&htim2);
		uint8_t motor_id;
		float xout_rad;

		if (DM4310_ParseOutputPositionReply(
				rx_id, data, &motor_id, &xout_rad))
		{
			(void)Joint_UpdateOutputPosition(
				motor_id,
				xout_rad,
				rx_us,
				rx_id);
		}
		else
		{
			canx_rx_parse(&feedback, data);

			(void)Joint_UpdateFeedback(
				&feedback,
				HAL_GetTick(),
				rx_id);
		}
	}
}

uint8_t rx_data2[8] = {0};
uint16_t rec_id2;



void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan == &hfdcan1)
	{
		fdcan1_rx_callback();
	}

}

static uint8_t usb_rx_buf[64];
static volatile uint8_t usb_rx_ready = 0;
static uint8_t enable[8] = {
	0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFC
};

static uint8_t disable[8] = {
	0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFD
};


void fdcan_usb_rx(uint8_t*buf,uint32_t len)
{
	if (buf[0] == 0xEE && len == 1)
	{
		fdcanx_send_data(&hfdcan1, 0x101, enable, 8);
		fdcanx_send_data(&hfdcan1, 0x102, enable, 8);
		fdcanx_send_data(&hfdcan1, 0x103, enable, 8);
		fdcanx_send_data(&hfdcan1, 0x104, enable, 8);
	}
	else if (buf[0] == 0xDD && len == 1)
	{
		fdcanx_send_data(&hfdcan1, 0x101, disable, 8);
		fdcanx_send_data(&hfdcan1, 0x102, disable, 8);
		fdcanx_send_data(&hfdcan1, 0x103, disable, 8);
		fdcanx_send_data(&hfdcan1, 0x104, disable, 8);
	}
	else
	{
		fdcanx_send_data(&hfdcan1, 0x7FF, buf, 4);
	}

}

void fdcan_usb_process(void)
{
	if (!usb_rx_ready)
		return;

	usb_rx_ready = 0;

	/* 在这里解析 usb_rx_buf，长度是 usb_rx_len */
	/* 得到 can_id、can_data、can_len 后： */
	fdcanx_send_data(&hfdcan1, 0x101, usb_rx_buf, 8);
	__NOP();
	fdcanx_send_data(&hfdcan1, 0x102, usb_rx_buf, 8);
	__NOP();
	fdcanx_send_data(&hfdcan1, 0x103, usb_rx_buf, 8);
	__NOP();
	fdcanx_send_data(&hfdcan1, 0x104, usb_rx_buf, 8);
}

bool DM4310_SendPositionVelocity(uint8_t id, float position, float velocity_limit)
{
	if (id <1U || id > 4U)
	{
		return false;
	}

	if (!isfinite(position) || !isfinite(velocity_limit) || velocity_limit <= 0.0f)
	{
		return false;
	}

	uint8_t data[8] = {0};

	memcpy(&data[0], &position, sizeof(float));
	memcpy(&data[4], &velocity_limit, sizeof(float));

	uint16_t can_id = POSITION_AND_SPEED_OFFSET + id;

	return fdcanx_send_data(&hfdcan1, can_id, data, 8) == 0U;
}

bool DM4310_RequestOutputPosition(uint8_t motor_id)
{
	uint8_t data[8] = {motor_id , 0x00U, 0x33U,0x51U};

	if (motor_id < 1U || motor_id > 4U)
	{
		return false;
	}

	return fdcanx_send_data(&hfdcan1, 0x7FFU, data, 4U) == 0U;
}

bool DM4310_ParseOutputPositionReply(uint16_t rx_id, const uint8_t data[8],uint8_t *motor_id, float *xout_rad)
{
	uint16_t queried_motor_id;
	float position;

	if (data == NULL || motor_id == NULL || xout_rad == NULL)
	{
		return false;
	}

	if (data[2] != 0x33U || data[3] != 0x51U)
	{
		return false;
	}

	queried_motor_id = (uint16_t)data[0] | ((uint16_t)data[1] << 8);

	if (queried_motor_id < 1U || queried_motor_id > 4U)
	{
		return false;
	}

	if (rx_id != (uint16_t)(0x50U + queried_motor_id))
	{
		return false;
	}

	memcpy(&position, &data[4], sizeof(float));

	if (!isfinite(position))
	{
		return false;
	}

	*motor_id = (uint8_t)queried_motor_id;
	*xout_rad = position;

	return true;
}










