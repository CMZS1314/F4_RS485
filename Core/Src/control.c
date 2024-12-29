#include "control.h"
//#include "gpio.h"
//#include "dmaPrintf.h"


uint8_t ctlCmd = CMD_OPEN_CONTROL;
uint8_t motorId = 1;
int64_t ctlValue = 0;
uint32_t commandSendCount = 0;
uint32_t commandUnreceivedCount = 0;

int8_t motorTemperature = 0;
int16_t motorPowerOrTorque = 0;
int16_t motorSpeed = 0;
uint16_t motorEncoder = 0;




uint8_t uart1TxBuffer[200];
uint8_t uart1RxBuffer[200];
uint8_t uart3TxBuffer[200];
uint8_t uart3RxBuffer[200];

uint8_t uart1TxDataSize = 0;
uint8_t uart1RxDataSize = 0;
uint8_t uart3TxDataSize = 0;
uint8_t uart3RxDataSize = 0;
/*********************************************************************************
* @brief	控制运行
* @param	None
* @retval	None
*********************************************************************************/
int c2;
void control_Run(void)
{
	static uint8_t motorState = 0;	// 电机状态
	
	uint8_t keyStatus = 0;
	
	//keyStatus = key_Read();
	
//	motorState = 1;
//	ctlValue = 400;
		if(KEY1_StateRead()==KEY_DOWN)
	  {
				motorState = 1;
	      ctlValue = 400;
		   	c2 =1;
    
  }
	if(KEY2_StateRead()==KEY_DOWN)
  {
			motorState = 2;
			ctlValue = 0;
		  c2 =3;
  }
	
	
	if (keyStatus)
	{
		if (motorState == 0)
		{
			motorState = 1;
			ctlValue = 400;
			c2 =1;
		}
		else if (motorState == 1)
		{
			motorState = 2;
			ctlValue = 0;
			c2 =2;
		}
		else if (motorState == 2)
		{
			motorState = 3;
			ctlValue = -400;
			c2 =3;
		}
		else if (motorState == 3)
		{
			motorState = 0;
			ctlValue = 0;
			c2 =4;
		}
	//	keyStatus = 0;
	}
}

/*********************************************************************************
* @brief	命令运行
* @param	None
* @retval	None
*********************************************************************************/
void command_Run(void)
{
	static uint16_t tick = 0;
	uint8_t receivedFlag = 0;	// 收到回复标志
	
	if (tick < 50)
	{
		if (tick == 0)
		{
			control_Send(ctlCmd, motorId, ctlValue);
			control_Receive();
		}
		else if (tick == 20)
		{
			receivedFlag = control_CheckReceivedData();			
			commandSendCount++;
			if (receivedFlag == 0)
			{
				commandUnreceivedCount++;	// 未收到回复计数
			}		
		}

		tick++;
	}
	else
	{
		tick = 0;
	}
}

/*********************************************************************************
* @brief	状态运行
* @param	None
* @retval	None
*********************************************************************************/
void status_Run(void)
{
	static uint16_t tick = 0;
	
	if (tick < 1000)
	{
		tick++;
		
		if (tick == 500)
		{
		//	LED1_TOGGLE();
			
	//		Uart3DmaPrintf("motorTemperature = %d, motorPowerOrTorque = %d, motorSpeed = %d, motorEncoder = %d\n", motorTemperature, motorPowerOrTorque, motorSpeed, motorEncoder);				
		}
		else if (tick == 1000)
		{
		//	LED1_TOGGLE();
			
			//Uart3DmaPrintf("commandSendCount = %d, unreceivedCount = %d\n", commandSendCount, commandUnreceivedCount);		
		}
	}
	else
	{
		
		tick = 0;
	}
}

/*********************************************************************************
* @brief	发送控制命令
* @param	None
* @retval	None
*********************************************************************************/
int c1;
void control_Send(uint8_t cmd, uint8_t id, int64_t value)
{
	uint8_t dataSize = 0;	// 命令数据长度
	int16_t openCtlData = 0;	// 开环控制数据
	int16_t torqueCtlData = 0;	// 力矩环控制数据
	int16_t speedCtlData = 0;	// 速度环控制数据
	int16_t angleCtlData = 0;	// 位置环控制数据
	
	c1=5;
	if (cmd == CMD_OPEN_CONTROL)
	{
		openCtlData = value;
		dataSize = 2;
		
		control_PackCmd(uart1TxBuffer, cmd, id, dataSize, (uint8_t *)&openCtlData);
		HAL_UART_Transmit_DMA(&huart3, uart1TxBuffer, uart1TxDataSize);
		uart1RxDataSize = LEAST_FRAME_SIZE + 7 + 1;
		c1 =1;
	}
	else if (cmd == CMD_TORQUE_CONTROL)
	{
		torqueCtlData = value;
		dataSize = 2;
		
		control_PackCmd(uart1TxBuffer, cmd, id, dataSize, (uint8_t *)&torqueCtlData);
		HAL_UART_Transmit_DMA(&huart3, uart1TxBuffer, uart1TxDataSize);
		uart1RxDataSize = LEAST_FRAME_SIZE + 7 + 1;
				c1 =2;

	}
	else if (cmd == CMD_SPEED_CONTROL)
	{
		speedCtlData = value;
		dataSize = 4;
		
		control_PackCmd(uart1TxBuffer, cmd, id, dataSize, (uint8_t *)&speedCtlData);
		HAL_UART_Transmit_DMA(&huart3, uart1TxBuffer, uart1TxDataSize);
		uart1RxDataSize = LEAST_FRAME_SIZE + 7 + 1;
		c1 =3;

	}	
	else if (cmd == CMD_ANGLE_CONTROL1)
	{
		angleCtlData = value;
		dataSize = 8;
		
		control_PackCmd(uart1TxBuffer, cmd, id, dataSize, (uint8_t *)&angleCtlData);
		HAL_UART_Transmit_DMA(&huart3, uart1TxBuffer, uart1TxDataSize);	
		uart1RxDataSize = LEAST_FRAME_SIZE + 7 + 1;
		c1 =4;
		
	}
}

/*********************************************************************************
* @brief	接收电机回复数据
* @param	None
* @retval	None
*********************************************************************************/
void control_Receive(void)
{
	HAL_UART_Receive_DMA(&huart3, uart1RxBuffer, uart1RxDataSize);
}

/*********************************************************************************
* @brief	检查电机回复数据
* @param	None
* @retval	None
*********************************************************************************/
uint8_t control_CheckReceivedData(void)
{
	uint8_t receiveSuccess = 0;
	uint8_t temp = 0;
	uint8_t i = 0;

	if (uart1RxBuffer[0] == CMD_HEAD)
	{
		temp = uart1RxBuffer[0] + uart1RxBuffer[1] + uart1RxBuffer[2] + uart1RxBuffer[3];
		if (uart1RxBuffer[4] == temp)
		{
			temp = uart1RxBuffer[5] + uart1RxBuffer[6] + uart1RxBuffer[7] + uart1RxBuffer[8] + uart1RxBuffer[9] + uart1RxBuffer[10] + uart1RxBuffer[11];
			if (uart1RxBuffer[12] == temp)
			{
				motorTemperature = (int8_t)uart1RxBuffer[5];
				motorPowerOrTorque = (int16_t)(uart1RxBuffer[6] + (uart1RxBuffer[7]<<8));
				motorSpeed = (int16_t)(uart1RxBuffer[8] + (uart1RxBuffer[9]<<8));		
				motorEncoder = (int16_t)(uart1RxBuffer[10] + (uart1RxBuffer[11]<<8));			
				receiveSuccess = 1;
			}
		}
	}
	
	for (i=0; i<uart1RxDataSize; i++)
		uart1RxBuffer[i] = 0;
	uart1RxDataSize = 0;
	
	return receiveSuccess;
}

/*********************************************************************************
* @brief	打包发送的数据
* @param	None
* @retval	None
*********************************************************************************/
void control_PackCmd(uint8_t *buffer, uint8_t cmd, uint8_t id, uint8_t size, uint8_t *data)
{
	uint8_t i = 0;

	buffer[0] = CMD_HEAD;
	buffer[1] = cmd;
	buffer[2] = id;
	buffer[3] = size;
	buffer[4] = 0;	// 需要先清0
	for (i=0; i<4; i++)
		buffer[4] += buffer[i];
	
	if (size != 0)
	{
		buffer[LEAST_FRAME_SIZE+size] = 0;	// 需要先清0
		for (i=0; i<size; i++)	// 复制数据并计算校验值
		{
			buffer[LEAST_FRAME_SIZE+i] = data[i];
			buffer[LEAST_FRAME_SIZE+size] += buffer[LEAST_FRAME_SIZE+i];
		}
		uart1TxDataSize = i + LEAST_FRAME_SIZE + 1;	// 需要发送的数据总长度
	}
	else
		uart1TxDataSize = LEAST_FRAME_SIZE ;	// 需要发送的数据总长度
}












