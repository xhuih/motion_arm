#include "ZDT.h"

uint8_t buf[20];


//date,size,timeout
void send(uint8_t *cmd,uint8_t len)
{
	HAL_UART_Transmit(&huart1,cmd,len,1000);
}




void position_control_zdt(uint8_t add,uint8_t dir,uint16_t vel,uint32_t pos,bool pos_flag,bool bot_flag)
{
	uint8_t cmd[15];
	
	cmd[0] = add;									//电机地址
	cmd[1] = 0xFB;								//功能码
	cmd[2] = dir;									//方向
	cmd[3] = (uint8_t)(vel>>8);		//速度,串口一次只能发八位，所以要分成两个八位
	cmd[4] = (uint8_t)(vel>>0);		//参数输入十进制，计算机会转成二进制的，（uint8_t）会把16位转成两个8位
	cmd[5] = (uint8_t)(pos>>24);	//转动的角度
	cmd[6] = (uint8_t)(pos>>16);	//输入的角度为32位的，要分成四个8位
	cmd[7] = (uint8_t)(pos>>8);		//位置
	cmd[8] = (uint8_t)(pos>>0);		//位置
	cmd[9] = pos_flag;						//相对绝对位置标志
	cmd[10] = bot_flag;						//多机同步标志
	cmd[11] = 0x6B;								//校验位
	
	send(cmd,12);
	
}


void Emm_test(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节 
  cmd[5]  =  acc;                       // 加速度，注意：0是直接启动
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  cmd[10] =  raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] =  snF;                       // 多机同步运动标志，false为不启用，true为启用
  cmd[12] =  0x6B;                      // 校验字节
  
  // 发送命令
  send(cmd, 13);
}
