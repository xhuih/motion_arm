#include "uart.h"

//指令最小长度
#define COMMAND_MIN_LENGTH 8


//循环缓冲区大小
#define BUFFER_SIZE 128
uint8_t buffer[BUFFER_SIZE];

//缓冲区读写索引index
//write是写进缓冲区，read是从缓冲区读出来
uint8_t read_flag = 0;
uint8_t write_flag = 0;


//缓冲区读索引
void add_readflag(uint16_t length)
{
	read_flag += length;				//发送完一位后读索引增加一个读取长度去读下一位
	read_flag %= BUFFER_SIZE;		//如果读索引增加后超过了缓冲区大小，那就回到起点循环
}


//读取第i位数据
uint8_t command_read(uint8_t i)
{
	uint8_t flag = i %  BUFFER_SIZE;		//当读取的数据位置超过缓冲区就回到起点
	return buffer[flag];
}


//查看缓冲区还有多少数据未处理
uint8_t get_length()
{
	return (write_flag + BUFFER_SIZE - read_flag) % BUFFER_SIZE;
}


//获取还有多少空余空间
uint8_t get_relength(void)
{
	return BUFFER_SIZE - get_length();
}


//向缓冲区写入数据，memcpy(写入的地址，要写的数据，写入的长度)
uint8_t write_proce(uint8_t *data,uint8_t length)
{
	//如果长度不足就返回0
	if(get_relength()<length) return 0;
	//使用memcpy函数将数据写入缓冲区
	if(write_flag+length<BUFFER_SIZE)
	{
		memcpy(buffer + write_flag,data,length);
		write_flag += length;
	}
	else
	{
		uint8_t first_length = BUFFER_SIZE - write_flag;
		memcpy(buffer + write_flag,data,first_length);
		memcpy(buffer,data + first_length,length - first_length);
		write_flag = length - first_length;		//memcpy?
	}
	return length;
}


//获取一条数据指令
//包头	包长	数据	校验和
uint8_t get_command(uint8_t *command)
{
	while(1)
	{
		//如果缓冲区长度小于COMMAND_MIN_LENGTH 则不可能有完整的指令
		if(get_length()<COMMAND_MIN_LENGTH) return 0;
		
		//如果不是包头，则跳过，重新开始找
		if(command_read(read_flag)!=0xAA)
		{
			add_readflag(1);
			continue;
		}
		
		//如果缓冲区长度小于指令长度则不可能有完整指令
		uint8_t length = 8;//command_read(read_flag+1);		//数据帧第二位是包长
		if(get_length()<length) return 0;
		
		//如果校验和不对则跳过
		uint8_t sum = 0;
		for(uint8_t i=0;i<length-1;i++) sum += command_read(read_flag+i);
		if((sum&0x00FF)!=command_read(read_flag+length-1))//这里是校验和的低八位
		{
			add_readflag(1);
			continue;
		}
		
		//如果找到完整指令则将指令写入command返回指令长度
		for(uint8_t i=0;i<length;i++) command[i] = command_read(read_flag+i);
		add_readflag(length);
		return length;
	}
}
