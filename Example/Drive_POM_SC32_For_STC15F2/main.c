#include <STC15F2K60S2.h>

#include <stdio.h>
#include <string.h>
#include <math.h>

//#define USER_REG_CMD 1
#define USER_HEX_CMD 1

void Uart1_Init(void)	//460800bps@11.0592MHz
{
	SCON = 0x50;		//8位数据,可变波特率
	AUXR |= 0x40;		//定时器时钟1T模式
	AUXR &= 0xFE;		//串口1选择定时器1为波特率发生器
	TMOD &= 0x0F;		//设置定时器模式
	TL1 = 0xFA;			//设置定时初始值
	TH1 = 0xFF;			//设置定时初始值
	ET1 = 0;			//禁止定时器中断
	TR1 = 1;			//定时器1开始计时
}

void Uart_Sendstring(unsigned char *pucStr,unsigned int NUM)
{
  while (NUM--)
  {
    SBUF = *pucStr++;
    while (TI == 0);
    TI = 0;
  }
}


#ifdef USER_REG_CMD
//REG list
#define RESET_REG 0x00
#define SERVO1_REG 0x01
#define SERVO2_REG 0x02
#define SERVO3_REG 0x03
#define SERVO4_REG 0x04
#define SERVO5_REG 0x05
#define SERVO6_REG 0x06
#define SERVO7_REG 0x07
#define SERVO8_REG 0x08
#define SERVO9_REG 0x09
#define SERVO10_REG 0x0A
#define SERVO11_REG 0x0B
#define SERVO12_REG 0x0C
#define SERVO13_REG 0x0D
#define SERVO14_REG 0x0E
#define SERVO15_REG 0x0F
#define SERVO16_REG 0x10
#define SERVO17_REG 0x11
#define SERVO18_REG 0x12
#define SERVO19_REG 0x13
#define SERVO20_REG 0x14
#define SERVO21_REG 0x15
#define SERVO22_REG 0x16
#define SERVO23_REG 0x17
#define SERVO24_REG 0x18
#define SERVO25_REG 0x19
#define SERVO26_REG 0x1A
#define SERVO27_REG 0x1B
#define SERVO28_REG 0x1C
#define SERVO29_REG 0x1D
#define SERVO30_REG 0x1E
#define SERVO31_REG 0x1F
#define SERVO32_REG 0x20
#define RED_REG 0x21
#define GREEN_REG 0x22
#define BLUE_REG 0x23
#define LED_REG 0x24
#define GROUP1_REG 0x25
#define GROUP2_REG 0x26
#define GROUP3_REG 0x27
#define GROUP4_REG 0x28
#define GROUP5_REG 0x29
#define GROUP6_REG 0x2A
#define TEMP1_REG 0x2B
#define TEMP2_REG 0x2C
#define TEMP3_REG 0x2D
#define TEMP4_REG 0x2E
#define TEMP5_REG 0x2F
#define TEMP6_REG 0x30
#define TEMP7_REG 0x31
#define TEMP8_REG 0x32
#define TEMP9_REG 0x33
#define TEMP10_REG 0x34
#define TEMP11_REG 0x35
#define TEMP12_REG 0x36
#define TEMP13_REG 0x37
#define TEMP14_REG 0x38
#define TEMP15_REG 0x39
#define TEMP16_REG 0x3A
#define TEMP17_REG 0x3B
#define TEMP18_REG 0x3C
#define TEMP19_REG 0x3D
#define TEMP20_REG 0x3E
#define TEMP21_REG 0x3F
#define TEMP22_REG 0x40
#define TEMP23_REG 0x41
#define TEMP24_REG 0x42
#define TEMP25_REG 0x43
#define TEMP26_REG 0x44
#define TEMP27_REG 0x45
#define TEMP28_REG 0x46
#define TEMP29_REG 0x47
#define TEMP30_REG 0x48
#define TEMP31_REG 0x49
#define TEMP32_REG 0x4A
#define ACTION_TIME_REG 0x4B
#define ACTION_GROUP_REG 0x4C
#define ACTION_NUM_REG 0x4D
#define FUNCTION_WRITE_REG 0x4E
#define FUNCTION_READ_REG 0x4F
#endif

#ifdef USER_HEX_CMD
void HEX_CMD_FUNC(unsigned int *NUM)
{
    xdata unsigned char TX_BUF[65];
    xdata unsigned char RX_BUF[2] = {0x00,0x00};
    xdata unsigned char Temp_num;
    memset(TX_BUF,0,65);
    for(Temp_num = 0;Temp_num < 32;Temp_num++)
    {
        TX_BUF[Temp_num*2] = NUM[Temp_num] >> 8;
        TX_BUF[Temp_num*2+1] = NUM[Temp_num] &= 0xFF;
        TX_BUF[64] = TX_BUF[64] + TX_BUF[Temp_num*2] +TX_BUF[Temp_num*2+1];
    }
    Uart_Sendstring(TX_BUF,65);
}
#endif


//frame head
#define FRAME_HEAD 0xAA


void REG_CMD_FUNC(unsigned char ADDR,unsigned int NUM)
{
    unsigned char TX_BUF[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
    unsigned char RX_BUF[2] = {0x00,0x00};
    TX_BUF[0] = FRAME_HEAD;
    TX_BUF[1] = 0x00;
    TX_BUF[2] = ADDR;
    TX_BUF[3] = NUM >> 8;
    TX_BUF[4] = NUM &= 0xFF;
    TX_BUF[5] = TX_BUF[0]+TX_BUF[1]+TX_BUF[2]+TX_BUF[3]+TX_BUF[4];
    Uart_Sendstring(TX_BUF,6);
}


int round_Angle(float num)//四舍五入
{
    int sign = 1;
    int fractional;
    int integer;
    if(num < 0)
    {
        sign = -1;
        num = -num;
    }
    num = num*100;
    // 获取小数部分（最后两位）
    fractional = (long int)num % 100;
    // 获取整数部分
    integer = (long int)num / 100;
    
    // 四舍五入
    if(fractional >= 50)
    {
        integer++;
    }
    
    return integer * sign;
}

unsigned char Angle_to_Duty(float Angle)
{
    if(Angle > 180.0)Angle = 180.0;
    return round_Angle((Angle * 5) / 9.0 +25);
}



void Timer0_Init(void)		//1毫秒@11.0592MHz
{
	AUXR |= 0x80;			//定时器时钟1T模式
	TMOD &= 0xF0;			//设置定时器模式
	TL0 = 0xCD;				//设置定时初始值
	TH0 = 0xD4;				//设置定时初始值
	TF0 = 0;				//清除TF0标志
	TR0 = 1;				//定时器0开始计时
	ET0 = 1;				//使能定时器0中断
}
void Delay20ms(void)	//@11.0592MHz
{
	unsigned char data i, j;

	i = 216;
	j = 37;
	do
	{
		while (--j);
	} while (--i);
}


double Temp_Time;
xdata unsigned int servo_buf[32];
unsigned char Temp_num_for_main1;

void main(void)
{
    Uart1_Init();
    Timer0_Init();
    EA = 1;
    REG_CMD_FUNC(0x25,50);
    REG_CMD_FUNC(0x26,50);
    REG_CMD_FUNC(0x27,50);
    REG_CMD_FUNC(0x28,50);
    REG_CMD_FUNC(0x29,50);
    REG_CMD_FUNC(0x2A,50);
    while(1)
    {
        #if USER_REG_CMD
            REG_CMD_FUNC(SERVO1_REG,Angle_to_Duty((sin(Temp_Time/1000.0)+1)*90));
            Delay20ms();
        #endif
        
        #if USER_HEX_CMD
            for(Temp_num_for_main1 = 0 ;Temp_num_for_main1 < 32 ;Temp_num_for_main1++)
            {
              servo_buf[Temp_num_for_main1] = Angle_to_Duty((sin(Temp_Time/1000.0)+1)*90);
            }
            HEX_CMD_FUNC(servo_buf);
            Delay20ms();
        #endif
    }
}
void Timer0_Isr(void) interrupt 1
{
    Temp_Time++;
}