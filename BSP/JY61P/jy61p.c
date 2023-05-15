// #include "jy61p.h"
// #include "string.h"

// SAcc 	ACC;
// SAngle 	Angle;
// SGyro 	Gyro;

// float angle_x;
// float angle_y;
// float angle_z;

// float acc_x;
// float acc_y;
// float acc_z;

// float gyro_x;
// float gyro_y;
// float gyro_z;

// void CopeSerial2Data(unsigned char ucData)
// {
// 	static unsigned char ucRxBuffer[250];
// 	static unsigned char ucRxCnt = 0;	
	
// 	ucRxBuffer[ucRxCnt++]=ucData;	// 将收到的数据存入缓冲区中
// 	if(ucRxBuffer[0]!=0x55)         // 数据头不对，则重新开始寻找0x55数据头
// 	{
// 		ucRxCnt=0;
// 		return;
// 	}
// 	if(ucRxCnt<11) 
// 	{
// 		return;                     // 数据不满11个，则返回
// 	}
// 	else
// 	{
// 		switch(ucRxBuffer[1])       // 判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
// 		{                           // memcpy为编译器自带的内存拷贝函数,将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。            
// 			case 0x51:	memcpy(&ACC,&ucRxBuffer[2],8);break;
// 			case 0x52:	memcpy(&Gyro,&ucRxBuffer[2],8);break;
// 			case 0x53:	memcpy(&Angle,&ucRxBuffer[2],8);break;
// 		}
// 		ucRxCnt=0;                  // 清空缓存区
// 	}
// }

// void get_jy61p_data()
// {
// 	// 姿态角
// 	acc_x = (float)ACC.a[0]/32768*16;
// 	acc_y = (float)ACC.a[1]/32768*16;
// 	acc_z = (float)ACC.a[2]/32768*16;
	
// 	// 加速度
// 	gyro_x = (float)Gyro.w[0]/32768*2000;
// 	gyro_y = (float)Gyro.w[1]/32768*2000;
// 	gyro_z = (float)Gyro.w[2]/32768*2000;
	
// 	// 角速度
// 	angle_x = (float)Angle.Angle[0]/32768*180;
// 	angle_y = (float)Angle.Angle[1]/32768*180;
// 	angle_z = (float)Angle.Angle[2]/32768*180;

// }

