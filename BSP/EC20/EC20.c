#include "ec20.h"
#include <stdlib.h>
#include <string.h>
#include "usart.h"	
#include <stdio.h>
#include "delay.h"
#include "cJSON.h"
#include "math.h"

int waterlevel;         // 水位
int lightning = 0;      // 雷电
float windspeed = 0;    // 风速
float winddirection = 0;// 风向
float turbidity = 0;    // 浊度
float ph = 0;           // ph值
int raindrop = 0;       // 降雨强度
float pitch = 0;        // 俯仰角
float roll = 0;         // 横滚角
float yaw = 0;          // 偏航角
double latitude = 0;    // 纬度
double longitude = 0;   // 经度
int distance = 0;       // 距离


static char *strx;
static uint8_t count = 0;  // 定义变量

void EC20_4G_Reset(void)  // 重启模块
{
    printf("AT+CFUN=1,1\r\n");  // 发送AT指令 重启模块 会断电
}

uint8_t EC20_init_check(uint8_t res)
{
	USART2_RX_Flag = 0;
    count = res;  // 读取初始化状态
    uint8_t reset_cnt = 0;  // 定义重启次数
    while (count != 0x0F)
    {    
        switch (count)
        {
            case 0:
                memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
                printf("ATI\r\n");  // 发送AT指令
                delay_ms(300);  // 延时300ms
                strx = strstr((const char *)&USART2_RX_BUF,(const char *)"OK");  // 判断是否有OK
                printf1(">--EC20-->> ATI\r\n");  // 发送AT指令
                printf1("<--EC20--<< \r\n");
                printf1(" %s\r\n",USART2_RX_BUF);
                delay_ms(300);  // 延时300ms
                HAL_Delay(500); // 延时500ms
                USART2_RX_Flag = 0;
                reset_cnt++;
                if(strx != NULL)  // 判断是否有OK
                {
                    count = 1;  // 有OK，进入下一步
                }
                else
                {
                    count = 0;  // 没有OK，重新发送AT指令
                    if(reset_cnt%3 == 1)
                    {
                        printf1(">--print-->> EC20 RESET\r\n");
                        printf("\r\n");
                        EC20_4G_Reset();  // 重启模块
                    }
                    delay_ms(3000);  // 延时3s
                    break;
                }
                // break;
            case 1:
                memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
                printf("ATE0\r\n");  // 发送ATE0指令 关闭回显
                delay_ms(300);  // 延时300ms
                strx = strstr((const char*)USART2_RX_BUF,(const char*)"OK");  // 判断是否有OK
                printf1(">--EC20-->> ATE0\r\n");  // 发送ATE0指令 关闭回显
                delay_ms(300);
                printf1("<--EC20--<< \r\n");
                printf1(" %s\r\n",USART2_RX_BUF);
                HAL_Delay(500); // 延时500ms
                USART2_RX_STA = 0;
                if(strx != NULL)
                {
                    count = 2;
                }
                else
                {
                    count = 1;
                    break;
                }
                // break;
            case 2:
                memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
                printf("AT+CPIN?\r\n");  // 发送AT+CPIN?指令 检测SIM卡是否插入
                delay_ms(300);  // 延时300ms
                strx = strstr((const char*)USART2_RX_BUF,(const char*)"READY");
                printf1(">--EC20-->> AT+CPIN?\r\n");  // 发送AT+CPIN?指令 检测SIM卡是否插入
                delay_ms(300);
                printf1("<--EC20--<< \r\n");
                printf1(" %s\r\n", USART2_RX_BUF);
                HAL_Delay(500); // 延时500ms
                USART2_RX_STA = 0;
                if(strx != NULL)
                {
                    count = 3;
                }
                else
                {
                    count = 2;
                    break;
                }
                // break;
            case 3: 
                memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
                printf("AT+CREG?\r\n");  // 发送AT+CREG?指令 检测网络是否注册
                delay_ms(300);  // 延时300ms
                strx = strstr((const char*)USART2_RX_BUF,(const char*)"+CREG: 0,1");
                printf1(">--EC20-->> AT+CREG?\r\n");  // 发送AT+CREG?指令 检测网络是否注册
                delay_ms(300);
                printf1("<--EC20--<< \r\n");
                printf1(" %s\r\n",USART2_RX_BUF);
                HAL_Delay(500); // 延时500ms
                USART2_RX_STA = 0;
                if(strx != NULL)
                {
                    count = 4;
                }
                else
                {
                    count = 3;
                    break;
                }
                // break;
            case 4:
                memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
                printf("AT+CSQ\r\n"); // 发送AT+CSQ指令 检测信号强度
                delay_ms(300);  // 延时300ms
                strx = strstr((const char*)USART2_RX_BUF,(const char*)"+CSQ:");
                printf1(">--EC20-->> AT+CSQ\r\n"); // 发送AT+CSQ指令 检测信号强度
                delay_ms(300);
                printf1("<--EC20--<< \r\n");
                printf1(" %s\r\n",USART2_RX_BUF);
                HAL_Delay(500); // 延时500ms
                USART2_RX_STA = 0;
                if(strx != NULL)
                {
                    count = 5;
                }
                else
                {
                    count = 4;
                    break;
                }
                // break;
            case 5:
                memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
                printf("AT+CGREG?\r\n");    // 发送AT+CGREG?指令 检测是否注册到GPRS网络
                delay_ms(300);  // 延时300ms
                strx = strstr((const char*)USART2_RX_BUF,(const char*)"+CGREG: 0,1");
                printf1(">--EC20-->> AT+CGREG?\r\n");    // 发送AT+CGREG?指令 检测是否注册到GPRS网络
                delay_ms(300);
                printf1("<--EC20--<< \r\n");
                printf1(" %s\r\n",USART2_RX_BUF);
                HAL_Delay(500); // 延时500ms
                USART2_RX_STA = 0;
                if(strx != NULL)
                {
                    count = 6;
                }
                else
                {
                    count = 5;
                    break;
                }
                // break;
            case 6:
                memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
                printf("AT+CGATT?\r\n");  // 发送AT+CGATT?指令 检测是否附着网络
                delay_ms(300);  // 延时300ms
                strx = strstr((const char*)USART2_RX_BUF,(const char*)"+CGATT: 1");
                printf1(">--EC20-->> AT+CGATT?\r\n");  // 发送AT+CGATT?指令 检测是否附着网络
                delay_ms(300);
                printf1("<--EC20--<< \r\n");
                printf1(" %s\r\n",USART2_RX_BUF);
                HAL_Delay(500); // 延时500ms
                USART2_RX_STA = 0;
                if(strx != NULL)
                {
                    count = 7;
                }
                else
                {
                    count = 6;
                    break;
                }
                // break;
            case 7:
                memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
                printf("AT+QGPS=1\r\n");  // 发送AT+QGPS=1指令 打开GPS
                delay_ms(300);  // 延时300ms
                strx = strstr((const char*)USART2_RX_BUF,(const char*)"OK");
                printf1(">--EC20-->> AT+QGPS=1\r\n");  // 发送AT+QGPS=1指令 打开GPS
                delay_ms(300);
                printf1("<--EC20--<< \r\n");
                printf1(" %s\r\n",USART2_RX_BUF);
                HAL_Delay(500); // 延时500ms
                USART2_RX_STA = 0;
                count = 8;
                // if(strx != NULL)
                // {
                //     count = 8;
                // }
                // else
                // {
                //     count = 7;
                //     break;
                // }
                // break;
            case 8:
                memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
                printf("AT+QMTDISC=0\r\n");  // 发送AT+QMTDISC=0指令 断开MQTT连接
                delay_ms(300);  // 延时300ms
                strx = strstr((const char*)USART2_RX_BUF,(const char*)"OK");
                printf1(">--EC20-->> AT+QMTDISC=0\r\n");  // 发送AT+QMTDISC=0指令 断开MQTT连接
                delay_ms(300);
                printf1("<--EC20--<< \r\n");
                printf1(" %s\r\n",USART2_RX_BUF);
                USART2_RX_STA = 0;
                HAL_Delay(500); // 延时500ms
                if(strx != NULL)
                {
                    count = 9;
                }
                else
                {
                    count = 8;
                    break;
                }
                // break;
            case 9:
                memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
                printf("AT+QIDEACT=1\r\n");  // 发送AT+QIDEACT=1指令 断开GPRS连接
                delay_ms(300);  // 延时300ms
                strx = strstr((const char*)USART2_RX_BUF,(const char*)"OK");
                printf1(">--EC20-->> AT+QIDEACT=1\r\n");  // 发送AT+QIDEACT=1指令 断开GPRS连接
                delay_ms(300);
                printf1("<--EC20--<< \r\n");
                printf1(" %s\r\n",USART2_RX_BUF);
                HAL_Delay(500); // 延时500ms
                USART2_RX_STA = 0;
                if(strx != NULL)
                {
                    count = 10;
                }
                else
                {
                    count = 8;
                    break;
                }
                // break;
            case 10:
                memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
                printf("AT+QIACT=1\r\n");  // 发送AT+QICLOSE=0指令 断开TCP连接
                delay_ms(300);  // 延时300ms
                strx = strstr((const char*)USART2_RX_BUF,(const char*)"OK");
                printf1(">--EC20-->> AT+QIACT=1\r\n");  // 发送AT+QICLOSE=0指令 断开TCP连接
                delay_ms(300);
                printf1("<--EC20--<< \r\n");
                printf1(" %s\r\n",USART2_RX_BUF);
                USART2_RX_STA = 0;
                HAL_Delay(500); // 延时500ms
                if(strx != NULL)
                {
                    count = 11;
                }
                else
                {
                    count = 8;
                    break;
                }
                // break;
            case 11:
                memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
                printf("AT+QMTCFG=\"qmtping\",0,30\r\n");  // 发送AT+QMTCFG='qmtping',0,30指令 设置MQTT心跳时间为30秒
                delay_ms(300);  // 延时300ms
                strx = strstr((const char*)USART2_RX_BUF,(const char*)"OK");
                printf1(">--EC20-->> AT+QMTCFG='qmtping',0,30\r\n");  // 发送AT+QMTCFG='qmtping',0,30指令 设置MQTT心跳时间为30秒
                delay_ms(300);
                printf1("<--EC20--<< \r\n");
                printf1(" %s\r\n",USART2_RX_BUF);
                USART2_RX_STA = 0;
                HAL_Delay(500); // 延时500ms
                if(strx != NULL)
                {
                    count = 12;
                }
                else
                {
                    count = 8;
                    break;
                }
                // break;
            case 12:
                memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
                printf("AT+QMTCFG=\"recv/mode\",0,0,1\r\n");  // 发送AT+QMTCFG='recv/mode',0,0,1指令 设置MQTT接收模式为1
                delay_ms(300);  // 延时300ms
                strx = strstr((const char*)USART2_RX_BUF,(const char*)"OK");
                printf1(">--EC20-->> AT+QMTCFG=\"recv/mode\",0,0,1\r\n");  // 发送AT+QMTCFG='recv/mode',0,0,1指令 设置MQTT接收模式为1
                delay_ms(300);
                printf1("<--EC20--<< \r\n");
                printf1(" %s\r\n",USART2_RX_BUF);
                USART2_RX_STA = 0;
                HAL_Delay(500); // 延时500ms
                if(strx != NULL)
                {
                    count = 13;
                }
                else
                {
                    count = 8;
                    break;
                }
                // break;
            case 13:
                memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
                printf("AT+QMTOPEN=0,\"124.220.4.71\",1883\r\n");  // 发送AT+QMTOPEN=0, "",1883指令 连接MQTT服务器
                delay_ms(300);  // 延时300ms
                printf1(">--EC20-->> AT+QMTOPEN=0,\"124.220.4.71\",1883\r\n");  // 发送AT+QMTOPEN=0, "",1883指令 连接MQTT服务器
                delay_ms(300);
                printf1("<--EC20--<< \r\n");
                strx = strstr((const char*)USART2_RX_BUF,(const char*)"+QMTOPEN: 0,0");
                printf1(" %s\r\n",USART2_RX_BUF);
                HAL_Delay(500); // 延时500ms
                USART2_RX_STA = 0;
                if(strx != NULL)
                {
                    count = 14;
                }
                else
                {
                    count = 8;
                    break;
                }
                // break;
            case 14:
                memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
                printf("AT+QMTCONN=0,\"port\",\"root\",\"123456\"\r\n"); // 发送AT+QMTCONN=0, "",1883指令 连接数据库
                delay_ms(300);  // 延时300ms
                strx = strstr((const char*)USART2_RX_BUF,(const char*)"+QMTCONN: 0,0,0");
                printf1(">--EC20-->> AT+QMTCONN=0,\"port\",\"root\",\"123456\"\r\n"); // 发送AT+QMTCONN=0, "",1883指令 连接数据库
                delay_ms(300);
                printf1("<--EC20--<< \r\n");
                printf1(" %s\r\n",USART2_RX_BUF);
                USART2_RX_STA = 0;
                HAL_Delay(500); // 延时500msHAL_Delay(500);
                if(strx != NULL)
                {
                    count = 15;
                }
                else
                {
                    count = 8;
                    break;
                }
                // break;
            case 15:
                printf1(">--print-->> EC20初始化成功\r\n");
                count = 15;
                return count;
            default:
                printf1(">--print-->> EC20初始化失败\r\n");
                break;
        }
    }
    return count;
}


void Mqtt_Senddata(void)
{
    count = 1;
    /* 构建JSON上报数据 */
    // 创建JSON数据对象
    cJSON *payload_data = cJSON_CreateObject();
    // 定义JSON字符串
    char *payload_message;  
    // 添加设备ID
    cJSON_AddItemToObject(payload_data, "DeviceID", cJSON_CreateString("1001"));
    // 添加温度
    cJSON_AddItemToObject(payload_data, "temperature", cJSON_CreateNumber(temperature));
    // 添加经度
    cJSON_AddItemToObject(payload_data, "lon", cJSON_CreateNumber(longitude));
    // 添加纬度
    cJSON_AddItemToObject(payload_data, "lat", cJSON_CreateNumber(latitude));
    // 添加俯仰角
    cJSON_AddItemToObject(payload_data, "pitch", cJSON_CreateNumber(pitch));
    // 添加横滚角
    cJSON_AddItemToObject(payload_data, "roll", cJSON_CreateNumber(roll));
    // 添加偏航角
    cJSON_AddItemToObject(payload_data, "yaw", cJSON_CreateNumber(yaw));
    // 添加风速
    cJSON_AddItemToObject(payload_data, "WindSpeed", cJSON_CreateNumber(windspeed));


    // 添加湿度
    cJSON_AddItemToObject(payload_data, "humidity", cJSON_CreateNumber(humidity));
    // 添加风向
    cJSON_AddItemToObject(payload_data, "WindDirection", cJSON_CreateNumber(winddirection));
    // 添加距离
    cJSON_AddItemToObject(payload_data, "distance", cJSON_CreateNumber(distance));
    // 添加雨滴
    cJSON_AddItemToObject(payload_data, "RainfallIntensity", cJSON_CreateNumber(raindrop));
    // 添加浊度
    cJSON_AddItemToObject(payload_data, "turbidity", cJSON_CreateNumber(turbidity));
    // 添加PH
    cJSON_AddItemToObject(payload_data, "PH", cJSON_CreateNumber(ph));
    // 添加水位
    cJSON_AddItemToObject(payload_data, "WaterLevel", cJSON_CreateNumber(waterlevel));

	cJSON *temperature_item = cJSON_GetObjectItem(payload_data, "temperature"); // 获取温度项
	if (temperature_item != NULL) { // 判断该项是否存在
		cJSON_SetNumberValue(temperature_item, temperature); // 更新温度值
	}
	cJSON *longitude_item = cJSON_GetObjectItem(payload_data, "lon"); // 获取经度项
	if (longitude_item != NULL) { // 判断该项是否存在
		cJSON_SetNumberValue(longitude_item, longitude); // 更新经度值
	}
	cJSON *latitude_item = cJSON_GetObjectItem(payload_data, "lat"); // 获取纬度项
	if (latitude_item != NULL) { // 判断该项是否存在
		cJSON_SetNumberValue(latitude_item, latitude); // 更新纬度值
	}
	cJSON *pitch_item = cJSON_GetObjectItem(payload_data, "pitch"); // 获取俯仰角项
	if (pitch_item != NULL) { // 判断该项是否存在
		cJSON_SetNumberValue(pitch_item, pitch); // 更新俯仰角值
	}
	cJSON *roll_item = cJSON_GetObjectItem(payload_data, "roll"); // 获取横滚角项
	if (roll_item != NULL) { // 判断该项是否存在
		cJSON_SetNumberValue(roll_item, roll); // 更新横滚角值
	}
	cJSON *yaw_item = cJSON_GetObjectItem(payload_data, "yaw"); // 获取偏航角项
	if (yaw_item != NULL) { // 判断该项是否存在
		cJSON_SetNumberValue(yaw_item, yaw); // 更新偏航角值
	}
	cJSON *windspeed_item = cJSON_GetObjectItem(payload_data, "WindSpeed"); // 获取风速项
	if (windspeed_item != NULL) { // 判断该项是否存在
		cJSON_SetNumberValue(windspeed_item, windspeed); // 更新风速值
	}cJSON *humidity_item = cJSON_GetObjectItem(payload_data, "humidity"); // 获取湿度项
	if (humidity_item != NULL) { // 判断该项是否存在
		cJSON_SetNumberValue(humidity_item, humidity); // 更新湿度值
	}
	cJSON *winddirection_item = cJSON_GetObjectItem(payload_data, "WindDirection"); // 获取风向项
	if (winddirection_item != NULL) { // 判断该项是否存在
		cJSON_SetNumberValue(winddirection_item, winddirection); // 更新风向值
	}
	cJSON *distance_item = cJSON_GetObjectItem(payload_data, "distance"); // 获取距离项
	if (distance_item != NULL) { // 判断该项是否存在
		cJSON_SetNumberValue(distance_item, distance); // 更新距离值
	}
	cJSON *raindrop_item = cJSON_GetObjectItem(payload_data, "RainfallIntensity"); // 获取雨滴项
	if (raindrop_item != NULL) { // 判断该项是否存在
		cJSON_SetNumberValue(raindrop_item, raindrop); // 更新雨滴值
	}
	cJSON *ph_item = cJSON_GetObjectItem(payload_data, "PH"); // 获取PH项
	if (ph_item != NULL) { // 判断该项是否存在
		cJSON_SetNumberValue(ph_item, ph); // 更新PH值
	}
	cJSON *turbidity_item = cJSON_GetObjectItem(payload_data, "turbidity"); // 获取浊度项
	if (turbidity_item != NULL) { // 判断该项是否存在
		cJSON_SetNumberValue(turbidity_item, turbidity); // 更新浊度值
	}
	cJSON *waterlevel_item = cJSON_GetObjectItem(payload_data, "WaterLevel"); // 获取水位项
	if (waterlevel_item != NULL) { // 判断该项是否存在
		cJSON_SetNumberValue(waterlevel_item, waterlevel); // 更新水位值
	}
	// 将JSON数据对象转换为字符串
	payload_message = cJSON_PrintUnformatted(payload_data);
    while(count != 0x02)
    {
        switch (count)
        {
        case 1:
            memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
            printf("AT+QMTPUBEX=0,0,0,0,\"port\",%d\r\n",strlen((char *)payload_message));  // 发送AT+QMTPUB=0,0,0,0,'port'指令 发布话题
            delay_ms(300);  // 延时300ms
            printf("%s\r\n",payload_message);  // 发送JSON字符串
            delay_ms(300);      // 延时300ms
            strx = strstr((const char*)USART2_RX_BUF,(const char*)"+QMTPUBEX: 0,0,0");
            printf1(">--EC20-->> AT+QMTPUB=0,0,0,0,\"port\",%d\r\n",strlen((char *)payload_message));  // 发送AT+QMTPUB=0,0,0,0,'port'指令 发布话题
            delay_ms(100);
            printf1(">--JSON-->> %s\r\n",payload_message);
            delay_ms(300);      // 延时300ms
            /********************/
            /*		释放内存	 */
            /********************/
            cJSON_Delete(payload_data);  // 释放JSON数据对象
            free(payload_message);  // 释放JSON字符串
            printf1("<--EC20--<< \r\n");
            printf1(" %s\r\n",USART2_RX_BUF);
            USART2_RX_STA = 0;
            if(strx != NULL)
            {
                count = 2;
            }
            else
            {
                count = 0;
                break;
            }
        case 2:
            printf1(">--print-->> 发布成功\r\n");
            break;
        case 0:
            printf1(">--print-->> 发布失败\r\n");
            printf1(">--print-->> MQTT连接重建\r\n");
            EC20_4G_Reset();  // EC20重启
            delay_ms(3000);
            EC20_init_check(0);  // MQTT连接重建
            break;
        default:
            printf1(">--print-->> 发布失败\r\n");
            printf1(">--print-->> MQTT连接重建\r\n");
            EC20_4G_Reset();  // EC20重启
            delay_ms(3000);
            EC20_init_check(0);  // MQTT连接重建
            break;
        }
    }
}

_GPRMC_Typedef GPRMC;	// 定义GPRMC结构体

// 解析UTC时间
void Parse_UTCTime(void)
{
    int hour, minute, second;
    int decimal;

    // 分离出 HH、MM、SS 和 SS.SS 部分
    sscanf(GPRMC.UTCtime, "%2d%2d%2d.%2d", &hour, &minute, &second, &decimal);

    // 计算时间
    sprintf(GPRMC.Time, "%2d:%2d:%2d.%2d", hour, minute, second, decimal);
}

// 解析经纬度
void Parse_gps_data(void)
{
    int lat_d, lon_d;  //dd分量
    int lat_temp0, lon_temp0, lat_temp1, lon_temp1;  

    // 分离出 ddmm.mmmm 部分
    sscanf(GPRMC.latitude, "%2d%2d.%4d", &lat_d, &lat_temp0, &lat_temp1);
    sscanf(GPRMC.longitude, "%3d%2d.%4d", &lon_d, &lon_temp0, &lon_temp1);

    // 计算
    GPRMC.lat = (double)lat_d + (double)lat_temp0 / 60 + (double)((double)lat_temp1 / 600000);
    GPRMC.lat = floor(GPRMC.lat * 10000000 + 0.5) / 10000000;
    GPRMC.lon = (double)lon_d + (double)lon_temp0 / 60 + (double)((double)lon_temp1 / 600000);
    GPRMC.lon = floor(GPRMC.lon * 10000000 + 0.5) / 10000000;
}


// 解析 $GPRMC 消息
void PARSE_GPRMC(void) 
{
	char *subString; 
	char *subStringNext;  
	char i = 0;
    memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
    sprintf((char *)USART2_RX_BUF, "$GPRMC,123519.12,A,4807.0388,N,12204.2817,W,0.0,0.0,230394,,,A*6A"); // 模拟GPRMC数据
    for (i = 0; i <= 6; i++)				//获取6段数据
    {
        if (i == 0)
        {
            subString = strstr((const char *)USART2_RX_BUF, ","); // 查找逗号
        }
        else
        {
            subString++;		//偏移到逗号后面
            if ((subStringNext = strstr(subString, ",")) != NULL)	//查找下一个逗号
            {
                switch(i)
                {
                    case 1:
                        memcpy(GPRMC.UTCtime, subString, subStringNext - subString); //获取UTC时间
                        break;	
                    case 2:
                        memcpy(GPRMC.GPS_sta, subString, subStringNext - subString); //获取定位状态，A=有效定位，V=无效定位
                        break;	
                    case 3:
                        memcpy(GPRMC.latitude, subString, subStringNext - subString); //获取纬度信息(度分)
                        break;	
                    case 4:
                        memcpy(GPRMC.latitudeHemi, subString, subStringNext - subString); //获取纬度半球(N/S)
                        break;	
                    case 5:
                        memcpy(GPRMC.longitude, subString, subStringNext - subString); //获取经度信息(度分)
                        break;	
                    case 6:
                        memcpy(GPRMC.longitudeHemi, subString, subStringNext - subString); //获取经度半球(E/W)
                        break;	
                    default:
                        break;
                }
                subString = subStringNext;
            }
        }
    }
    // printf1("UTCtime:%s\r\n",GPRMC.UTCtime); // 打印UTC时间
    // printf1("GPS_sta:%s\r\n",GPRMC.GPS_sta); // 打印定位状态
    // printf1("latitude:%s\r\n",GPRMC.latitude); // 打印纬度信息(度分)
    // printf1("latitudeHemi:%s\r\n",GPRMC.latitudeHemi); // 打印纬度半球(N/S)
    // printf1("longitude:%s\r\n",GPRMC.longitude); // 打印经度信息(度分)
    // printf1("longitudeHemi:%s\r\n",GPRMC.longitudeHemi); // 打印经度半球(E/W)
    Parse_UTCTime();  // 解析UTC时间
    Parse_gps_data();  // 解析经纬度
    printf1("Time:%s\r\n",GPRMC.Time); // 打印UTC时间
    printf1("lat:%f\r\n",GPRMC.lat); // 打印纬度信息(度分)
    printf1("lon:%f\r\n",GPRMC.lon); // 打印经度信息(度分)
    latitude = GPRMC.lat;  // 纬度
    longitude = GPRMC.lon;  // 经度
}



// GPS数据解析
void GPS_Analysis(void)
{
    static uint8_t GPS_ENABLE = 0;  // GPS使能标志位

    memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
    printf("AT+QGPSGNMEA=\"RMC\"\r\n");     // 发送AT+QGPSGNMEA="RMC"指令 获取GPS数据
    delay_ms(300);  // 延时300ms 
    strx = strstr((const char*)USART2_RX_BUF,(const char*)"OK");
    printf1(">--EC20-->> AT+QGPSGNMEA=\"RMC\"\r\n");     // 发送AT+QGPSGNMEA="RMC"指令 获取GPS数据
    delay_ms(100);  // 延时100ms
    printf1("<--EC20--<< \r\n");
    printf1(" %s\r\n",USART2_RX_BUF);
    USART2_RX_STA = 0;
    if(strx != NULL)
    {
        GPS_ENABLE = 0;  // GPS使能
         // 如果接收到完整的 $GPRMC 消息
        if (strstr((const char*)USART2_RX_BUF, "$GPRMC") != NULL && strchr((const char*)USART2_RX_BUF, '\n') != NULL) 
        {
            // 解析消息
            PARSE_GPRMC();
        }
    }
    else
    {
        if(ENABLE > 6)
        {
            printf1(">--printf1-->> GPS重启\r\n");   // 重启成功
            memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
            printf("AT+QGPSEND\r\n");     // 发送AT+QGPSEND指令 关闭GPS
            delay_ms(300);  // 延时300ms
            strx = strstr((const char*)USART2_RX_BUF,(const char*)"OK");
            printf1(">--EC20-->> AT+QGPSEND\r\n");     // 发送AT+QGPSEND指令 关闭GPS
            delay_ms(100);  // 延时100ms
            printf1("<--EC20--<< \r\n");
            printf1(" %s\r\n",USART2_RX_BUF);
            USART2_RX_STA = 0;
            memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
            printf("AT+QGPS=0\r\n");     // 发送AT+QGPS=0指令 关闭GPS
            delay_ms(300);  // 延时300ms
            /* GPS重启 */
            memset(USART2_RX_BUF,0,sizeof(USART2_RX_BUF));  // 清空缓存
            printf("AT+QGPS=1\r\n");     // 发送AT+QGPS=1指令 打开GPS
            delay_ms(300);  // 延时300ms
            strx = strstr((const char*)USART2_RX_BUF,(const char*)"OK");
            printf("AT+QGPSCFG=\"gpsnmeatype\",1\r\n");     // 发送AT+QGPSCFG="gpsnmeatype",1指令 设置GPS输出数据格式为GGA
            delay_ms(300);  // 延时300ms
            strx = strstr((const char*)USART2_RX_BUF,(const char*)"OK");
            if (strx != NULL)
            {
                printf1(">--printf1-->> GPS重启成功\r\n");   // 重启成功
            }
            else
            {
                printf1(">--printf1-->> GPS重启失败\r\n");   // 重启失败
            }
        }
        printf1(">--printf1-->> GPS数据正在加载,请稍后...\r\n");   // 重启成功
    }
    GPS_ENABLE++;  // GPS使能标志位自增
}


