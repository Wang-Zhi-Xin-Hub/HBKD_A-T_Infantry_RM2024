/*!
 * @file     IMU.c
 * @brief    陀螺仪模块
 */
#include "IMU.h"
#include "string.h"
#include "Variate.h"
#include "time.h"

/* IMU数据结构体定义 */
IMU_Typedef IMU;

void IMU_Receive(IMU_Typedef *Dst, const uint8_t *Data)
{
	if (Data[0] == 0x55)
	{
		for (uint16_t i = 1; i < IMU_Usart2_Len; i += 11)
		{
			switch (Data[i])
			{
                
#if Time_EN == 1 // 时间(片上时间，不是当前时区时间，实际用处不大,需校准)
			case kItemTime:
				Dst->Time.YY =  Data[i + 1];
				Dst->Time.MM =  Data[i + 2];
				Dst->Time.DD =  Data[i + 3];
				Dst->Time.HH =  Data[i + 4];
				Dst->Time.MN =  Data[i + 5];
				Dst->Time.SS =  Data[i + 6];
				Dst->Time.MS =  (int16_t)(Data[i + 8] << 8) | Data[i + 7];
//                Dst->Time.TimeStamp = standard_to_stamp(&IMU);
				break;
#endif

#if Quaternions_EN == 1 // 四元数
			case kItemQuaternions:
				Dst->Quaternions.W = ((int16_t)(Data[i + 2] << 8) | Data[i + 1]) / 32768.0f;
				Dst->Quaternions.X = ((int16_t)(Data[i + 4] << 8) | Data[i + 3]) / 32768.0f;
				Dst->Quaternions.Y = ((int16_t)(Data[i + 6] << 8) | Data[i + 5]) / 32768.0f;
				Dst->Quaternions.Z = ((int16_t)(Data[i + 8] << 8) | Data[i + 7]) / 32768.0f;
				break;
#endif

#if Acceleration_EN == 1 // 加速度
			case kItemAcceleration:
				Dst->Acceleration.X = ((int16_t)(Data[i + 2] << 8) | Data[i + 1]) / 32768.0f * 16 * 9.8f;
				Dst->Acceleration.Y = ((int16_t)(Data[i + 4] << 8) | Data[i + 3]) / 32768.0f * 16 * 9.8f;
				Dst->Acceleration.Z = ((int16_t)(Data[i + 6] << 8) | Data[i + 5]) / 32768.0f * 16 * 9.8f;
				break;
#endif

#if AngularVelocity_EN == 1 // 角速度
			case kItemAngularVelocity:
				Dst->AngularVelocity.X = ((int16_t)(Data[i + 2] << 8) | Data[i + 1]) / 32768.0f * 2000;
				Dst->AngularVelocity.Y = ((int16_t)(Data[i + 4] << 8) | Data[i + 3]) / 32768.0f * 2000;
				Dst->AngularVelocity.Z = ((int16_t)(Data[i + 6] << 8) | Data[i + 5]) / 32768.0f * 2000;
				break;
#endif

#if EulerAngle_EN == 1 // 欧拉角
			case kItemEulerAngler:
				Dst->EulerAngler.Roll = ((int16_t)(Data[i + 2] << 8) | Data[i + 1]) / 32768.0f * 180;
				Dst->EulerAngler.Pitch = ((int16_t)(Data[i + 4] << 8) | Data[i + 3]) / 32768.0f * 180;
				Dst->EulerAngler.Yaw = ((int16_t)(Data[i + 6] << 8) | Data[i + 5]) / 32768.0f * 180;

				float diff = Dst->EulerAngler.Yaw - Dst->EulerAngler.LsatAngle;
				if (diff > 100)
					Dst->EulerAngler.r--;
				else if (diff < -100)
					Dst->EulerAngler.r++;

				Dst->EulerAngler.ContinuousYaw = Dst->EulerAngler.r * 360.0f + Dst->EulerAngler.Yaw;
				Dst->EulerAngler.LsatAngle = Dst->EulerAngler.Yaw;
				break;
#endif
			}
		}
	}
}

#if Time_EN == 1
/* 标准时间转换为时间戳 */
int64_t standard_to_stamp(IMU_Typedef *Dst)
{
	struct tm stm;
	stm.tm_year=Dst->Time.YY + 100;
	stm.tm_mon=Dst->Time.MM - 1;
	stm.tm_mday=Dst->Time.DD;
	stm.tm_hour=Dst->Time.HH - 8 ;
	stm.tm_min=Dst->Time.MN;
	stm.tm_sec=Dst->Time.SS;
//  	return (int64_t)mktime(&stm);   //秒时间戳
	return (int64_t)mktime(&stm) * 1000 + Dst->Time.MS; //毫秒时间戳
}
#endif
