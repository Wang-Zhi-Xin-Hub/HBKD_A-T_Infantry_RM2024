/*!
 * @file     IMU.h
 * @brief    陀螺仪模块头文件
 */

#ifndef __IMU_H_
#define __IMU_H_

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || \
	defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || \
	defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F410Tx) || defined(STM32F410Cx) || \
	defined(STM32F410Rx) || defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F469xx) || \
	defined(STM32F479xx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F412Rx) || \
	defined(STM32F412Vx) || defined(STM32F413xx) || defined(STM32F423xx)
#include <stm32f4xx.h>
#elif defined(STM32F030x6) || defined(STM32F030x8) || defined(STM32F031x6) || defined(STM32F038xx) || \
	defined(STM32F042x6) || defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) ||   \
	defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) ||   \
	defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx) || defined(STM32F030xC)
#include <stm32f0xx.h>
#elif defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101x6) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102x6) || defined(STM32F102xB) ||   \
	defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) ||   \
	defined(STM32F105xC) || defined(STM32F107xC)
#include <stm32f1xx.h>
#elif defined(STM32F301x8) || defined(STM32F302x8) || defined(STM32F302xC) || defined(STM32F302xE) || \
	defined(STM32F303x8) || defined(STM32F303xC) || defined(STM32F303xE) || defined(STM32F373xC) ||   \
	defined(STM32F334x8) || defined(STM32F318xx) || defined(STM32F328xx) || defined(STM32F358xx) ||   \
	defined(STM32F378xx) || defined(STM32F398xx)
#include <stm32f3xx.h>
#elif defined(STM32H743xx) || defined(STM32H753xx) || defined(STM32H750xx) || defined(STM32H742xx) || \
	defined(STM32H745xx) || defined(STM32H755xx) || defined(STM32H747xx) || defined(STM32H757xx) ||   \
	defined(STM32H7B0xx) || defined(STM32H7B0xxQ) || defined(STM32H7A3xx) || defined(STM32H7B3xx) ||  \
	defined(STM32H7A3xxQ) || defined(STM32H7B3xxQ) || defined(STM32H735xx) || defined(STM32H733xx) || \
	defined(STM32H730xx) || defined(STM32H730xxQ) || defined(STM32H725xx) || defined(STM32H723xx)
#include <stm32h7xx.h>
#endif

/** @brief 对应陀螺仪开启的功能 */
#define Time_EN 0					//!< @brief 时间
#define Acceleration_EN 0			//!< @brief 加速度
#define AngularVelocity_EN 1		//!< @brief 角速度
#define EulerAngle_EN 1				//!< @brief 欧拉角
#define MagneticFieldIntensity_EN 0 //!< @brief 磁场强度
#define Pressure_Height_EN 0		//!< @brief 气压、高度
#define Quaternions_EN 1			//!< @brief 四元数

#define IMU_LEN     ( Time_EN + Acceleration_EN + AngularVelocity_EN + EulerAngle_EN + MagneticFieldIntensity_EN + Pressure_Height_EN + Quaternions_EN ) * 11
/*! @brief 陀螺仪数据结构体 */
typedef struct
{

#if Time_EN == 1
	struct
	{
        uint8_t YY;               //!< @brief 年
        uint8_t MM;               //!< @brief 月       
        uint8_t DD;               //!< @brief 日
        uint8_t HH;               //!< @brief 时
        uint8_t MN;               //!< @brief 分
        uint8_t SS;               //!< @brief 秒
		uint8_t MS;               //!< @brief 毫秒
        int64_t TimeStamp;        //!< @brief 时间戳
        
	} Time; //!< @brief 时间
#endif

#if Acceleration_EN == 1
	struct
	{
		int16_t X;
		int16_t Y;
		int16_t Z;
	} Acceleration; //!< @brief 加速度
#endif

#if AngularVelocity_EN == 1
	struct
	{
		float X;
		float Y;
		float Z;
	} AngularVelocity; //!< @brief 角速度
#endif

#if EulerAngle_EN == 1
	struct
	{
		float Pitch;
		float Roll;
		float Yaw;
		int16_t r;
		float LsatAngle;
		float ContinuousYaw;
		float Yawoffset;
	} EulerAngler; //!< @brief 欧拉角
#endif

#if MagneticFieldIntensity_EN == 1
	struct
	{
		int16_t X;
		int16_t Y;
		int16_t Z;
	} MagneticFieldIntensity; //!< @brief 磁场强度
#endif

#if Pressure_Height_EN == 1
	struct
	{
		int16_t P;
		int16_t H;
	} Pressure_Height; //!< @brief 气压、高度
#endif

#if Quaternions_EN == 1
	struct
	{
		float W;
		float X;
		float Y;
		float Z;
	} Quaternions; //!< @brief 四元数
#endif

} IMU_Typedef;
extern IMU_Typedef IMU;

/** @brief 陀螺仪数据包包头 */
typedef enum
{
	kItemTime = 0x50,					//!< @brief 时间数据包头
	kItemAcceleration = 0x51,			//!< @brief 加速度数据包头
	kItemAngularVelocity = 0x52,		//!< @brief 角速度数据包头
	kItemEulerAngler = 0x53,			//!< @brief 欧拉角数据包头
	kItemMagneticFieldIntensity = 0x54, //!< @brief 磁场强度数据包头
	kItemPressure_Height = 0x56,		//!< @brief 气压、高度磁场强度数据包头
	kItemQuaternions = 0x59,			//!< @brief 四元数磁场强度
} ItemID_t;
/*!
 * @brief       陀螺仪解包函数
 * @param[out]  IMU_Typedef *Dst
 * @param[in]   串口源数据
*/
void IMU_Receive(IMU_Typedef *Dst, const uint8_t *Data);

int64_t standard_to_stamp(IMU_Typedef *Dst);

#endif
