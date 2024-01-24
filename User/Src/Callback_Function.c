#include "Callback_Function.h"

/* CAN1接收回调函数 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    uint16_t CAN1_ID = CAN_Receive_DataFrame(&hcan1, CAN1_buff);
    switch (CAN1_ID)
    {
        case 0x201:M2006_Receive(&Pluck_Motor, CAN1_buff);Feed_Dog(&Pluck_Dog);break;
        case 0x202:RM3508_Receive(&Shoot_Motor[LEFT], CAN1_buff);Feed_Dog(&Shoot_Dog[LEFT]);break;   
        case 0x203:RM3508_Receive(&Shoot_Motor[RIGHT], CAN1_buff);Feed_Dog(&Shoot_Dog[RIGHT]);break;   
        case 0x101:memcpy(&Referee_data_Rx, CAN1_buff, sizeof(Referee_data_Rx));Feed_Dog(&Down_Dog);break;
        default:break;
    }
  }
}

/* CAN2接收回调函数 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN2)
  {
    uint16_t CAN2_ID = CAN_Receive_DataFrame(&hcan2, CAN2_buff);
    switch (CAN2_ID)
    {
        case 0x205:GM6020_Receive(&Gimbal_Motor[PITCH], CAN2_buff);Feed_Dog(&Gimbal_Dog[PITCH]);break;
        case 0x206:GM6020_Receive(&Gimbal_Motor[YAW], CAN2_buff);Feed_Dog(&Gimbal_Dog[YAW]);break;
        default:break;
    }
  }
}

/* 喂狗回调函数 */
void FeedDog_CallBack(WatchDogp handle)
{
  /*
  ID
  1    WatchDog_Init(&Remote_Dog,30);
  2    WatchDog_Init(&IMU_Dog,15);
  3    WatchDog_Init(&Gimbal_Dog[YAW],10);
  4    WatchDog_Init(&Gimbal_Dog[PITCH],10);
  5    WatchDog_Init(&Shoot_Dog[Left],10);
  6    WatchDog_Init(&Shoot_Dog[Right],10);
  7    WatchDog_Init(&Pluck_Dog,10);
  8    WatchDog_Init(&Down_Dog,15);
  9    WatchDog_Init(&PC_Dog,50);
  */
  switch (handle->ID)
  {

      case 1:
            if (REMOTE_IfDataError() == osError)
            {
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
              Remote_State = Device_Error;
            }
            else
            {
              Remote_State = Device_Online;
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
            }
        break;

      case 2:
            if (IMU_IfDataError() == osError)
            {
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
              IMU_State = Device_Error;
            }
            else
            {
              IMU_State = Device_Online;
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
            }
        break;

      case 3:
            if (GM6020_Motor_Temp(&Gimbal_Motor[YAW]) == osError)
            {
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
              Gimbal_State[YAW] = Device_Error;
            }
            else
            {
              Gimbal_State[YAW] = Device_Online;
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
            }
        break;

      case 4:
            if (GM6020_Motor_Temp(&Gimbal_Motor[PITCH]) == osError)
            {
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
              Gimbal_State[PITCH] = Device_Error;
            }
            else
            {
              Gimbal_State[PITCH] = Device_Online;
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
            }
        break;

      case 5:
           if (RM3508_Motor_Temp(&Shoot_Motor[LEFT]) == osError)
            {
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
              Shoot_State[LEFT] = Device_Error;
            }
            else
            {
              Shoot_State[LEFT] = Device_Online;
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
            }
        break;

      case 6:
            if (RM3508_Motor_Temp(&Shoot_Motor[RIGHT]) == osError)
            {
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
              Shoot_State[RIGHT] = Device_Error;
            }
            else
            {
              Shoot_State[RIGHT] = Device_Online;
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
            }
        break;

      case 7:
          Pluck_State = Device_Online;
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
        break;

      case 8:
            Down_State = Device_Online;
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        break;

      case 9:
            PC_State = Device_Online;
        break;
  }
}

/* 饿死回调函数 */
void WatchDog_CallBack(WatchDogp handle)
{
  /*
  ID
  1    WatchDog_Init(&Remote_Dog,30)
  2    WatchDog_Init(&IMU_Dog,15);
  3    WatchDog_Init(&Gimbal_Dog[YAW],10);
  4    WatchDog_Init(&Gimbal_Dog[PITCH],10);
  5    WatchDog_Init(&Shoot_Dog[Left],10);
  6    WatchDog_Init(&Shoot_Dog[Right],10);
  7    WatchDog_Init(&Pluck_Dog,10);
  8    WatchDog_Init(&Down_Dog,15);
  9    WatchDog_Init(&PC_Dog,50);
  */
  switch (handle->ID)
  {

  case 1:
        Remote_State = Device_Offline;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    break;

  case 2:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
        IMU_State = Device_Offline;
    break;

  case 3:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
        Gimbal_State[YAW] = Device_Offline;
    break;

  case 4:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
        Gimbal_State[PITCH] = Device_Offline;
    break;

  case 5:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
        Shoot_State[LEFT] = Device_Offline;
    break;

  case 6:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
        Shoot_State[RIGHT] = Device_Offline;
    break;

  case 7:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
        Pluck_State = Device_Offline;
    break;

  case 8:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        Down_State = Device_Offline;
    break;

  case 9:
        PC_State = Device_Offline;
    break;
  }
}
