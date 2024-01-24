#include "Referee_offcial.h"

typedef struct {
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
}__attribute__((packed)) PACK_DEAD_t;

ext_game_status_t ext_game_state;                                  //!<@brief 比赛状态数据
ext_game_result_t ext_game_result;                                 //!<@brief 比赛状结果数据
ext_game_robot_HP_t ext_game_robot_HP;                             //!<@brief 比赛机器人血量数据
ext_event_data_t ext_event_data;                                   //!<@brief 场地事件数据
ext_supply_projectile_action_t ext_supply_projectile_action;       //!<@brief 场地补给站动作标识数据，动作改变后发送
ext_referee_warning_t ext_referee_warning;                         //!<@brief 裁判警告数据
ext_dart_remaining_time_t ext_dart_remaining_time;                 //!<@brief 飞镖发射口倒计时
ext_game_robot_status_t ext_game_robot_state;                      //!<@brief 机器人状态数据
ext_power_heat_data_t ext_power_heat_data;                         //!<@brief 实时功率热量数据
ext_game_robot_pos_t ext_game_robot_pos;                           //!<@brief 机器人位置数据
ext_buff_t ext_buff_musk;                                          //!<@brief 机器人增益数据
ext_robot_hurt_t ext_robot_hurt;                                   //!<@brief 伤害状态数据
ext_shoot_data_t ext_shoot_data;                                   //!<@brief 实时射击数据

ext_aerial_robot_energy_t ext_aerial_robot_energy;                 //!<@brief 空中机器人能量状态数据
ext_bullet_remaining_t ext_bullet_remaining;                       //!<@brief 子弹剩余发送数
ext_dart_client_cmd_t ext_dart_client_cmd;                         //!<@brief 飞镖机器人客户端指令书

#if ROBOT_TYPE == 4
float shooter_bili1 = 0, shooter_bili2 = 0;
#endif

uint8_t referee_system_halfbuff[USART2_BUFLEN];
uint8_t referee_system_buff[USART2_BUFLENx2];
//Shooter shooter;

/**
 * 裁判系统状态
 * @param RxMsg
 * @return 数据包长度，0 CRC错误
 */
int referee_system_Rx(uint8_t *RxMsg) {
    PACK_DEAD_t *head = (PACK_DEAD_t *)RxMsg;
    uint16_t CMD_code = *(uint16_t *)(RxMsg + 5);
    uint8_t *Msg = &RxMsg[7];
    
    if (head->data_length > USART2_BUFLEN - 9)
        return 0;
    uint16_t Referee_CRC = *(uint16_t *)(RxMsg + 7 + head->data_length);
    if (Referee_CRC == Verify_CRC16_Check_Sum(RxMsg, head->data_length + 7)) {
        switch (CMD_code) {
#if Match_status_data==REFEREE_SYSTEM_ENABLE //比赛状态
        case CMD_code_Match_status_data:
            memcpy(&ext_game_state, Msg, sizeof(ext_game_status_t));
                break;
#endif

#if Result_data==REFEREE_SYSTEM_ENABLE//比赛结果
        case CMD_code_Result_data:
            memcpy(&ext_game_result, Msg, sizeof(ext_game_result_t));
            break;
#endif

#if Match_robot_HP_data==REFEREE_SYSTEM_ENABLE
        case CMD_code_Match_robot_HP_data:
            memcpy(&ext_game_robot_HP, Msg, sizeof(ext_game_robot_HP_t));
            break;
#endif

#if Site_event_data==REFEREE_SYSTEM_ENABLE//场中事件点数据
        case CMD_code_Site_event_data:
            memcpy(&ext_event_data, Msg, sizeof(ext_event_data_t));
            break;
#endif

#if Site_supply_station_action_identification_data==REFEREE_SYSTEM_ENABLE//补给站动作标识
        case CMD_code_Site_supply_station_action_identification_data:
            memcpy(&ext_supply_projectile_action, Msg, sizeof(ext_supply_projectile_action_t));
            break;
#endif

#if Referee_warning_data==REFEREE_SYSTEM_ENABLE//裁判警告
        case CMD_code_Referee_Warning_data:
            memcpy(&ext_game_result, Msg, sizeof(ext_game_result_t));
            break;
#endif
        
#if Darts_shoot_timeremind_data == REFEREE_SYSTEM_ENABLE//飞镖发射口倒计时
        case CMD_code_Darts_shoot_timeremind_data:
            memcpy(&ext_dart_remaining_time, Msg, sizeof(ext_dart_remaining_time_t));
            break;
#endif
            
#if Robot_state_data==REFEREE_SYSTEM_ENABLE//机器人状态
        case CMD_code_Robot_state_data:
            memcpy(&ext_game_robot_state, Msg, sizeof(ext_game_robot_status_t));
            break;
#endif

#if Real_time_power_and_heat_data == REFEREE_SYSTEM_ENABLE //实时功率&热量
        case CMD_code_Real_time_power_and_heat_data:
            memcpy(&ext_power_heat_data, Msg, sizeof(ext_power_heat_data));
            break;
#endif

#if Robot_position_data==REFEREE_SYSTEM_ENABLE //机器人位置数据
        case CMD_code_Robot_position_data:
            memcpy(&ext_game_robot_pos, Msg, sizeof(ext_game_robot_pos_t));
            break;
#endif

#if Robot_gain_data==REFEREE_SYSTEM_ENABLE //机器人Buff数据
        case CMD_code_Robot_gain_data:
            memcpy(&ext_buff_musk, Msg, sizeof(ext_buff_t));
            break;
#endif

#if Air_robot_energy_status_data==REFEREE_SYSTEM_ENABLE //飞机能量数据
        case CMD_code_Air_robot_energy_status_data:
            memcpy(&ext_aerial_robot_energy, Msg, sizeof(ext_aerial_robot_energy_t));
            break;
#endif

#if Damage_status_data==REFEREE_SYSTEM_ENABLE //伤害状态
        case CMD_code_Damage_status_data:
            memcpy(&ext_robot_hurt, Msg, sizeof(ext_robot_hurt_t));
            break;
#endif

#if Real_time_shooting_data==REFEREE_SYSTEM_ENABLE//实时射击数据
        case CMD_code_Real_time_shooting_data:
            memcpy(&ext_shoot_data, Msg, sizeof(ext_shoot_data_t));
            break;
#endif
        
#if Ammo_remind_data==REFEREE_SYSTEM_ENABLE//空中机器人及哨兵机器人剩余弹量
        case CMD_code_Ammo_remind_data:
            memcpy(&ext_bullet_remaining, Msg, sizeof(ext_bullet_remaining_t));
            break;
#endif
        
#if Darts_client_cmd == REFEREE_SYSTEM_ENABLE//飞镖机器人客户端指令数据
        case CMD_code_Darts_client_cmd_data:
            memcpy(&ext_dart_client_cmd, Msg, sizeof(ext_dart_client_cmd_t));
            break;
#endif
        }
        return head->data_length + 9;
    } else return 0;
}

uint8_t unpack_refree_system_data(RMQueue_Handle *Queue) {
    uint8_t *referee_ptr = RMQueueTop(Queue);
    if (referee_ptr) {
        memcpy(referee_system_buff, referee_system_buff + USART2_BUFLEN, USART2_BUFLEN);
        memcpy(referee_system_buff + USART2_BUFLEN, referee_ptr, USART2_BUFLEN);
        RMQueuePop(Queue);
        for (uint8_t i = 0; i < USART2_BUFLEN; i++) {
            if (referee_system_buff[i] == 0xA5)
                i += referee_system_Rx(&referee_system_buff[i]);
        }
        return 1;
    }
    return 0;
}

float refree_shooter_limit_bili() {
#if ROBOT_TYPE == 2
    if(ext_power_heat_data.shooter_id1_17mm_cooling_heat > 
        ext_game_robot_state.shooter_id1_17mm_cooling_limit * 0.75) {
        if(ext_power_heat_data.shooter_id1_17mm_cooling_heat < ext_game_robot_state.shooter_id1_17mm_cooling_limit)
            return (ext_game_robot_state.shooter_id1_17mm_cooling_limit - ext_power_heat_data.shooter_id1_17mm_cooling_heat)
                / (ext_game_robot_state.shooter_id1_17mm_cooling_limit * 0.25);
        else
            return 0;
    }    
    else 
            return 1;
#endif

#if ROBOT_TYPE == 4
    if(ext_power_heat_data.shooter_id1_17mm_cooling_heat > 
        ext_game_robot_state.shooter_id1_17mm_cooling_limit * 0.75) {
        if(ext_power_heat_data.shooter_id1_17mm_cooling_heat < ext_game_robot_state.shooter_id1_17mm_cooling_limit * 0.9)
            shooter_bili1 = (ext_game_robot_state.shooter_id1_17mm_cooling_limit - ext_power_heat_data.shooter_id1_17mm_cooling_heat)
                / (ext_game_robot_state.shooter_id1_17mm_cooling_limit * 0.25);
        else
            shooter_bili1 = 0;
    }  
    else
        shooter_bili1 = 1;
    
    if(ext_power_heat_data.shooter_id2_17mm_cooling_heat > 
        ext_game_robot_state.shooter_id2_17mm_cooling_limit * 0.75) {
        if(ext_power_heat_data.shooter_id2_17mm_cooling_heat < ext_game_robot_state.shooter_id2_17mm_cooling_limit * 0.9)
            shooter_bili2 = (ext_game_robot_state.shooter_id2_17mm_cooling_limit - ext_power_heat_data.shooter_id2_17mm_cooling_heat)
                / (ext_game_robot_state.shooter_id2_17mm_cooling_limit * 0.25);
        else
            shooter_bili2 = 0;
    }
    else
        shooter_bili2 = 1;
#endif

#if ROBOT_TYPE == 1
    if(ext_power_heat_data.shooter_id1_42mm_cooling_heat < 100)  return 0;
    else return 1;
#endif

    return 1;
}

#if ROBOT_TYPE == 4
float refree_power_limit_bili() {
    if(ext_power_heat_data.chassis_power_buffer < 100)
    {
        return ext_power_heat_data.chassis_power_buffer / 200.0f;
    }
    return 1;
}
#endif

