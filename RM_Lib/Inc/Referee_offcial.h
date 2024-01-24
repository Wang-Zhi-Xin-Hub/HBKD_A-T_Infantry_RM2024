#ifndef _REFEREE_SYSTEM_H_
#define _REFEREE_SYSTEM_H_
#include "stm32f4xx_hal.h"
#include "RMQueue.h"
#include "CRC.h"

#define USART2_BUFLENx2 (USART2_BUFLEN * 2)
#define USART2_BUFLEN 40

extern uint8_t referee_system_buff[USART2_BUFLENx2]; // 裁判系统缓存
extern uint8_t referee_system_halfbuff[USART2_BUFLEN]; // 裁判系统半缓存
/**************V1.2****************/

typedef enum {
    REFEREE_SYSTEM_DISABLE = 0, //裁判系统使能
    REFEREE_SYSTEM_ENABLE,
} REFEREE_SYSTEM_SWITCH;  //裁判模式枚举


/* 比赛数据 */
#define Match_status_data                                       REFEREE_SYSTEM_DISABLE      //!<@brief 比赛状态数据
#define Result_data                                             REFEREE_SYSTEM_DISABLE      //!<@brief 比赛状结果数据
#define Match_robot_HP_data                                     REFEREE_SYSTEM_DISABLE
#define Site_event_data                                         REFEREE_SYSTEM_DISABLE      //!<@brief 场地事件数据
#define Site_supply_station_action_identification_data          REFEREE_SYSTEM_DISABLE      //!<@brief 场地补给站动作标识数据，动作改变后发送
#define Field_resupply_station_booking_bullet_data              REFEREE_SYSTEM_DISABLE      //!<@brief 场地补给站补弹数据（由参赛队发送）
#define Referee_warning_data                                    REFEREE_SYSTEM_DISABLE      //!<@brief 裁判警告数据
#define Darts_shoot_timeremind_data                             REFEREE_SYSTEM_DISABLE      //!<@brief 飞镖发射口倒计时
#define Robot_state_data                                        REFEREE_SYSTEM_ENABLE       //!<@brief 机器人状态数据
#define Real_time_power_and_heat_data                           REFEREE_SYSTEM_ENABLE       //!<@brief 实时功率热量数据
#define Robot_position_data                                     REFEREE_SYSTEM_DISABLE      //!<@brief 机器人位置数据
#define Robot_gain_data                                         REFEREE_SYSTEM_ENABLE       //!<@brief 机器人增益数据
#define Air_robot_energy_status_data                            REFEREE_SYSTEM_DISABLE      //!<@brief 空中机器人能量状态数据
#define Damage_status_data                                      REFEREE_SYSTEM_ENABLE       //!<@brief 伤害状态数据
#define Real_time_shooting_data                                 REFEREE_SYSTEM_ENABLE       //!<@brief 实时射击数据
#define Ammo_remind_data                                        REFEREE_SYSTEM_ENABLE       //!<@brief 子弹剩余发送数
#define RFID_data                                               REFEREE_SYSTEM_DISABLE      //!<@brief 机器人RFID状态
#define Darts_client_cmd                                        REFEREE_SYSTEM_DISABLE      //!<@brief 飞镖机器人客户端指令书
#define Robots_interact_with_data                               REFEREE_SYSTEM_ENABLE       //!<@brief 机器人间交互数据
#define Custom_controller_data                                  REFEREE_SYSTEM_DISABLE      //!<@brief 自定义控制器交互数据接口
#define Client_minimap_data                                     REFEREE_SYSTEM_DISABLE      //!<@brief 客户端小地图交互数据
#define Keyborad_mouse_data                                     REFEREE_SYSTEM_DISABLE      //!<@brief 键盘、鼠标信息
/* 命令代码内容同上 */
#define CMD_code_Match_status_data                                ((uint16_t)0x0001)          //!<@brief 比赛状态数据
#define CMD_code_Result_data                                      ((uint16_t)0x0002)          //!<@brief 比赛状结果数据
#define CMD_code_Match_robot_HP_data                              ((uint16_t)0x0003)          //!<@brief 比赛机器人血量数据
#define CMD_code_Site_event_data                                  ((uint16_t)0x0101)          //!<@brief 场地事件数据
#define CMD_code_Site_supply_station_action_identification_data   ((uint16_t)0x0102)          //!<@brief 场地补给站动作标识数据，动作改变后发送
#define CMD_code_Field_resupply_station_booking_bullet_data       ((uint16_t)0x0103)          //!<@brief 场地补给站补弹数据（由参赛队发送）
#define CMD_code_Referee_Warning_data                             ((uint16_t)0x0104)          //!<@brief 裁判警告数据
#define CMD_code_Darts_shoot_timeremind_data                      ((uint16_t)0x0105)          //!<@brief 飞镖发射口倒计时
#define CMD_code_Robot_state_data                                 ((uint16_t)0x0201)          //!<@brief 机器人状态数据
#define CMD_code_Real_time_power_and_heat_data                    ((uint16_t)0x0202)          //!<@brief 实时功率热量数据
#define CMD_code_Robot_position_data                              ((uint16_t)0x0203)          //!<@brief 机器人位置数据
#define CMD_code_Robot_gain_data                                  ((uint16_t)0x0204)          //!<@brief 机器人增益数据
#define CMD_code_Air_robot_energy_status_data                     ((uint16_t)0x0205)          //!<@brief 空中机器人能量状态数据
#define CMD_code_Damage_status_data                               ((uint16_t)0x0206)          //!<@brief 伤害状态数据
#define CMD_code_Real_time_shooting_data                          ((uint16_t)0x0207)          //!<@brief 实时射击数据
#define CMD_code_Ammo_remind_data                                 ((uint16_t)0x0208)          //!<@brief 子弹剩余发送数
#define CMD_code_RFID_data                                        ((uint16_t)0x0209)          //!<@brief 机器人RFID状态
#define CMD_code_Darts_client_cmd_data                            ((uint16_t)0x020A)          //!<@brief 飞镖机器人客户端指令书
#define CMD_code_Robots_interact_with_data                        ((uint16_t)0x0301)          //!<@brief 机器人间交互数据
#define CMD_code_Custom_controller_data                           ((uint16_t)0x0302)          //!<@brief 自定义控制器交互数据接口
#define CMD_code_Client_minimap_data                              ((utin16_t)0x0303)          //!<@brief 客户端小地图交互数据
#define CMD_code_Keyborad_mouse_data                              ((uint16_t)0x0304)          //!<@brief 键盘、鼠标信息

/**
 * 机器人ID枚举
 */
typedef enum {
    R_Hero = 1,//红方
    R_Engineering = 2,
    R_Infantry1 = 3,
    R_Infantry2 = 4,
    R_Infantry3 = 5,
    R_Air = 6,
    R_Sentinel = 7,
    R_Darts = 8,
    R_Radar = 9,
    B_Hero = 101, //蓝方
    B_Engineering = 102,
    B_Infantry1 = 103,
    B_Infantry2 = 104,
    B_Infantry3 = 105,
    B_Air = 106,
    B_Sentinel = 107,
    B_Darts = 108,
    B_Radar = 109,
} Robot_ID;

/*Match_status_data*/
typedef struct {
    uint8_t game_type: 4;
    uint8_t game_progress: 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_status_t;

/*Result_data*/
typedef struct {
    uint8_t winner;
} ext_game_result_t;

/**
 * 机器人血量数据：0x0003。发送频率：1Hz
 */
typedef struct {
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/*Site_event_data*/
typedef enum {
    Not_occupy = 0,
    occupation,
} occupations;

typedef enum {
    not_occupied_not_activated = 0,
    occupied_not_activated,
    not_occupied_activated,
    occupied_and_activated,
} Power_off_state;

typedef enum {
    no_defense = 0,
    sentry_defence = 1,
    invincible = 3,
} Base_defense;

/**
 * 场地事件数据：0x0101。发送频率：1Hz
 */
typedef struct {
    uint32_t depot_point1: 1;                //!<@brief 补给站 1 号补血点占领状态 1 为已占领；
    uint32_t depot_point2: 1;                //!<@brief 补给站 2 号补血点占领状态 1 为已占领；
    uint32_t depot_point3: 1;                //!<@brief 补给站 3 号补血点占领状态 1 为已占领；

    uint32_t strike_point_capture: 3;        //!<@brief 打击点占领状态，1 为占领；
    uint32_t smallEnergy_authorities: 1;     //!<@brief 小能量机关激活状态，1 为已激活；
    uint32_t bigEnergy_authorities: 1;       //!<@brief 大能量机关激活状态，1 为已激活；

    uint32_t annular_highlands_2: 1;         //!<@brief 己方侧 R2/B2 环形高地占领状态 1 为已占领；
    uint32_t trapezoid_highlands_3: 1;       //!<@brief 己方侧 R3/B3 梯形高地占领状态 1 为已占领
    uint32_t trapezoid_highlands_4: 1;       //!<@brief 己方侧 R4/B4 梯形高地占领状态 1 为已占领；

    uint32_t base_shield: 1;                  //!<@brief 己方基地护盾状态, 1 为基地有虚拟护盾血量, 0 为基地无虚拟护盾血量
    uint32_t outposts: 1;                    //!<@brief 己方前哨战状态：1 为前哨战存活
}__attribute__((packed)) ext_event_data_t;

/**
 * 补给站动作标识：0x0102。发送频率：动作改变后发送, 发送范围：己方机器人
 */
typedef struct {
    uint8_t supply_projectile_id;             //!<@brief 补给站口 ID
    uint8_t supply_robot_id;                  //!<@brief 补弹机器人 ID
    uint8_t supply_projectile_step;           //!<@brief 出弹口开闭状态
    uint8_t supply_projectile_num;            //!<@brief 补弹数量：
}__attribute__((packed)) ext_supply_projectile_action_t;

/**
 *  裁判警告信息：cmd_id (0x0104)。发送频率：己方警告发生后发送
 */
typedef struct {
    uint8_t level;                            //!<@brief 警告等级
    uint8_t foul_robot_id;                    //!<@brief 犯规机器人 ID, 判负时，机器人 ID 为 0
}__attribute__((packed)) ext_referee_warning_t;

/**
 * 飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz 周期发送，发送范围：己方机器人
 */
typedef struct {
    uint8_t dart_remaining_time;               //!<@brief 15s 倒计时
}__attribute__((packed)) ext_dart_remaining_time_t;

/**
 * 比赛机器人状态：0x0201。发送频率：10Hz
 */
typedef struct {
    uint8_t robot_id;                           //!<@brief 本机器人 ID
    uint8_t robot_level;                        //!<@brief 机器人等级
    uint16_t remain_HP;                         //!<@brief 机器人剩余血量
    uint16_t max_HP;                            //!<@brief 机器人上限血量
    uint16_t shooter_id1_17mm_cooling_rate;     //!<@brief 机器人 1 号 17mm 枪口每秒冷却值
    uint16_t shooter_id1_17mm_cooling_limit;    //!<@brief 机器人 1 号 17mm 枪口热量上限
    uint16_t shooter_id1_17mm_speed_limit;      //!<@brief 机器人 1 号 17mm 枪口上限速度 单位 m/s
    uint16_t shooter_id2_17mm_cooling_rate;     //!<@brief 机器人 2 号 17mm 枪口每秒冷却值
    uint16_t shooter_id2_17mm_cooling_limit;    //!<@brief 机器人 2 号 17mm 枪口热量上限
    uint16_t shooter_id2_17mm_speed_limit;      //!<@brief 机器人 2 号 17mm 枪口上限速度 单位 m/s
    uint16_t shooter_id1_42mm_cooling_rate;     //!<@brief 机器人 42mm 枪口每秒冷却值
    uint16_t shooter_id1_42mm_cooling_limit;    //!<@brief 机器人 42mm 枪口热量上限
    uint16_t shooter_id1_42mm_speed_limit;      //!<@brief 机器人 42mm 枪口上限速度 单位 m/s
    uint16_t chassis_power_limit;               //!<@brief 机器人底盘功率限制上限

     struct mains_power_t {
        uint8_t gimbal_output: 1;
        uint8_t chassis_output: 1;
        uint8_t shooter_output: 1;
    } mains_power; //!<@brief 主控电源输出情况：
}__attribute__((packed)) ext_game_robot_status_t;


/**
 * 实时功率热量数据：0x0202。发送频率：50Hz
 */
typedef struct {
    uint16_t chassis_volt;                    //!<@brief 底盘输出电压 单位 毫伏
    uint16_t chassis_current;                 //!<@brief 底盘输出电流 单位 毫安
    float chassis_power;                      //!<@brief 底盘输出功率 单位 W 瓦
    uint16_t chassis_power_buffer;            //!<@brief 底盘功率缓冲 单位 J 焦耳 {@note 飞坡根据规则增加至 250J}
    uint16_t shooter_id1_17mm_cooling_heat;   //!<@brief 1 号 17mm 枪口热量
    uint16_t shooter_id2_17mm_cooling_heat;   //!<@brief 2 号 17mm 枪口热量
    uint16_t shooter_id1_42mm_cooling_heat;   //!<@brief 42mm 枪口热量
}__attribute__((packed)) ext_power_heat_data_t;

/**
 * 机器人位置：0x0203。发送频率：10Hz
 */
typedef struct {
    float x;   //!<@brief 位置 x 坐标，单位 m
    float y;   //!<@brief 位置 y 坐标，单位 m
    float z;   //!<@brief 位置 z 坐标，单位 m
    float yaw; //!<@brief 位置枪口，单位度
}__attribute__((packed)) ext_game_robot_pos_t;


/**
 * 机器人增益：0x0204。发送频率：1Hz
 */
typedef struct {
    uint8_t blood: 1;             //!<@brief 机器人血量补血状态
    uint8_t cooling_speed: 1;     //!<@brief 枪口热量冷却加速
    uint8_t Defense_bonus: 1;     //!<@brief 机器人防御加成
    uint8_t Bonus_to_attack: 1;   //!<@brief 机器人攻击加成
}__attribute__((packed)) ext_buff_t;

/**
 * 空中机器人能量状态：0x0205。发送频率：10Hz
 */
typedef struct {
    uint8_t attack_time;          //!<@brief 可攻击时间 单位 s。30s 递减至 0
}__attribute__((packed)) ext_aerial_robot_energy_t;

/**
 * 血量变化类型
 */
enum Types_of_blood {
    Armor_damage = 0,
    Module_dropped_thread,      //!<@brief 模块掉线
    Excess_velocity_shooter,    //!<@brief 超射速
    Excess_heat_shooter,        //!<@brief 枪口过热
    Super_chassis_power,        //!<@brief 底盘功率过高
    Armor_Strike,               //!<@brief 装甲撞击
};


/**
 * 伤害状态：0x0206。发送频率：伤害发生后发送
 */
typedef struct {
    uint8_t armor_id: 4;     //!<@brief 当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的五个装甲片，其他血量变化类型，该变量数值为 0。
    uint8_t hurt_type: 4;    //!<@brief 血量变化类型 {@see Types_of_blood}
}__attribute__((packed)) ext_robot_hurt_t;

/**
 * 实时射击信息：0x0207。发送频率：射击后发送
 */
typedef struct {
    uint8_t bullet_type;    //!<@brief 子弹类型: 1：17mm 弹丸 2：42mm 弹丸
    uint8_t shooter_id;     //!<@brief 发射机构 ID：1：1 号 17mm 发射机构 2：2 号 17mm 发射机构 3：42mm 发射机构
    uint8_t bullet_freq;    //!<@brief 子弹射频 单位 Hz
    float bullet_speed;     //!<@brief 子弹射速 单位 m/s
}__attribute__((packed)) ext_shoot_data_t;


/**
 * 子弹剩余发射数：0x0208。发送频率：10Hz 周期发送，所有机器人发送
 */
typedef struct {
    uint16_t bullet_remaining_num_17mm;   //!<@brief 17mm 子弹剩余发射数目
    uint16_t bullet_remaining_num_42mm;   //!<@brief 42mm 子弹剩余发射数目
    uint16_t coin_remaining_num;          //!<@brief 剩余金币数量
} __attribute__((packed)) ext_bullet_remaining_t;

/**
 * 飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人
 */
typedef struct {
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint16_t operate_launch_cmd_time;
}__attribute__((packed)) ext_dart_client_cmd_t;


extern ext_game_status_t ext_game_state;
extern ext_game_result_t ext_game_result;
extern ext_game_robot_HP_t ext_game_robot_HP;
extern ext_event_data_t ext_event_data;
extern ext_supply_projectile_action_t ext_supply_projectile_action;
extern ext_referee_warning_t ext_referee_warning;
extern ext_dart_remaining_time_t ext_dart_remaining_time;
extern ext_game_robot_status_t ext_game_robot_state;
extern ext_power_heat_data_t ext_power_heat_data;
extern ext_game_robot_pos_t ext_game_robot_pos;
extern ext_buff_t ext_buff_musk;
extern ext_robot_hurt_t ext_robot_hurt;
extern ext_shoot_data_t ext_shoot_data;

int referee_system_Rx(uint8_t *RxMsg);

#if ROBOT_TYPE == 4
extern float shooter_bili1, shooter_bili2;
#endif


uint8_t unpack_refree_system_data(RMQueue_Handle *Queue);
void call_refree_data(uint32_t tim);
float refree_shooter_limit_bili(void);
float refree_power_limit_bili(void);

#endif
