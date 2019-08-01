#ifndef __robomaster_H
#define __robomaster_H
#ifdef __cplusplus
extern "C" {
#endif
    
#include "can.h"
#include "utils.h"    
    
//#define Limit(value,max)     if(value>max)value=max;else if(value<- (max+max))value=-(max+max)

typedef enum{
    _M2006,
    _M3508
}ROBOMASTER_TYPE;

 
typedef struct{
    int16_t	 	speed_rpm;
    float  	real_current;
    int16_t  	given_current;
    uint8_t  	hall;
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;	//abs angle range:[0,8191]
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
    int target_position;
    int target_speed;
    ROBOMASTER_TYPE type;
}RoboMaster;

extern PID_Struct chassis_PID;
extern PID_Struct speed_x_dir_pid;
extern PID_Struct position_y_dir_pid;
extern RoboMaster robomaster[4];
extern PID_Struct Robomaster_Speed_PID[4];
extern PID_Struct Robomaster_Position_PID[4];
extern int robomaster_flag;
extern int canmtrid;

void robomaster_set_current2(int id,int16_t iq);
void can_robomaster_rcv(CanRxMsgTypeDef* pRxMsg);
void RoboconMaster_Control();
float robomaster_pid_control(int id);
void robomaster_set_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void robomaster_PID_init();
#ifdef __cplusplus
}
#endif
#endif /*__ track_H */