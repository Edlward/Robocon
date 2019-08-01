#ifndef __cmd_func_H
#define __cmd_func_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f1xx_hal.h"
#include "main.h"
#include "usart.h"
#include "cmd.h"
#include "stdlib.h"
#include "math.h"
#include "robomaster.h"
#define PI 3.1415926535
   
typedef struct{
  char * var_name;
  void * value_ptr;
}Var_Edit_Struct;

void cmd_help_func(int argc,char *argv[]);//

void cmd_version_func(int argc,char *argv[]);//

void cmd_write_can_func(int argc,char *argv[]);//

void cmd_motor_mode_func(int argc,char *argv[]);//

void cmd_set_duty_func(int argc,char *argv[]);//

void cmd_speed_func(int argc,char *argv[]);//

void cmd_position_func(int argc,char *argv[]);//

void cmd_speed_pid_func(int argc,char *argv[]);//

void cmd_position_pid_func(int argc,char *argv[]);//

void cmd_send_wave_func(int argc,char *argv[]);//
   
void cmd_state_func(int argc,char *argv[]);//

void cmd_max_speed_func(int argc,char *argv[]);//

void cmd_int_limit_func(int argc,char *argv[]);//

void cmd_inipos_func(int argc,char *argv[]);//

void cmd_rst_func(int argc,char *argv[]);

void cmd_canmtrid_func(int argc,char *argv[]);

#ifdef __cplusplus
}
#endif
#endif /*__ cmd_func_H */
