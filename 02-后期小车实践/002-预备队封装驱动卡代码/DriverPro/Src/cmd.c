#include "cmd.h"


/*
  *存放命令的结构体
  * 若需添加命令，需要在此加上：
  * CMD_ADD("命令名","命令使用方法（可为空格,但不能不加双引号）",对应命令的执行函数名)
  * 注意最后一个不需要逗号，前面的都需要逗号
  */
static cmd_struct cmd_tbl[] = {
  CMD_ADD("help","help",cmd_help_func),
  CMD_ADD("version","return the version of driver",cmd_version_func),
  CMD_ADD("canid","write can id to flash, use canid <Param> to set can id.",cmd_write_can_func), 
  CMD_ADD("mtr","change motor and encoder type, use mtr <Param> <Param2> to set, the first param is the type of encoder,and the second param is the type of motor.",cmd_motor_mode_func),
  CMD_ADD("canmtrid","write robomaster can id",cmd_canmtrid_func),
  CMD_ADD("speed_pid","set speed pid, use speed_pid <Param> <Param2> <Param3> to set, three params correspond to p, i and d respectively.",cmd_speed_pid_func), 
  CMD_ADD("position_pd","set position pid,use position_pd <Param> <Param2> to set, two params correspond to p and d respectively",cmd_position_pid_func),
  CMD_ADD("max_speed","set max speed of speed-position mode",cmd_max_speed_func),
  CMD_ADD("asoffset","change the zero pos of as5047p, use asoffset <Param> to set.",cmd_inipos_func),
  CMD_ADD("int_limit","set max integral of speed pid, use int_limit <Param> to set",cmd_int_limit_func),
  CMD_ADD("duty", "set_duty of motor,use duty <Param> to set.", cmd_set_duty_func),
  CMD_ADD("speed","set target speed, use speed <Param> to set.",cmd_speed_func),
  CMD_ADD("position","set target position, use position <Param> to set",cmd_position_func),
  CMD_ADD("rst","not used",cmd_rst_func),
  CMD_ADD("wave", "output oscilloscope wave, use wave <Param> to select which form you want.", cmd_send_wave_func),
  CMD_ADD("status","output status of the driver",cmd_state_func)
};

char cmd_line[MAX_CMD_LINE_LENGTH + 1];
char *cmd_argv[MAX_ARGC]; 

void cmd_init()
{
  for(int i = 0;i < MAX_ARGC;i++){
    cmd_argv[i] = (char *)malloc(MAX_CMD_ARG_LENGTH + 1);//不确定输入数据的内存空间，所以分配一块
  }
}
/*
*解析命令函数
*/
int cmd_parse(char *cmd_line,int *argc,char *argv[]){
  char c_temp;
  int i = 0,arg_index = 0;
  int arg_cnt = 0;
  c_temp = cmd_line[i++];  
  while(c_temp != '\r'){
    if(c_temp == ' '){
      //空格为参数或者命令的分隔符
      argv[arg_cnt][arg_index] = 0;
      arg_cnt++;
      arg_index = 0;
      c_temp = cmd_line[i++];
      if(i>90)
      {
        return -3;
      }
      continue;
    }
    argv[arg_cnt][arg_index++] = c_temp;
    c_temp = cmd_line[i++];
  }
  //最后一个参数的结束没有在上面的while循环中解析到
  argv[arg_cnt++][arg_index] = 0;
  *argc = arg_cnt;//命令数
  return 0;
}

int cmd_exec(int argc,char *argv[]){
  int cmd_index = 0;
  uint32_t cmd_num;
  
  cmd_num = sizeof(cmd_tbl)/sizeof(cmd_tbl[0]);
  
  for(cmd_index = 0;cmd_index < cmd_num;cmd_index++){   //查找命令
    if(strcmp((char *)(cmd_tbl[cmd_index].cmd_name),(char *)argv[0]) == 0){  //如果找到了命令，则执行命令相对应的函数
      uprintf("*******************************************************************************\r\n");
      cmd_tbl[cmd_index].cmd_func(argc,argv);
      memset(USART_RX_BUF,0,MAX_CMD_LINE_LENGTH + 1);
      return 0;
    }
  }
  return -2;
}

void cmd_help_func(int argc,char *argv[]){
  int i;
  uint32_t cmd_num;
  cmd_num = sizeof(cmd_tbl)/sizeof(cmd_tbl[0]);
  if(argc > 1){
    uprintf("msg:\n help too many arguments\r\n\r\n");      
    return;         
  }
  for(i = 0;i < cmd_num;i++){
    uprintf("cmd:%s\r\n",cmd_tbl[i].cmd_name);
    uprintf("usage:%s\r\n\r\n",cmd_tbl[i].cmd_usage);
  }
}

uint8_t compare_string(const char *s1,char * s2)
{
  int i=0;
  while(s1[i]==s2[i]&&s1[i]&&s2[i]){
    i++;
  }
  if(!s1[i]&&!s2[i]){
    return 1;
  }else{
    return 0;
  }  
}

