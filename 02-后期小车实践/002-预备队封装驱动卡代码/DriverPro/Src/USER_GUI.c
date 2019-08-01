
#include "USER_GUI.h"
#include "encoder.h"
#include "string.h"
#include "utils.h"
#include "stdlib.h"

void Disp_Title(void) {
  //------重绘数据显示界面---------------------
  //OLED_Clear;
  Show_Str(0, 0, "BUPT", 16, 0);
  Show_Str(0, 2, "ROBOCON", 16, 0);
  Show_Str(0, 4, "2019", 16, 0); 
  Show_Str(0, 6, "by yirongjie", 16, 0); 
}
void Disp_Info(){
  //char *Mtemp="Nothing  ";
  //char *Etemp="Nothing  ";
  char Mtemp[9];
  char Etemp[9];
  if(Motor == BRUSH)
    strcpy(Mtemp, "BrushDC  ");
  else if(Motor == BRUSHLESS)
    strcpy(Mtemp, "BLDC  ");
  else if(Motor == BRUSHLESS_NONSENSOR)
    strcpy(Mtemp, "BLDC-NS  ");
  else
    strcpy(Mtemp, "Nothing  ");
  
  if (Encoder==INCREMENT)
    strcpy(Etemp, "INC");
  else if (Encoder==ABSOLUTE)
    strcpy(Etemp, "ABS");
  else if (Encoder==HALLINC)
    strcpy(Etemp, "HALL");
  else if (Encoder==HALLABS)
    strcpy(Etemp, "HALL&ABS");
  else
    strcpy(Etemp, "Nothing");
  char *iname = (char *) malloc(strlen(Mtemp) + strlen(Etemp));
  sprintf(iname, "%s%s", Mtemp, Etemp);
  Show_Str(0, 0, (uint8_t *)iname, 16, 0);
  char temp[200] = {0};
  sprintf(temp,"Cur %.2f %.2f %.2f",Current_PID.KP,Current_PID.KI,Current_PID.KD);
  Show_Str(0, 2, (uint8_t *)temp, 16, 0); 
  sprintf(temp,"Spd %.2f %.2f %.2f",Speed_PID.KP,Speed_PID.KI,Speed_PID.KD);
  Show_Str(0, 4, (uint8_t *)temp, 16, 0);
  sprintf(temp,"Pos %.2f %.2f %.2f",Position_PID.KP,Position_PID.KI,Position_PID.KD);
  Show_Str(0, 6, (uint8_t *)temp, 16, 0); 
}