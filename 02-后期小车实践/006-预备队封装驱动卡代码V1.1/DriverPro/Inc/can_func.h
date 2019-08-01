#ifndef __can_func_H
#define __can_func_H
#ifdef __cplusplus
 extern "C" {
#endif
   
#include "stm32f1xx_hal.h"
#include "main.h"
void can_handle_func(CanRxMsgTypeDef* pRxMsg);

   
 #ifdef __cplusplus
}
#endif
#endif /*__ can_func_H */