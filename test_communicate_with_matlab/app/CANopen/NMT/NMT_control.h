#ifndef __NMT_CONTROL_H
#define __NMT_CONTROL_H

#include "can.h"
#include "def.h"


#define UNS8   unsigned char
#define UNS16  unsigned short
#define UNS32  unsigned long
#define UNS24  unsigned long
#define UNS40  unsigned long long
#define UNS48  unsigned long long
#define UNS56  unsigned long long
#define UNS64  unsigned long long


void NMT_State_Change(UNS8 Node_ID,UNS8 cs);

#endif
