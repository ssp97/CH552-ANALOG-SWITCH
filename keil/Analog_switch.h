#ifndef __ANALOG_SWITCH_H__
#define __ANALOG_SWITCH_H__

#include "CH552.H"
#include <stdio.h>
#include <string.h>
#include <intrins.h>

#define DAP_PACKET_COUNT 2
#define DAP_PACKET_SIZE 64 //THIS_ENDP0_SIZE

extern UINT8I Ep2Oi;
extern UINT8I Ep2Oo;
extern UINT8I Ep2Ii;
extern UINT8I Ep2Io;

extern UINT8I Ep2Is[];
extern UINT8X Ep2DataO[][DAP_PACKET_SIZE];
extern UINT8X Ep2DataI[][DAP_PACKET_SIZE];

extern UINT8I turnaround;
extern UINT8I data_phase;
extern UINT8I idle_cycles;

#endif