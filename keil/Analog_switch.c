#include "Analog_switch.h"

#define SWITCH_VER      0x01U 
#define SWITCH_SN       0x02U
#define SWITCH_READ     0x10U
#define SWITCH_WRITE    0X11U

sbit S0 = P3 ^ 2;
sbit S1 = P1 ^ 4;
sbit S2 = P1 ^ 5;
sbit S3 = P1 ^ 6;
sbit S4 = P1 ^ 7;
sbit S5 = P3 ^ 4;
sbit S6 = P3 ^ 3;
sbit S7 = P1 ^ 1;

extern UINT8I MySerNumber[];

void analog_switch_init(){
    
    P3_MOD_OC = P3_MOD_OC & ~(1 << 2);
    P1_MOD_OC = P1_MOD_OC & ~(1 << 4);
    P1_MOD_OC = P1_MOD_OC & ~(1 << 5);
    P1_MOD_OC = P1_MOD_OC & ~(1 << 6);
    P1_MOD_OC = P1_MOD_OC & ~(1 << 7);
    P3_MOD_OC = P3_MOD_OC & ~(1 << 4);
    P3_MOD_OC = P3_MOD_OC & ~(1 << 3);
    P1_MOD_OC = P1_MOD_OC & ~(1 << 1);
    
    P3_DIR_PU = P3_DIR_PU | (1 << 2);
    P1_DIR_PU = P1_DIR_PU | (1 << 4);
    P1_DIR_PU = P1_DIR_PU | (1 << 5);
    P1_DIR_PU = P1_DIR_PU | (1 << 6);
    P1_DIR_PU = P1_DIR_PU | (1 << 7);
    P3_DIR_PU = P3_DIR_PU | (1 << 4);
    P3_DIR_PU = P3_DIR_PU | (1 << 3);
    P1_DIR_PU = P1_DIR_PU | (1 << 1);
    
    
    sprintf(MySerNumber,"%.2X%.2X%.2X%.2X%.2X%.2X",
            CHIP_ID,
            *(UINT8 *)(0x3FFB),
            *(UINT8 *)(0x3FFC),
            *(UINT8 *)(0x3FFD),
            *(UINT8 *)(0x3FFE),
            *(UINT8 *)(0x3FFF));
}

void analog_switch_write(UINT8 action){
    S0 = action&0x01?1:0;
    S1 = action&0x02?1:0;
    S2 = action&0x04?1:0;
    S3 = action&0x08?1:0;
    S4 = action&0x10?1:0;
    S5 = action&0x20?1:0;
    S6 = action&0x40?1:0;
    S7 = action&0x80?1:0;
    return;
}

UINT8 analog_switch_read(){
    UINT8 status=0;
    status |= S0?0x01:0;
    status |= S1?0x02:0;
    status |= S2?0x04:0;
    status |= S3?0x08:0;
    status |= S4?0x10:0;
    status |= S5?0x20:0;
    status |= S6?0x40:0;
    status |= S7?0x80:0;
    return status;
}

void analog_switch_thread(){
    UINT8 num;
    
    if (Ep2Oi != Ep2Oo){
        PUINT8 req = Ep2DataO[Ep2Oo++];
        PUINT8 res = Ep2DataI[Ep2Ii];
        
        if (Ep2Oo >= DAP_PACKET_COUNT)
            Ep2Oo = 0;
        
        *res++ = *req; //回复命令字
        
        switch(*req++)
        {
            case SWITCH_VER:
                *res ++ = 0x01;
                *res ++ = 0x00;
                *res ++ = 0x08;
                num = 3;
                break;
            case SWITCH_SN:
                *res ++ = CHIP_ID;
                *res ++ = *(UINT8 *)(0x3FFB);
                *res ++ = *(UINT8 *)(0x3FFC);
                *res ++ = *(UINT8 *)(0x3FFD);
                *res ++ = *(UINT8 *)(0x3FFE);
                *res ++ = *(UINT8 *)(0x3FFF);
                num = 6;
                break;
            case SWITCH_READ:
                *res = analog_switch_read();
                num = 1;
                break;
            case SWITCH_WRITE:
                analog_switch_write(*res);
                num = 0; // 无需返回
                break;
        }
        
        Ep2Is[Ep2Ii++] = num + 1;
        if (Ep2Ii >= DAP_PACKET_COUNT)
            Ep2Ii = 0;
        
    }
}