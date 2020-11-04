
/********************************** (C) COPYRIGHT *******************************
* File Name          :CompatibilityHID.C
* Author             : WCH
* Version            : V1.2
* Date               : 2018/02/28
* Description        : CH554模拟HID兼容设备，支持中断上下传，支持控制端点上下传，支持设置全速，低速
*******************************************************************************/

#include "CH552.H"
#include "Debug.H"
#include "Analog_switch.h"

#define Fullspeed 1
#define WINUSB 1
#define THIS_ENDP0_SIZE 64

UINT8X Ep0Buffer[THIS_ENDP0_SIZE] _at_ 0x0000;  //端点0 OUT&IN缓冲区，必须是偶地址
UINT8X Ep2BufferO[THIS_ENDP0_SIZE] _at_ 0x0040; //端点2 OUT双缓冲区,必须是偶地址
UINT8X Ep2BufferI[THIS_ENDP0_SIZE] _at_ 0x0080; //端点2 IN双缓冲区,必须是偶地址

UINT8X Ep2DataO[DAP_PACKET_COUNT][THIS_ENDP0_SIZE];
UINT8X Ep2DataI[DAP_PACKET_COUNT][THIS_ENDP0_SIZE];

UINT8I Ep2Oi, Ep2Oo;            //OUT 索引
UINT8I Ep2Ii, Ep2Io;            //IN 索引
UINT8I Ep2Is[DAP_PACKET_COUNT]; //发送包长

PUINT8 pDescr; //USB配置标志
UINT8I Endp2Busy = 0;
UINT8I SetupReq, SetupLen, Ready, Count, UsbConfig;
#define UsbSetupBuf ((PUSB_SETUP_REQ)Ep0Buffer)

#if (WINUSB == 1)
UINT8C DevDesc[] =
{
    0x12, 0x01, 0x10, 0x02, 0xEF, 0x02, 0x01, THIS_ENDP0_SIZE,
    0x28, 0x0D, 0x04, 0x02, 0x00, 0x01, 0x01, 0x02,
    0x03, 0x01
};
UINT8C CfgDesc[] =
{
    0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0x80, 0xfa, //配置描述符
    0x09, 0x04, 0x00, 0x00, 0x02, 0xff, 0x00, 0x00, 0x04, //接口描述符

    0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00, //端点描述符
    0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00,
};
#else
#Err no USB interface define
#endif

/*字符串描述符 略*/
// 语言描述符
UINT8C MyLangDescr[] = {0x04, 0x03, 0x09, 0x04};
// 厂家信息:ARM
UINT8C MyManuInfo[] = {0x08, 0x03, 0x41, 0x00, 0x52, 0x00, 0x4D, 0x00};
// 产品信息: DAP-Link-II
UINT8C MyProdInfo[] =
{
    36,
    0x03,
    'D', 0, 'A', 0, 'P', 0, 'L', 0, 'i', 0, 'n', 0, 'k', 0, ' ', 0,
    'C', 0, 'M', 0, 'S', 0, 'I', 0, 'S', 0, '-', 0, 'D', 0, 'A', 0,
    'P', 0
};
// 序列号: 0001A0000000
UINT8I MySerNumber[] = {0x1A, 0x03, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x31, 0x00, 0x41, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00};
// 接口: CMSIS-DAP
UINT8C MyInterface[] =
{
    26,
    0x03,
    'C', 0, 'M', 0, 'S', 0, 'I', 0, 'S', 0, '-', 0, 'D', 0, 'A', 0, 'P', 0, ' ', 0, 'v', 0, '2', 0
};

UINT8C USB_BOSDescriptor[] =
{
    0x05, 0x0F, 0x28, 0x00, 0x02, 0x07, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x10, 0x05, 0x00,
    0xDF, 0x60, 0xDD, 0xD8, 0x89, 0x45, 0xC7,
    0x4C, 0x9C, 0xD2, 0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F, 0x00, 0x00, 0x03, 0x06, 0xA2, 0x00, 0x20,
    0x00
};

UINT8C WINUSB_Descriptor[] =
{
    0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x06, 0xA2, 0x00,
    0x14, 0x00, 0x03, 0x00, 0x57, 0x49, 0x4E, 0x55, 0x53, 0x42, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 0x04, 0x00, 0x07, 0x00, 0x2A, 0x00, 0x44, 0x00,
    0x65, 0x00, 0x76, 0x00, 0x69, 0x00, 0x63, 0x00, 0x65, 0x00, 0x49, 0x00, 0x6E, 0x00, 0x74, 0x00,
    0x65, 0x00, 0x72, 0x00, 0x66, 0x00, 0x61, 0x00, 0x63, 0x00, 0x65, 0x00, 0x47, 0x00, 0x55, 0x00,
    0x49, 0x00, 0x44, 0x00, 0x73, 0x00, 0x00, 0x00, 0x50, 0x00, 0x7B, 0x00, 0x43, 0x00, 0x44, 0x00,
    0x42, 0x00, 0x33, 0x00, 0x42, 0x00, 0x35, 0x00, 0x41, 0x00, 0x44, 0x00, 0x2D, 0x00, 0x32, 0x00,
    0x39, 0x00, 0x33, 0x00, 0x42, 0x00, 0x2D, 0x00, 0x34, 0x00, 0x36, 0x00, 0x36, 0x00, 0x33, 0x00,
    0x2D, 0x00, 0x41, 0x00, 0x41, 0x00, 0x33, 0x00, 0x36, 0x00, 0x2D, 0x00, 0x31, 0x00, 0x41, 0x00,
    0x41, 0x00, 0x45, 0x00, 0x34, 0x00, 0x36, 0x00, 0x34, 0x00, 0x36, 0x00, 0x33, 0x00, 0x37, 0x00,
    0x37, 0x00, 0x36, 0x00, 0x7D, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*******************************************************************************
* Function Name  : USBDeviceInit()
* Description    : USB设备模式配置,设备模式启动，收发端点配置，中断开启
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceInit()
{
    IE_USB = 0;
    USB_CTRL = 0x00;        // 先设定USB设备模式
    UDEV_CTRL = bUD_PD_DIS; // 禁止DP/DM下拉电阻
#ifndef Fullspeed
    UDEV_CTRL |= bUD_LOW_SPEED; //选择低速1.5M模式
    USB_CTRL |= bUC_LOW_SPEED;
#else
    UDEV_CTRL &= ~bUD_LOW_SPEED; //选择全速12M模式，默认方式
    USB_CTRL &= ~bUC_LOW_SPEED;
#endif
    UEP2_DMA = Ep2BufferO;                                     //端点2数据传输地址
    UEP2_3_MOD |= bUEP2_TX_EN | bUEP2_RX_EN;                   //端点2发送接收使能
    UEP2_3_MOD |= ~bUEP2_BUF_MOD;                              //端点2收发各64字节缓冲区
    UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK; //端点2自动翻转同步标志位，IN事务返回NAK，OUT返回ACK
    UEP0_DMA = Ep0Buffer;                                      //端点0数据传输地址
    UEP4_1_MOD &= ~(bUEP4_RX_EN | bUEP4_TX_EN);                //端点0单64字节收发缓冲区
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                 //OUT事务返回ACK，IN事务返回NAK

    USB_DEV_AD = 0x00;
    USB_CTRL |= bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN; // 启动USB设备及DMA，在中断期间中断标志未清除前自动返回NAK
    UDEV_CTRL |= bUD_PORT_EN;                              // 允许USB端口
    USB_INT_FG = 0xFF;                                     // 清中断标志
    USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
    IE_USB = 1;
}

/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description    : CH559USB中断处理函数
*******************************************************************************/
void DeviceInterrupt(void) interrupt INT_NO_USB using 1 //USB中断服务程序,使用寄存器组1
{
    UINT8 len;
    if (UIF_TRANSFER) //USB传输完成标志
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 2: //endpoint 2# 端点批量上传
            UEP2_T_LEN = 0;      //预使用发送长度一定要清空
            Endp2Busy = 0;
            UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; //默认应答NAK
            break;
        case UIS_TOKEN_OUT | 2: //endpoint 2# 端点批量下传
            if (U_TOG_OK)         // 不同步的数据包将丢弃
            {
                len = USB_RX_LEN;
                memcpy(Ep2DataO[Ep2Oi++], Ep2BufferO, len); //待优化
                if (Ep2Oi >= DAP_PACKET_COUNT)
                    Ep2Oi = 0;
            }
            break;
        case UIS_TOKEN_SETUP | 0: //SETUP事务
            len = USB_RX_LEN;
            if (len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;
                if (UsbSetupBuf->wLengthH || SetupLen > 0xF0)
                    SetupLen = 0xF0; // 限制总长度
                len = 0;           // 默认为成功并且上传0长度
                SetupReq = UsbSetupBuf->bRequest;
                if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) /*HID类命令*/
                {
                    if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE)
                    {
                        switch (SetupReq)
                        {
                        case 0x20:                         //GetReport
                            if (UsbSetupBuf->wIndexL == 0x07)
                            {
                                pDescr = WINUSB_Descriptor; //把设备描述符送到要发送的缓冲区
                                len = sizeof(WINUSB_Descriptor);
                            }
                            break;
                        default:
                            len = 0xFF; /*命令不支持*/
                            break;
                        }
                    }
                    if (SetupLen > len)
                    {
                        SetupLen = len; //限制总长度
                    }
                    len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; //本次传输长度
                    memcpy(Ep0Buffer, pDescr, len);                                 //加载上传数据
                    SetupLen -= len;
                    pDescr += len;
                }
                else //标准请求
                {
                    switch (SetupReq) //请求码
                    {
                    case USB_GET_DESCRIPTOR:
                        switch (UsbSetupBuf->wValueH)
                        {
                        case 1:             //设备描述符
                            pDescr = DevDesc; //把设备描述符送到要发送的缓冲区
                            len = sizeof(DevDesc);
                            break;
                        case 2:             //配置描述符
                            pDescr = CfgDesc; //把设备描述符送到要发送的缓冲区
                            len = sizeof(CfgDesc);
                            break;
                        case 3: // 字符串描述符
                            switch (UsbSetupBuf->wValueL)
                            {
                            case 0:
                                pDescr = (PUINT8)(&MyLangDescr[0]);
                                len = sizeof(MyLangDescr);
                                break;
                            case 1:
                                pDescr = (PUINT8)(&MyManuInfo[0]);
                                len = sizeof(MyManuInfo);
                                break;
                            case 2:
                                pDescr = (PUINT8)(&MyProdInfo[0]);
                                len = sizeof(MyProdInfo);
                                break;
                            case 3:
                                pDescr = (PUINT8)(&MySerNumber[0]);
                                len = sizeof(MySerNumber);
                                break;
                            case 4:
                                pDescr = (PUINT8)(&MyInterface[0]);
                                len = 26;//sizeof(MyInterface);
                                break;
                            default:
                                len = 0xFF; // 不支持的字符串描述符
                                break;
                            }
                            break;
                        case 15:
                            pDescr = (PUINT8)(&USB_BOSDescriptor[0]);
                            len = sizeof(USB_BOSDescriptor);
                            break;
                        default:
                            len = 0xff; //不支持的命令或者出错
                            break;
                        }
                        if (SetupLen > len)
                        {
                            SetupLen = len; //限制总长度
                        }
                        len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; //本次传输长度
                        memcpy(Ep0Buffer, pDescr, len);                                 //加载上传数据
                        SetupLen -= len;
                        pDescr += len;
                        break;
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL; //暂存USB设备地址
                        break;
                    case USB_GET_CONFIGURATION:
                        Ep0Buffer[0] = UsbConfig;
                        if (SetupLen >= 1)
                        {
                            len = 1;
                        }
                        break;
                    case USB_SET_CONFIGURATION:
                        UsbConfig = UsbSetupBuf->wValueL;
                        if (UsbConfig)
                        {
                            Ready = 1; //set config命令一般代表usb枚举完成的标志
                        }
                        break;
                    case 0x0A:
                        break;
                    case USB_CLEAR_FEATURE:                                                       //Clear Feature
                        if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) // 端点
                        {
                            switch (UsbSetupBuf->wIndexL)
                            {
                            case 0x82:
                                UEP2_CTRL = UEP2_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                break;
                            case 0x81:
                                UEP1_CTRL = UEP1_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                break;
                            case 0x02:
                                UEP2_CTRL = UEP2_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
                                break;
                            default:
                                len = 0xFF; // 不支持的端点
                                break;
                            }
                        }
                        else
                        {
                            len = 0xFF; // 不是端点不支持
                        }
                        break;
                    case USB_SET_FEATURE:                             /* Set Feature */
                        if ((UsbSetupBuf->bRequestType & 0x1F) == 0x00) /* 设置设备 */
                        {
                            if ((((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x01)
                            {
                                if (CfgDesc[7] & 0x20)
                                {
                                    /* 设置唤醒使能标志 */
                                }
                                else
                                {
                                    len = 0xFF; /* 操作失败 */
                                }
                            }
                            else
                            {
                                len = 0xFF; /* 操作失败 */
                            }
                        }
                        else if ((UsbSetupBuf->bRequestType & 0x1F) == 0x02) /* 设置端点 */
                        {
                            if ((((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x00)
                            {
                                switch (((UINT16)UsbSetupBuf->wIndexH << 8) | UsbSetupBuf->wIndexL)
                                {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* 设置端点2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* 设置端点2 OUT Stall */
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* 设置端点1 IN STALL */
                                    break;
                                default:
                                    len = 0xFF; /* 操作失败 */
                                    break;
                                }
                            }
                            else
                            {
                                len = 0xFF; /* 操作失败 */
                            }
                        }
                        else
                        {
                            len = 0xFF; /* 操作失败 */
                        }
                        break;
                    case USB_GET_STATUS:
                        Ep0Buffer[0] = 0x00;
                        Ep0Buffer[1] = 0x00;
                        if (SetupLen >= 2)
                        {
                            len = 2;
                        }
                        else
                        {
                            len = SetupLen;
                        }
                        break;
                    default:
                        len = 0xff; //操作失败
                        break;
                    }
                }
            }
            else
            {
                len = 0xff; //包长度错误
            }
            if (len == 0xff)
            {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; //STALL
            }
            else if (len <= THIS_ENDP0_SIZE) //上传数据或者状态阶段返回0长度包
            {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; //默认数据包是DATA1，返回应答ACK
            }
            else
            {
                UEP0_T_LEN = 0;                                                      //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; //默认数据包是DATA1,返回应答ACK
            }
            break;
        case UIS_TOKEN_IN | 0: //endpoint0 IN
            switch (SetupReq)
            {
            case USB_GET_DESCRIPTOR:
            case 0x20:
                len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; //本次传输长度
                memcpy(Ep0Buffer, pDescr, len);                                 //加载上传数据
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG; //同步标志位翻转
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
                UEP0_T_LEN = 0; //状态阶段完成中断或者是强制上传0长度数据包结束控制传输
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            }
            break;
        case UIS_TOKEN_OUT | 0: // endpoint0 OUT
            len = USB_RX_LEN;
            if (SetupReq == 0x09)
            {
                if (Ep0Buffer[0])
                {
                }
                else if (Ep0Buffer[0] == 0)
                {
                }
            }
            UEP0_CTRL ^= bUEP_R_TOG; //同步标志位翻转
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0; //写0清空中断
    }
    if (UIF_BUS_RST) //设备模式USB总线复位中断
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        Endp2Busy = 0;
        UIF_BUS_RST = 0; //清中断标志
    }
    if (UIF_SUSPEND) //USB总线挂起/唤醒完成
    {
        UIF_SUSPEND = 0;
        if (USB_MIS_ST & bUMS_SUSPEND) //挂起
        {
        }
    }
    else
    {
        //意外的中断,不可能发生的情况
        USB_INT_FG = 0xFF; //清中断标志
    }
}

void main(void)
{
    CfgFsys();   //CH559时钟选择配置
    mDelaymS(5); //修改主频等待内部晶振稳定,必加
//  mInitSTDIO();

    USBDeviceInit(); //USB设备模式初始化
    EA = 1;          //允许单片机中断
    UEP1_T_LEN = 0;  //预使用发送长度一定要清空
    UEP2_T_LEN = 0;  //预使用发送长度一定要清空
    Ready = 0;

    Ep2Oi = 0;
    Ep2Oo = 0;
    Ep2Ii = 0;
    Ep2Io = 0;

    analog_switch_init();

    while (1)
    {
        //DAP_Thread();
        analog_switch_thread();
        
        if (Endp2Busy != 1 && Ep2Ii != Ep2Io)
        {
            Endp2Busy = 1;
            memcpy(Ep2BufferI, Ep2DataI[Ep2Io], Ep2Is[Ep2Io]); //加载上传数据
            UEP2_T_LEN = Ep2Is[Ep2Io++];                       //上传最大包长度
            if (Ep2Io >= DAP_PACKET_COUNT)
                Ep2Io = 0;
            UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK; //有数据时上传数据并应答ACK
        }
    }
}
