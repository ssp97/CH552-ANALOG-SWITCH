
/********************************** (C) COPYRIGHT *******************************
* File Name          :CompatibilityHID.C
* Author             : WCH
* Version            : V1.2
* Date               : 2018/02/28
* Description        : CH554ģ��HID�����豸��֧���ж����´���֧�ֿ��ƶ˵����´���֧������ȫ�٣�����
*******************************************************************************/

#include "CH552.H"
#include "Debug.H"
#include "Analog_switch.h"

#define Fullspeed 1
#define WINUSB 1
#define THIS_ENDP0_SIZE 64

UINT8X Ep0Buffer[THIS_ENDP0_SIZE] _at_ 0x0000;  //�˵�0 OUT&IN��������������ż��ַ
UINT8X Ep2BufferO[THIS_ENDP0_SIZE] _at_ 0x0040; //�˵�2 OUT˫������,������ż��ַ
UINT8X Ep2BufferI[THIS_ENDP0_SIZE] _at_ 0x0080; //�˵�2 IN˫������,������ż��ַ

UINT8X Ep2DataO[DAP_PACKET_COUNT][THIS_ENDP0_SIZE];
UINT8X Ep2DataI[DAP_PACKET_COUNT][THIS_ENDP0_SIZE];

UINT8I Ep2Oi, Ep2Oo;            //OUT ����
UINT8I Ep2Ii, Ep2Io;            //IN ����
UINT8I Ep2Is[DAP_PACKET_COUNT]; //���Ͱ���

PUINT8 pDescr; //USB���ñ�־
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
    0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0x80, 0xfa, //����������
    0x09, 0x04, 0x00, 0x00, 0x02, 0xff, 0x00, 0x00, 0x04, //�ӿ�������

    0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00, //�˵�������
    0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00,
};
#else
#Err no USB interface define
#endif

/*�ַ��������� ��*/
// ����������
UINT8C MyLangDescr[] = {0x04, 0x03, 0x09, 0x04};
// ������Ϣ:ARM
UINT8C MyManuInfo[] = {0x08, 0x03, 0x41, 0x00, 0x52, 0x00, 0x4D, 0x00};
// ��Ʒ��Ϣ: DAP-Link-II
UINT8C MyProdInfo[] =
{
    36,
    0x03,
    'D', 0, 'A', 0, 'P', 0, 'L', 0, 'i', 0, 'n', 0, 'k', 0, ' ', 0,
    'C', 0, 'M', 0, 'S', 0, 'I', 0, 'S', 0, '-', 0, 'D', 0, 'A', 0,
    'P', 0
};
// ���к�: 0001A0000000
UINT8I MySerNumber[] = {0x1A, 0x03, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x31, 0x00, 0x41, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00};
// �ӿ�: CMSIS-DAP
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
* Description    : USB�豸ģʽ����,�豸ģʽ�������շ��˵����ã��жϿ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceInit()
{
    IE_USB = 0;
    USB_CTRL = 0x00;        // ���趨USB�豸ģʽ
    UDEV_CTRL = bUD_PD_DIS; // ��ֹDP/DM��������
#ifndef Fullspeed
    UDEV_CTRL |= bUD_LOW_SPEED; //ѡ�����1.5Mģʽ
    USB_CTRL |= bUC_LOW_SPEED;
#else
    UDEV_CTRL &= ~bUD_LOW_SPEED; //ѡ��ȫ��12Mģʽ��Ĭ�Ϸ�ʽ
    USB_CTRL &= ~bUC_LOW_SPEED;
#endif
    UEP2_DMA = Ep2BufferO;                                     //�˵�2���ݴ����ַ
    UEP2_3_MOD |= bUEP2_TX_EN | bUEP2_RX_EN;                   //�˵�2���ͽ���ʹ��
    UEP2_3_MOD |= ~bUEP2_BUF_MOD;                              //�˵�2�շ���64�ֽڻ�����
    UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK; //�˵�2�Զ���תͬ����־λ��IN���񷵻�NAK��OUT����ACK
    UEP0_DMA = Ep0Buffer;                                      //�˵�0���ݴ����ַ
    UEP4_1_MOD &= ~(bUEP4_RX_EN | bUEP4_TX_EN);                //�˵�0��64�ֽ��շ�������
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                 //OUT���񷵻�ACK��IN���񷵻�NAK

    USB_DEV_AD = 0x00;
    USB_CTRL |= bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN; // ����USB�豸��DMA�����ж��ڼ��жϱ�־δ���ǰ�Զ�����NAK
    UDEV_CTRL |= bUD_PORT_EN;                              // ����USB�˿�
    USB_INT_FG = 0xFF;                                     // ���жϱ�־
    USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
    IE_USB = 1;
}

/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description    : CH559USB�жϴ�������
*******************************************************************************/
void DeviceInterrupt(void) interrupt INT_NO_USB using 1 //USB�жϷ������,ʹ�üĴ�����1
{
    UINT8 len;
    if (UIF_TRANSFER) //USB������ɱ�־
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 2: //endpoint 2# �˵������ϴ�
            UEP2_T_LEN = 0;      //Ԥʹ�÷��ͳ���һ��Ҫ���
            Endp2Busy = 0;
            UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; //Ĭ��Ӧ��NAK
            break;
        case UIS_TOKEN_OUT | 2: //endpoint 2# �˵������´�
            if (U_TOG_OK)         // ��ͬ�������ݰ�������
            {
                len = USB_RX_LEN;
                memcpy(Ep2DataO[Ep2Oi++], Ep2BufferO, len); //���Ż�
                if (Ep2Oi >= DAP_PACKET_COUNT)
                    Ep2Oi = 0;
            }
            break;
        case UIS_TOKEN_SETUP | 0: //SETUP����
            len = USB_RX_LEN;
            if (len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;
                if (UsbSetupBuf->wLengthH || SetupLen > 0xF0)
                    SetupLen = 0xF0; // �����ܳ���
                len = 0;           // Ĭ��Ϊ�ɹ������ϴ�0����
                SetupReq = UsbSetupBuf->bRequest;
                if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) /*HID������*/
                {
                    if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE)
                    {
                        switch (SetupReq)
                        {
                        case 0x20:                         //GetReport
                            if (UsbSetupBuf->wIndexL == 0x07)
                            {
                                pDescr = WINUSB_Descriptor; //���豸�������͵�Ҫ���͵Ļ�����
                                len = sizeof(WINUSB_Descriptor);
                            }
                            break;
                        default:
                            len = 0xFF; /*���֧��*/
                            break;
                        }
                    }
                    if (SetupLen > len)
                    {
                        SetupLen = len; //�����ܳ���
                    }
                    len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; //���δ��䳤��
                    memcpy(Ep0Buffer, pDescr, len);                                 //�����ϴ�����
                    SetupLen -= len;
                    pDescr += len;
                }
                else //��׼����
                {
                    switch (SetupReq) //������
                    {
                    case USB_GET_DESCRIPTOR:
                        switch (UsbSetupBuf->wValueH)
                        {
                        case 1:             //�豸������
                            pDescr = DevDesc; //���豸�������͵�Ҫ���͵Ļ�����
                            len = sizeof(DevDesc);
                            break;
                        case 2:             //����������
                            pDescr = CfgDesc; //���豸�������͵�Ҫ���͵Ļ�����
                            len = sizeof(CfgDesc);
                            break;
                        case 3: // �ַ���������
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
                                len = 0xFF; // ��֧�ֵ��ַ���������
                                break;
                            }
                            break;
                        case 15:
                            pDescr = (PUINT8)(&USB_BOSDescriptor[0]);
                            len = sizeof(USB_BOSDescriptor);
                            break;
                        default:
                            len = 0xff; //��֧�ֵ�������߳���
                            break;
                        }
                        if (SetupLen > len)
                        {
                            SetupLen = len; //�����ܳ���
                        }
                        len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; //���δ��䳤��
                        memcpy(Ep0Buffer, pDescr, len);                                 //�����ϴ�����
                        SetupLen -= len;
                        pDescr += len;
                        break;
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL; //�ݴ�USB�豸��ַ
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
                            Ready = 1; //set config����һ�����usbö����ɵı�־
                        }
                        break;
                    case 0x0A:
                        break;
                    case USB_CLEAR_FEATURE:                                                       //Clear Feature
                        if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) // �˵�
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
                                len = 0xFF; // ��֧�ֵĶ˵�
                                break;
                            }
                        }
                        else
                        {
                            len = 0xFF; // ���Ƕ˵㲻֧��
                        }
                        break;
                    case USB_SET_FEATURE:                             /* Set Feature */
                        if ((UsbSetupBuf->bRequestType & 0x1F) == 0x00) /* �����豸 */
                        {
                            if ((((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x01)
                            {
                                if (CfgDesc[7] & 0x20)
                                {
                                    /* ���û���ʹ�ܱ�־ */
                                }
                                else
                                {
                                    len = 0xFF; /* ����ʧ�� */
                                }
                            }
                            else
                            {
                                len = 0xFF; /* ����ʧ�� */
                            }
                        }
                        else if ((UsbSetupBuf->bRequestType & 0x1F) == 0x02) /* ���ö˵� */
                        {
                            if ((((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x00)
                            {
                                switch (((UINT16)UsbSetupBuf->wIndexH << 8) | UsbSetupBuf->wIndexL)
                                {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* ���ö˵�2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* ���ö˵�2 OUT Stall */
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* ���ö˵�1 IN STALL */
                                    break;
                                default:
                                    len = 0xFF; /* ����ʧ�� */
                                    break;
                                }
                            }
                            else
                            {
                                len = 0xFF; /* ����ʧ�� */
                            }
                        }
                        else
                        {
                            len = 0xFF; /* ����ʧ�� */
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
                        len = 0xff; //����ʧ��
                        break;
                    }
                }
            }
            else
            {
                len = 0xff; //�����ȴ���
            }
            if (len == 0xff)
            {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; //STALL
            }
            else if (len <= THIS_ENDP0_SIZE) //�ϴ����ݻ���״̬�׶η���0���Ȱ�
            {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; //Ĭ�����ݰ���DATA1������Ӧ��ACK
            }
            else
            {
                UEP0_T_LEN = 0;                                                      //��Ȼ��δ��״̬�׶Σ�������ǰԤ���ϴ�0�������ݰ��Է�������ǰ����״̬�׶�
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; //Ĭ�����ݰ���DATA1,����Ӧ��ACK
            }
            break;
        case UIS_TOKEN_IN | 0: //endpoint0 IN
            switch (SetupReq)
            {
            case USB_GET_DESCRIPTOR:
            case 0x20:
                len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; //���δ��䳤��
                memcpy(Ep0Buffer, pDescr, len);                                 //�����ϴ�����
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG; //ͬ����־λ��ת
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
                UEP0_T_LEN = 0; //״̬�׶�����жϻ�����ǿ���ϴ�0�������ݰ��������ƴ���
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
            UEP0_CTRL ^= bUEP_R_TOG; //ͬ����־λ��ת
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0; //д0����ж�
    }
    if (UIF_BUS_RST) //�豸ģʽUSB���߸�λ�ж�
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        Endp2Busy = 0;
        UIF_BUS_RST = 0; //���жϱ�־
    }
    if (UIF_SUSPEND) //USB���߹���/�������
    {
        UIF_SUSPEND = 0;
        if (USB_MIS_ST & bUMS_SUSPEND) //����
        {
        }
    }
    else
    {
        //������ж�,�����ܷ��������
        USB_INT_FG = 0xFF; //���жϱ�־
    }
}

void main(void)
{
    CfgFsys();   //CH559ʱ��ѡ������
    mDelaymS(5); //�޸���Ƶ�ȴ��ڲ������ȶ�,�ؼ�
//  mInitSTDIO();

    USBDeviceInit(); //USB�豸ģʽ��ʼ��
    EA = 1;          //������Ƭ���ж�
    UEP1_T_LEN = 0;  //Ԥʹ�÷��ͳ���һ��Ҫ���
    UEP2_T_LEN = 0;  //Ԥʹ�÷��ͳ���һ��Ҫ���
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
            memcpy(Ep2BufferI, Ep2DataI[Ep2Io], Ep2Is[Ep2Io]); //�����ϴ�����
            UEP2_T_LEN = Ep2Is[Ep2Io++];                       //�ϴ���������
            if (Ep2Io >= DAP_PACKET_COUNT)
                Ep2Io = 0;
            UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK; //������ʱ�ϴ����ݲ�Ӧ��ACK
        }
    }
}