/******************************************************************************
 * Copyright (C) 2021, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************/

/******************************************************************************
 * @file   main.c
 *
 * @brief  Source file for TEMPLATE example
 *
 * @author MADS Team 
 *
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "sysctrl.h"
#include "gpio.h"
#include "can.h"
#include "spi.h"
#include "reset.h"
#include "w5500.h"
#include "wizchip_conf.h"
#include "socket.h"
#include "uart.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define SLAVEADDR   0xc0

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void App_SysClkInit(void);
static void App_CanGpioInit(void);
static void App_CanInit(void);
static void App_GpioInit(void);
static void App_SPIInit(void);
void App_PortInit(void);
void App_UartCfg(void);
void W5500_Init(void);
void ReceiveData(void);
void SendData(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
///CAN
stc_can_rxframe_t       stcRxFrame;
stc_can_txframe_t       stcTxFrame;
uint8_t                 u8RxFlag = FALSE;
///ETH
uint8_t rx_buf[8] = {0};
uint8_t tx_buf[8] = "asd";
uint8_t txsize[8] = {2, 2, 2, 2, 2, 2, 2, 2}; // 每个Socket的发送缓冲区大小为2KB
uint8_t rxsize[8] = {2, 2, 2, 2, 2, 2, 2, 2}; // 每个Socket的接收缓冲区大小为2KB
///UART-RS485
volatile uint8_t u8TxData[2] = {0xaa,0x55};
volatile uint8_t u8RxData[2] = {0x00};
uint8_t u8TxCnt=0,u8RxCnt=0;
/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/


/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/


/*******************************************************************************
 * 中断服务函数
 ******************************************************************************/
void Can_IRQHandler(void)
{
    if(TRUE == CAN_IrqFlgGet(CanRxIrqFlg))
    {
        CAN_IrqFlgClr(CanRxIrqFlg);
        CAN_IrqCmd(CanRxIrqEn, FALSE);

        CAN_Receive(&stcRxFrame);

        u8RxFlag = TRUE;
    }

}

//UART0中断
void Uart0_IRQHandler(void)
{
    if(Uart_GetStatus(M0P_UART0, UartRC))
    {
        Uart_ClrStatus(M0P_UART0, UartRC);              //清除中断状态位
        u8RxData[u8RxCnt]=Uart_ReceiveData(M0P_UART0);  //发送数据
        u8RxCnt++;
        if(u8RxCnt>1)                                   //如果已接收两个字节
        {
            Uart_DisableIrq(M0P_UART0,UartRxIrq);       //禁止接收中断
        }
    }
    
    if(Uart_GetStatus(M0P_UART0, UartTC))
    {
        Uart_ClrStatus(M0P_UART0, UartTC);              //清除中断状态位
        Uart_SendDataIt(M0P_UART0, u8TxData[u8TxCnt++]);//发送数据
        if(u8TxCnt>1)                                   //如果已发送两个字节
        {
            u8TxCnt = 0;
            u8RxCnt = 0;
            Uart_DisableIrq(M0P_UART0,UartTxIrq);       //禁止发送中断
            Uart_EnableIrq(M0P_UART0,UartRxIrq);        //使能接收中断
        } 
    }

}

/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** This sample
 **
 ******************************************************************************/

int32_t main(void)
{    
    uint8_t u8Idx = 0;

    ///< 系统时钟初始化(8MHz for CanClk)
    App_SysClkInit();
	  ///< 端口初始化
    App_GpioInit();
    ///< SPI初始化
    App_SPIInit(); 
    ///< CAN GPIO 配置
    App_CanGpioInit();
    ///< CAN 初始化配置
    App_CanInit();
	  ///< w5500 初始化配置
    W5500_Init();
	  //UART端口初始化
    App_PortInit();
    //串口模块配置
    App_UartCfg();
	
	
	
	
    ///UART0-RS485
    //发送地址后，触发中断，后续发送数据 
    M0P_UART0->SBUF = (1<<8)|SLAVEADDR;   
 
	
    ///< ETH接收信息
    ///< 片选，开始通讯
    Spi_SetCS(M0P_SPI1, FALSE);
    ///< 主机接收从机数据
    Spi_ReceiveBuf(M0P_SPI1, rx_buf, 10);
    ///< 结束通信
    Spi_SetCS(M0P_SPI1, TRUE);
		///< ETH接收信息
    ///< 片选，开始通讯
    Spi_SetCS(M0P_SPI1, FALSE);
    ///< 主机接收从机数据
    Spi_SendBuf(M0P_SPI1, tx_buf, 10);
    ///< 结束通信
    Spi_SetCS(M0P_SPI1, TRUE);
    
		
		///< SPI0接收信息
    ///< 片选，开始通讯
    Spi_SetCS(M0P_SPI0, FALSE);
    ///< 主机接收从机数据
    Spi_ReceiveBuf(M0P_SPI0, rx_buf, 10);
    ///< 结束通信
    Spi_SetCS(M0P_SPI0, TRUE);
  
    
	
	
    while (1)
		{
			
			
		}
		
}

static void App_SysClkInit(void)
{
    ///< 切换时钟前（根据外部高速晶振）设置XTH频率范围,配置晶振参数，使能目标时钟，此处为8MHz
    Sysctrl_SetXTHFreq(SysctrlXthFreq4_8MHz);
    Sysctrl_XTHDriverCfg(SysctrlXtalDriver1);
    Sysctrl_SetXTHStableTime(SysctrlXthStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTH, TRUE);
    delay1ms(10);

    ///< 时钟切换
    Sysctrl_SysClkSwitch(SysctrlClkXTH);

}

static void App_CanGpioInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

    ///< 端口方向配置->输入
    stcGpioCfg.enDir = GpioDirIn;
    ///< 端口驱动能力配置->高驱动能力
    stcGpioCfg.enDrv = GpioDrvL;
    ///< 端口上下拉配置->无
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    ///< 端口开漏输出配置->开漏输出关闭
    stcGpioCfg.enOD = GpioOdDisable;
    ///< 端口输入/输出值寄存器总线控制模式配置->AHB
    stcGpioCfg.enCtrlMode = GpioAHB;

    Gpio_Init(EVB_CAN_RX_PORT, EVB_CAN_RX_PIN, &stcGpioCfg);
    stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(EVB_CAN_TX_PORT, EVB_CAN_TX_PIN, &stcGpioCfg);
    Gpio_Init(EVB_CAN_STB_PORT, EVB_CAN_STB_PIN, &stcGpioCfg);

    ///<CAN RX\TX复用功能配置
    Gpio_SetAfMode(EVB_CAN_RX_PORT, EVB_CAN_RX_PIN, GpioAf1);
    Gpio_SetAfMode(EVB_CAN_TX_PORT, EVB_CAN_TX_PIN, GpioAf1);

    ///<STB 低-PHY有效
    Gpio_ClrIO(EVB_CAN_STB_PORT, EVB_CAN_STB_PIN);
}

static void App_CanInit(void)
{
    stc_can_init_config_t   stcCanInitCfg;
    stc_can_filter_t        stcFilter;


    Sysctrl_SetPeripheralGate(SysctrlPeripheralCan, TRUE);

    //<<CAN 波特率配置
    stcCanInitCfg.stcCanBt.PRESC = 1-1;
    stcCanInitCfg.stcCanBt.SEG_1 = 5-2;
    stcCanInitCfg.stcCanBt.SEG_2 = 3-1;
    stcCanInitCfg.stcCanBt.SJW   = 3-1;

    stcCanInitCfg.stcWarningLimit.CanErrorWarningLimitVal = 16-1;
    stcCanInitCfg.stcWarningLimit.CanWarningLimitVal = 10;

    stcCanInitCfg.enCanRxBufAll  = CanRxNormal;
    stcCanInitCfg.enCanRxBufMode = CanRxBufNotStored;
    stcCanInitCfg.enCanSTBMode   = CanSTBFifoMode;

    CAN_Init(&stcCanInitCfg);

    //<<CAN 滤波器配置
    stcFilter.enAcfFormat = CanAllFrames;
    stcFilter.enFilterSel = CanFilterSel1;
    stcFilter.u32CODE     = 0x00000352;
    stcFilter.u32MASK     = 0x1FFFFFFF;
    CAN_FilterConfig(&stcFilter, TRUE);


    //<<Can Irq Enable
    CAN_IrqCmd(CanRxIrqEn, TRUE);

    EnableNvic(CAN_IRQn, IrqLevel0, TRUE);

}

/**
 ******************************************************************************
 ** \brief  初始化外部GPIO引脚
 **
 ** \return 无
 ******************************************************************************/
static void App_GpioInit(void)
{
    stc_gpio_cfg_t GpioInitStruct;
    DDL_ZERO_STRUCT(GpioInitStruct);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
    
    ///< SPI0引脚配置:主机
    GpioInitStruct.enDrv = GpioDrvH;
    GpioInitStruct.enDir = GpioDirOut;   

    Gpio_Init(EVB_SPI0_CS_PORT, EVB_SPI0_CS_PIN, &GpioInitStruct);
    Gpio_SetAfMode(EVB_SPI0_CS_PORT, EVB_SPI0_CS_PIN, GpioAf2);             ///<配置SPI0_CS
                                                               
    Gpio_Init(EVB_SPI0_SCK_PORT, EVB_SPI0_SCK_PIN, &GpioInitStruct);            
    Gpio_SetAfMode(EVB_SPI0_SCK_PORT, EVB_SPI0_SCK_PIN, GpioAf2);           ///<配置SPI0_SCK
                                                               
    Gpio_Init(EVB_SPI0_MOSI_PORT, EVB_SPI0_MOSI_PIN, &GpioInitStruct);           
    Gpio_SetAfMode(EVB_SPI0_MOSI_PORT, EVB_SPI0_MOSI_PIN, GpioAf2);         ///<配置SPI0_MOSI
                                                               
    GpioInitStruct.enDir = GpioDirIn;                          
    Gpio_Init(EVB_SPI0_MISO_PORT, EVB_SPI0_MISO_PIN, &GpioInitStruct);            
    Gpio_SetAfMode(EVB_SPI0_MISO_PORT, EVB_SPI0_MISO_PIN, GpioAf2);         ///<配置SPI0_MISO
    
		///< SPI1-ETH引脚配置:主机
    GpioInitStruct.enDrv = GpioDrvH;
    GpioInitStruct.enDir = GpioDirOut;   

    Gpio_Init(EVB_SPI1_CS_PORT, EVB_SPI1_CS_PIN, &GpioInitStruct);
    Gpio_SetAfMode(EVB_SPI1_CS_PORT, EVB_SPI1_CS_PIN, GpioAf2);             ///<配置SPI0_CS
                                                               
    Gpio_Init(EVB_SPI1_SCK_PORT, EVB_SPI1_SCK_PIN, &GpioInitStruct);            
    Gpio_SetAfMode(EVB_SPI1_SCK_PORT, EVB_SPI1_SCK_PIN, GpioAf2);           ///<配置SPI0_SCK
                                                               
    Gpio_Init(EVB_SPI1_MOSI_PORT, EVB_SPI1_MOSI_PIN, &GpioInitStruct);           
    Gpio_SetAfMode(EVB_SPI1_MOSI_PORT, EVB_SPI1_MOSI_PIN, GpioAf2);         ///<配置SPI0_MOSI
                                                               
    GpioInitStruct.enDir = GpioDirIn;                          
    Gpio_Init(EVB_SPI1_MISO_PORT, EVB_SPI1_MISO_PIN, &GpioInitStruct);            
    Gpio_SetAfMode(EVB_SPI1_MISO_PORT, EVB_SPI1_MISO_PIN, GpioAf2);         ///<配置SPI0_MISO
		
    
    ///< 端口方向配置->输入
    GpioInitStruct.enDir = GpioDirIn;
    ///< 端口驱动能力配置->高驱动能力
    GpioInitStruct.enDrv = GpioDrvL;
    ///< 端口上下拉配置->无
    GpioInitStruct.enPu = GpioPuDisable;
    GpioInitStruct.enPd = GpioPdDisable;
    ///< 端口开漏输出配置->开漏输出关闭
    GpioInitStruct.enOD = GpioOdDisable;
    ///< 端口输入/输出值寄存器总线控制模式配置->AHB
    GpioInitStruct.enCtrlMode = GpioAHB;
    ///< GPIO IO USER KEY初始化
    Gpio_Init(EVB_KEY1_PORT, EVB_KEY1_PIN, &GpioInitStruct); 
    
    
    //PD14:板上LED
    GpioInitStruct.enDrv  = GpioDrvH;
    GpioInitStruct.enDir  = GpioDirOut;
    Gpio_Init(EVB_LEDR_PORT, EVB_LEDR_PIN, &GpioInitStruct);
    Gpio_WriteOutputIO(EVB_LEDR_PORT, EVB_LEDR_PIN, FALSE);     //输出高，熄灭LED        
}

/**
 ******************************************************************************
 ** \brief  初始化SPI0,1
 **
 ** \return 无
 ******************************************************************************/
static void App_SPIInit(void)
{
    stc_spi_cfg_t  SpiInitStruct;    
    
    ///< 打开外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralSpi0,TRUE);
  	Sysctrl_SetPeripheralGate(SysctrlPeripheralSpi1,TRUE);
    
    ///<复位模块
    Reset_RstPeripheral0(ResetMskSpi0);
	  Reset_RstPeripheral0(ResetMskSpi1);
    
    //SPI0,1模块配置：主机
    SpiInitStruct.enSpiMode = SpiMskMaster;     //配置位主机模式
    SpiInitStruct.enPclkDiv = SpiClkMskDiv128;    //波特率：PCLK/2
    SpiInitStruct.enCPHA    = SpiMskCphafirst;  //第一边沿采样
    SpiInitStruct.enCPOL    = SpiMskcpollow;    //极性为低
    Spi_Init(M0P_SPI0, &SpiInitStruct);
	  Spi_Init(M0P_SPI1, &SpiInitStruct);
}

void W5500_Init(void) {
    uint8_t temp;
    wiz_NetInfo netInfo = {
        .mac = {0x00, 0x08, 0xdc, 0x01, 0x02, 0x03},
        .ip = {172, 25, 80, 1},
        .sn = {255, 255, 255, 0},
        .gw = {192, 168, 1, 1},
    };
    wizchip_init(txsize, rxsize);
    wizchip_setnetinfo(&netInfo);
    ctlwizchip(CW_GET_PHYLINK, (void*)&temp);
}

void SendData(void) {
    int32_t ret;
    uint8_t data[] = "Hello, W5500!";
    ret = send(0, data, sizeof(data));
    if (ret < 0) {
        // 错误处理
    }
}

void ReceiveData(void) {
    uint8_t buffer[512];
    int32_t ret = recv(0, buffer, sizeof(buffer));
    if (ret > 0) {
        // 成功接收数据，处理接收到的数据
    }
}

//UART0串口模块配置
void App_UartCfg(void)
{
    stc_uart_cfg_t  stcCfg;
    stc_uart_baud_t stcBaud;

    DDL_ZERO_STRUCT(stcCfg);
    DDL_ZERO_STRUCT(stcBaud);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart0,TRUE); ///<使能UART0外设时钟门控开关
    
    ///<UART Init
    stcCfg.enRunMode        = UartMskMode3;                 ///<模式3
    stcCfg.enStopBit        = UartMsk1bit;                  ///<1bit停止位
    stcCfg.enMmdorCk        = UartMskDataOrAddr;            ///<多机模式时
    stcCfg.stcBaud.u32Baud  = 9600;                         ///<波特率9600
    stcCfg.stcBaud.enClkDiv = UartMsk8Or16Div;              ///<通道采样分频配置
    stcCfg.stcBaud.u32Pclk  = Sysctrl_GetPClkFreq();        ///</<获得外设时钟（PCLK）频率值
    Uart_Init(M0P_UART0, &stcCfg);                          ///<串口初始化
    
    ///<UART中断使能
    Uart_ClrStatus(M0P_UART0,UartRC);                       ///<清接收请求
    Uart_ClrStatus(M0P_UART0,UartTC);                       ///<清接收请求
    Uart_EnableIrq(M0P_UART0,UartTxIrq);                    ///<使能串口接收中断    
    EnableNvic(UART1_3_IRQn, IrqLevel3, TRUE);              ///<系统中断使能

}

//串口引脚配置
void App_PortInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    DDL_ZERO_STRUCT(stcGpioCfg); 
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);  ///<使能GPIO外设时钟门控开关
    
    stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPortB,GpioPin6,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortB,GpioPin6,GpioAf1);             ///<配置PB06 为UART0 TX
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortB,GpioPin7,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortB,GpioPin7,GpioAf1);             ///<配置PB07 为UART0 RX
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
