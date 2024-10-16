/******************************************************************************
* Copyright (C) 2019, Xiaohua Semiconductor Co.,Ltd All rights reserved.
*
* This software is owned and published by:
* Xiaohua Semiconductor Co.,Ltd ("XHSC").
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software contains source code for use with XHSC
* components. This software is licensed by XHSC to be adapted only
* for use in systems utilizing XHSC components. XHSC shall not be
* responsible for misuse or illegal use of this software for devices not
* supported herein. XHSC is providing this software "AS IS" and will
* not be responsible for issues arising from incorrect user implementation
* of the software.
*
* Disclaimer:
* XHSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
* REGARDING THE SOFTWARE (INCLUDING ANY ACOOMPANYING WRITTEN MATERIALS),
* ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
* WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
* WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
* WARRANTY OF NONINFRINGEMENT.
* XHSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
* NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
* LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
* LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
* INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
* SAVINGS OR PROFITS,
* EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
* INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
* FROM, THE SOFTWARE.
*
* This software may be replicated in part or whole for the licensed use,
* with the restriction that this Disclaimer and Copyright notice must be
* included with each copy of this software, whether used in part or whole,
* at all times.
*/
/******************************************************************************/
/** \file s25fl132k_spi_flash.h
 **
 ** A detailed description is available at
 ** @link Sample Group Some description @endlink
 **
 **   - 2019-08-07   linsq   First version
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#ifndef S25FL132K_SPI_FLASH_H__ 
#define S25FL132K_SPI_FLASH_H__  

#include "hc32f072.h"
   
//大小定义   
#define SPI_FLASH_PageSize                  4096        
#define SPI_FLASH_PerWritePageSize          256     
   
//指令定义   
#define S25FL132K_WriteEnable       0x06    
#define S25FL132K_WriteDisable      0x04    
#define S25FL132K_ReadStatusReg     0x05    
#define S25FL132K_WriteStatusReg    0x01    
#define S25FL132K_ReadData          0x03    
#define S25FL132K_FastReadData      0x0B    
#define S25FL132K_FastReadDual      0x3B    
#define S25FL132K_PageProgram       0x02    
#define S25FL132K_BlockErase        0xD8    
#define S25FL132K_SectorErase       0x20    
#define S25FL132K_ChipErase         0xC7    
#define S25FL132K_PowerDown         0xB9    
#define S25FL132K_ReleasePowerDown  0xAB    
#define S25FL132K_DeviceID          0xAB    
#define S25FL132K_ManufactDeviceID  0x90    
#define S25FL132K_JedecDeviceID     0x9F      
#define WIP_Flag                    0x01      
#define Dummy_Byte                  0xA5      
#define Flash_DeviceID              0x014016   
   
 
//函数声明
extern void S25FL132K_SPI_FLASH_Init(void); 
extern void S25FL132K_SPI_FLASH_SectorErase(uint32_t SectorAddr); 
extern void S25FL132K_SPI_FLASH_BulkErase(void); 
extern void S25FL132K_SPI_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite); 
extern void S25FL132K_SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite); 
extern void S25FL132K_SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead); 
extern uint32_t S25FL132K_SPI_FLASH_ReadID(void); 
extern uint32_t S25FL132K_SPI_FLASH_ReadDeviceID(void); 
extern void S25FL132K_SPI_FLASH_StartReadSequence(uint32_t ReadAddr); 
extern void S25FL132K_SPI_Flash_PowerDown(void); 
extern void S25FL132K_SPI_Flash_WAKEUP(void);   
extern uint8_t S25FL132K_SPI_FLASH_ReadByte(void); 
extern uint8_t S25FL132K_SPI_FLASH_SendByte(uint8_t byte); 
extern void S25FL132K_SPI_FLASH_WriteEnable(void); 
extern void S25FL132K_SPI_FLASH_WaitForWriteEnd(void); 
 
#endif //S25FL132K_SPI_FLASH_H__ 
 
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
