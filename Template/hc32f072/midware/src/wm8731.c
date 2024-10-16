/*******************************************************************************
 * Copyright (C) 2016, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software is owned and published by:
 * Xiaohua Semiconductor Co., Ltd. ("XHSC").
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
/** \file wm8731.c
 **
 ** \brief I2S codec WM8731 driver API functions
 **
 **   - 2019-08-13 Linsq   First version
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "wm8731.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/


/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 ******************************************************************************
    ** \brief  通过I2C1发送2字节数据
    ** \param  i2cx: M0P_I2C0或者M0P_I2C1
    ** \param  buf：所要发送的数据缓存
    ** \retval len：发送的数据字节数
    **
 ******************************************************************************/
void I2C_WriteWm8731R(M0P_I2C_TypeDef *i2cx, uint8_t *buf, uint8_t len)
{
	uint8_t u8State = 0;
	uint8_t tmp=0, tmp_1;
    I2C_SetFunc(i2cx,I2cStart_En);
	while(1)
	{
		while(1!=i2cx->CR_f.SI)
        {}
        u8State = I2C_GetState(i2cx);
		switch(u8State)
		{
			case 0x08:
                I2C_ClearFunc(i2cx,I2cStart_En);
                I2C_WriteByte(i2cx,I2c_SlaveWt);
				break;
			case 0x10:
                I2C_ClearFunc(i2cx,I2cStart_En);
                I2C_WriteByte(i2cx,I2c_SlaveWt);
				break;
			case 0x18:
				tmp_1=buf[tmp++];
                I2C_ClearFunc(i2cx,I2cStart_En);
                I2C_WriteByte(i2cx,tmp_1);
				break;
			case 0x20:
			case 0x38:
			case 0x30:
			case 0x48:
                I2C_SetFunc(i2cx,I2cStart_En);
                I2C_SetFunc(i2cx,I2cStop_En);
			break;
			case 0x28:
				if(tmp>=len)//如果已经发送完2字节，则停止I2C
				{
                    I2C_SetFunc(i2cx,I2cStop_En);
                    I2C_ClearIrq(i2cx);
					return;
				}
                I2C_WriteByte(i2cx,buf[tmp++]);//发送寄存器内容
				break;
		}
	 	I2C_ClearIrq(i2cx);
	}
}

/**
 ******************************************************************************
    * @brief  通过I2Cx初始化WM8731，主要配置内部的各个寄存器
    * \param  i2cx: M0P_I2C0或者M0P_I2C1
    * @param  pstcReg: 用于初始化寄存器的结构体
    * @retval None
    **
 ******************************************************************************/
void WM8731_Init(M0P_I2C_TypeDef *i2cx, stc_wm8731_reg_t* pstcReg)
{
	uint8_t buf[2];
	uint8_t tmp;

	//Reset Register 的提取和配置
	tmp = (uint8_t)((pstcReg->RESET>>8) & 0x01);
	buf[0] = (WM8731_REG_RESET<<1) & (~1);
	buf[0] = buf[0] | tmp;
	buf[1] = (uint8_t)(pstcReg->RESET & 0xff);
	I2C_WriteWm8731R(i2cx, buf, 2);          //配置Reset Register寄存器
	//Left Line In 的提取和配置
	tmp = (uint8_t)((pstcReg->LLIN>>8) & 0x01);
	buf[0] = (WM8731_REG_LLINE_IN<<1) & (~1);
	buf[0] = buf[0] | tmp;
	buf[1] = (uint8_t)(pstcReg->LLIN & 0xff);
	I2C_WriteWm8731R(i2cx, buf, 2);          //配置Left Line In寄存器
	//Right Line In的提取和配置
	tmp = (uint8_t)((pstcReg->RLIN>>8) & 0x01);
	buf[0] = (WM8731_REG_RLINE_IN<<1) & (~1);
	buf[0] = buf[0] | tmp;
	buf[1] = (uint8_t)(pstcReg->RLIN & 0xff);
	I2C_WriteWm8731R(i2cx, buf, 2);          //配置Right Line In寄存器
	//Left Headphone Out 的提取和配置
	tmp = (uint8_t)((pstcReg->LHOUT>>8) & 0x01);
	buf[0] = (WM8731_REG_LHPHONE_OUT<<1) & (~1);
	buf[0] = buf[0] | tmp;
	buf[1] = (uint8_t)(pstcReg->LHOUT & 0xff);
	I2C_WriteWm8731R(i2cx, buf, 2);          //配置Left Headphone Out寄存器
	//Right Headphone Out 的提取和配置
	tmp = (uint8_t)((pstcReg->RHOUT>>8) & 0x01);
	buf[0] = (WM8731_REG_RHPHONE_OUT<<1) & (~1);
	buf[0] = buf[0] | tmp;
	buf[1] = (uint8_t)(pstcReg->RHOUT & 0xff);
	I2C_WriteWm8731R(i2cx, buf, 2);          //配置Right Headphone Out 寄存器
	//Analogue Audio Path Control的提取和配置
	tmp = (uint8_t)((pstcReg->AAPC>>8) & 0x01);
	buf[0] = (WM8731_REG_ANALOG_PATH<<1) & (~1);
	buf[0] = buf[0] | tmp;
	buf[1] = (uint8_t)(pstcReg->AAPC & 0xff);
	I2C_WriteWm8731R(i2cx, buf, 2);          //配置Analogue Audio Path Control寄存器
	//Digital Audio Path Control 的提取和配置
	tmp = (uint8_t)((pstcReg->DAPC>>8) & 0x01);
	buf[0] = (WM8731_REG_DIGITAL_PATH<<1) & (~1);
	buf[0] = buf[0] | tmp;
	buf[1] = (uint8_t)(pstcReg->DAPC & 0xff);
	I2C_WriteWm8731R(i2cx, buf, 2);          //配置Digital Audio Path Control寄存器
	//Power Down Control 的提取和配置
	tmp = (uint8_t)((pstcReg->PDC>>8) & 0x01);
	buf[0] = (WM8731_REG_PDOWN_CTRL<<1) & (~1);
	buf[0] = buf[0] | tmp;
	buf[1] = (uint8_t)(pstcReg->PDC & 0xff);
	I2C_WriteWm8731R(i2cx, buf, 2);          //配置Power Down Control寄存器
	//Digital Audio Interface Format 的提取和配置
	tmp = (uint8_t)((pstcReg->DAIF>>8) & 0x01);
	buf[0] = (WM8731_REG_DIGITAL_IF<<1) & (~1);
	buf[0] = buf[0] | tmp;
	buf[1] = (uint8_t)(pstcReg->DAIF & 0xff);
	I2C_WriteWm8731R(i2cx, buf, 2);           //配置Digital Audio Interface Format寄存器
	//Sampling Control 的提取和配置
	tmp = (uint8_t)((pstcReg->SC>>8) & 0x01);
	buf[0] = (WM8731_REG_SAMPLING_CTRL<<1) & (~1);
	buf[0] = buf[0] | tmp;
	buf[1] = (uint8_t)(pstcReg->SC & 0xff);
	I2C_WriteWm8731R(i2cx, buf, 2);           //配置Sampling Control 寄存器
	//Active Control 的提取和配置
	tmp = (uint8_t)((pstcReg->AC>>8) & 0x01);
	buf[0] = (WM8731_REG_ACTIVE_CTRL<<1) & (~1);
	buf[0] = buf[0] | tmp;
	buf[1] = (uint8_t)(pstcReg->AC & 0xff);
	I2C_WriteWm8731R(i2cx, buf, 2);           //配置Active Control 寄存器
}

/**
 ******************************************************************************
    * @brief  通过I2Cx调整左右耳机通道的输出音量，主要配置Left Headphone Out和Right Headphone Out寄存器
    * \param  i2cx: M0P_I2C0或者M0P_I2C1
    * @param  pstcReg: 用于初始化寄存器的结构体
    * @param  lhpvol : 左耳机输出通道的音量，范围位0-2^7
    * @param  rhpvol : 右耳机输出通道的音量，范围位0-2^7
    * @retval None
    **
 ******************************************************************************/
void WM8731_HPhoneVolume(M0P_I2C_TypeDef *i2cx, stc_wm8731_reg_t* pstcReg, uint8_t lhpvol, uint8_t rhpvol)
{
	uint8_t buf[2];
	uint8_t tmp;

	pstcReg->LHOUT_f.LHPVOL=lhpvol;
	tmp = (uint8_t)((pstcReg->LHOUT>>8) & 0x01);
	buf[0] = (WM8731_REG_LHPHONE_OUT<<1) & (~1);
	buf[0] = buf[0] | tmp;
	buf[1] = (uint8_t)(pstcReg->LHOUT & 0xff);
	I2C_WriteWm8731R(i2cx, buf, 2);

	pstcReg->RHOUT_f.RHPVOL=rhpvol;
	tmp = (uint8_t)((pstcReg->RHOUT>>8) & 0x01);
	buf[0] = (WM8731_REG_RHPHONE_OUT<<1) & (~1);
	buf[0] = buf[0] | tmp;
	buf[1] = (uint8_t)(pstcReg->RHOUT & 0xff);
	I2C_WriteWm8731R(i2cx, buf, 2);
}
/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
