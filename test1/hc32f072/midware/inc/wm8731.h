/*****************************************************************************
 * Copyright (C) 2016, Xiaohua Semiconductor Co.,Ltd All rights reserved.
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
/** \file wm8731.h
 **
 ** A detailed description is available at
 ** @link Wm8731Group WM8731 description @endlink
 **
 **   - 2019-08-13 Linsq   First version
 **
 ******************************************************************************/
#ifndef __WM8731_H__
#define __WM8731_H__

#include "hc32f072.h"
#include "i2c.h"
/*******************************************************************************
 * Include files
 ******************************************************************************/

/**
 *******************************************************************************
 ** \defgroup Wm8731Group Wm8731 CODECs with an integrated headphone driver
 **
 ******************************************************************************/


/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/*****************************************************************************/
/* Global pre-processor symbols/macros ('define')                            */
/*****************************************************************************/
#define I2c_SlaveWt                (0x34)       // WM8731 chip address on I2C bus

#define WM8731_REG_LLINE_IN        (0x00)       // Left Channel Line Input Volume Control
#define WM8731_REG_RLINE_IN        (0x01)       // Right Channel Line Input Volume Control
#define WM8731_REG_LHPHONE_OUT     (0x02)       // Left Channel Headphone Output Volume Control
#define WM8731_REG_RHPHONE_OUT     (0x03)       // Right Channel Headphone Output Volume Control
#define WM8731_REG_ANALOG_PATH     (0x04)       // Analog Audio Path Control
#define WM8731_REG_DIGITAL_PATH    (0x05)       // Digital Audio Path Control
#define WM8731_REG_PDOWN_CTRL      (0x06)       // Power Down Control Register
#define WM8731_REG_DIGITAL_IF      (0x07)       // Digital Audio Interface Format
#define WM8731_REG_SAMPLING_CTRL   (0x08)       // Sampling Control Register
#define WM8731_REG_ACTIVE_CTRL     (0x09)       // Active Control
#define WM8731_REG_RESET           (0x0F)       // Reset register

/******************************************************************************
 * WM8731 ¼Ä´æÆ÷Î»Óò¶¨Òå
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief ¼Ä´æÆ÷£ºLeft Line In  µØÖ·£º0
 ******************************************************************************/
typedef struct  
{
    uint16_t LINVOL   : 5;
    uint16_t RESERVED : 2;
    uint16_t LINMUTE  : 1;
    uint16_t LRINBOTH : 1;

}stc_wm8731_llin_field_t;

/**
 *******************************************************************************
 ** \brief ¼Ä´æÆ÷£ºRight Line In  µØÖ·£º1
 ******************************************************************************/
typedef struct  
{
    uint16_t RINVOL   : 5;
    uint16_t RESERVED : 2;
    uint16_t RINMUTE  : 1;
    uint16_t RINBOTH  : 1;

}stc_wm8731_rlin_field_t;

/**
 *******************************************************************************
 ** \brief ¼Ä´æÆ÷£ºLeft Headphone Out   µØÖ·£º2
 ******************************************************************************/
typedef struct  
{
    uint16_t LHPVOL   : 7;
    uint16_t LZCEN    : 1;
    uint16_t LRHPBOTH : 1;

}stc_wm8731_lhout_field_t;

/**
 *******************************************************************************
 ** \brief ¼Ä´æÆ÷£ºRight Headphone Out  µØÖ·£º3
 ******************************************************************************/
typedef struct  
{
    uint16_t RHPVOL   : 7;
    uint16_t RZCEN    : 1;
    uint16_t RLHPBOTH : 1;

}stc_wm8731_rhout_field_t;

/**
 *******************************************************************************
 ** \brief ¼Ä´æÆ÷£ºAnalogue Audio Path Control   µØÖ·£º4
 ******************************************************************************/
typedef struct  
{
    uint16_t MICBOOST   : 1;
    uint16_t MUTEMIC    : 1;
    uint16_t INSEL      : 1;
    uint16_t BYPASS     : 1;
    uint16_t DACSEL     : 1;
    uint16_t SIDETONE   : 1;
    uint16_t SIDEATT    : 2;

}stc_wm8731_aapc_field_t;

/**
 *******************************************************************************
 ** \brief ¼Ä´æÆ÷£ºDigital Audio Path Control   µØÖ·£º5
 ******************************************************************************/
typedef struct  
{
    uint16_t ADCHPD   : 1;
    uint16_t DEEMP    : 2;
    uint16_t DACMU    : 1;
    uint16_t HPOR     : 1;

}stc_wm8731_dapc_field_t;

/**
 *******************************************************************************
 ** \brief ¼Ä´æÆ÷£ºPower Down Control   µØÖ·£º6
 ******************************************************************************/
typedef struct  
{
    uint16_t LINEINPD   : 1;
    uint16_t MICPD      : 1;
    uint16_t ADCPD      : 1;
    uint16_t DACPD      : 1;
    uint16_t OUTPD      : 1;
    uint16_t OSCPD      : 1;
    uint16_t CLKOUTPD   : 1;
    uint16_t POWEROFF   : 1;

}stc_wm8731_pdc_field_t;

/**
 *******************************************************************************
 ** \brief ¼Ä´æÆ÷£ºDigital Audio Interface Format  µØÖ·£º7
 ******************************************************************************/
typedef struct  
{
    uint16_t FORMAT     : 2;
    uint16_t IWL        : 2;
    uint16_t LRP        : 1;
    uint16_t LRSWAP     : 1;
    uint16_t MS         : 1;
    uint16_t BCLKINV    : 1;

}stc_wm8731_daif_field_t;

/**
 *******************************************************************************
 ** \brief ¼Ä´æÆ÷£ºSampling Control   µØÖ·£º8
 ******************************************************************************/
typedef struct  
{
    uint16_t NORMAL_USB : 1;
    uint16_t BOSR       : 1;
    uint16_t SR         : 4;
    uint16_t CLKDIV2    : 1;
    uint16_t CLKODIV2   : 1;
}stc_wm8731_sc_field_t;

/**
 *******************************************************************************
 ** \brief ¼Ä´æÆ÷£ºActive Control     µØÖ·£º9
 ******************************************************************************/
typedef struct  
{
    uint16_t ACTIVE : 1;

}stc_wm8731_ac_field_t;

/******************************************************************************
 * WM8731 ¿ØÖÆ¼Ä´æÆ÷½á¹¹Ìå»úÆ÷Ïà¹Ø¼Ä´æÆ÷µÄÎ»Óò¶¨Òå
 ******************************************************************************/
typedef struct 
{
  union {
    __IO  uint16_t LLIN;                    // ¼Ä´æÆ÷£ºLeft line in register
    stc_wm8731_llin_field_t LLIN_f;
  };

  union {
    __IO  uint16_t RLIN;                   // ¼Ä´æÆ÷£ºRight line in register
    stc_wm8731_rlin_field_t RLIN_f;
  };

  union {
    __IO  uint16_t LHOUT;                  // ¼Ä´æÆ÷£ºLeft headphone out register
    stc_wm8731_lhout_field_t LHOUT_f;
  };

  union {
    __IO  uint16_t RHOUT;                  // ¼Ä´æÆ÷£ºRight headphone out register
    stc_wm8731_rhout_field_t RHOUT_f;
  };

  union {
    __IO  uint16_t AAPC;                   // ¼Ä´æÆ÷£ºAnalog audio path control register
    stc_wm8731_aapc_field_t AAPC_f;
  };

  union {
    __IO  uint16_t DAPC;                   // ¼Ä´æÆ÷£ºDigital audio path control register
    stc_wm8731_dapc_field_t DAPC_f;
  };

  union {
    __IO  uint16_t PDC;                    // ¼Ä´æÆ÷£ºPower down control register
    stc_wm8731_pdc_field_t PDC_f;
  };

  union {
    __IO  uint16_t DAIF;                   // ¼Ä´æÆ÷£ºDigital audio interface format register
    stc_wm8731_daif_field_t DAIF_f;
  };

  union {
    __IO  uint16_t SC;                     // ¼Ä´æÆ÷£ºSampling control register
    stc_wm8731_sc_field_t SC_f;
  };

  union {
    __IO  uint16_t AC;                     // ¼Ä´æÆ÷£ºActive control register
    stc_wm8731_ac_field_t AC_f;
  };

  __IO  uint16_t RESET;                   // ¼Ä´æÆ÷£ºReset register

}stc_wm8731_reg_t;


/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
 * Global function prototypes (definition in C source)
 ******************************************************************************/
extern void WM8731_Init(M0P_I2C_TypeDef *i2cx, stc_wm8731_reg_t* pstcReg);
extern void WM8731_HPhoneVolume(M0P_I2C_TypeDef *i2cx, stc_wm8731_reg_t* pstcReg, uint8_t lhpvol, uint8_t rhpvol);
extern void I2C_WriteWm8731R(M0P_I2C_TypeDef *i2cx, uint8_t *buf, uint8_t len);

#endif /* __WM8731_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
