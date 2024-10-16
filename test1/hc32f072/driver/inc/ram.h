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
 * @file   ram.h
 *
 * @brief  Header file for RAM functions
 *
 * @author MADS Team 
 *
 ******************************************************************************/

#ifndef __RAM_H__
#define __RAM_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "ddl.h"

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup RamGroup Ram Controller (Ram)
 **
 **
 ******************************************************************************/
//@{

/******************************************************************************
 * Global type definitions
 ******************************************************************************/

/**
 ******************************************************************************
 ** \brief Redefinition of RAM register structure 
 ******************************************************************************/

/******************************************************************************
 * Global definitions
 ******************************************************************************/

/******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions ('extern')                                     
 ******************************************************************************/

/******************************************************************************
 * Global function prototypes (definition in C source)                        
 ******************************************************************************/


///<中断相关函数
///<中断使能/禁止
void Ram_EnableIrq(void);
void Ram_DisableIrq(void);
///<中断标志获取
boolean_t Ram_GetIntFlag(void);
///<中断标志清除
void Ram_ClearIntFlag(void);
///<奇偶校验出错地址获取
uint32_t Ram_ErrAddrGet(void);

//@} // RamGroup

#ifdef __cplusplus
}
#endif

#endif /* __RAM_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
