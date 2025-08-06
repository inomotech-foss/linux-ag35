/**
 * @file        quectel_product_config.h
 * @brief       Quectel PRODUCT MACROs Define.
 * @author      Running.qian
 * @copyright   Copyright (c) 2017-2020 @ Quectel Wireless Solutions Co., Ltd.
 */

#ifndef QL_PRODUCT_CONFIG_H
#define QL_PRODUCT_CONFIG_H

/******************  QUECTEL PRODUCT MACRO, Priority Level: Low1  ***********************************///
//#if QL_G_PRODUCT_EC20C_CFA
/******************* ReDefine FUNCS MACRO  ********************************/
//    #undef    QL_G_FUNC_XXX
//    #define   QL_G_FUNC_XXX     1

/******************* SPECIFIC FUNCS MACRO  ********************************/
//    #define   QL_G_FUNC_YYY
//#endif


#if defined(QL_G_PRODUCT_MODEL_AG35)
/******************* ReDefine FUNCS MACRO  ********************************/

/******************* SPECIFIC FUNCS MACRO  ********************************/

#else //error
//#error "<<<<<<<<<<<<<<<<<<Undefine PRODUCT NAME>>>>>>>>>>>>>>>>>>>>>>>>>>>"
#endif

#endif //#ifdef  QL_G_PRODUCT_CONFIG_H
