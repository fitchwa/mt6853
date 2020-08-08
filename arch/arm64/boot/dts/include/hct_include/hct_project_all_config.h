
#ifndef __HCT_PROJECT_ALL_CONFIG_H__
#define __HCT_PROJECT_ALL_CONFIG_H__

#include "hct_common_config.h"
#include "hct_board_config.h"
#include "hct_custom_config.h"



//Jay: when you add new macro for project , you need defined default macro valut here, so you need not defined in code logic
//Next is Begin :::::::::



///////////////////
/////  next pls do not modify
////////////////////
#ifndef PINMUX_GPIO1__FUNC_GPIO1

	#if defined(CONFIG_MACH_MT6873)
	#include "mt6873-pinfunc.h"
	#endif
	#if defined(CONFIG_MACH_MT6853)
	#include "mt6853-pinfunc.h"
	#endif

#endif

#include "hct_board_dts_config.h"


#ifndef __HCT_GPIO_LCM_POWER_ENN_PINMUX__
#define __HCT_GPIO_LCM_POWER_ENN_PINMUX__ PUNMUX_GPIO_NONE_FUNC_NONE
#endif

#ifndef __HCT_GPIO_LCM_POWER_ENP_PINMUX__
#define __HCT_GPIO_LCM_POWER_ENP_PINMUX__ PUNMUX_GPIO_NONE_FUNC_NONE
#endif

#ifndef __HCT_GPIO_LCM_POWER_DM_PINMUX__
#define __HCT_GPIO_LCM_POWER_DM_PINMUX__ PUNMUX_GPIO_NONE_FUNC_NONE
#endif

#ifndef __HCT_GPIO_LCM_POWER_DP_PINMUX__
#define __HCT_GPIO_LCM_POWER_DP_PINMUX__ PUNMUX_GPIO_NONE_FUNC_NONE
#endif

#ifndef __HCT_GPIO_CTP_RST_PIN__
#define __HCT_GPIO_CTP_RST_PIN__ PUNMUX_GPIO_NONE_FUNC_NONE
#endif

#ifndef __HCT_DUAL_CAMERA_USEDBY_YUV_MODE__
#define __HCT_DUAL_CAMERA_USEDBY_YUV_MODE__  HCT_NO
#endif

#ifndef __HCT_DUAL_SUB_CAMERA__
#define __HCT_DUAL_SUB_CAMERA__  HCT_NO
#endif

#ifndef __HCT_MAIN_FAKE_I2C_USE_SUB__
#define __HCT_MAIN_FAKE_I2C_USE_SUB__  HCT_NO
#endif

#ifndef __HCT_MAIN_FAKE_I2C_USE_SUB2__
#define __HCT_MAIN_FAKE_I2C_USE_SUB2__  HCT_NO
#endif

#ifndef __HCT_SUB_FAKE_I2C_USE_MAIN__
#define __HCT_SUB_FAKE_I2C_USE_MAIN__  HCT_NO
#endif

#ifndef __HCT_FAKE_CAMERA_USE__MCLK3__
#define __HCT_FAKE_CAMERA_USE__MCLK3__ HCT_NO
#endif

#endif
