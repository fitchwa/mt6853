#ifndef __SF_HCT_H__
#define __SF_HCT_H__

#define HCT_DRIVER

extern int hct_finger_probe_isok;//add for hct finger jianrong

/*0:power off 1:power on*/
extern int hct_finger_set_power(int cmd);

/*0:power off 1:power on*/
extern int hct_finger_set_18v_power(int cmd);

/*0:output low 1:output high*/
extern int hct_finger_set_reset(int cmd);

//not used
extern int hct_finger_set_irq(int cmd);

// cmd = 1
//extern int hct_finger_set_eint(int cmd);

/*0:gpio mode 1:spi mode*/
extern int hct_finger_set_spi_mode(int cmd);

#endif
