/*
 * Copyright (C) 2018 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/cpu.h>

#include <mt-plat/charger_class.h>
#include "inc/tcpci.h"
#include "inc/tcpci_typec.h"

#ifdef CONFIG_USB_POWER_DELIVERY
#include "inc/tcpci_event.h"
#endif /* CONFIG_USB_POWER_DELIVERY */

#ifdef CONFIG_DUAL_ROLE_USB_INTF
#include <linux/usb/class-dual-role.h>
#endif /* CONFIG_DUAL_ROLE_USB_INTF */
#include <linux/hct_include/hct_project_all_config.h>
/*
 * [BLOCK] TCPCI IRQ Handler
 */
#if __HCT_TYPEC_ACCDET_MIC_SUPPORT__
	#include <linux/interrupt.h>
	#define EINT_PIN_PLUG_IN        (1)
	#define EINT_PIN_PLUG_OUT       (0)
	int accdet_typec_state = EINT_PIN_PLUG_OUT;

	extern int pmic_eint_queue_work(int eintID);
	extern int typec_recheck_flag;
#ifdef __HCT_WIRELESS_CHARGER_WHEN_TPC_IN_SUPPORT__
#if CONFIG_MTK_GAUGE_VERSION == 30
	static struct charger_device *primary_charger;
	extern int charger_get_vbus(void);	
#endif
#endif

#endif
static int tcpci_alert_cc_changed(struct tcpc_device *tcpc_dev)
{
	return tcpc_typec_handle_cc_change(tcpc_dev);
}

#ifdef CONFIG_TCPC_VSAFE0V_DETECT_IC

static inline int tcpci_alert_vsafe0v(struct tcpc_device *tcpc_dev)
{
	tcpc_typec_handle_vsafe0v(tcpc_dev);

#ifdef CONFIG_USB_POWER_DELIVERY
#ifdef CONFIG_USB_PD_SAFE0V_DELAY
	tcpc_enable_timer(tcpc_dev, PD_TIMER_VSAFE0V_DELAY);
#else
	pd_put_vbus_safe0v_event(tcpc_dev);
#endif	/* CONFIG_USB_PD_SAFE0V_DELAY */
#endif	/* CONFIG_USB_POWER_DELIVERY */

	return 0;
}

#endif	/* CONFIG_TCPC_VSAFE0V_DETECT_IC */

static inline void __tcpci_vbus_level_refresh(struct tcpc_device *tcpc_dev)
{
	tcpc_dev->vbus_level = tcpc_dev->vbus_present ? TCPC_VBUS_VALID :
			       TCPC_VBUS_INVALID;

#ifdef CONFIG_TCPC_VSAFE0V_DETECT_IC
	if (tcpc_dev->vbus_safe0v) {
		if (tcpc_dev->vbus_level == TCPC_VBUS_INVALID)
			tcpc_dev->vbus_level = TCPC_VBUS_SAFE0V;
		else
			TCPC_INFO("ps_confused: %d\r\n", tcpc_dev->vbus_level);
	}
#endif	/* CONFIG_TCPC_VSAFE0V_DETECT_IC */
}

static inline void tcpci_vbus_level_refresh(struct tcpc_device *tcpc_dev)
{
	mutex_lock(&tcpc_dev->access_lock);
	__tcpci_vbus_level_refresh(tcpc_dev);
	mutex_unlock(&tcpc_dev->access_lock);
}

void tcpci_vbus_level_init(struct tcpc_device *tcpc_dev, uint16_t status)
{
	mutex_lock(&tcpc_dev->access_lock);

	tcpc_dev->vbus_present = status & TCPC_REG_POWER_STATUS_VBUS_PRES ?
				 true : false;
	__tcpci_vbus_level_refresh(tcpc_dev);
	mutex_unlock(&tcpc_dev->access_lock);
}

static inline int tcpci_vbus_level_changed(struct tcpc_device *tcpc_dev)
{
	int rv = 0;
	bool show_msg = true;

#ifdef CONFIG_USB_PD_DIRECT_CHARGE
	if (tcpc_dev->pd_during_direct_charge && tcpc_dev->vbus_level != 0)
		show_msg = false;
#endif	/* CONFIG_USB_PD_DIRECT_CHARGE */

	if (show_msg)
		TCPC_INFO("ps_change=%d\r\n", tcpc_dev->vbus_level);

	rv = tcpc_typec_handle_ps_change(tcpc_dev, tcpc_dev->vbus_level);
	if (rv < 0)
		return rv;

#ifdef CONFIG_USB_POWER_DELIVERY
	pd_put_vbus_changed_event(tcpc_dev, true);
#endif /* CONFIG_USB_POWER_DELIVERY */

#ifdef CONFIG_TCPC_VSAFE0V_DETECT_IC
	if (tcpc_dev->vbus_level == TCPC_VBUS_SAFE0V)
		rv = tcpci_alert_vsafe0v(tcpc_dev);
#endif	/* CONFIG_TCPC_VSAFE0V_DETECT_IC */
	return rv;
}

#ifdef __HCT_WIRELESS_CHARGER_WHEN_TPC_IN_SUPPORT__
extern bool usbc_otg_state_get(void);
#endif

static int tcpci_alert_power_status_changed(struct tcpc_device *tcpc_dev)
{
	int rv = 0;
	uint16_t status = 0;
#ifdef __HCT_WIRELESS_CHARGER_WHEN_TPC_IN_SUPPORT__
	bool otg_state;
#endif
	rv = tcpci_get_power_status(tcpc_dev, &status);
	if (rv < 0)
		return rv;

	tcpc_dev->vbus_present = (status & TCPC_REG_POWER_STATUS_VBUS_PRES) ?
				 true : false;
#ifdef __HCT_WIRELESS_CHARGER_WHEN_TPC_IN_SUPPORT__
/* after plug in/out headset many times with back clamp charger in,
 * it shows charging logo while plug out back clamp charger.
 * Because pwr_rdy is not changed in __mt6370_chgdet_handler func.
 * So,we redetect the charger while vbus changed.
 * lijia add 20181120.
 */
	msleep(100);
	otg_state = usbc_otg_state_get();
	TCPC_INFO("ps_change_1=%d, otg_state=%d\r\n", tcpc_dev->vbus_level, otg_state);
	if ((tcpc_dev->vbus_level != TCPC_VBUS_VALID) && (otg_state != true)){
		charger_dev_enable_chg_type_det(primary_charger, false);
		TCPC_INFO("hct--charger_dev_enable_chg_type_det false.\n");
	}
	if ((tcpc_dev->vbus_level == TCPC_VBUS_VALID) && ( otg_state != true)){
		charger_dev_enable_chg_type_det(primary_charger, true);
		TCPC_INFO("hct--charger_dev_enable_chg_type_det true.\n");
	}
#endif
	return rv;
}

#ifdef CONFIG_USB_POWER_DELIVERY
static int tcpci_alert_tx_success(struct tcpc_device *tcpc_dev)
{
	uint8_t tx_state;

	struct pd_event evt = {
		.event_type = PD_EVT_CTRL_MSG,
		.msg = PD_CTRL_GOOD_CRC,
		.pd_msg = NULL,
	};

	mutex_lock(&tcpc_dev->access_lock);
	tx_state = tcpc_dev->pd_transmit_state;
	tcpc_dev->pd_transmit_state = PD_TX_STATE_GOOD_CRC;
	mutex_unlock(&tcpc_dev->access_lock);

	if (tx_state == PD_TX_STATE_WAIT_CRC_VDM)
		pd_put_vdm_event(tcpc_dev, &evt, false);
	else
		pd_put_event(tcpc_dev, &evt, false);

	return 0;
}

static int tcpci_alert_tx_failed(struct tcpc_device *tcpc_dev)
{
	uint8_t tx_state;

	mutex_lock(&tcpc_dev->access_lock);
	tx_state = tcpc_dev->pd_transmit_state;
	tcpc_dev->pd_transmit_state = PD_TX_STATE_NO_GOOD_CRC;
	mutex_unlock(&tcpc_dev->access_lock);

	if (tx_state == PD_TX_STATE_WAIT_CRC_VDM)
		vdm_put_hw_event(tcpc_dev, PD_HW_TX_FAILED);
	else
		pd_put_hw_event(tcpc_dev, PD_HW_TX_FAILED);

	return 0;
}

static int tcpci_alert_tx_discard(struct tcpc_device *tcpc_dev)
{
	uint8_t tx_state;
	bool retry_crc_discard = false;

	mutex_lock(&tcpc_dev->access_lock);
	tx_state = tcpc_dev->pd_transmit_state;
	tcpc_dev->pd_transmit_state = PD_TX_STATE_DISCARD;
	mutex_unlock(&tcpc_dev->access_lock);

	TCPC_INFO("Discard\r\n");

	if (tx_state == PD_TX_STATE_WAIT_CRC_VDM)
		pd_put_last_vdm_event(tcpc_dev);
	else {
		retry_crc_discard =
			(tcpc_dev->tcpc_flags &
					TCPC_FLAGS_RETRY_CRC_DISCARD) != 0;

		if (retry_crc_discard) {
#ifdef CONFIG_USB_PD_RETRY_CRC_DISCARD
			tcpc_dev->pd_discard_pending = true;
			tcpc_enable_timer(tcpc_dev, PD_TIMER_DISCARD);
#else
			TCPC_ERR("RETRY_CRC_DISCARD\r\n");
#endif	/* CONFIG_USB_PD_RETRY_CRC_DISCARD */
		} else {
			pd_put_hw_event(tcpc_dev, PD_HW_TX_FAILED);
		}
	}
	return 0;
}

static int tcpci_alert_recv_msg(struct tcpc_device *tcpc_dev)
{
	int retval;
	struct pd_msg *pd_msg;
	enum tcpm_transmit_type type;

	const uint32_t alert_rx =
		TCPC_REG_ALERT_RX_STATUS | TCPC_REG_ALERT_RX_BUF_OVF;

	pd_msg = pd_alloc_msg(tcpc_dev);
	if (pd_msg == NULL) {
		tcpci_alert_status_clear(tcpc_dev, alert_rx);
		return -EINVAL;
	}

	retval = tcpci_get_message(tcpc_dev,
		pd_msg->payload, &pd_msg->msg_hdr, &type);
	if (retval < 0) {
		TCPC_INFO("recv_msg failed: %d\r\n", retval);
		pd_free_msg(tcpc_dev, pd_msg);
		return retval;
	}

	pd_msg->frame_type = (uint8_t) type;
	pd_put_pd_msg_event(tcpc_dev, pd_msg);
	return 0;
}

static int tcpci_alert_rx_overflow(struct tcpc_device *tcpc_dev)
{
	int rv;
	uint32_t alert_status;

	TCPC_INFO("RX_OVERFLOW\r\n");

	rv = tcpci_get_alert_status(tcpc_dev, &alert_status);
	if (rv)
		return rv;

	if (alert_status & TCPC_REG_ALERT_RX_STATUS)
		return tcpci_alert_recv_msg(tcpc_dev);

	return 0;
}

static int tcpci_alert_recv_hard_reset(struct tcpc_device *tcpc_dev)
{
	TCPC_INFO("HardResetAlert\r\n");
	pd_put_recv_hard_reset_event(tcpc_dev);
	tcpci_init_alert_mask(tcpc_dev);
	return 0;
}

#endif /* CONFIG_USB_POWER_DELIVERY */

static int tcpci_alert_vendor_defined(struct tcpc_device *tcpc_dev)
{
	tcpci_alert_vendor_defined_handler(tcpc_dev);
	return 0;
}

static int tcpci_alert_fault(struct tcpc_device *tcpc_dev)
{
	uint8_t status = 0;

	tcpci_get_fault_status(tcpc_dev, &status);
	TCPC_INFO("FaultAlert=0x%x\r\n", status);
	tcpci_fault_status_clear(tcpc_dev, status);
	return 0;
}

#if 0 /* alert_v10 */
#ifdef CONFIG_TYPEC_CAP_LPM_WAKEUP_WATCHDOG
static int tcpci_alert_wakeup(struct tcpc_device *tcpc_dev)
{
	if (tcpc_dev->tcpc_flags & TCPC_FLAGS_LPM_WAKEUP_WATCHDOG) {
		TCPC_INFO("Wakeup\r\n");

		if (tcpc_dev->typec_remote_cc[0] == TYPEC_CC_DRP_TOGGLING)
			tcpc_enable_wakeup_timer(tcpc_dev, true);
	}

	return 0;
}
#endif /* CONFIG_TYPEC_CAP_LPM_WAKEUP_WATCHDOG */

#ifdef CONFIG_TYPEC_CAP_RA_DETACH
static int tcpci_alert_ra_detach(struct tcpc_device *tcpc_dev)
{
	if (tcpc_dev->tcpc_flags & TCPC_FLAGS_CHECK_RA_DETACHE) {
		TCPC_DBG("RA_DETACH\r\n");

		if (tcpc_dev->typec_remote_cc[0] == TYPEC_CC_DRP_TOGGLING)
			tcpc_typec_enter_lpm_again(tcpc_dev);
	}

	return 0;
}
#endif /* CONFIG_TYPEC_CAP_RA_DETACH */
#endif

struct tcpci_alert_handler {
	uint32_t bit_mask;
	int (*handler)(struct tcpc_device *tcpc_dev);
};

#define DECL_TCPCI_ALERT_HANDLER(xbit, xhandler) {\
		.bit_mask = 1 << xbit,\
		.handler = xhandler, \
	}

static const struct tcpci_alert_handler tcpci_alert_handlers[] = {
	DECL_TCPCI_ALERT_HANDLER(15, tcpci_alert_vendor_defined),
#ifdef CONFIG_USB_POWER_DELIVERY
	DECL_TCPCI_ALERT_HANDLER(4, tcpci_alert_tx_failed),
	DECL_TCPCI_ALERT_HANDLER(5, tcpci_alert_tx_discard),
	DECL_TCPCI_ALERT_HANDLER(6, tcpci_alert_tx_success),
	DECL_TCPCI_ALERT_HANDLER(2, tcpci_alert_recv_msg),
	DECL_TCPCI_ALERT_HANDLER(7, NULL),
	DECL_TCPCI_ALERT_HANDLER(8, NULL),
	DECL_TCPCI_ALERT_HANDLER(3, tcpci_alert_recv_hard_reset),
	DECL_TCPCI_ALERT_HANDLER(10, tcpci_alert_rx_overflow),
#endif /* CONFIG_USB_POWER_DELIVERY */

	DECL_TCPCI_ALERT_HANDLER(9, tcpci_alert_fault),
	DECL_TCPCI_ALERT_HANDLER(0, tcpci_alert_cc_changed),
	DECL_TCPCI_ALERT_HANDLER(1, tcpci_alert_power_status_changed),
};

#ifdef CONFIG_USB_POWER_DELIVERY
static inline bool tcpci_check_hard_reset_complete(
	struct tcpc_device *tcpc_dev, uint32_t alert_status)
{
	if ((alert_status & TCPC_REG_ALERT_HRESET_SUCCESS)
			== TCPC_REG_ALERT_HRESET_SUCCESS) {
		pd_put_sent_hard_reset_event(tcpc_dev);
		return true;
	}

	if (alert_status & TCPC_REG_ALERT_TX_DISCARDED) {
		TCPC_INFO("HResetFailed\r\n");
		tcpci_transmit(tcpc_dev, TCPC_TX_HARD_RESET, 0, NULL);
		return false;
	}

	return false;
}
#endif	/* CONFIG_USB_POWER_DELIVERY */
static inline int __tcpci_alert(struct tcpc_device *tcpc_dev)
{
	int rv, i;
	uint32_t alert_status;
	uint32_t alert_mask;

	rv = tcpci_get_alert_status(tcpc_dev, &alert_status);
	if (rv)
		return rv;

	rv = tcpci_get_alert_mask(tcpc_dev, &alert_mask);
	if (rv)
		return rv;

	/* mask all alert */
	rv = tcpci_set_alert_mask(tcpc_dev, 0);
	if (rv)
		return rv;

#ifdef CONFIG_USB_PD_DBG_ALERT_STATUS
	if (alert_status != 0)
		TCPC_INFO("Alert:0x%04x, Mask:0x%04x\r\n",
			  alert_status, alert_mask);
#endif /* CONFIG_USB_PD_DBG_ALERT_STATUS */

	alert_status &= alert_mask;

	tcpci_alert_status_clear(tcpc_dev,
		alert_status & (~TCPC_REG_ALERT_RX_MASK));

	if (tcpc_dev->typec_role == TYPEC_ROLE_UNKNOWN)
		return 0;

#ifdef CONFIG_USB_POWER_DELIVERY
	if (tcpc_dev->pd_transmit_state == PD_TX_STATE_WAIT_HARD_RESET) {
		tcpci_check_hard_reset_complete(tcpc_dev, alert_status);
		alert_status &= ~TCPC_REG_ALERT_TX_MASK;
	}
#endif	/* CONFIG_USB_POWER_DELIVERY */

#ifndef CONFIG_USB_PD_DBG_SKIP_ALERT_HANDLER
	for (i = 0; i < ARRAY_SIZE(tcpci_alert_handlers); i++) {
		if (tcpci_alert_handlers[i].bit_mask & alert_status) {
			if (tcpci_alert_handlers[i].handler != 0)
				tcpci_alert_handlers[i].handler(tcpc_dev);
		}
	}
#endif /* CONFIG_USB_PD_DBG_SKIP_ALERT_HANDLER */

	/* unmask alert */
	rv = tcpci_set_alert_mask(tcpc_dev, alert_mask);
	if (rv)
		return rv;

	tcpci_vbus_level_refresh(tcpc_dev);
	tcpci_vbus_level_changed(tcpc_dev);
	return 0;
}

int tcpci_alert(struct tcpc_device *tcpc_dev)
{
	int ret;

#ifdef CONFIG_TCPC_IDLE_MODE
	tcpci_idle_poll_ctrl(tcpc_dev, true, 0);
#endif /* CONFIG_TCPC_IDLE_MODE */

	ret = __tcpci_alert(tcpc_dev);

#ifdef CONFIG_TCPC_IDLE_MODE
	tcpci_idle_poll_ctrl(tcpc_dev, false, 0);
#endif /* CONFIG_TCPC_IDLE_MODE */

	return ret;
}

/*
 * [BLOCK] TYPEC device changed
 */

static inline void tcpci_attach_wake_lock(struct tcpc_device *tcpc)
{
#ifdef CONFIG_TCPC_ATTACH_WAKE_LOCK_TOUT
	__pm_wakeup_event(&tcpc->attach_wake_lock,
					     CONFIG_TCPC_ATTACH_WAKE_LOCK_TOUT);
#else
	__pm_stay_awake(&tcpc->attach_wake_lock);
#endif	/* CONFIG_TCPC_ATTACH_WAKE_LOCK_TOUT */
}

int tcpci_set_wake_lock(
	struct tcpc_device *tcpc, bool pd_lock, bool user_lock)
{
	bool ori_lock, new_lock;

	if (tcpc->wake_lock_pd && tcpc->wake_lock_user)
		ori_lock = true;
	else
		ori_lock = false;

	if (pd_lock && user_lock)
		new_lock = true;
	else
		new_lock = false;

	if (new_lock != ori_lock) {
		if (new_lock) {
			TCPC_DBG("wake_lock=1\r\n");
			tcpci_attach_wake_lock(tcpc);
			if (tcpc->typec_watchdog)
				tcpci_set_intrst(tcpc, true);
		} else {
			TCPC_DBG("wake_lock=0\r\n");
			if (tcpc->typec_watchdog)
				tcpci_set_intrst(tcpc, false);
			__pm_relax(&tcpc->attach_wake_lock);
		}
		return 1;
	}

	return 0;
}

static inline int tcpci_set_wake_lock_pd(
	struct tcpc_device *tcpc, bool pd_lock)
{
	uint8_t wake_lock_pd;

	mutex_lock(&tcpc->access_lock);

	wake_lock_pd = tcpc->wake_lock_pd;

	if (pd_lock)
		wake_lock_pd++;
	else if (wake_lock_pd > 0)
		wake_lock_pd--;

	if (wake_lock_pd == 0)
		__pm_wakeup_event(&tcpc->dettach_temp_wake_lock, 5000);

	tcpci_set_wake_lock(tcpc, wake_lock_pd, tcpc->wake_lock_user);

	if (wake_lock_pd == 1)
		__pm_relax(&tcpc->dettach_temp_wake_lock);

	tcpc->wake_lock_pd = wake_lock_pd;
	mutex_unlock(&tcpc->access_lock);
	return 0;
}

static inline int tcpci_report_usb_port_attached(struct tcpc_device *tcpc)
{
	TCPC_INFO("usb_port_attached\r\n");

#ifdef CONFIG_DUAL_ROLE_USB_INTF
	switch (tcpc->typec_attach_new) {
	case TYPEC_ATTACHED_SNK:
	case TYPEC_ATTACHED_CUSTOM_SRC:
		tcpc->dual_role_pr = DUAL_ROLE_PROP_PR_SNK;
		tcpc->dual_role_dr = DUAL_ROLE_PROP_DR_DEVICE;
		tcpc->dual_role_mode = DUAL_ROLE_PROP_MODE_UFP;
		tcpc->dual_role_vconn = DUAL_ROLE_PROP_VCONN_SUPPLY_NO;
		break;
	case TYPEC_ATTACHED_SRC:
		tcpc->dual_role_pr = DUAL_ROLE_PROP_PR_SRC;
		tcpc->dual_role_dr = DUAL_ROLE_PROP_DR_HOST;
		tcpc->dual_role_mode = DUAL_ROLE_PROP_MODE_DFP;
		tcpc->dual_role_vconn = DUAL_ROLE_PROP_VCONN_SUPPLY_YES;
		break;
	default:
		break;
	}
	dual_role_instance_changed(tcpc->dr_usb);
#endif /* CONFIG_DUAL_ROLE_USB_INTF */
#if __HCT_TYPEC_ACCDET_MIC_SUPPORT__
	if(TYPEC_ATTACHED_AUDIO == tcpc->typec_attach_new){ 
		TCPC_INFO("usb_port_attached accdet_typec_state --> ACCDET_PIN_PLUG_IN.\n");
		typec_recheck_flag =1;
		pmic_eint_queue_work(1);
		accdet_typec_state = EINT_PIN_PLUG_IN;
		TCPC_INFO("usb_port_attached ex_eint_handler end.\n");
#ifdef __HCT_WIRELESS_CHARGER_WHEN_TPC_IN_SUPPORT__
#if CONFIG_MTK_GAUGE_VERSION == 30                                   
		primary_charger = get_charger_by_name("primary_chg");            
		if (!primary_charger) {                                          
			pr_err("%s: get primary charger device failed\n", __func__); 
			return -ENODEV;                                              
		}                                                                
		//printk("hct_jk [%s] vbus = %d !\n",__func__,charger_get_vbus());                                  
		if(charger_get_vbus() > 4500) {
			charger_dev_enable_chg_type_det(primary_charger, true);
		}
#endif
#endif	
	}
#endif

	tcpci_set_wake_lock_pd(tcpc, true);

#ifdef CONFIG_USB_POWER_DELIVERY

#ifdef CONFIG_USB_PD_DISABLE_PE
	if (tcpc->disable_pe)
		return 0;
#endif	/* CONFIG_USB_PD_DISABLE_PE */

	/* MTK Only */
	if (tcpc->pd_inited_flag)
		pd_put_cc_attached_event(tcpc, tcpc->typec_attach_new);
#endif /* CONFIG_USB_POWER_DLEIVERY */

	return 0;
}

static inline int tcpci_report_usb_port_detached(struct tcpc_device *tcpc)
{
	TCPC_INFO("usb_port_detached\r\n");

#ifdef CONFIG_DUAL_ROLE_USB_INTF
	tcpc->dual_role_pr = DUAL_ROLE_PROP_PR_NONE;
	tcpc->dual_role_dr = DUAL_ROLE_PROP_DR_NONE;
	tcpc->dual_role_mode = DUAL_ROLE_PROP_MODE_NONE;
	tcpc->dual_role_vconn = DUAL_ROLE_PROP_VCONN_SUPPLY_NO;
	dual_role_instance_changed(tcpc->dr_usb);
#endif /* CONFIG_DUAL_ROLE_USB_INTF */

#if __HCT_TYPEC_ACCDET_MIC_SUPPORT__
	if(EINT_PIN_PLUG_IN == accdet_typec_state){ 
		TCPC_INFO("usb_port_detached audio accessory detached.\n");
		typec_recheck_flag = 0;
		pmic_eint_queue_work(1);
		accdet_typec_state = EINT_PIN_PLUG_OUT;
		TCPC_INFO("usb_port_detached accdet_typec_state --> ACCDET_PIN_PLUG_OUT .\n");
#ifdef __HCT_WIRELESS_CHARGER_WHEN_TPC_IN_SUPPORT__	
		//printk("hct_jk [%s] vbus = %d !\n",__func__,charger_get_vbus());
		if(charger_get_vbus() > 4500) {
			charger_dev_enable_chg_type_det(primary_charger, false);
		}
#endif
	}
#endif

#ifdef CONFIG_USB_POWER_DELIVERY
	/* MTK Only */
	if (tcpc->pd_inited_flag)
		pd_put_cc_detached_event(tcpc);
	else {
		pd_event_buf_reset(tcpc);
		tcpc_enable_timer(tcpc, TYPEC_RT_TIMER_PE_IDLE);
	}
#endif /* CONFIG_USB_POWER_DELIVERY */

	tcpci_set_wake_lock_pd(tcpc, false);
	return 0;
}

int tcpci_report_usb_port_changed(struct tcpc_device *tcpc)
{
	tcpci_notify_typec_state(tcpc);

	if (tcpc->typec_attach_old == TYPEC_UNATTACHED)
		tcpci_report_usb_port_attached(tcpc);
	else if (tcpc->typec_attach_new == TYPEC_UNATTACHED)
		tcpci_report_usb_port_detached(tcpc);
	else
		TCPC_DBG2("TCPC Attach Again\r\n");

	return 0;
}

/*
 * [BLOCK] TYPEC power control changed
 */

int tcpci_report_power_control_on(struct tcpc_device *tcpc)
{
	tcpci_set_wake_lock_pd(tcpc, true);

#ifdef CONFIG_TYPEC_CAP_AUTO_DISCHARGE

#ifdef CONFIG_TCPC_AUTO_DISCHARGE_EXT
	tcpci_enable_ext_discharge(tcpc, false);
#endif	/* CONFIG_TCPC_AUTO_DISCHARGE_EXT */

#ifdef CONFIG_TCPC_AUTO_DISCHARGE_IC
	tcpci_enable_auto_discharge(tcpc, true);
#endif	/* CONFIG_TCPC_AUTO_DISCHARGE_IC */

	tcpc_disable_timer(tcpc, TYPEC_RT_TIMER_AUTO_DISCHARGE);
#endif	/* CONFIG_TYPEC_CAP_AUTO_DISCHARGE */

	return 0;
}

int tcpci_report_power_control_off(struct tcpc_device *tcpc)
{
#ifdef CONFIG_USB_POWER_DELIVERY
#ifdef CONFIG_TCPC_FORCE_DISCHARGE_IC
	tcpci_disable_force_discharge(tcpc);
#endif	/* CONFIG_TCPC_FORCE_DISCHARGE_IC */
#endif	/* CONFIG_USB_POWER_DELIVERY */

#ifdef CONFIG_TYPEC_CAP_AUTO_DISCHARGE

#ifdef CONFIG_TCPC_AUTO_DISCHARGE_EXT
	tcpci_enable_ext_discharge(tcpc, true);
#endif	/* CONFIG_TCPC_AUTO_DISCHARGE_EXT */

	tcpc_enable_timer(tcpc, TYPEC_RT_TIMER_AUTO_DISCHARGE);
#endif	/* CONFIG_TYPEC_CAP_AUTO_DISCHARGE */

	tcpci_set_wake_lock_pd(tcpc, false);
	return 0;
}

int tcpci_report_power_control(struct tcpc_device *tcpc, bool en)
{
	if (tcpc->typec_power_ctrl == en)
		return 0;

	tcpc->typec_power_ctrl = en;

	if (en)
		tcpci_report_power_control_on(tcpc);
	else
		tcpci_report_power_control_off(tcpc);

	return 0;
}
