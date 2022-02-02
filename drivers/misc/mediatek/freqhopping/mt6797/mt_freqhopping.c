/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/sched_clock.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include "mach/mt_freqhopping.h"
#include "mach/mt_fhreg.h"
#include "sync_write.h"
#include "mt_freqhopping_drv.h"
#ifdef HP_EN_REG_SEMAPHORE_PROTECT
#include "mt_cpufreq_hybrid.h"
#endif
#include <linux/seq_file.h>
#include <linux/of_address.h>

/***********************************/
/* Other global variable           */
/***********************************/
static unsigned int g_initialize;	/* [True]: Init done */
static DEFINE_SPINLOCK(g_fh_lock);
/* Everest 0x1001AXXX bus access issue. All access 0x1001AXXX driver should use the lock */
static DEFINE_SPINLOCK(g_mt6797_0x1001AXXX_lock);


/*********************************/
/* FHCTL related IP base address */
/*********************************/

static void __iomem *g_fhctl_base;
static void __iomem *g_mcu_fhctl_base;
static void __iomem *g_apmixed_base;
static void __iomem *g_mcumixed_base;

/*********************************/
/* Utility Macro */
/*********************************/
#define MASK21b (0x1FFFFF)
#define MASK32b (0xFFFFFFFF)
#define BIT32   (1U<<31)

#define VALIDATE_PLLID(id) BUG_ON(id >= FH_PLL_NUM)
#define VALIDATE_DDS(dds)  BUG_ON(dds > 0x1ffff)
#define PERCENT_TO_DDSLMT(dDS, pERCENT_M10) (((dDS * pERCENT_M10) >> 5) / 100)

/*********************************/
/* FHCTL PLL Setting ID */
/*********************************/
#define PLL_SETTING_IDX__USER (0x9)	/* Magic number, no any special indication */
#define PLL_SETTING_IDX__DEF    (0x1)	/* Default Setting, Magic number, indicate table position 1. */


/*********************************/
/* Track the status of all FHCTL PLL */
/*********************************/
static fh_pll_t g_fh_pll[FH_PLL_NUM] = { };	/* init during run time. */


/*********************************/
/* FHCTL PLL name                */
/*********************************/
static const char *g_pll_name[FH_PLL_NUM] = {
	"CAX0PLL",
	"CAX1PLL",
	"CAX2PLL",
	"CAX3PLL",
	"VDECPLL",
	"MPLL",
	"MAINPLL",
	"MEMPLL",
	"MSDCPLL",
	"MFGPLL",
	"IMGPLL",
	"TVDPLL",
	"CODECPLL",
};

/*********************************/
/* FHCTL PLL SSC Setting Table   */
/*********************************/
#define UNINIT_DDS   0x0
#define MAX_DDS      0x1fffff	/* 21 bit */

/* [For Everest] Should be setting according to HQA de-sense result.  */
static const int g_pll_ssc_init_tbl[FH_PLL_NUM] = {
	/*
	 *  [FH_SSC_DEF_DISABLE]: Default SSC disable,
	 *  [FH_SSC_DEF_ENABLE_SSC]: Default enable SSC.
	 */
	FH_SSC_DEF_DISABLE,	/* MCUFHCTL PLL0 */
	FH_SSC_DEF_DISABLE,	/* MCUFHCTL PLL1 */
	FH_SSC_DEF_DISABLE,	/* MCUFHCTL PLL2 */
	FH_SSC_DEF_DISABLE,	/* MCUFHCTL PLL3 */
	FH_SSC_DEF_DISABLE,	/* FHCTL PLL0 */
	FH_SSC_DEF_ENABLE_SSC,	/* FHCTL PLL1 */
	FH_SSC_DEF_DISABLE,	/* FHCTL PLL2 */
	FH_SSC_DEF_DISABLE,	/* FHCTL PLL3 */
	FH_SSC_DEF_DISABLE,	/* FHCTL PLL4 */
	FH_SSC_DEF_DISABLE,	/* FHCTL PLL5 */
	FH_SSC_DEF_DISABLE,	/* FHCTL PLL6 */
	FH_SSC_DEF_DISABLE,	/* FHCTL PLL7 */
	FH_SSC_DEF_DISABLE,	/* FHCTL PLL8 */
};

/* [For Everest] */
static const struct freqhopping_ssc g_pll_ssc_setting_tbl[FH_PLL_NUM][4] = {
	/* MCU FH PLL0 */
	{
	 /* magic_index, dt, df, upbnd, lowbnd, dds */
	 {0, 0, 0, 0, 0, 0},	/* Means disable */
	 {PLL_SETTING_IDX__DEF, 0, 9, 0, 0, UNINIT_DDS},	/* Default 0%(upbnd) ~ -0%(lowbnd) */
	 },

	/* MCU FH PLL1 */
	{
	 {0, 0, 0, 0, 0, 0},
	 {PLL_SETTING_IDX__DEF, 0, 9, 0, 0, UNINIT_DDS},	/* Default 0%(upbnd) ~ -0%(lowbnd) */
	 },

	/* MCU FH PLL2 */
	{
	 {0, 0, 0, 0, 0, 0},
	 {PLL_SETTING_IDX__DEF, 0, 9, 0, 0, UNINIT_DDS},	/* Default 0%(upbnd) ~ -0%(lowbnd) */
	 },

	/* MCU FH PLL3 */
	{
	 {0, 0, 0, 0, 0, 0},
	 {PLL_SETTING_IDX__DEF, 0, 9, 0, 0, UNINIT_DDS},	/* Default 0%(upbnd) ~ -0%(lowbnd) */
	 },

	/* FH PLL0 */
	{
	 {0, 0, 0, 0, 0, 0},
	 {PLL_SETTING_IDX__DEF, 0, 9, 0, 0, UNINIT_DDS},	/* Default 0%(upbnd) ~ -0%(lowbnd) */
	 },

	/* FH PLL1 */
	{
	 {0, 0, 0, 0, 0, 0},
	 /* SSC Slope [dys]:0.015625 [dts]:1.808000 [slope]:0.096619 Mhz/us */
	 /* double slope = ((DYS[dy]*26)/DTS[df])*0.43; Test by from Yulia */
	 {PLL_SETTING_IDX__DEF, 6, 8, 0, 2, UNINIT_DDS},	/* Default 0%(upbnd) ~ -2%(lowbnd) */
	 },

	/* FH PLL2 */
	{
	 {0, 0, 0, 0, 0, 0},
	 {PLL_SETTING_IDX__DEF, 0, 9, 0, 0, UNINIT_DDS},	/* Default 0%(upbnd) ~ -0%(lowbnd) */
	 },

	/* FH PLL3 */
	{
	 {0, 0, 0, 0, 0, 0},
	 {PLL_SETTING_IDX__DEF, 0, 9, 0, 0, UNINIT_DDS},	/* Default 0%(upbnd) ~ -0%(lowbnd) */
	 },

	/* FH PLL4 */
	{
	 {0, 0, 0, 0, 0, 0},
	 {PLL_SETTING_IDX__DEF, 0, 9, 0, 0, UNINIT_DDS},	/* Default 0%(upbnd) ~ -0%(lowbnd) */
	 },

	/* FH PLL5 */
	{
	 {0, 0, 0, 0, 0, 0},
	 {PLL_SETTING_IDX__DEF, 0, 9, 0, 0, UNINIT_DDS},	/* Default 0%(upbnd) ~ -0%(lowbnd) */
	 },

	/* FH PLL6 */
	{
	 {0, 0, 0, 0, 0, 0},
	 {PLL_SETTING_IDX__DEF, 0, 9, 0, 0, UNINIT_DDS},	/* Default 0%(upbnd) ~ -0%(lowbnd) */
	 },

	/* FH PLL7 */
	{
	 {0, 0, 0, 0, 0, 0},
	 {PLL_SETTING_IDX__DEF, 0, 9, 0, 0, UNINIT_DDS},	/* Default 0%(upbnd) ~ -0%(lowbnd) */
	 },

	/* FH PLL8 */
	{
	 {0, 0, 0, 0, 0, 0},
	 {PLL_SETTING_IDX__DEF, 0, 9, 0, 0, UNINIT_DDS},	/* Default 0%(upbnd) ~ -0%(lowbnd) */
	 },
};


/***********************************/
/* MCU FHCTL/FHCTL HP CON Register */
/***********************************/
/* [For Everest] */
static const int pllid_to_hp_con[] = { 0, 1, 2, 3, 0, 1, 2, 3, 4, 5, 6, 7, 8 };

static struct freqhopping_ssc mt_ssc_fhpll_userdefined[FH_PLL_NUM];	/* freq, dt, df, upbnd, lowbnd, dds */

/*************************************/
/* FHCTL Register table              */
/* - Dynamic assign address based on */
/*   Device tree IP address          */
/*************************************/
static unsigned long g_reg_dds[FH_PLL_NUM];
static unsigned long g_reg_cfg[FH_PLL_NUM];
static unsigned long g_reg_updnlmt[FH_PLL_NUM];
static unsigned long g_reg_mon[FH_PLL_NUM];
static unsigned long g_reg_dvfs[FH_PLL_NUM];
static unsigned long g_reg_pll_con0[FH_PLL_NUM];
static unsigned long g_reg_pll_con1[FH_PLL_NUM];

/*****************************************************************************/
/* Special Function for Everest BUS issue */
/*****************************************************************************/
#define hs_read32(reg)          readl((void __iomem *)reg)
#define hs_write32(reg, val)    mt_reg_sync_writel((val), (reg))

static void __iomem *g_sema_base;
static unsigned long g_reg_sema3_m0;
static unsigned long g_reg_cspm_poweron_en;
#define SEMA_GET_TIMEOUT  2000	/* us */

/* 0x1001AXXX mistake-proofing mechanism. */
/* Whole system only dispatch VA of 0x1001A000 in the API */
static int __init mt6797_0x1001AXXX_sw_protect_init(void)
{
	FH_MSG_DEBUG("init_mcumixedsys_base_addr+++");
	/* Init APMIXED base address */

	g_mcumixed_base = ioremap_nocache(0x1001A000, 0x1000);
	if (!g_mcumixed_base) {
		FH_MSG("Error, MCUMIXED iomap failed");
		BUG_ON(1);
	}

	g_mcu_fhctl_base = g_mcumixed_base + 0xf00;

	/* DVFSP HW semaphore init. */
	{
		g_sema_base = ioremap_nocache(0x11015000, 0x1000);
		g_reg_sema3_m0 = (unsigned long)g_sema_base + (0x440);
		g_reg_cspm_poweron_en = (unsigned long)g_sema_base;
		hs_write32(g_reg_cspm_poweron_en, 0x0b160001);	/* mt6797-dvfsp enable internal CG bit */
	}

	FH_MSG_DEBUG("g_mcumixed_base:0x%lx\n", (unsigned long)g_mcumixed_base);
	FH_MSG_DEBUG("g_mcu_fhctl_base:0x%lx\n", (unsigned long)g_mcu_fhctl_base);
	FH_MSG_DEBUG("g_reg_sema3_m0:0x%lx\n", (unsigned long)g_reg_sema3_m0);
	FH_MSG_DEBUG("g_reg_cspm_poweron_en:0x%lx\n", (unsigned long)g_reg_cspm_poweron_en);

	return 0;
}
core_initcall(mt6797_0x1001AXXX_sw_protect_init);

/* HW semaphore3 M0 get.
 *  For ATF, SPM and kernel protecting 0x1001AXXX access
 */

static int mt6797_0x1001AXXX_get_semaphore(void)
{
	int i;
	int n = DIV_ROUND_UP(SEMA_GET_TIMEOUT, 10);

	FH_MSG_DEBUG("mt6797_0x1001AXXX_get_semaphore+");

	/* mt6797-dvfsp enable internal CG bit */
	hs_write32(g_reg_cspm_poweron_en, 0x0b160001);


	FH_MSG_DEBUG("0x1001AXXX sema get %lx\n", g_reg_sema3_m0);

	for (i = 0; i < n; i++) {
		hs_write32(g_reg_sema3_m0, 0x1);
		if (hs_read32(g_reg_sema3_m0) & 0x1)
			return 0;

		udelay(10);
	}

	FH_MSG("0x1001AXXX SEMA_USER GET TIMEOUT");
	BUG_ON(1);

	return -EBUSY;
}

/* HW semaphore3 M0 release.
 *  For ATF, SPM and kernel protecting 0x1001AXXX access
 */
static void mt6797_0x1001AXXX_release_semaphore(void)
{
	FH_MSG_DEBUG("0x1001AXXX sema release\n");

	if (hs_read32(g_reg_sema3_m0) & 0x1) {
		hs_write32(g_reg_sema3_m0, 0x1);
		BUG_ON(hs_read32(g_reg_sema3_m0) & 0x1);	/* semaphore release failed */
	}
}

/* 0x1001AXXX register write API
 *  Provide to MCUMIXEDSYS access. (Clock related driver)
 */
void mt6797_0x1001AXXX_reg_write(unsigned long reg_offset, unsigned int value)
{
	volatile unsigned long reg;
	unsigned long flags = 0;

	BUG_ON(reg_offset > 0xfff);

	reg = (unsigned long)g_mcumixed_base + reg_offset;

	local_irq_save(flags);
	mt6797_0x1001AXXX_lock();	/* To protect between ATF and SPM. */

	mt_reg_sync_writel(value, reg);
	ndelay(200);		/* DE workaround, for first read after sequential write. */

	mt6797_0x1001AXXX_unlock();	/* To protect between ATF and SPM. */
	local_irq_restore(flags);
}

/* 0x1001AXXX register read API
 *  Provide to MCUMIXEDSYS access.  (Clock related driver)
 */
unsigned int mt6797_0x1001AXXX_reg_read(unsigned long reg_offset)
{
	volatile unsigned int value;
	volatile unsigned long reg;
	unsigned long flags = 0;

	BUG_ON(reg_offset > 0xfff);

	/* g_mcumixed_base value from VA protect mechanism */
	reg = (unsigned long)g_mcumixed_base + reg_offset;

	local_irq_save(flags);
	mt6797_0x1001AXXX_lock();	/* To protect between ATF and SPM. */

	ndelay(200);		/* DE workaround, for first read after sequential write. */
	value = readl((void __iomem *)reg);

	mt6797_0x1001AXXX_unlock();	/* To protect between ATF and SPM. */
	local_irq_restore(flags);

	return value;
}

void mt6797_0x1001AXXX_reg_set(unsigned long reg_offset, unsigned int field, unsigned int val)
{
	volatile unsigned long reg;
	volatile unsigned int temp_val;
	unsigned long flags = 0;

	BUG_ON(reg_offset > 0xfff);

	reg = (unsigned long)g_mcumixed_base + reg_offset;

	local_irq_save(flags);
	mt6797_0x1001AXXX_lock();	/* To protect between ATF and SPM. */

	if (field > 0) {
		ndelay(200);	/* DE workaround, for first read after sequential write. */
		temp_val = readl((void __iomem *)reg);
		temp_val &= ~(field);
		temp_val |= ((val) << (uffs((unsigned int)field) - 1));
		mt_reg_sync_writel(temp_val, reg);
		ndelay(200);	/* DE workaround, for first read after sequential write. */
	}
	mt6797_0x1001AXXX_unlock();	/* To protect between ATF and SPM. */
	local_irq_restore(flags);
}

/* 0x1001AXX bus access should use the API to protect
 * All clock driver might call the API when access 0x1001AXX address.
 */
void mt6797_0x1001AXXX_lock(void)
{
	spin_lock(&g_mt6797_0x1001AXXX_lock);


	if (0 != mt6797_0x1001AXXX_get_semaphore()) {
		if (0 != mt6797_0x1001AXXX_get_semaphore()) {
			FH_MSG("[ERROR] HW sema time out 4ms");
			BUG_ON(1);
		}
	}

}

void mt6797_0x1001AXXX_unlock(void)
{
	mt6797_0x1001AXXX_release_semaphore();

	spin_unlock(&g_mt6797_0x1001AXXX_lock);
}

/*****************************************************************************/
/* Function */
/*****************************************************************************/


static int fh_dumpregs_read(enum FH_PLL_ID pll_id)
{
	/* FH_MSG("FHCTL dumpregs: %s", __func__); */
	unsigned long flags = 0;
	unsigned int mon;

	local_irq_save(flags);

	mon = fh_read32(g_reg_mon[pll_id]);

	FH_MSG("[PLL]:%d [CFG]:%08x [UPDNLMT]:%08x [DVFS]:%08x [DDS]:%08x [MON]:%08x",
	       pll_id, fh_read32(g_reg_cfg[pll_id]), fh_read32(g_reg_updnlmt[pll_id]),
	       fh_read32(g_reg_dvfs[pll_id]), fh_read32(g_reg_dds[pll_id]), mon);

	FH_MSG("[CON0]:%08x [CON1]:%08x",
	       fh_read32(g_reg_pll_con0[pll_id]), fh_read32(g_reg_pll_con1[pll_id]));

	FH_MSG
	    ("FHCTL [HP_EN]:0x%08x [CLK_CON]:0x%08x [SLOPE0]:0x%08x [SLOPE1]:0x%08x [DSSC_CFG]:0x%08x",
	     fh_read32(REG_FHCTL_HP_EN), fh_read32(REG_FHCTL_CLK_CON),
	     fh_read32(REG_FHCTL_SLOPE0), fh_read32(REG_FHCTL_SLOPE1),
	     fh_read32(REG_FHCTL_DSSC_CFG));

	local_irq_restore(flags);
	return 0;
}

static int fh_mcu_dumpregs_read(enum FH_PLL_ID pll_id)
{
	/* FH_MSG("FHCTL dumpregs: %s", __func__); */
	unsigned long flags = 0;
	unsigned int mon;

	local_irq_save(flags);


	mon = mcu_fh_read32(g_reg_mon[pll_id]);

	FH_MSG("[PLL]:%d [CFG]:%08x [UPDNLMT]:%08x [DVFS]:%08x [DDS]:%08x [MON]:%08x",
	       pll_id, mcu_fh_read32(g_reg_cfg[pll_id]),
	       mcu_fh_read32(g_reg_updnlmt[pll_id]), mcu_fh_read32(g_reg_dvfs[pll_id]),
	       mcu_fh_read32(g_reg_dds[pll_id]), mon);

	FH_MSG("[CON0]:%08x [CON1]:%08x",
	       fh_read32(g_reg_pll_con0[pll_id]), fh_read32(g_reg_pll_con1[pll_id]));


	FH_MSG
	    ("MCU_FHCTL [HP_EN]:0x%08x [CLK_CON]:0x%08x [SLOPE0]:0x%08x [DSSC_CFG]:0x%08x",
	     mcu_fh_read32(REG_MCU_FHCTL_HP_EN), mcu_fh_read32(REG_MCU_FHCTL_CLK_CON),
	     mcu_fh_read32(REG_MCU_FHCTL_SLOPE0), mcu_fh_read32(REG_MCU_FHCTL_DSSC_CFG));

	local_irq_restore(flags);

	return 0;
}


static void mt_fh_hal_default_conf(void)
{
	int id;

	FH_MSG_DEBUG("%s", __func__);


	/* According to setting to enable PLL SSC during init FHCTL. */
	for (id = 0; id < FH_PLL_NUM; id++) {

		g_fh_pll[id].pll_status = FH_PLL_ENABLE;

		if (g_pll_ssc_init_tbl[id] == FH_SSC_DEF_ENABLE_SSC) {
			FH_MSG("[Default ENABLE SSC] PLL_ID:%d", id);
			g_fh_pll[id].fh_status = FH_FH_ENABLE_SSC;
			freqhopping_config(id, PLL_SETTING_IDX__DEF, true);	/* MAINPLL */
		} else {
			g_fh_pll[id].fh_status = FH_FH_DISABLE;
		}
	}
}

static void fh_switch2fhctl(enum FH_PLL_ID pll_id, int i_control)
{
	unsigned int mask = 0;

	VALIDATE_PLLID(pll_id);

	mask = 0x1U << pllid_to_hp_con[pll_id];

	/* Release software reset */
	/* fh_set_field(REG_FHCTL_RST_CON, mask, 0); */

	if (isFHCTL(pll_id)) {
		/* Original FHCTL */
		fh_set_field(REG_FHCTL_HP_EN, mask, i_control);
	} else {
		FH_MSG("ERROR! Don't use fh_switch2fhctl() for MCU FHCTL\n");
		BUG_ON(1);
	}			/* if - else */
}

static void fh_sync_ncpo_to_fhctl_dds(enum FH_PLL_ID pll_id)
{
	unsigned long reg_src = 0;
	unsigned long reg_dst = 0;

	VALIDATE_PLLID(pll_id);

	if (pll_id == FH_PLL3) {
		/* [For Everest] W/o MEMPLL */
		return;
	}

	reg_src = g_reg_pll_con1[pll_id];
	reg_dst = g_reg_dds[pll_id];

	if (isFHCTL(pll_id)) {
		/* FHCTL */
		fh_write32(reg_dst, (fh_read32(reg_src) & MASK21b) | BIT32);
	} else {
		/* MCU FHCTL */
		FH_MSG("Don't use fh_sync_ncpo_to_fhctl_dds for MCU FHCTL");
		BUG_ON(1);
	}

}

static void __enable_ssc(unsigned int pll_id, const struct freqhopping_ssc *setting)
{
	unsigned long flags = 0;
	const unsigned long reg_cfg = g_reg_cfg[pll_id];
	const unsigned long reg_updnlmt = g_reg_updnlmt[pll_id];
	const unsigned long reg_dds = g_reg_dds[pll_id];

	FH_MSG_DEBUG("%s: %x~%x df:%d dt:%d dds:%x",
		     __func__, setting->lowbnd, setting->upbnd, setting->df, setting->dt,
		     setting->dds);

	if (!isFHCTL(pll_id)) {
		FH_MSG("MCU FHCTL is forbidden to call __enable_ssc()");
		g_fh_pll[pll_id].fh_status = FH_FH_ENABLE_SSC;
		return;
	}

	mb();

	g_fh_pll[pll_id].fh_status = FH_FH_ENABLE_SSC;

	local_irq_save(flags);
	/* spin_lock(&g_fh_lock); */

	/* Set the relative parameter registers (dt/df/upbnd/downbnd) */
	fh_set_field(reg_cfg, MASK_FRDDSX_DYS, setting->df);
	fh_set_field(reg_cfg, MASK_FRDDSX_DTS, setting->dt);

	fh_sync_ncpo_to_fhctl_dds(pll_id);

	/* TODO: Not setting upper due to they are all 0? */
	fh_write32(reg_updnlmt,
		   (PERCENT_TO_DDSLMT((fh_read32(reg_dds) & MASK21b), setting->lowbnd) << 16));

	/* Switch to FHCTL */
	fh_switch2fhctl(pll_id, 1);
	mb();

	/* Enable SSC */
	fh_set_field(reg_cfg, FH_FRDDSX_EN, 1);
	/* Enable Hopping control */
	fh_set_field(reg_cfg, FH_FHCTLX_EN, 1);

	local_irq_restore(flags);
	/* spin_unlock(&g_fh_lock); */
}

static void __disable_ssc(unsigned int pll_id, const struct freqhopping_ssc *ssc_setting)
{
	unsigned long flags = 0;
	unsigned long reg_cfg = g_reg_cfg[pll_id];

	FH_MSG_DEBUG("Calling %s", __func__);

	if (!isFHCTL(pll_id)) {
		FH_MSG("MCU FHCTL is forbidden to call __disable_ssc()");
		g_fh_pll[pll_id].fh_status = FH_FH_DISABLE;
		return;
	}


	local_irq_save(flags);
	/* spin_lock(&g_fh_lock); */


	/* Set the relative registers */
	fh_set_field(reg_cfg, FH_FRDDSX_EN, 0);
	fh_set_field(reg_cfg, FH_FHCTLX_EN, 0);
	mb();
	fh_switch2fhctl(pll_id, 0);
	g_fh_pll[pll_id].fh_status = FH_FH_DISABLE;

	local_irq_restore(flags);
	/* spin_unlock(&g_fh_lock); */
	mb();

}

/* Just to use special index pattern to find right setting. */
static noinline int __freq_to_index(enum FH_PLL_ID pll_id, int setting_idx_pattern)
{
	unsigned int retVal = 0;
	unsigned int i = PLL_SETTING_IDX__DEF;	/* start from 1 */
	const unsigned int size =
	    sizeof(g_pll_ssc_setting_tbl[pll_id]) / sizeof(g_pll_ssc_setting_tbl[pll_id][0]);

	while (i < size) {
		if (setting_idx_pattern == g_pll_ssc_setting_tbl[pll_id][i].idx_pattern) {
			retVal = i;
			break;
		}
		++i;
	}

	return retVal;
}

/* Hook to g_fh_hal_drv.mt_fh_hal_ctrl function point.
 * Common drv freqhopping_config() will call the HAL API.
 */
static int __freqhopping_ctrl(struct freqhopping_ioctl *fh_ctl, bool enable)
{
	const struct freqhopping_ssc *pSSC_setting = NULL;
	unsigned int ssc_setting_id = 0;
	int retVal = 1;
	fh_pll_t *pfh_pll = NULL;

	FH_MSG("%s for pll %d", __func__, fh_ctl->pll_id);

	if (!isFHCTL(fh_ctl->pll_id)) {
		FH_MSG("MCU FHCTL is forbidden to call __freqhopping_ctrl()");
	}


	/* Check the out of range of frequency hopping PLL ID */
	VALIDATE_PLLID(fh_ctl->pll_id);

	pfh_pll = &g_fh_pll[fh_ctl->pll_id];

	pfh_pll->setting_idx_pattern = PLL_SETTING_IDX__DEF;

	if ((enable == true) && (pfh_pll->fh_status == FH_FH_ENABLE_SSC)) {
		__disable_ssc(fh_ctl->pll_id, pSSC_setting);
	} else if ((enable == false) && (pfh_pll->fh_status == FH_FH_DISABLE)) {
		retVal = 0;
		goto Exit;
	}
	/* enable freq. hopping @ fh_ctl->pll_id */
	if (enable == true) {
		if (pfh_pll->pll_status == FH_PLL_DISABLE) {
			pfh_pll->fh_status = FH_FH_ENABLE_SSC;
			retVal = 0;
			goto Exit;
		} else {
			if (pfh_pll->user_defined == true) {
				FH_MSG("Apply user defined setting");

				pSSC_setting = &mt_ssc_fhpll_userdefined[fh_ctl->pll_id];
				pfh_pll->setting_id = PLL_SETTING_IDX__USER;
			} else {
				if (pfh_pll->setting_idx_pattern != 0) {
					ssc_setting_id = pfh_pll->setting_id =
					    __freq_to_index(fh_ctl->pll_id,
							    pfh_pll->setting_idx_pattern);
				} else {
					ssc_setting_id = 0;
				}

				if (ssc_setting_id == 0) {
					FH_MSG("!!! No corresponding setting found !!!");

					/* just disable FH & exit */
					__disable_ssc(fh_ctl->pll_id, pSSC_setting);
					goto Exit;
				}

				pSSC_setting =
				    &g_pll_ssc_setting_tbl[fh_ctl->pll_id][ssc_setting_id];

			}	/* user defined */

			if (pSSC_setting == NULL) {
				FH_MSG("SSC_setting is NULL!");

				/* disable FH & exit */
				__disable_ssc(fh_ctl->pll_id, pSSC_setting);
				goto Exit;
			}

			__enable_ssc(fh_ctl->pll_id, pSSC_setting);
			retVal = 0;
		}
	} else {		/* disable req. hopping @ fh_ctl->pll_id */
		__disable_ssc(fh_ctl->pll_id, pSSC_setting);
		retVal = 0;
	}

Exit:
	return retVal;
}

/* For FHCTL hopping only*/
static void wait_dds_stable(enum FH_PLL_ID pll_id, unsigned int target_dds, unsigned long reg_mon,
			    unsigned int wait_count)
{
	unsigned long flags = 0;
	unsigned int fh_dds = 0;
	unsigned int i = 0;

	local_irq_save(flags);
	fh_dds = fh_read32(reg_mon) & MASK21b;
	local_irq_restore(flags);

	while ((target_dds != fh_dds) && (i < wait_count)) {
		udelay(10);

		local_irq_save(flags);
		fh_dds = (fh_read32(reg_mon)) & MASK21b;
		local_irq_restore(flags);

		if ((i == 40) || (i == 60) || (i == 80)) {
			/* Might have something wrong during hopping */
			FH_MSG
			    ("[Warning]wait_dds_stable() pll = %d target_dds = 0x%x, fh_dds = 0x%x, i = %d",
			     pll_id, target_dds, fh_dds, i);

			fh_dumpregs_read(pll_id);
			WARN_ON(1);
		}
		++i;
	}
	if (i >= wait_count) {
		/* Something wrong during hopping */
		FH_MSG
		    ("[Warning]wait_dds_stable() pll = %d target_dds = 0x%x, fh_dds = 0x%x, i = %d",
		     pll_id, target_dds, fh_dds, i);
		fh_dumpregs_read(pll_id);

		BUG_ON(1);

	}
}

/* For MCU FHCTL hopping only*/
static void wait_mcu_fh_dds_stable(enum FH_PLL_ID pll_id, unsigned int target_dds,
				   unsigned long reg_mon, unsigned int wait_count)
{
	unsigned long flags = 0;
	unsigned int fh_dds = 0;
	unsigned int i = 0;

	local_irq_save(flags);
	fh_dds = mcu_fh_read32(reg_mon) & MASK21b;
	local_irq_restore(flags);

	while ((target_dds != fh_dds) && (i < wait_count)) {
		udelay(10);

		local_irq_save(flags);
		fh_dds = mcu_fh_read32(reg_mon) & MASK21b;
		local_irq_restore(flags);

		if ((i == 60) || (i == 80)) {
			/* Might have something wrong during hopping */
			FH_MSG
			    ("[Warning]wait_dds_stable() pll = %d target_dds = 0x%x, fh_dds = 0x%x, i = %d",
			     pll_id, target_dds, fh_dds, i);

			fh_mcu_dumpregs_read(pll_id);
			WARN_ON(1);
		}
		++i;
	}
	if (i >= wait_count) {
		/* Something wrong during hopping */
		FH_MSG
		    ("[Warning]wait_dds_stable() pll = %d target_dds = 0x%x, fh_dds = 0x%x, i = %d",
		     pll_id, target_dds, fh_dds, i);
		fh_mcu_dumpregs_read(pll_id);

		BUG_ON(1);
	}
}


/* Please add lock between the API for protecting FHCLT register atomic operation.
 *     spin_lock(&g_fh_lock);
 *     mt_fh_hal_hopping();
 *     spin_unlock(&g_fh_lock);
 *     for FHCTL only.
 */
static int mt_fh_hal_hopping(enum FH_PLL_ID pll_id, unsigned int dds_value)
{
	unsigned long flags = 0;

	FH_MSG_DEBUG("%s for pll %d:", __func__, pll_id);

	if (pll_id == FH_PLL3) {
		/* [For Everest] W/o MEMPLL */
		return 0;
	}

	if (pll_id <= MCU_FH_PLL3) {
		FH_MSG("[ERROR] mt_fh_hal_hopping() cannot hopping for pll_id:%d ", pll_id);
		BUG_ON(1);
	}

	VALIDATE_PLLID(pll_id);

	/* local_irq_save(flags); */
	local_irq_save(flags);

	/* 1. sync ncpo to DDS of FHCTL */
	fh_sync_ncpo_to_fhctl_dds(pll_id);

	/* FH_MSG("1. sync ncpo to DDS of FHCTL"); */
	FH_MSG_DEBUG("[Before DVFS] FHCTL%d_DDS: 0x%08x", pll_id,
		     (fh_read32(g_reg_dds[pll_id]) & MASK21b));

	/* 2. enable DVFS and Hopping control */
	{
		unsigned long reg_cfg = g_reg_cfg[pll_id];

		if (isFHCTL(pll_id)) {
			fh_set_field(reg_cfg, FH_SFSTRX_EN, 1);	/* enable dvfs mode */
			fh_set_field(reg_cfg, FH_FHCTLX_EN, 1);	/* enable hopping control */
		} else {
			/* for MCU FHCTL only */
			volatile unsigned int reg_val;

			reg_val = fh_read32(reg_cfg);
			reg_val |= (FH_SFSTRX_EN | FH_FHCTLX_EN);
			fh_write32(reg_cfg, reg_val);

			if (fh_read32(reg_cfg) != reg_val) {
				/* Retry again */
				fh_write32(reg_cfg, reg_val);
				FH_MSG("[Warning] MCU FHCTL CFG write retry 1");
				if (fh_read32(reg_cfg) != reg_val) {
					/* Retry again */
					fh_write32(reg_cfg, reg_val);
					FH_MSG("[Warning] MCU FHCTL CFG write retry 2");
				}
			}
			/* if */
		}		/* if-else */
	}

	/* for slope setting. */
	/* For FHCTL */
	fh_write32(REG_FHCTL_SLOPE0, 0x6003c97);
	fh_write32(REG_FHCTL_SLOPE1, 0x6003c97);

	/* FH_MSG("2. enable DVFS and Hopping control"); */

	/* 3. switch to hopping control */
	fh_switch2fhctl(pll_id, 1);
	mb();

	/* FH_MSG("3. switch to hopping control"); */

	/* 4. set DFS DDS */
	{
		unsigned long dvfs_req = g_reg_dvfs[pll_id];

		fh_write32(dvfs_req, (dds_value) | (BIT32));	/* set dds */

		/* FH_MSG("4. set DFS DDS"); */
		FH_MSG_DEBUG("[After DVFS] FHCTL%d_DDS: 0x%08x", pll_id,
			     (fh_read32(dvfs_req) & MASK21b));
		FH_MSG_DEBUG("FHCTL%d_DVFS: 0x%08x", pll_id, (fh_read32(dvfs_req) & MASK21b));
	}
	local_irq_restore(flags);

	/* 4.1 ensure jump to target DDS */
	wait_dds_stable(pll_id, dds_value, g_reg_mon[pll_id], 100);
	/* FH_MSG("4.1 ensure jump to target DDS"); */

	local_irq_save(flags);

	/* 5. write back to ncpo */
	/* FH_MSG("5. write back to ncpo"); */
	{
		unsigned long reg_dvfs = 0;
		unsigned long reg_pll_con1 = 0;


		reg_pll_con1 = g_reg_pll_con1[pll_id];
		reg_dvfs = g_reg_dvfs[pll_id];
		FH_MSG_DEBUG("PLL_CON1: 0x%08x", (fh_read32(reg_pll_con1) & MASK21b));

		fh_write32(reg_pll_con1, (fh_read32(g_reg_mon[pll_id]) & MASK21b)
			   | (fh_read32(reg_pll_con1) & 0xFFE00000) | (BIT32));
		FH_MSG_DEBUG("PLL_CON1: 0x%08x", (fh_read32(reg_pll_con1) & MASK21b));
	}

	/* 6. switch to register control */
	fh_switch2fhctl(pll_id, 0);
	mb();

	/* FH_MSG("6. switch to register control"); */
	local_irq_restore(flags);

	return 0;
}

static int mt_fh_hal_hopping_mcu(enum FH_PLL_ID pll_id, unsigned int dds_value)
{
	unsigned long flags = 0;

	FH_MSG_DEBUG("%s for pll %d:", __func__, pll_id);


  /*********************************************/
	local_irq_save(flags);

	{
		/* 1. sync ncpo to DDS of FHCTL */
		/* fh_sync_ncpo_to_fhctl_dds(pll_id); */
		volatile unsigned int reg_val;
		unsigned long reg_con1 = 0;
		unsigned long reg_dds = 0;

		reg_con1 = g_reg_pll_con1[pll_id];
		reg_dds = g_reg_dds[pll_id];
		reg_val = (mcu_fh_read32(reg_con1) & MASK21b);
		mcu_fh_write32(reg_dds, (reg_val | BIT32), MASK21b);
	}

	FH_MSG_DEBUG("[Before DVFS] FHCTL%d_DDS: 0x%08x", pll_id,
		     (mcu_fh_read32(g_reg_dds[pll_id]) & MASK21b));

	/* 2. enable DVFS and Hopping control */
	{
		unsigned long reg_cfg = g_reg_cfg[pll_id];

		/* for MCU FHCTL only */
		volatile unsigned int reg_val;

		/* reg_val = mcu_fh_read32(reg_cfg); */
		reg_val = (FH_SFSTRX_EN | FH_FHCTLX_EN);
		mcu_fh_write32(reg_cfg, reg_val, MASK32b);

	}


	/* For MCU FHCTL, only has slope0 */
	mcu_fh_write32(REG_MCU_FHCTL_SLOPE0, 0x6003c97, MASK32b);


	/* 3. switch to hopping control */
	{

		/* fh_switch2fhctl(pll_id, 1); */
		unsigned int mask = 0;

		mask = 0x1U << pllid_to_hp_con[pll_id];
		mcu_fh_set_field(REG_MCU_FHCTL_HP_EN, mask, 1);
	}

	mb();

	/* 4. set DFS DDS */
	{
		unsigned long dvfs_req = g_reg_dvfs[pll_id];

		mcu_fh_write32(dvfs_req, (dds_value | BIT32), MASK21b);	/* set dds */

		/* FH_MSG("4. set DFS DDS"); */
		FH_MSG_DEBUG("[After DVFS] FHCTL%d_DDS: 0x%08x", pll_id,
			     (mcu_fh_read32(dvfs_req) & MASK21b));
		FH_MSG_DEBUG("FHCTL%d_DVFS: 0x%08x", pll_id, (mcu_fh_read32(dvfs_req) & MASK21b));
	}
	local_irq_restore(flags);
    /*********************************************/

	/* 4.1 ensure jump to target DDS */
	wait_mcu_fh_dds_stable(pll_id, dds_value, g_reg_mon[pll_id], 100);


    /*********************************************/
	local_irq_save(flags);

	/* 5. write back to ncpo */
	{
		unsigned long reg_dvfs = 0;
		unsigned long reg_mon = 0;
		unsigned long reg_pll_con1 = 0;
		volatile unsigned int mon_reg_val;
		volatile unsigned int con1_reg_val;
		volatile unsigned int val;

		reg_pll_con1 = g_reg_pll_con1[pll_id];
		reg_dvfs = g_reg_dvfs[pll_id];
		reg_mon = g_reg_mon[pll_id];
		FH_MSG_DEBUG("PLL_CON1: 0x%08x", (mcu_fh_read32(reg_pll_con1) & MASK21b));

		/* Write MON DDS value to MCU_MIXEDSYS */
		mon_reg_val = mcu_fh_read32(reg_mon) & MASK21b;
		con1_reg_val = mcu_fh_read32(reg_pll_con1) & 0xFFE00000;
		val = mon_reg_val | con1_reg_val | (BIT32);
		mcu_fh_write32(reg_pll_con1, val, MASK21b);

		if ((mcu_fh_read32(reg_pll_con1) & MASK21b) != mon_reg_val) {
			FH_MSG("ERROR1 PLL:%d con1_reg:0x%lx mon_reg:0x%lx",
			       pll_id, reg_pll_con1, reg_mon);
			FH_MSG("ERROR1 PLL:%d dds_value:0x%08x mon_dds:0x%08x con1:0x%08x",
			       pll_id, dds_value, mon_reg_val, mcu_fh_read32(reg_pll_con1));
			fh_mcu_dumpregs_read(pll_id);
			BUG_ON(1);
		}

		if ((dds_value & MASK21b) != (mcu_fh_read32(reg_pll_con1) & MASK21b)) {
			FH_MSG("ERROR2 PLL:%d con1_reg:0x%lx mon_reg:0x%lx",
			       pll_id, reg_pll_con1, reg_mon);
			FH_MSG("ERROR2 PLL:%d dds_value:0x%08x mon_dds%08x con1:0x%08x",
			       pll_id, dds_value, mon_reg_val, mcu_fh_read32(reg_pll_con1));
			fh_mcu_dumpregs_read(pll_id);
			BUG_ON(1);
		}

		FH_MSG_DEBUG("PLL_CON1: 0x%08x", (mcu_fh_read32(reg_pll_con1) & MASK21b));
	}

	/* 6. switch to register control */
	{
		/* fh_switch2fhctl(pll_id, 0); */
		unsigned int mask = 0;

		mask = 0x1U << pllid_to_hp_con[pll_id];
		mcu_fh_set_field(REG_MCU_FHCTL_HP_EN, mask, 0);
	}
	mb();

	local_irq_restore(flags);
    /*********************************************/

	return 0;
}


/*
   armpll dfs mdoe
*/
static int mt_fh_hal_dfs_armpll(unsigned int coreid, unsigned int dds)
{
	/* unsigned long flags = 0; */
	unsigned long reg_cfg = 0;
	unsigned int pll = coreid;

	if (g_initialize == 0) {
		FH_MSG("(Warning) %s FHCTL isn't ready.", __func__);
		return -1;
	}

	if (dds > MAX_DDS) {
		FH_MSG("[ERROR] Overflow! [%s] [pll_id]:%d [dds]:0x%x", __func__, pll, dds);
		/* Check dds overflow (21 bit) */
		BUG_ON(1);
	}

	switch (pll) {
	case MCU_FH_PLL0:
	case MCU_FH_PLL1:
	case MCU_FH_PLL2:
	case MCU_FH_PLL3:
		break;
	default:
		FH_MSG("[ERROR] %s [pll_id]:%d is not CAXPLL. ", __func__, pll);
		BUG_ON(1);
		return 1;
	};


  /***************************************************/
	reg_cfg = g_reg_cfg[pll];

	mt6797_0x1001AXXX_lock();
	/* spin_lock(&g_fh_lock); */

	/* MCU FHCTL reg should read two times, so add disable IRQ to protect. */
	mcu_fh_write32(reg_cfg, 0, MASK32b);	/* disable SSC mode, disable dvfs mode  and disable hopping control */

	mt_fh_hal_hopping_mcu(pll, dds);

	mcu_fh_write32(reg_cfg, 0, MASK32b);	/* disable SSC mode, disable dvfs mode  and disable hopping control */

	/* spin_unlock(&g_fh_lock); */
	mt6797_0x1001AXXX_unlock();
  /***************************************************/

	return 0;
}

/* [For Everest] GPU CLK hopping, MFGPLL */
static int mt_fh_hal_dfs_mmpll(unsigned int target_dds)
{
	/* unsigned long flags = 0; */
	/* [For Everest] MFGPLL, confirmed with GPU CLK owner Owen.Chen */
	const unsigned int pll_id = FH_PLL5;
	const unsigned long reg_cfg = g_reg_cfg[pll_id];

	if (g_initialize == 0) {
		FH_MSG("(Warning) %s FHCTL isn't ready. ", __func__);
		return -1;
	}

	if (target_dds > MAX_DDS) {
		FH_MSG("[ERROR] Overflow! [%s] [pll_id]:%d [dds]:0x%x", __func__, pll_id,
		       target_dds);
		/* Check dds overflow (21 bit) */
		BUG_ON(1);
	}

	FH_MSG("%s, current dds(MMPLL_CON1): 0x%x, target dds %d",
	       __func__, (fh_read32(g_reg_pll_con1[pll_id]) & MASK21b), target_dds);

	spin_lock(&g_fh_lock);

	if (g_fh_pll[pll_id].fh_status == FH_FH_ENABLE_SSC) {
		unsigned int pll_dds = 0;
		unsigned int fh_dds = 0;

		/* only when SSC is enable, turn off MEMPLL hopping */
		fh_set_field(reg_cfg, FH_FRDDSX_EN, 0);	/* disable SSC mode */
		fh_set_field(reg_cfg, FH_SFSTRX_EN, 0);	/* disable dvfs mode */
		fh_set_field(reg_cfg, FH_FHCTLX_EN, 0);	/* disable hopping control */

		pll_dds = (fh_read32(g_reg_dds[pll_id])) & MASK21b;
		fh_dds = (fh_read32(g_reg_mon[pll_id])) & MASK21b;

		FH_MSG(">p:f< %x:%x", pll_dds, fh_dds);

		wait_dds_stable(pll_id, pll_dds, g_reg_mon[pll_id], 100);
	}


	FH_MSG("target dds: 0x%x", target_dds);
	mt_fh_hal_hopping(pll_id, target_dds);

	if (g_fh_pll[pll_id].fh_status == FH_FH_ENABLE_SSC) {
		const struct freqhopping_ssc *p_setting =
		    &g_pll_ssc_setting_tbl[pll_id][PLL_SETTING_IDX__DEF];

		fh_set_field(reg_cfg, FH_FRDDSX_EN, 0);	/* disable SSC mode */
		fh_set_field(reg_cfg, FH_SFSTRX_EN, 0);	/* disable dvfs mode */
		fh_set_field(reg_cfg, FH_FHCTLX_EN, 0);	/* disable hopping control */

		fh_sync_ncpo_to_fhctl_dds(pll_id);

		FH_MSG("Enable GPU PLL SSC mode");
		FH_MSG("DDS: 0x%08x", (fh_read32(g_reg_dds[pll_id]) & MASK21b));

		fh_set_field(reg_cfg, MASK_FRDDSX_DYS, p_setting->df);
		fh_set_field(reg_cfg, MASK_FRDDSX_DTS, p_setting->dt);

		fh_write32(g_reg_updnlmt[pll_id],
			   (PERCENT_TO_DDSLMT
			    ((fh_read32(g_reg_dds[pll_id]) & MASK21b), p_setting->lowbnd) << 16));
		FH_MSG("UPDNLMT: 0x%08x", fh_read32(g_reg_updnlmt[pll_id]));

		fh_switch2fhctl(pll_id, 1);

		fh_set_field(reg_cfg, FH_FRDDSX_EN, 1);	/* enable SSC mode */
		fh_set_field(reg_cfg, FH_FHCTLX_EN, 1);	/* enable hopping control */

		FH_MSG("CFG: 0x%08x", fh_read32(reg_cfg));

	}
	spin_unlock(&g_fh_lock);

	return 0;
}

static int mt_fh_hal_dfs_vencpll(unsigned int target_freq)
{
	FH_BUG_ON(1);
	return 0;
}

static int mt_fh_hal_l2h_dvfs_mempll(void)
{
	FH_BUG_ON(1);
	return 0;
}

static int mt_fh_hal_h2l_dvfs_mempll(void)
{
	FH_BUG_ON(1);
	return 0;
}

static int mt_fh_hal_dram_overclock(int clk)
{
	FH_BUG_ON(1);
	return 0;
}

static int mt_fh_hal_get_dramc(void)
{
	FH_BUG_ON(1);
	return 0;
}

/* General purpose PLL hopping and SSC enable API. */
static int mt_fh_hal_general_pll_dfs(enum FH_PLL_ID pll_id, unsigned int target_dds)
{
	const unsigned long reg_cfg = g_reg_cfg[pll_id];

	VALIDATE_PLLID(pll_id);

	if (g_initialize == 0) {
		FH_MSG("(Warning) %s FHCTL isn't ready. ", __func__);
		return -1;
	}

	if (target_dds > MAX_DDS) {
		FH_MSG("[ERROR] Overflow! [%s] [pll_id]:%d [dds]:0x%x", __func__, pll_id,
		       target_dds);
		/* Check dds overflow (21 bit) */
		BUG_ON(1);
	}

	/* [Everest Only] All new platform should confirm again!!! */
	switch (pll_id) {
	case MCU_FH_PLL0:	/* MCU */
	case MCU_FH_PLL1:
	case MCU_FH_PLL2:
	case MCU_FH_PLL3:
	case FH_PLL1:		/* MPLL for DRAMC */
	case FH_PLL3:		/* MEMPLL */
		FH_MSG("ERROR! The [PLL_ID]:%d was forbidden hopping by Everest FHCTL.", pll_id);
		BUG_ON(1);
		break;
	default:
		break;
	}

	FH_MSG("%s, [Pll_ID]:%d [current dds(CON1)]:0x%x, [target dds]:%d",
	       __func__, pll_id, (fh_read32(g_reg_pll_con1[pll_id]) & MASK21b), target_dds);

	spin_lock(&g_fh_lock);

	if (g_fh_pll[pll_id].fh_status == FH_FH_ENABLE_SSC) {
		unsigned int pll_dds = 0;
		unsigned int fh_dds = 0;

		/* only when SSC is enable, turn off MEMPLL hopping */
		fh_set_field(reg_cfg, FH_FRDDSX_EN, 0);	/* disable SSC mode */
		fh_set_field(reg_cfg, FH_SFSTRX_EN, 0);	/* disable dvfs mode */
		fh_set_field(reg_cfg, FH_FHCTLX_EN, 0);	/* disable hopping control */

		pll_dds = (fh_read32(g_reg_dds[pll_id])) & MASK21b;
		fh_dds = (fh_read32(g_reg_mon[pll_id])) & MASK21b;

		/* FH_MSG(">p:f< %x:%x", pll_dds, fh_dds); */

		wait_dds_stable(pll_id, pll_dds, g_reg_mon[pll_id], 100);
	}

	mt_fh_hal_hopping(pll_id, target_dds);

	if (g_fh_pll[pll_id].fh_status == FH_FH_ENABLE_SSC) {
		const struct freqhopping_ssc *p_setting =
		    &g_pll_ssc_setting_tbl[pll_id][PLL_SETTING_IDX__DEF];

		fh_set_field(reg_cfg, FH_FRDDSX_EN, 0);	/* disable SSC mode */
		fh_set_field(reg_cfg, FH_SFSTRX_EN, 0);	/* disable dvfs mode */
		fh_set_field(reg_cfg, FH_FHCTLX_EN, 0);	/* disable hopping control */

		fh_sync_ncpo_to_fhctl_dds(pll_id);

		/* FH_MSG("Enable PLL SSC mode"); */
		/* FH_MSG("DDS: 0x%08x", (fh_read32(g_reg_dds[pll_id]) & MASK21b)); */

		fh_set_field(reg_cfg, MASK_FRDDSX_DYS, p_setting->df);
		fh_set_field(reg_cfg, MASK_FRDDSX_DTS, p_setting->dt);

		fh_write32(g_reg_updnlmt[pll_id],
			   (PERCENT_TO_DDSLMT
			    ((fh_read32(g_reg_dds[pll_id]) & MASK21b), p_setting->lowbnd) << 16));
		/* FH_MSG("UPDNLMT: 0x%08x", fh_read32(g_reg_updnlmt[pll_id])); */

		fh_switch2fhctl(pll_id, 1);

		fh_set_field(reg_cfg, FH_FRDDSX_EN, 1);	/* enable SSC mode */
		fh_set_field(reg_cfg, FH_FHCTLX_EN, 1);	/* enable hopping control */

		/* FH_MSG("CFG: 0x%08x", fh_read32(reg_cfg)); */

	}
	spin_unlock(&g_fh_lock);

	return 0;

}

static void mt_fh_hal_popod_save(void)
{
	/* [ For Everest] */
	/* MAINPLL Only, This is legacy function. Nobody know what purpose is. */
	const unsigned int pll_id = FH_PLL2;

	FH_MSG_DEBUG("EN: %s", __func__);

	/* disable maipll SSC mode */
	if (g_fh_pll[pll_id].fh_status == FH_FH_ENABLE_SSC) {
		unsigned int fh_dds = 0;
		unsigned int pll_dds = 0;
		const unsigned long reg_cfg = g_reg_cfg[pll_id];

		/* only when SSC is enable, turn off MAINPLL hopping */
		fh_set_field(reg_cfg, FH_FRDDSX_EN, 0);	/* disable SSC mode */
		fh_set_field(reg_cfg, FH_SFSTRX_EN, 0);	/* disable dvfs mode */
		fh_set_field(reg_cfg, FH_FHCTLX_EN, 0);	/* disable hopping control */

		pll_dds = (fh_read32(g_reg_dds[pll_id])) & MASK21b;
		fh_dds = (fh_read32(g_reg_mon[pll_id])) & MASK21b;

		FH_MSG("Org pll_dds:%x fh_dds:%x", pll_dds, fh_dds);

		wait_dds_stable(pll_id, pll_dds, g_reg_mon[pll_id], 100);


		/* write back to ncpo, only for MAINPLL. */
		/* [For Everest] */
		fh_write32(g_reg_pll_con1[pll_id],
			   (fh_read32(g_reg_dds[pll_id]) & MASK21b) |
			   ((fh_read32(REG_FH_PLL2_CON1) & 0xFFE00000)) | (BIT32));

		FH_MSG("MAINPLL_CON1: 0x%08x", (fh_read32(g_reg_pll_con1[pll_id]) & MASK21b));

		/* switch to register control */
		fh_switch2fhctl(pll_id, 0);

		mb();
	}
}


static void mt_fh_hal_popod_restore(void)
{
	/* [ For Everest] */
	/* MAINPLL Only, This is legacy function. Nobody know what purpose is. */
	const unsigned int pll_id = FH_PLL2;

	FH_MSG_DEBUG("EN: %s", __func__);

	/* enable maipll SSC mode */
	if (g_fh_pll[pll_id].fh_status == FH_FH_ENABLE_SSC) {

		/* Default setting index is 2 */
		const struct freqhopping_ssc *p_setting =
		    &g_pll_ssc_setting_tbl[pll_id][PLL_SETTING_IDX__DEF];
		const unsigned long reg_cfg = g_reg_cfg[pll_id];

		fh_set_field(reg_cfg, FH_FRDDSX_EN, 0);	/* disable SSC mode */
		fh_set_field(reg_cfg, FH_SFSTRX_EN, 0);	/* disable dvfs mode */
		fh_set_field(reg_cfg, FH_FHCTLX_EN, 0);	/* disable hopping control */

		fh_sync_ncpo_to_fhctl_dds(pll_id);

		FH_MSG("Enable mainpll SSC mode");
		FH_MSG("sync ncpo to DDS of FHCTL");
		FH_MSG("FHCTL1_DDS: 0x%08x", (fh_read32(g_reg_dds[pll_id]) & MASK21b));

		fh_set_field(reg_cfg, MASK_FRDDSX_DYS, p_setting->df);
		fh_set_field(reg_cfg, MASK_FRDDSX_DTS, p_setting->dt);

		fh_write32(g_reg_updnlmt[pll_id],
			   (PERCENT_TO_DDSLMT
			    ((fh_read32(g_reg_dds[pll_id]) & MASK21b), p_setting->lowbnd) << 16));
		FH_MSG("REG_FHCTL2_UPDNLMT: 0x%08x", fh_read32(g_reg_updnlmt[pll_id]));

		fh_switch2fhctl(pll_id, 1);

		fh_set_field(reg_cfg, FH_FRDDSX_EN, 1);	/* enable SSC mode */
		fh_set_field(reg_cfg, FH_FHCTLX_EN, 1);	/* enable hopping control */

		FH_MSG("REG_FHCTL2_CFG: 0x%08x", fh_read32(reg_cfg));
	}
}

static int fh_dvfs_proc_read(struct seq_file *m, void *v)
{
	int i = 0;

	FH_MSG("EN: %s", __func__);

	seq_puts(m, "DVFS:\r\n");
	seq_puts(m, "CFG: 0x3 is SSC mode;  0x5 is DVFS mode \r\n");

	mt6797_0x1001AXXX_lock();
	for (i = 0; i <= MCU_FH_PLL3; ++i) {
		seq_printf(m, "MCU FHCTL%d:   CFG:0x%08x    DVFS:0x%08x\r\n",
			   i, mcu_fh_read32(g_reg_cfg[i]), mcu_fh_read32(g_reg_dvfs[i]));
	}
	mt6797_0x1001AXXX_unlock();

	for (i = FH_PLL0; i < FH_PLL_NUM; ++i) {
		seq_printf(m, "FHCTL%d:   CFG:0x%08x    DVFS:0x%08x\r\n",
			   i, fh_read32(g_reg_cfg[i]), fh_read32(g_reg_dvfs[i]));
	}
	return 0;
}

static int fh_dvfs_proc_write(struct file *file, const char *buffer, unsigned long count,
			      void *data)
{
	unsigned int p1, p2, p3, p4, p5;

	p1 = p2 = p3 = p4 = p5 = 0;

	FH_MSG("EN: %s", __func__);

	if (count == 0)
		return -1;

	FH_MSG("EN: p1=%d p2=%d p3=%d", p1, p2, p3);

	switch (p1) {
	case MCU_FH_PLL0:
	case MCU_FH_PLL1:
	case MCU_FH_PLL2:
	case MCU_FH_PLL3:
		mt_fh_hal_dfs_armpll(p2, p3);
		FH_MSG("ARMCA7PLL DVFS completed\n");
		break;
	case 4370:		/* TODO: and what this case for? Nobody know! */
		{
			unsigned long reg_cfg = 0;

			VALIDATE_PLLID(p2);

			reg_cfg = g_reg_cfg[p2];

			FH_MSG("pllid=%d dt=%d df=%d lowbnd=%d", p2, p3, p4, p5);
			fh_set_field(reg_cfg, FH_FRDDSX_EN, 0);	/* disable SSC mode */
			fh_set_field(reg_cfg, FH_SFSTRX_EN, 0);	/* disable dvfs mode */
			fh_set_field(reg_cfg, FH_FHCTLX_EN, 0);	/* disable hopping control */

			fh_sync_ncpo_to_fhctl_dds(p2);

			FH_MSG("Enable FHCTL%d SSC mode", p2);
			FH_MSG("DDS: 0x%08x", (fh_read32(reg_cfg) & MASK21b));

			fh_set_field(reg_cfg, MASK_FRDDSX_DYS, p4);
			fh_set_field(reg_cfg, MASK_FRDDSX_DTS, p3);

			fh_write32(g_reg_updnlmt[p2],
				   (PERCENT_TO_DDSLMT((fh_read32(reg_cfg) & MASK21b), p5) << 16));
			FH_MSG("UPDNLMT: 0x%08x", fh_read32(g_reg_updnlmt[p2]));

			fh_switch2fhctl(p2, 1);

			fh_set_field(reg_cfg, FH_FRDDSX_EN, 1);	/* enable SSC mode */
			fh_set_field(reg_cfg, FH_FHCTLX_EN, 1);	/* enable hopping control */

			FH_MSG("CFG: 0x%08x", fh_read32(reg_cfg));
		}
		break;
	case 2222:
		/* TODO: and what this case for? Nobody know! */
		if (p2 == 0)	/* disable */
			mt_fh_hal_popod_save();
		else if (p2 == 1)	/* enable */
			mt_fh_hal_popod_restore();
		break;
	default:
		spin_lock(&g_fh_lock);
		mt_fh_hal_hopping(p1, p2);
		spin_unlock(&g_fh_lock);
		break;
	};

	return count;
}

/* #define UINT_MAX (unsigned int)(-1) */
static int fh_dumpregs_proc_read(struct seq_file *m, void *v)
{
	int i = 0;
	unsigned long flags = 0;
	static unsigned int dds_max[FH_PLL_NUM] = { 0 };
	static unsigned int dds_min[FH_PLL_NUM] = { 0 };

	if (g_initialize != 1) {
		FH_MSG("[ERROR] %s fhctl didn't init. Please check!!!", __func__);
		return -1;
	}

	FH_MSG("EN: %s", __func__);

	mt6797_0x1001AXXX_lock();
	spin_lock(&g_fh_lock);

	local_irq_save(flags);

  /***************** MCU FCHTL *****************/
	for (i = 0; i <= MCU_FH_PLL3; ++i) {
		const unsigned int mon = mcu_fh_read32(g_reg_mon[i]);
		const unsigned int dds = mon & MASK21b;

		seq_printf(m, "MCU_FHCTL%d CFG, UPDNLMT, DVFS, DDS, MON\r\n", i);
		seq_printf(m, "0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\r\n",
			   mcu_fh_read32(g_reg_cfg[i]), mcu_fh_read32(g_reg_updnlmt[i]),
			   mcu_fh_read32(g_reg_dvfs[i]), mcu_fh_read32(g_reg_dds[i]), mon);

		if (dds > dds_max[i])
			dds_max[i] = dds;
		if ((dds < dds_min[i]) || (dds_min[i] == 0))
			dds_min[i] = dds;
	}

	seq_printf(m, "\r\nMCU_FHCTL_HP_EN:\r\n0x%08x\r\n", mcu_fh_read32(REG_MCU_FHCTL_HP_EN));
	seq_printf(m, "\r\nMCU_FHCTL_CLK_CON:\r\n0x%08x\r\n", mcu_fh_read32(REG_MCU_FHCTL_CLK_CON));

	seq_puts(m, "\r\nPLL_CON0 :\r\n");
	for (i = 0; i <= MCU_FH_PLL3; ++i)
		seq_printf(m, "PLL%d;0x%08x ", i, mcu_fh_read32(g_reg_pll_con0[i]));


	seq_puts(m, "\r\nPLL_CON1 :\r\n");
	for (i = 0; i <= MCU_FH_PLL3; ++i)
		seq_printf(m, "PLL%d;0x%08x ", i, mcu_fh_read32(g_reg_pll_con1[i]));


	seq_puts(m, "\r\nRecorded dds range\r\n");

	for (i = 0; i < MCU_FH_PLL3; ++i)
		seq_printf(m, "Pll%d dds max 0x%06x, min 0x%06x\r\n", i, dds_max[i], dds_min[i]);


  /***************** Original FCHTL *****************/
	for (i = FH_PLL0; i < FH_PLL_NUM; ++i) {
		const unsigned int mon = fh_read32(g_reg_mon[i]);
		const unsigned int dds = mon & MASK21b;

		seq_printf(m, "FHCTL%d CFG, UPDNLMT, DVFS, DDS, MON\r\n", i);
		seq_printf(m, "0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\r\n",
			   fh_read32(g_reg_cfg[i]), fh_read32(g_reg_updnlmt[i]),
			   fh_read32(g_reg_dvfs[i]), fh_read32(g_reg_dds[i]), mon);

		if (dds > dds_max[i])
			dds_max[i] = dds;
		if ((dds < dds_min[i]) || (dds_min[i] == 0))
			dds_min[i] = dds;
	}

	seq_printf(m, "\r\nFHCTL_HP_EN:\r\n0x%08x\r\n", fh_read32(REG_FHCTL_HP_EN));
	seq_printf(m, "\r\nFHCTL_CLK_CON:\r\n0x%08x\r\n", fh_read32(REG_FHCTL_CLK_CON));

	seq_puts(m, "\r\nPLL_CON0 :\r\n");
	for (i = FH_PLL0; i < FH_PLL_NUM; ++i)
		seq_printf(m, "PLL%d;0x%08x ", i, fh_read32(g_reg_pll_con0[i]));


	seq_puts(m, "\r\nPLL_CON1 :\r\n");
	for (i = FH_PLL0; i < FH_PLL_NUM; ++i)
		seq_printf(m, "PLL%d;0x%08x ", i, fh_read32(g_reg_pll_con1[i]));



	seq_puts(m, "\r\nRecorded dds range\r\n");

	for (i = FH_PLL0; i < FH_PLL_NUM; ++i)
		seq_printf(m, "Pll%d dds max 0x%06x, min 0x%06x\r\n", i, dds_max[i], dds_min[i]);


  /**************************************************/
	local_irq_restore(flags);

	spin_unlock(&g_fh_lock);
	mt6797_0x1001AXXX_unlock();

	return 0;
}


static void __reg_tbl_init(void)
{
	int id = 0;

    /****************************************/
	/* Should porting for specific platform. */
    /****************************************/

	const unsigned long reg_dds[] = {
		REG_MCU_FHCTL0_DDS, REG_MCU_FHCTL1_DDS, REG_MCU_FHCTL2_DDS, REG_MCU_FHCTL3_DDS,
		REG_FHCTL0_DDS, REG_FHCTL1_DDS, REG_FHCTL2_DDS, REG_FHCTL3_DDS,
		REG_FHCTL4_DDS, REG_FHCTL5_DDS, REG_FHCTL6_DDS, REG_FHCTL7_DDS,
		REG_FHCTL8_DDS
	};

	const unsigned long reg_cfg[] = {
		REG_MCU_FHCTL0_CFG, REG_MCU_FHCTL1_CFG, REG_MCU_FHCTL2_CFG, REG_MCU_FHCTL3_CFG,
		REG_FHCTL0_CFG, REG_FHCTL1_CFG, REG_FHCTL2_CFG, REG_FHCTL3_CFG,
		REG_FHCTL4_CFG, REG_FHCTL5_CFG, REG_FHCTL6_CFG, REG_FHCTL7_CFG,
		REG_FHCTL8_CFG
	};

	const unsigned long reg_updnlmt[] = {
		REG_MCU_FHCTL0_UPDNLMT, REG_MCU_FHCTL1_UPDNLMT, REG_MCU_FHCTL2_UPDNLMT,
		REG_MCU_FHCTL3_UPDNLMT,
		REG_FHCTL0_UPDNLMT, REG_FHCTL1_UPDNLMT, REG_FHCTL2_UPDNLMT, REG_FHCTL3_UPDNLMT,
		REG_FHCTL4_UPDNLMT, REG_FHCTL5_UPDNLMT, REG_FHCTL6_UPDNLMT, REG_FHCTL7_UPDNLMT,
		REG_FHCTL8_UPDNLMT
	};

	const unsigned long reg_mon[] = {
		REG_MCU_FHCTL0_MON, REG_MCU_FHCTL1_MON, REG_MCU_FHCTL2_MON, REG_MCU_FHCTL3_MON,
		REG_FHCTL0_MON, REG_FHCTL1_MON, REG_FHCTL2_MON, REG_FHCTL3_MON,
		REG_FHCTL4_MON, REG_FHCTL5_MON, REG_FHCTL6_MON, REG_FHCTL7_MON,
		REG_FHCTL8_MON
	};

	const unsigned long reg_dvfs[] = {
		REG_MCU_FHCTL0_DVFS, REG_MCU_FHCTL1_DVFS, REG_MCU_FHCTL2_DVFS, REG_MCU_FHCTL3_DVFS,
		REG_FHCTL0_DVFS, REG_FHCTL1_DVFS, REG_FHCTL2_DVFS, REG_FHCTL3_DVFS,
		REG_FHCTL4_DVFS, REG_FHCTL5_DVFS, REG_FHCTL6_DVFS, REG_FHCTL7_DVFS,
		REG_FHCTL8_DVFS
	};

	const unsigned long reg_pll_con0[] = {
		REG_MCU_FH_PLL0_CON0, REG_MCU_FH_PLL1_CON0, REG_MCU_FH_PLL2_CON0,
		REG_MCU_FH_PLL3_CON0,
		REG_FH_PLL0_CON0, REG_FH_PLL1_CON0, REG_FH_PLL2_CON0, REG_FH_PLL3_CON0,
		REG_FH_PLL4_CON0, REG_FH_PLL5_CON0, REG_FH_PLL6_CON0, REG_FH_PLL7_CON0,
		REG_FH_PLL8_CON0
	};

	const unsigned long reg_pll_con1[] = {
		REG_MCU_FH_PLL0_CON1, REG_MCU_FH_PLL1_CON1, REG_MCU_FH_PLL2_CON1,
		REG_MCU_FH_PLL3_CON1,
		REG_FH_PLL0_CON1, REG_FH_PLL1_CON1, REG_FH_PLL2_CON1, REG_FH_PLL3_CON1,
		REG_FH_PLL4_CON1, REG_FH_PLL5_CON1, REG_FH_PLL6_CON1, REG_FH_PLL7_CON1,
		REG_FH_PLL8_CON1
	};

    /****************************************/

	FH_MSG_DEBUG("EN: %s", __func__);


	for (id = 0; id < FH_PLL_NUM; ++id) {
		g_reg_dds[id] = reg_dds[id];
		g_reg_cfg[id] = reg_cfg[id];
		g_reg_updnlmt[id] = reg_updnlmt[id];
		g_reg_mon[id] = reg_mon[id];
		g_reg_dvfs[id] = reg_dvfs[id];
		g_reg_pll_con0[id] = reg_pll_con0[id];
		g_reg_pll_con1[id] = reg_pll_con1[id];
	}
}

/* Device Tree Initialize */
static int __reg_base_addr_init(void)
{
	struct device_node *fhctl_node;
	/* struct device_node *mcu_fhctl_node; */
	struct device_node *apmixed_node;
	/* struct device_node *mcumixed_node; */

	FH_MSG("(b) g_fhctl_base:0x%lx", (unsigned long)g_fhctl_base);
	FH_MSG("(b) g_mcu_fhctl_base:0x%lx", (unsigned long)g_mcu_fhctl_base);
	FH_MSG("(b) g_apmixed_base:0x%lx", (unsigned long)g_apmixed_base);
	FH_MSG("(b) g_mcumixed_base:0x%lx", (unsigned long)g_mcumixed_base);


	/* Init FHCTL base address */
	fhctl_node = of_find_compatible_node(NULL, NULL, "mediatek,fhctl");
	g_fhctl_base = of_iomap(fhctl_node, 0);
	if (!g_fhctl_base) {
		FH_MSG_DEBUG("Error, FHCTL iomap failed");
		BUG_ON(1);
	}
#if 0				/* move to init_mcumixedsys_base_addr() */
	/* Init MCU FHCTL base address */
	mcu_fhctl_node = of_find_compatible_node(NULL, NULL, "mediatek,mcufhctl");
	g_mcu_fhctl_base = of_iomap(mcu_fhctl_node, 0);
	if (!g_mcu_fhctl_base) {
		FH_MSG_DEBUG("Error, MCU FHCTL iomap failed");
		BUG_ON(1);
	}
#endif

	/* Init APMIXED base address */
	apmixed_node = of_find_compatible_node(NULL, NULL, "mediatek,apmixed");
	g_apmixed_base = of_iomap(apmixed_node, 0);
	if (!g_apmixed_base) {
		FH_MSG_DEBUG("Error, APMIXED iomap failed");
		BUG_ON(1);
	}
#if 0				/* move to init_mcumixedsys_base_addr() */
	/* Init APMIXED base address */
	mcumixed_node = of_find_compatible_node(NULL, NULL, "mediatek,mcumixed");
	g_mcumixed_base = of_iomap(mcumixed_node, 0);
	if (!g_mcumixed_base) {
		FH_MSG_DEBUG("Error, MCUMIXED iomap failed");
		BUG_ON(1);
	}
#endif

	FH_MSG("g_fhctl_base:0x%lx", (unsigned long)g_fhctl_base);
	FH_MSG("g_mcu_fhctl_base:0x%lx", (unsigned long)g_mcu_fhctl_base);
	FH_MSG("g_apmixed_base:0x%lx", (unsigned long)g_apmixed_base);
	FH_MSG("g_mcumixed_base:0x%lx", (unsigned long)g_mcumixed_base);


	__reg_tbl_init();

	return 0;
}

static void __global_var_init(void)
{

}

/* TODO: __init void mt_freqhopping_init(void) */
static void mt_fh_hal_init(void)
{
	int i = 0;


	FH_MSG_DEBUG("EN: %s", __func__);

	if (g_initialize == 1)
		return;

	/* Init relevant register base address by device tree */
	__reg_base_addr_init();

	/* Global Variable Init */
	__global_var_init();

	g_initialize = 1;

	/* FHCTL IP Init */
	for (i = 0; i < FH_PLL_NUM; ++i) {

		unsigned int mask;

		if (isFHCTL(i)) {
			spin_lock(&g_fh_lock);
			/* For FHCTL */
			/* Turn on all clock */
			mask = 1 << (i - FH_PLL0);

			fh_set_field(REG_FHCTL_CLK_CON, mask, 1);

			/* Release software-reset to reset */
			fh_set_field(REG_FHCTL_RST_CON, mask, 0);
			fh_set_field(REG_FHCTL_RST_CON, mask, 1);

			g_fh_pll[i].setting_id = 0;
			fh_write32(g_reg_cfg[i], 0x00000000);	/* No SSC and FH enabled */
			fh_write32(g_reg_updnlmt[i], 0x00000000);	/* clear all the settings */
			fh_write32(g_reg_dds[i], 0x00000000);	/* clear all the settings */

			spin_unlock(&g_fh_lock);
		} else {
			mt6797_0x1001AXXX_lock();

			mask = 1 << i;

			/* For MCU FHCTL [For Everest] */
			/* Turn on all clock */
			mcu_fh_set_field(REG_MCU_FHCTL_CLK_CON, mask, 1);

			/* Release software-reset to reset */
			mcu_fh_set_field(REG_MCU_FHCTL_RST_CON, mask, 0);
			mcu_fh_set_field(REG_MCU_FHCTL_RST_CON, mask, 1);
			g_fh_pll[i].setting_id = 0;
			mcu_fh_write32(g_reg_cfg[i], 0x00000000, 0);	/* No SSC and FH enabled */
			mcu_fh_write32(g_reg_updnlmt[i], 0x00000000, 0);	/* clear all the settings */
			mcu_fh_write32(g_reg_dds[i], 0x00000000, 0);	/* clear all the settings */

			mt6797_0x1001AXXX_unlock();
		}		/* if-else */
	}			/* for */

	FH_MSG("mt_fh_hal_init done");
}

static void mt_fh_hal_lock(unsigned long *flags)
{
	spin_lock(&g_fh_lock);
}

static void mt_fh_hal_unlock(unsigned long *flags)
{
	spin_unlock(&g_fh_lock);
}


static int mt_fh_hal_get_init(void)
{
	return g_initialize;
}

static int mt_fh_hal_is_support_DFS_mode(void)
{
	return true;
}

/* TODO: module_init(mt_freqhopping_init); */
/* TODO: module_exit(cpufreq_exit); */

/* Engineer mode will use the proc msg to create UI!!! */
static int __fh_debug_proc_read(struct seq_file *m, void *v, fh_pll_t *pll)
{
	int id;

	FH_MSG("EN: %s", __func__);

	/* [WWK] Should remove PLL name to save porting time. */
	/* [WWK] Could print ENG ID and PLL mapping */

	seq_puts(m, "\r\n[freqhopping debug flag]\r\n");
	seq_puts(m, "[1st Status] FH_FH_UNINIT:0, FH_FH_DISABLE: 1, FH_FH_ENABLE_SSC:2 \r\n");
	seq_puts(m, "[2nd Setting_id] Disable:0, Default:1, PLL_SETTING_IDX__USER:9 \r\n");
	seq_puts(m, "===============================================\r\n");

    /****** String Format sensitive for EM mode ******/
	seq_puts(m, "id");
	for (id = 0; id < FH_PLL_NUM; ++id)
		seq_printf(m, "=%s", g_pll_name[id]);

	seq_puts(m, "\r\n");



	for (id = 0; id < FH_PLL_NUM; ++id) {
		/* "  =%04d==%04d==%04d==%04d=\r\n" */
		if (id == 0)
			seq_puts(m, "  =");
		else
			seq_puts(m, "==");

		seq_printf(m, "%04d", pll[id].fh_status);

		if (id == (FH_PLL_NUM - 1))
			seq_puts(m, "=");
	}
	seq_puts(m, "\r\n");


	for (id = 0; id < FH_PLL_NUM; ++id) {
		/* "  =%04d==%04d==%04d==%04d=\r\n" */
		if (id == 0)
			seq_puts(m, "  =");
		else
			seq_puts(m, "==");

		seq_printf(m, "%04d", pll[id].setting_id);

		if (id == (FH_PLL_NUM - 1))
			seq_puts(m, "=");
	}
    /*************************************************/

	seq_puts(m, "\r\n");

	return 0;
}


/* *********************************************************************** */
/* This function would support special request. */
/* [History] */
/* We implement API mt_freqhopping_devctl() to */
/* complete -2~-4% SSC. (DVFS to -2% freq and enable 0~-2% SSC) */
/*  */
/* *********************************************************************** */
static int fh_ioctl_dvfs_ssc(unsigned int ctlid, void *arg)
{
	struct freqhopping_ioctl *fh_ctl = arg;

	mt6797_0x1001AXXX_lock();
	spin_lock(&g_fh_lock);

	switch (ctlid) {
	case FH_DCTL_CMD_DVFS:	/* < PLL DVFS */
		{
			if (isFHCTL(fh_ctl->pll_id))
				mt_fh_hal_hopping(fh_ctl->pll_id, fh_ctl->ssc_setting.dds);
			else
				mt_fh_hal_hopping_mcu(fh_ctl->pll_id, fh_ctl->ssc_setting.dds);
		}
		break;
	case FH_DCTL_CMD_DVFS_SSC_ENABLE:	/* PLL DVFS and enable SSC */
		{
			__disable_ssc(fh_ctl->pll_id, &(fh_ctl->ssc_setting));

			if (isFHCTL(fh_ctl->pll_id))
				mt_fh_hal_hopping(fh_ctl->pll_id, fh_ctl->ssc_setting.dds);
			else
				BUG_ON(1);
			__enable_ssc(fh_ctl->pll_id, &(fh_ctl->ssc_setting));
		}
		break;
	case FH_DCTL_CMD_DVFS_SSC_DISABLE:	/* PLL DVFS and disable SSC */
		{
			__disable_ssc(fh_ctl->pll_id, &(fh_ctl->ssc_setting));

			if (isFHCTL(fh_ctl->pll_id))
				mt_fh_hal_hopping(fh_ctl->pll_id, fh_ctl->ssc_setting.dds);
			else
				BUG_ON(1);

		}
		break;
	case FH_DCTL_CMD_SSC_ENABLE:	/* SSC enable */
		{
			if (!isFHCTL(fh_ctl->pll_id))
				BUG_ON(1);

			__enable_ssc(fh_ctl->pll_id, &(fh_ctl->ssc_setting));
		}
		break;
	case FH_DCTL_CMD_SSC_DISABLE:	/* SSC disable */
		{
			if (!isFHCTL(fh_ctl->pll_id))
				BUG_ON(1);

			__disable_ssc(fh_ctl->pll_id, &(fh_ctl->ssc_setting));
		}
		break;
	case FH_DCTL_CMD_GENERAL_DFS:
		{
			if (!isFHCTL(fh_ctl->pll_id))
				BUG_ON(1);

			mt_fh_hal_general_pll_dfs(fh_ctl->pll_id, fh_ctl->ssc_setting.dds);
		}
		break;
	default:
		break;
	};

	spin_unlock(&g_fh_lock);
	mt6797_0x1001AXXX_unlock();

	return 0;
}


static void __ioctl(unsigned int ctlid, void *arg)
{
	switch (ctlid) {
	case FH_IO_PROC_READ:
		{
			FH_IO_PROC_READ_T *tmp = (FH_IO_PROC_READ_T *) (arg);

			__fh_debug_proc_read(tmp->m, tmp->v, tmp->pll);
		}
		break;
	case FH_DCTL_CMD_DVFS:	/* PLL DVFS */
	case FH_DCTL_CMD_DVFS_SSC_ENABLE:	/* PLL DVFS and enable SSC */
	case FH_DCTL_CMD_DVFS_SSC_DISABLE:	/* PLL DVFS and disable SSC */
	case FH_DCTL_CMD_SSC_ENABLE:	/* SSC enable */
	case FH_DCTL_CMD_SSC_DISABLE:	/* SSC disable */
	case FH_DCTL_CMD_GENERAL_DFS:
		{
			fh_ioctl_dvfs_ssc(ctlid, arg);
		}
		break;

	default:
		FH_MSG("Unrecognized ctlid %d", ctlid);
		break;
	};
}

static struct mt_fh_hal_driver g_fh_hal_drv = {
	.fh_pll = g_fh_pll,
	.fh_usrdef = mt_ssc_fhpll_userdefined,
	.pll_cnt = FH_PLL_NUM,
	.proc.dumpregs_read = fh_dumpregs_proc_read,
	.proc.dvfs_read = fh_dvfs_proc_read,
	.proc.dvfs_write = fh_dvfs_proc_write,
	.mt_fh_hal_init = mt_fh_hal_init,
	.mt_fh_hal_ctrl = __freqhopping_ctrl,
	.mt_fh_lock = mt_fh_hal_lock,
	.mt_fh_unlock = mt_fh_hal_unlock,
	.mt_fh_get_init = mt_fh_hal_get_init,
	.mt_fh_popod_restore = mt_fh_hal_popod_restore,
	.mt_fh_popod_save = mt_fh_hal_popod_save,
	.mt_l2h_mempll = NULL,
	.mt_h2l_mempll = NULL,
	.mt_dfs_armpll = mt_fh_hal_dfs_armpll,
	.mt_dfs_mmpll = mt_fh_hal_dfs_mmpll,
	.mt_dfs_vencpll = mt_fh_hal_dfs_vencpll,	/* TODO: should set to NULL */
	.mt_is_support_DFS_mode = mt_fh_hal_is_support_DFS_mode,
	.mt_l2h_dvfs_mempll = mt_fh_hal_l2h_dvfs_mempll,	/* TODO: should set to NULL */
	.mt_h2l_dvfs_mempll = mt_fh_hal_h2l_dvfs_mempll,	/* TODO: should set to NULL */
	.mt_dram_overclock = mt_fh_hal_dram_overclock,
	.mt_get_dramc = mt_fh_hal_get_dramc,
	.mt_fh_default_conf = mt_fh_hal_default_conf,
	.mt_dfs_general_pll = mt_fh_hal_general_pll_dfs,
	.ioctl = __ioctl
};

struct mt_fh_hal_driver *mt_get_fh_hal_drv(void)
{
	return &g_fh_hal_drv;
}

/* SS13 request to provide the pause ARMPLL API */
/* [Purpose]: control PLL for each cluster */
int mt_pause_armpll(unsigned int pll, unsigned int pause)
{
	/* unsigned long flags = 0; */
	unsigned long reg_cfg = 0;
	unsigned long flags = 0;

	if (g_initialize == 0) {
		FH_MSG("(Warning) %s FHCTL isn't ready.", __func__);
		return -1;
	}

	FH_MSG_DEBUG("%s for pll %d pause %d", __func__, pll, pause);

	switch (pll) {
	case MCU_FH_PLL0:
	case MCU_FH_PLL1:
	case MCU_FH_PLL2:
	case MCU_FH_PLL3:
		break;
	default:
		BUG_ON(1);
		return 1;
	};

  /********************************************************/
	local_irq_save(flags);
	mt6797_0x1001AXXX_lock();


	reg_cfg = g_reg_cfg[pll];
	if (pause)
		mcu_fh_set_field(reg_cfg, FH_FHCTLX_CFG_PAUSE, 1);	/* pause  */
	else
		mcu_fh_set_field(reg_cfg, FH_FHCTLX_CFG_PAUSE, 0);	/* no pause  */

	mt6797_0x1001AXXX_unlock();
	local_irq_restore(flags);
    /********************************************************/

	return 0;
}


/* TODO: module_exit(cpufreq_exit); */
