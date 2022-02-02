#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"
#include "lcm_i2c.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mt-plat/mt_gpio.h>
#endif

// ---------------------------------------------------------------------------
//  Local Constantsq
// ---------------------------------------------------------------------------

#define FRAME_WIDTH                                         (720)
#define FRAME_HEIGHT                                        (1440)

#define REGFLAG_DELAY                           0XFE
#define REGFLAG_END_OF_TABLE                    0xFFF   // END OF REGISTERS MARKER
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)          lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)             lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                            lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                        lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                             lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                     lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    


struct LCM_setting_table {
	unsigned int cmd;
	unsigned int count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {

	{ 0x11, 0x01, {0x00}},
	{ REGFLAG_DELAY, 120, {0x00}},
	{ 0x29, 0x01, {0x00}},
	{ REGFLAG_END_OF_TABLE, 0x00, {0x00}}
};

static struct LCM_setting_table lcm_sleep_in_setting[] = {
	{ 0x28, 0x01, {0x00}},
	{ REGFLAG_DELAY, 50, {0x00}},
	{ 0x10, 0x01, {0x00}},
	{ REGFLAG_DELAY, 120, {0x00}},
	{ REGFLAG_END_OF_TABLE, 0x00, {0x00}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++)
	{   
		unsigned cmd;
		cmd = table[i].cmd;   

		switch (cmd)
		{     
			case REGFLAG_DELAY :
			MDELAY(table[i].count);
			break;

			case REGFLAG_END_OF_TABLE :
			break;

			default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	} 
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS * util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS * params){

	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = 2;
	params->dsi.LANE_NUM = 4;
	params->dsi.data_format.format = 2;
	params->dsi.PS = 2;
	params->dsi.vertical_sync_active = 8;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->dbi.te_mode = 1;
	params->dsi.mode = 1;
	params->dsi.packet_size = 256;
	params->dsi.vertical_backporch = 106;
	params->dsi.vertical_frontporch = 240;
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	params->dsi.horizontal_sync_active = 14;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_disable = 1;
	params->dsi.cont_clock = 1;
	params->dsi.clk_lp_per_line_enable = 1;
	params->dsi.horizontal_backporch = 25;
	params->dsi.horizontal_frontporch = 45;
	params->dsi.PLL_CLOCK = 260;
}

extern int mt_dsi_pinctrl_set(unsigned int pin , unsigned int level);

static void lcm_suspend(void){
	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	SET_RESET_PIN(1);
	MDELAY(120);
	mt_dsi_pinctrl_set(4,0);
	mt_dsi_pinctrl_set(3,0);
	MDELAY(2);
	mt_dsi_pinctrl_set(2, 0);
	MDELAY(2);
	mt_dsi_pinctrl_set(1, 0);
}

static void lcm_init(void)
{
	mt_dsi_pinctrl_set(1,1);
	MDELAY(2);
	mt_dsi_pinctrl_set(2,1);
	MDELAY(2);
	SET_RESET_PIN(1);
	MDELAY(10);
	mt_dsi_pinctrl_set(3,1);
	mt_dsi_pinctrl_set(4,1);
	MDELAY(2);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120);
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}



static void lcm_resume(void){
	lcm_init();
}

static unsigned int lcm_compare_id(void){
	return 1;
}

LCM_DRIVER ft8006m_hd1440_dsi_vdo_boe_57_lcm_drv = {
	.name                   = "ft8006m_hd1440_dsi_vdo_boe_57",
	.set_util_funcs         = lcm_set_util_funcs,
	.get_params         = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id
};
