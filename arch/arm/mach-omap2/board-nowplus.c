/*
 * linux/arch/arm/mach-omap2/board-nowplus.c
 *
 * Author: Joerie de Gram <j.de.gram@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>

#include <linux/regulator/machine.h>
#include <linux/i2c/twl4030.h>

#include <linux/spi/spi_gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/tl2796.h>

#include <linux/input.h>
#include <linux/input/matrix_keypad.h>

#include <linux/leds-bd2802.h>
#include <linux/max17040_battery.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/mcspi.h>
#include <plat/mux.h>
#include <plat/board.h>
#include <plat/usb.h>
#include <plat/common.h>
#include <plat/dma.h>
#include <plat/gpmc.h>
#include <plat/display.h>

//#include "mux.h"
#include "mmc-twl4030.h"
#include "sdram-nowplus.h"
#include "omap3-opp.h"

#define NOWPLUS_CHARGER_ENABLE_GPIO	157
#define NOWPLUS_CHARGING_STATUS_GPIO	16

static struct bd2802_led_platform_data nowplus_led_data = {
	.reset_gpio = 151,
	.rgb_time = 3, /* refer to application note */
};

static int board_keymap[] = {
	KEY(0, 0, KEY_FRONT),
	KEY(1, 0, KEY_SEARCH),
	KEY(2, 0, KEY_CAMERA_FOCUS),
	KEY(0, 1, KEY_PHONE),
	KEY(2, 1, KEY_CAMERA),
	KEY(0, 2, KEY_EXIT),
	KEY(1, 2, KEY_VOLUMEUP),
	KEY(2, 2, KEY_VOLUMEDOWN),
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size		= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data nowplus_kp_data = {
	.keymap_data	= &board_map_data,
	.rows		= 3,
	.cols		= 3,
	.rep		= 1,
};

static struct gpio_keys_button nowplus_gpio_keys[] = {
	{
		.desc			= "Power button",
		.type			= EV_KEY,
		.code			= KEY_POWER,
		.gpio			= 24,
		.active_low		= 0,
		.debounce_interval	= 0,
	},
};

static struct gpio_keys_platform_data nowplus_gpio_keys_data = {
	.buttons	= nowplus_gpio_keys,
	.nbuttons	= ARRAY_SIZE(nowplus_gpio_keys),
};

static struct platform_device nowplus_gpio_keys_device = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &nowplus_gpio_keys_data,
	},
};

static struct twl4030_hsmmc_info nowplus_mmc[] = {
	{
		.mmc		= 1,
		.wires		= 4,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{
		.mmc		= 2,
		.wires		= 4,
		.nonremovable	= 1,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{
		.mmc		= 3,
		.wires		= 4,
		.nonremovable	= 1,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply nowplus_vmmc1_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply nowplus_vmmc2_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply nowplus_vaux2_supply = {
	.supply			= "vaux2",
};

static struct regulator_consumer_supply nowplus_vaux3_supply = {
	.supply			= "vaux3",
};

static struct regulator_consumer_supply nowplus_vaux4_supply = {
	.supply			= "vaux4",
};

static struct regulator_consumer_supply nowplus_vpll2_supply = {
	.supply			= "vpll2",
};

static struct regulator_consumer_supply nowplus_vsim_supply = {
	.supply			= "vmmc",
};

/* VMMC1 for MMC1 card */
static struct regulator_init_data nowplus_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &nowplus_vmmc1_supply,
};

/* VMMC2 for MoviNAND */
static struct regulator_init_data nowplus_vmmc2 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &nowplus_vmmc2_supply,
};

/* VAUX2 for touch screen */
static struct regulator_init_data nowplus_vaux2 = {
	.constraints = {
		.name			= "VAUX2",
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.always_on		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &nowplus_vaux2_supply,
};

/* VAUX3 for display */
static struct regulator_init_data nowplus_vaux3 = {
	.constraints = {
		.name			= "VLCD_IO",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &nowplus_vaux3_supply,
};

/* VAUX4 for display */
static struct regulator_init_data nowplus_vaux4 = {
	.constraints = {
		.name			= "VLCD_CORE",
		.min_uV			= 3150000,
		.max_uV			= 3150000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &nowplus_vaux4_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_init_data nowplus_vpll2 = {
	.constraints = {
		.name			= "VPLL2",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &nowplus_vpll2_supply,
};

/* VSIM for WiFi SDIO */
static struct regulator_init_data nowplus_vsim = {
	.constraints = {
		.name			= "VSIM",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &nowplus_vsim_supply,
};

static int nowplus_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	twl4030_mmc_init(nowplus_mmc);

	/* link regulators to MMC adapters */
	nowplus_vmmc1_supply.dev = nowplus_mmc[0].dev;
	nowplus_vmmc2_supply.dev = nowplus_mmc[1].dev;
	nowplus_vsim_supply.dev = nowplus_mmc[2].dev;

	return 0;
}

static struct twl4030_gpio_platform_data nowplus_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.setup		= nowplus_twl_gpio_setup,
};

static struct twl4030_usb_data nowplus_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_codec_audio_data nowplus_audio_data = {
	.audio_mclk = 26000000,
};

static struct twl4030_codec_data nowplus_codec_data = {
	.audio_mclk = 26000000,
	.audio = &nowplus_audio_data,
};

static struct twl4030_platform_data nowplus_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	.keypad		= &nowplus_kp_data,
	.gpio		= &nowplus_gpio_data,
	.usb		= &nowplus_usb_data,
	.codec		= &nowplus_codec_data,

	.vaux2		= &nowplus_vaux2,
	.vaux3		= &nowplus_vaux3,
	.vaux4		= &nowplus_vaux4,
	.vpll2		= &nowplus_vpll2,
	.vmmc1		= &nowplus_vmmc1,
	.vmmc2		= &nowplus_vmmc2,
	.vsim		= &nowplus_vsim,
};

static int nowplus_battery_online(void) {
	return 1;
};

static int nowplus_charger_online(void) {
	return 1;
};

static int nowplus_charger_enable(void) {
	//gpio_set_value(NOWPLUS_CHARGER_ENABLE_GPIO, 0);
	return 1;
};

static struct max17040_platform_data nowplus_max17040_data = {
	.battery_online = &nowplus_battery_online,
	.charger_online = &nowplus_charger_online,
	.charger_enable = &nowplus_charger_enable,
};

static struct i2c_board_info __initdata nowplus_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl5030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &nowplus_twldata,
	},
};

static struct i2c_board_info __initdata nowplus_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("s5ka3dfx", 0x62),
	},
	{
		I2C_BOARD_INFO("mb91688", 0x1f),
	},
	{
		I2C_BOARD_INFO("s5ka3dfx-pm", 0x7d),
	},
	{
		I2C_BOARD_INFO("kxsd9", 0x18),
	},
	{
		I2C_BOARD_INFO("si470x", 0x10),
		.irq = OMAP_GPIO_IRQ(3),
	},
	{
		I2C_BOARD_INFO("BD2802", 0x1A),
		.platform_data = &nowplus_led_data,
	},
	{
		I2C_BOARD_INFO("akm8976", 0x1c),
	},
	{
		I2C_BOARD_INFO("microusbic", 0x25),
	},
	{
		I2C_BOARD_INFO("max17040", 0x36),
		.platform_data = &nowplus_max17040_data,
	},
	{
		I2C_BOARD_INFO("gp2ap002a00f", 0x44),
	},
	{
		I2C_BOARD_INFO("max9877", 0x4d),
	},
};

static struct i2c_board_info __initdata nowplus_i2c3_boardinfo[] = {
	{
		I2C_BOARD_INFO("synaptics-rmi4", 0x2C),
	},
};

static int __init nowplus_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, nowplus_i2c1_boardinfo,
			ARRAY_SIZE(nowplus_i2c1_boardinfo));
	omap_register_i2c_bus(2, 400, nowplus_i2c2_boardinfo,
			ARRAY_SIZE(nowplus_i2c2_boardinfo));
	omap_register_i2c_bus(3, 400, nowplus_i2c3_boardinfo,
			ARRAY_SIZE(nowplus_i2c3_boardinfo));
	return 0;
}

static struct tl2796_platform_data tl2796_data = {
	.reset_gpio	= 170,
	.reg_reset_gpio	= 99,
	.io_supply	= "vaux3",
	.core_supply	= "vaux4",
	.pll_supply	= "vpll2",
};

static struct omap2_mcspi_device_config tl2796_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_board_info nowplus_spi_board_info[] __initdata = {
	{
		.modalias		= "tl2796_bl",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 3000000,
		.controller_data	= &tl2796_mcspi_config,
		.irq			= 0,
		.platform_data		= &tl2796_data,
	}
};

static struct platform_device nowplus_bl_device = {
	.name		= "tl2796_bl",
	.id		= -1,
};

static int nowplus_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	return 0;
}

static void nowplus_panel_disable_lcd(struct omap_dss_device *dssdev)
{
}

static struct omap_dss_device nowplus_lcd_device = {
	.name			= "lcd",
	.driver_name		= "tl2796_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.platform_enable	= nowplus_panel_enable_lcd,
	.platform_disable	= nowplus_panel_disable_lcd,
};

static struct omap_dss_device *nowplus_dss_devices[] = {
	&nowplus_lcd_device,
};

static struct omap_dss_board_info nowplus_dss_data = {
	.num_devices	= ARRAY_SIZE(nowplus_dss_devices),
	.devices	= nowplus_dss_devices,
	.default_device	= &nowplus_lcd_device,
};

static struct platform_device nowplus_dss_device = {
	.name	= "omapdss",
	.id		= -1,
	.dev	= {
		.platform_data = &nowplus_dss_data,
	},
};

static struct omap_board_config_kernel nowplus_config[] __initdata = {
};

static struct platform_device *nowplus_devices[] __initdata = {
	&nowplus_dss_device,
	&nowplus_bl_device,
	&nowplus_gpio_keys_device,
};

static void __init nowplus_init_irq(void)
{
	omap_board_config = nowplus_config;
	omap_board_config_size = ARRAY_SIZE(nowplus_config);
	omap2_init_common_hw(nowplus_sdrc_params, nowplus_sdrc_params,omap3_mpu_rate_table,
				 omap3_dsp_rate_table, omap3_l3_rate_table);

	omap_init_irq();
	omap_gpio_init();
}

#ifdef CONFIG_OMAP_MUX
//static struct omap_board_mux board_mux[] __initdata = {
//	{ .reg_offset = OMAP_MUX_TERMINATOR },
//};
#else
#define board_mux	NULL
#endif

static void __init nowplus_init(void)
{
	//omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	nowplus_i2c_init();

	platform_add_devices(nowplus_devices,
			ARRAY_SIZE(nowplus_devices));

	spi_register_board_info(nowplus_spi_board_info,
			ARRAY_SIZE(nowplus_spi_board_info));

	usb_musb_init();

	/* Charger GPIOs */
//	gpio_request(NOWPLUS_CHARGING_STATUS_GPIO, "nCHG");
//	gpio_direction_input(NOWPLUS_CHARGING_STATUS_GPIO);
//	gpio_request(NOWPLUS_CHARGER_ENABLE_GPIO, "CHG_EN");
//	gpio_direction_output(NOWPLUS_CHARGER_ENABLE_GPIO, 1);

	/* Ensure SDRC pins are mux'd for self-refresh */
//	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
//	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);
	omap_cfg_reg(H16_34XX_SDRC_CKE0);
	omap_cfg_reg(H17_34XX_SDRC_CKE1);
}

static void __init nowplus_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(NOWPLUS, "Samsung NOWPLUS board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= nowplus_map_io,
	.init_irq	= nowplus_init_irq,
	.init_machine	= nowplus_init,
	.timer		= &omap_timer,
MACHINE_END
