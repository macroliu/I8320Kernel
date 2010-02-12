/*
 * LCD panel driver for Devkit8000 4.3" LCD pannel
 *
 * Copyright (C) 2010 Thomas Weber <weber@corscience.de>
 * Copyright (C) 2010 Kan-Ru Chen <kanru@0xlab.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>

#include <mach/display.h>

static struct omap_video_timings devkit8000_lcd_timings = {
	.x_res 		= 480,
	.y_res 		= 272,

	.pixel_clock	= 10000,

	.hsw		= 41,
	.hfp		= 2,
	.hbp		= 2,

	.vsw		= 10,
	.vfp		= 2,
	.vbp		= 2,
};

static int devkit8000_lcd_panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS;
	dssdev->panel.acb = 0x28;
	dssdev->panel.timings = devkit8000_lcd_timings;

	return 0;
}

static void devkit8000_lcd_panel_remove(struct omap_dss_device *dssdev)
{
}

static int devkit8000_lcd_panel_enable(struct omap_dss_device *dssdev)
{	
	if (dssdev->platform_enable)
		dssdev->platform_enable(dssdev);

	return 0;
}

static void devkit8000_lcd_panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static int devkit8000_lcd_panel_suspend(struct omap_dss_device *dssdev)
{
	devkit8000_lcd_panel_disable(dssdev);

	return 0;
}

static int devkit8000_lcd_panel_resume(struct omap_dss_device *dssdev)
{
	return devkit8000_lcd_panel_enable(dssdev);
}

static struct omap_dss_driver devkit8000_lcd_driver = {
	.probe		= devkit8000_lcd_panel_probe,
	.remove		= devkit8000_lcd_panel_remove,

	.enable		= devkit8000_lcd_panel_enable,
	.disable	= devkit8000_lcd_panel_disable,
	.suspend	= devkit8000_lcd_panel_suspend,
	.resume		= devkit8000_lcd_panel_resume,

	.driver         = {
		.name   = "devkit8000_lcd_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init devkit8000_lcd_panel_drv_init(void)
{
	return omap_dss_register_driver(&devkit8000_lcd_driver);
}

static void __exit devkit8000_lcd_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&devkit8000_lcd_driver);
}

module_init(devkit8000_lcd_panel_drv_init);
module_exit(devkit8000_lcd_panel_drv_exit);
MODULE_LICENSE("GPL");
