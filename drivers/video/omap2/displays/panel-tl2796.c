/*
 * tl2796 panel support
 *
 * Author: Joerie de Gram <j.de.gram@gmail.com>
 *
 * Derived from drivers/video/omap2/displays/panel-generic.c
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

#include <plat/display.h>

static struct omap_video_timings tl2796_panel_timings = {
	.x_res		= 480,
	.y_res		= 800,
	.pixel_clock	= 24576,
	.hfp		= 8,
	.hsw		= 1,
	.hbp		= 8,
	.vfp		= 8,
	.vsw		= 1,
	.vbp		= 8,
};

static int tl2796_panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
				OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_RF |
				OMAP_DSS_LCD_ONOFF;
	dssdev->panel.timings = tl2796_panel_timings;
	dssdev->panel.recommended_bpp = 16;

	return 0;
}

static void tl2796_panel_remove(struct omap_dss_device *dssdev)
{
}

static int tl2796_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	if (dssdev->platform_enable)
		r = dssdev->platform_enable(dssdev);

	return r;
}

static void tl2796_panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static int tl2796_panel_suspend(struct omap_dss_device *dssdev)
{
	tl2796_panel_disable(dssdev);
	return 0;
}

static int tl2796_panel_resume(struct omap_dss_device *dssdev)
{
	return tl2796_panel_enable(dssdev);
}

static struct omap_dss_driver tl2796_driver = {
	.probe		= tl2796_panel_probe,
	.remove		= tl2796_panel_remove,

	.enable		= tl2796_panel_enable,
	.disable	= tl2796_panel_disable,
	.suspend	= tl2796_panel_suspend,
	.resume		= tl2796_panel_resume,

	.driver         = {
		.name   = "tl2796_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init tl2796_panel_drv_init(void)
{
	return omap_dss_register_driver(&tl2796_driver);
}

static void __exit tl2796_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&tl2796_driver);
}

module_init(tl2796_panel_drv_init);
module_exit(tl2796_panel_drv_exit);
MODULE_LICENSE("GPL");
