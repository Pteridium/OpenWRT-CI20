/*
 * Copyright (C) 2014 Imagination Technologies
 * Author: Zubair Lutfullah Kakakhel <Zubair.Kakakhel@imgtec.com>
 *
 * LCDC DRM driver for Ingenic JZ4780, based on the tilcdc driver
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

#include "jz4780_drv.h"
#include "jz4780_regs.h"

#include "drm_fb_helper.h"
#include "dwc_hdmi.h"

static struct of_device_id jz4780_of_match[];

static struct drm_framebuffer *jz4780_fb_create(struct drm_device *dev,
		struct drm_file *file_priv, struct drm_mode_fb_cmd2 *mode_cmd)
{
	return drm_fb_cma_create(dev, file_priv, mode_cmd);
}

static void jz4780_fb_output_poll_changed(struct drm_device *dev)
{
	struct jz4780_drm_private *priv = dev->dev_private;

	if (priv->fbdev)
		drm_fbdev_cma_hotplug_event(priv->fbdev);
}

static const struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create = jz4780_fb_create,
	.output_poll_changed = jz4780_fb_output_poll_changed,
};

void modeset_init(struct drm_device *dev)
{
	struct jz4780_drm_private *priv = dev->dev_private;

	priv->crtc = jz4780_crtc_create(dev);

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;
	dev->mode_config.max_width = 2048;
	dev->mode_config.max_height = 2048;
	dev->mode_config.funcs = &mode_config_funcs;
}

/*
 * DRM operations:
 */

static int jz4780_unload(struct drm_device *dev)
{
	struct jz4780_drm_private *priv = dev->dev_private;

	drm_kms_helper_poll_fini(dev);
	drm_mode_config_cleanup(dev);
	drm_vblank_cleanup(dev);

	pm_runtime_get_sync(dev->dev);
	drm_irq_uninstall(dev);
	pm_runtime_put_sync(dev->dev);

	if (priv->clk)
		clk_put(priv->clk);

	if (priv->mmio)
		iounmap(priv->mmio);

	flush_workqueue(priv->wq);
	destroy_workqueue(priv->wq);

	dev->dev_private = NULL;

	pm_runtime_disable(dev->dev);

	kfree(priv);

	return 0;
}

static int jz4780_load(struct drm_device *dev, unsigned long flags)
{
	struct platform_device *pdev = dev->platformdev;
	struct platform_device *hdmi_pdev;
	struct device_node *node = pdev->dev.of_node;
	struct jz4780_drm_private *priv;
	struct resource *res;
	struct device_node *hdmi_node;
	struct dwc_hdmi *hdmi;
	uint32_t hdmi_phandle;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		DRM_DEBUG_DRIVER("failed to allocate private data\n");
		return -ENOMEM;
	}

	dev->dev_private = priv;

	priv->wq = alloc_ordered_workqueue("jz4780", 0);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		DRM_DEBUG_DRIVER("failed to get memory resource\n");
		ret = -EINVAL;
		goto fail;
	}

	priv->mmio = ioremap_nocache(res->start, resource_size(res));
	if (!priv->mmio) {
		DRM_DEBUG_DRIVER("failed to ioremap\n");
		ret = -ENOMEM;
		goto fail;
	}

	priv->clk = clk_get(dev->dev, "lcd_clk");
	if (IS_ERR(priv->clk)) {
		DRM_DEBUG_DRIVER("failed to get lcd clock\n");
		ret = -ENODEV;
		goto fail;
	}

	clk_prepare_enable(priv->clk);

	priv->disp_clk = clk_get(dev->dev, "lcd_pixclk");
	if (IS_ERR(priv->clk)) {
		DRM_DEBUG_DRIVER("failed to get pixel clock\n");
		ret = -ENODEV;
		goto fail;
	}

	clk_prepare_enable(priv->disp_clk);

	if (of_property_read_u32(node, "max-bandwidth", &priv->max_bandwidth))
		priv->max_bandwidth = JZ4780_DEFAULT_MAX_BANDWIDTH;

	if (of_property_read_u32(node, "max-width", &priv->max_width))
		priv->max_width = JZ4780_DEFAULT_MAX_WIDTH;

	if (of_property_read_u32(node, "max-pixelclock",
					&priv->max_pixelclock))
		priv->max_pixelclock = JZ4780_DEFAULT_MAX_PIXELCLOCK;

	drm_mode_config_init(dev);

	if (of_property_read_u32(node, "hdmi", &hdmi_phandle)) {
		dev_warn(&pdev->dev, "could not get hdmi phandle\n");
	} else {
		hdmi_node = of_find_node_by_phandle(hdmi_phandle);
		if (!hdmi_node) {
			dev_err(&pdev->dev, "could not get hdmi node\n");
			return -ENODEV;
		}

		hdmi_pdev = of_find_device_by_node(hdmi_node);
		if (!hdmi_pdev) {
			dev_err(&pdev->dev, "hdmi platform device not found\n");
			return -ENODEV;
		}

		hdmi = platform_get_drvdata(hdmi_pdev);
		hdmi->drm = dev;

		dwc_hdmi_register(hdmi, dev);
		priv->encoders[priv->num_encoders++] = &hdmi->encoder;
		priv->connectors[priv->num_connectors++] = &hdmi->connector;

	}

	modeset_init(dev);

	ret = drm_vblank_init(dev, 1);
	if (ret < 0) {
		DRM_DEBUG_DRIVER("failed to initialize vblank\n");
		goto fail;
	}

	pm_runtime_get_sync(dev->dev);
	ret = drm_irq_install(dev, platform_get_irq(dev->platformdev, 0));
	pm_runtime_put_sync(dev->dev);
	if (ret < 0) {
		DRM_DEBUG_DRIVER("failed to install IRQ handler\n");
		goto fail;
	}

	platform_set_drvdata(pdev, dev);

	priv->fbdev = drm_fbdev_cma_init(dev, 32,
			dev->mode_config.num_crtc,
			dev->mode_config.num_connector);

	drm_kms_helper_poll_init(dev);

	return 0;

fail:
	jz4780_unload(dev);
	return ret;
}

static void jz4780_preclose(struct drm_device *dev, struct drm_file *file)
{
	struct jz4780_drm_private *priv = dev->dev_private;

	jz4780_crtc_cancel_page_flip(priv->crtc, file);
}

static void jz4780_lastclose(struct drm_device *dev)
{
	struct jz4780_drm_private *priv = dev->dev_private;

	drm_fbdev_cma_restore_mode(priv->fbdev);
}

static irqreturn_t jz4780_irq(int irq, void *arg)
{
	struct drm_device *dev = arg;
	struct jz4780_drm_private *priv = dev->dev_private;

	return jz4780_crtc_irq(priv->crtc);
}

static void enable_vblank(struct drm_device *dev, bool enable)
{
	u32 tmp;

	/* clear previous EOF flag */
	tmp = jz4780_read(dev, LCDC_STATE);
	jz4780_write(dev, LCDC_STATE, tmp & ~LCDC_STATE_EOF);

	/* enable end of frame interrupt */
	tmp = jz4780_read(dev, LCDC_CTRL);
	if (enable)
		jz4780_write(dev, LCDC_CTRL, tmp | LCDC_CTRL_EOFM);
	else
		jz4780_write(dev, LCDC_CTRL, tmp & ~LCDC_CTRL_EOFM);

}

static int jz4780_enable_vblank(struct drm_device *dev, int crtc)
{
	enable_vblank(dev, true);
	return 0;
}

static void jz4780_disable_vblank(struct drm_device *dev, int crtc)
{
	enable_vblank(dev, false);
}

static const struct file_operations fops = {
	.owner              = THIS_MODULE,
	.open               = drm_open,
	.release            = drm_release,
	.unlocked_ioctl     = drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl       = drm_compat_ioctl,
#endif
	.poll               = drm_poll,
	.read               = drm_read,
	.llseek             = no_llseek,
	.mmap               = drm_gem_cma_mmap,
};

static struct drm_driver jz4780_driver = {
	.driver_features    = DRIVER_HAVE_IRQ | DRIVER_GEM | DRIVER_MODESET,
	.load               = jz4780_load,
	.unload             = jz4780_unload,
	.preclose           = jz4780_preclose,
	.lastclose          = jz4780_lastclose,
	.set_busid          = drm_platform_set_busid,
	.irq_handler        = jz4780_irq,
	.get_vblank_counter = drm_vblank_count,
	.enable_vblank      = jz4780_enable_vblank,
	.disable_vblank     = jz4780_disable_vblank,
	.gem_free_object    = drm_gem_cma_free_object,
	.gem_vm_ops         = &drm_gem_cma_vm_ops,
	.dumb_create        = drm_gem_cma_dumb_create,
	.dumb_map_offset    = drm_gem_cma_dumb_map_offset,
	.dumb_destroy       = drm_gem_dumb_destroy,
	.fops               = &fops,
	.name               = "jz4780",
	.desc               = "Ingenic LCD Controller DRM",
	.date               = "20140623",
	.major              = 1,
	.minor              = 0,
};

/*
 * Platform driver:
 */
static int jz4780_pdev_probe(struct platform_device *pdev)
{
	struct device_node *hdmi_node;
	struct device_node *ddc_node;
	uint32_t hdmi_phandle;
	void * ddc;
	/* bail out early if no DT data: */
	if (!pdev->dev.of_node) {
		DRM_DEBUG_DRIVER("device-tree data is missing\n");
		return -ENXIO;
	}

	ddc_node = of_parse_phandle(pdev->dev.of_node, "ddc", 0);
	if (ddc_node) {
		ddc = of_find_i2c_adapter_by_node(ddc_node);
		if (!ddc) {
			dev_dbg(&pdev->dev, "failed to read i2c ddc node\n");
			return -EPROBE_DEFER;
		}
	}

	if (of_property_read_u32(pdev->dev.of_node, "hdmi", &hdmi_phandle)) {
		dev_warn(&pdev->dev, "could not get hdmi phandle\n");
	} else {
		hdmi_node = of_find_node_by_phandle(hdmi_phandle);
		if (!hdmi_node) {
			dev_err(&pdev->dev, "could not get hdmi node\n");
			return -EPROBE_DEFER;
		}
	}

	return drm_platform_init(&jz4780_driver, pdev);
}

static int jz4780_pdev_remove(struct platform_device *pdev)
{
	drm_put_dev(platform_get_drvdata(pdev));
	return 0;
}

static struct of_device_id jz4780_of_match[] = {
		{ .compatible = "ingenic,jz4780-lcd", },
		{ },
};
MODULE_DEVICE_TABLE(of, jz4780_of_match);

static struct platform_driver jz4780_platform_driver = {
	.probe      = jz4780_pdev_probe,
	.remove     = jz4780_pdev_remove,
	.driver     = {
		.owner  = THIS_MODULE,
		.name   = "jz4780_drm",
		.of_match_table = jz4780_of_match,
	},
};

static int __init jz4780_drm_init(void)
{
	DRM_DEBUG_DRIVER("init");
	return platform_driver_register(&jz4780_platform_driver);
}

static void __exit jz4780_drm_fini(void)
{
	DRM_DEBUG_DRIVER("fini");
	platform_driver_unregister(&jz4780_platform_driver);
}
late_initcall(jz4780_drm_init);
module_init(jz4780_drm_init);
module_exit(jz4780_drm_fini);

MODULE_AUTHOR("Zubair Lutfullah Kakakhel <Zubair.Kakakhel@imgtec.com>");
MODULE_DESCRIPTION("Ingenic JZ4780 LCD/HDMI Driver");
MODULE_LICENSE("GPL");
