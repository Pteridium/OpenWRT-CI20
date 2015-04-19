/*
 *  Copyright (C) 2009-2010, Lars-Peter Clausen <lars@metafoo.de>
 *  Copyright (C) 2010, Paul Cercueil <paul@crapouillou.net>
 *  Copyright (C) 2013, Imagination Technologies
 *	 JZ4740 SoC RTC driver
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of  the GNU General Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#define JZ_REG_RTC_CTRL		0x00
#define JZ_REG_RTC_SEC		0x04
#define JZ_REG_RTC_SEC_ALARM	0x08
#define JZ_REG_RTC_REGULATOR	0x0C
#define JZ_REG_RTC_HIBERNATE	0x20
#define JZ_REG_RTC_SCRATCHPAD	0x34
#define JZ_REG_RTC_WENR		0x3C
#define JZ_REG_RTC_CKPCR	0x40

#define JZ_RTC_CTRL_WRDY	BIT(7)
#define JZ_RTC_CTRL_1HZ		BIT(6)
#define JZ_RTC_CTRL_1HZ_IRQ	BIT(5)
#define JZ_RTC_CTRL_AF		BIT(4)
#define JZ_RTC_CTRL_AF_IRQ	BIT(3)
#define JZ_RTC_CTRL_AE		BIT(2)
#define JZ_RTC_CTRL_ENABLE	BIT(0)

#define JZ_RTC_WENR_PAT		0xA55A
#define JZ_RTC_WENR_WEN		BIT(31)

#define JZ_RTC_CKPCR_CK32CTL_SHIFT	1
#define JZ_RTC_CKPCR_CK32CTL_INPUT	0x0
#define JZ_RTC_CKPCR_CK32CTL_OUTPUT	0x1
#define JZ_RTC_CKPCR_CK32CTL_GPIO	0x2
#define JZ_RTC_CKPCR_CK32CTL_CLK32K	0x3
#define JZ_RTC_CKPCR_CK32CTL_CK32PULL	BIT(4)

enum jz4740_rtc_version {
	JZ_RTC_JZ4740,
	JZ_RTC_JZ4780,
};

struct jz4740_rtc {
	void __iomem *base;
	enum jz4740_rtc_version version;

	struct rtc_device *rtc;

	int irq;

	spinlock_t lock;

#ifdef CONFIG_COMMON_CLK
	struct clk_hw clkout_hw;
#endif

};

static inline uint32_t jz4740_rtc_reg_read(struct jz4740_rtc *rtc, size_t reg)
{
	return readl(rtc->base + reg);
}

static int jz4740_rtc_wait_write_ready(struct jz4740_rtc *rtc)
{
	uint32_t ctrl;
	int timeout = 10000;

	do {
		ctrl = jz4740_rtc_reg_read(rtc, JZ_REG_RTC_CTRL);
	} while (!(ctrl & JZ_RTC_CTRL_WRDY) && --timeout);

	return timeout ? 0 : -EIO;
}

static inline int jz4740_rtc_reg_write(struct jz4740_rtc *rtc, size_t reg,
	uint32_t val)
{
	int ret;
	uint32_t wenr;
	int timeout = 10000;

	/*
	 * The 4780 has a write enable register which must have a pattern
	 * written to it to enable writing to certain registers. Some actions
	 * (undocumented) appear to cause registers to become unwritable, so to
	 * be on the safe side do this before each write.
	 */
	if (rtc->version >= JZ_RTC_JZ4780) {
		ret = jz4740_rtc_wait_write_ready(rtc);
		if (ret)
			return ret;

		writel(JZ_RTC_WENR_PAT, rtc->base + JZ_REG_RTC_WENR);

		ret = jz4740_rtc_wait_write_ready(rtc);
		if (ret)
			return ret;

		do {
			wenr = jz4740_rtc_reg_read(rtc, JZ_REG_RTC_WENR);
		} while (!(wenr & JZ_RTC_WENR_WEN) && --timeout);

		if (!timeout)
			return -EIO;
	} else {
		ret = jz4740_rtc_wait_write_ready(rtc);
		if (ret)
			return ret;
	}

	writel(val, rtc->base + reg);

	return 0;
}

static int jz4740_rtc_ctrl_set_bits(struct jz4740_rtc *rtc, uint32_t mask,
	bool set)
{
	int ret;
	unsigned long flags;
	uint32_t ctrl;

	spin_lock_irqsave(&rtc->lock, flags);

	ctrl = jz4740_rtc_reg_read(rtc, JZ_REG_RTC_CTRL);

	/* Don't clear interrupt flags by accident */
	ctrl |= JZ_RTC_CTRL_1HZ | JZ_RTC_CTRL_AF;

	if (set)
		ctrl |= mask;
	else
		ctrl &= ~mask;

	ret = jz4740_rtc_reg_write(rtc, JZ_REG_RTC_CTRL, ctrl);

	spin_unlock_irqrestore(&rtc->lock, flags);

	return ret;
}

static int jz4740_rtc_read_time(struct device *dev, struct rtc_time *time)
{
	struct jz4740_rtc *rtc = dev_get_drvdata(dev);
	uint32_t secs, secs2;
	int timeout = 5;

	/* If the seconds register is read while it is updated, it can contain a
	 * bogus value. This can be avoided by making sure that two consecutive
	 * reads have the same value.
	 */
	secs = jz4740_rtc_reg_read(rtc, JZ_REG_RTC_SEC);
	secs2 = jz4740_rtc_reg_read(rtc, JZ_REG_RTC_SEC);

	while (secs != secs2 && --timeout) {
		secs = secs2;
		secs2 = jz4740_rtc_reg_read(rtc, JZ_REG_RTC_SEC);
	}

	if (timeout == 0)
		return -EIO;

	rtc_time_to_tm(secs, time);

	return rtc_valid_tm(time);
}

static int jz4740_rtc_set_mmss(struct device *dev, unsigned long secs)
{
	struct jz4740_rtc *rtc = dev_get_drvdata(dev);

	return jz4740_rtc_reg_write(rtc, JZ_REG_RTC_SEC, secs);
}

static int jz4740_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct jz4740_rtc *rtc = dev_get_drvdata(dev);
	uint32_t secs;
	uint32_t ctrl;

	secs = jz4740_rtc_reg_read(rtc, JZ_REG_RTC_SEC_ALARM);

	ctrl = jz4740_rtc_reg_read(rtc, JZ_REG_RTC_CTRL);

	alrm->enabled = !!(ctrl & JZ_RTC_CTRL_AE);
	alrm->pending = !!(ctrl & JZ_RTC_CTRL_AF);

	rtc_time_to_tm(secs, &alrm->time);

	return rtc_valid_tm(&alrm->time);
}

static int jz4740_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	int ret;
	struct jz4740_rtc *rtc = dev_get_drvdata(dev);
	unsigned long secs;

	rtc_tm_to_time(&alrm->time, &secs);

	ret = jz4740_rtc_reg_write(rtc, JZ_REG_RTC_SEC_ALARM, secs);
	if (!ret)
		ret = jz4740_rtc_ctrl_set_bits(rtc,
			JZ_RTC_CTRL_AE | JZ_RTC_CTRL_AF_IRQ, alrm->enabled);

	return ret;
}

static int jz4740_rtc_alarm_irq_enable(struct device *dev, unsigned int enable)
{
	struct jz4740_rtc *rtc = dev_get_drvdata(dev);
	return jz4740_rtc_ctrl_set_bits(rtc, JZ_RTC_CTRL_AF_IRQ, enable);
}

static struct rtc_class_ops jz4740_rtc_ops = {
	.read_time	= jz4740_rtc_read_time,
	.set_mmss	= jz4740_rtc_set_mmss,
	.read_alarm	= jz4740_rtc_read_alarm,
	.set_alarm	= jz4740_rtc_set_alarm,
	.alarm_irq_enable = jz4740_rtc_alarm_irq_enable,
};

static irqreturn_t jz4740_rtc_irq(int irq, void *data)
{
	struct jz4740_rtc *rtc = data;
	uint32_t ctrl;
	unsigned long events = 0;

	ctrl = jz4740_rtc_reg_read(rtc, JZ_REG_RTC_CTRL);

	if (ctrl & JZ_RTC_CTRL_1HZ)
		events |= (RTC_UF | RTC_IRQF);

	if (ctrl & JZ_RTC_CTRL_AF)
		events |= (RTC_AF | RTC_IRQF);

	rtc_update_irq(rtc->rtc, 1, events);

	jz4740_rtc_ctrl_set_bits(rtc, JZ_RTC_CTRL_1HZ | JZ_RTC_CTRL_AF, false);

	return IRQ_HANDLED;
}

/*
 * Handling of the clkout
 */

#ifdef CONFIG_COMMON_CLK

static int jz4740_rtc_clkout_enable(struct clk_hw *hw)
{
	struct jz4740_rtc *rtc = container_of(hw, struct jz4740_rtc, clkout_hw);
	int reg = (JZ_RTC_CKPCR_CK32CTL_CLK32K << JZ_RTC_CKPCR_CK32CTL_SHIFT)
		   & ~JZ_RTC_CKPCR_CK32CTL_CK32PULL;

	jz4740_rtc_reg_write(rtc, JZ_REG_RTC_CKPCR, reg);
	return 0;
}

static void jz4740_rtc_clkout_disable(struct clk_hw *hw)
{
	struct jz4740_rtc *rtc = container_of(hw, struct jz4740_rtc, clkout_hw);
	int reg = JZ_RTC_CKPCR_CK32CTL_CK32PULL;

	jz4740_rtc_reg_write(rtc, JZ_REG_RTC_CKPCR, reg);
}

static const struct clk_ops jz4740_rtc_clkout_ops = {
	.enable = jz4740_rtc_clkout_enable,
	.disable = jz4740_rtc_clkout_disable,
};

static struct clk *jz4740_rtc_register_clkout(struct device *dev,
					   struct jz4740_rtc *rtc)
{
	struct device_node *node = dev->of_node;
	struct clk *clk;
	struct clk_init_data init;

	init.name = "jz4740-rtc32k-clkout";
	init.ops = &jz4740_rtc_clkout_ops;
	init.flags = CLK_IS_ROOT;
	init.parent_names = NULL;
	init.num_parents = 0;
	rtc->clkout_hw.init = &init;

	/* register the clock */
	clk = devm_clk_register(dev, &rtc->clkout_hw);

	if (!IS_ERR(clk)) {
		clk_register_clkdev(clk, init.name, NULL);
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
	}

	return clk;
}
#endif


static const struct of_device_id jz4740_rtc_of_match[] = {
	{ .compatible = "ingenic,jz4740-rtc", .data = (void *)JZ_RTC_JZ4740 },
	{ .compatible = "ingenic,jz4780-rtc", .data = (void *)JZ_RTC_JZ4780 },
	{},
};
MODULE_DEVICE_TABLE(of, jz4740_rtc_of_match);

static int jz4740_rtc_probe(struct platform_device *pdev)
{
	int ret;
	struct jz4740_rtc *rtc;
	const struct of_device_id *match;
	uint32_t scratchpad;
	struct resource *mem;

	rtc = devm_kzalloc(&pdev->dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	match = of_match_device(jz4740_rtc_of_match, &pdev->dev);
	if (match)
		rtc->version = (enum jz4740_rtc_version)match->data;
	else
		rtc->version = platform_get_device_id(pdev)->driver_data;

	rtc->irq = platform_get_irq(pdev, 0);
	if (rtc->irq < 0) {
		dev_err(&pdev->dev, "Failed to get platform irq\n");
		return -ENOENT;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rtc->base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(rtc->base))
		return PTR_ERR(rtc->base);

	spin_lock_init(&rtc->lock);

	platform_set_drvdata(pdev, rtc);

	device_init_wakeup(&pdev->dev, 1);

	rtc->rtc = devm_rtc_device_register(&pdev->dev, pdev->name,
					&jz4740_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc)) {
		ret = PTR_ERR(rtc->rtc);
		dev_err(&pdev->dev, "Failed to register rtc device: %d\n", ret);
		return ret;
	}

	ret = devm_request_irq(&pdev->dev, rtc->irq, jz4740_rtc_irq, 0,
				pdev->name, rtc);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request rtc irq: %d\n", ret);
		return ret;
	}

	scratchpad = jz4740_rtc_reg_read(rtc, JZ_REG_RTC_SCRATCHPAD);
	if (scratchpad != 0x12345678) {
		ret = jz4740_rtc_reg_write(rtc, JZ_REG_RTC_SCRATCHPAD, 0x12345678);
		ret = jz4740_rtc_reg_write(rtc, JZ_REG_RTC_SEC, 0);
		if (ret) {
			dev_err(&pdev->dev, "Could not write to RTC registers\n");
			return ret;
		}
	}

#ifdef CONFIG_COMMON_CLK
	if (rtc->version >= JZ_RTC_JZ4780) {
		jz4740_rtc_register_clkout(&pdev->dev, rtc);
		clk_prepare_enable(rtc->clkout_hw.clk);
	}
#endif
	return 0;
}

static int jz4740_rtc_remove(struct platform_device *pdev)
{
	struct jz4740_rtc *rtc = platform_get_drvdata(pdev);

#ifdef CONFIG_COMMON_CLK
	if (rtc->version >= JZ_RTC_JZ4780) {
		clk_disable_unprepare(rtc->clkout_hw.clk);
		clk_unregister(rtc->clkout_hw.clk);
	}
#endif

	return 0;
}

#ifdef CONFIG_PM
static int jz4740_rtc_suspend(struct device *dev)
{
	struct jz4740_rtc *rtc = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(rtc->irq);
	return 0;
}

static int jz4740_rtc_resume(struct device *dev)
{
	struct jz4740_rtc *rtc = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(rtc->irq);
	return 0;
}

static const struct dev_pm_ops jz4740_pm_ops = {
	.suspend = jz4740_rtc_suspend,
	.resume  = jz4740_rtc_resume,
};
#define JZ4740_RTC_PM_OPS (&jz4740_pm_ops)

#else
#define JZ4740_RTC_PM_OPS NULL
#endif  /* CONFIG_PM */

static struct platform_driver jz4740_rtc_driver = {
	.probe	 = jz4740_rtc_probe,
	.remove	 = jz4740_rtc_remove,
	.driver	 = {
		.name  = "jz4740-rtc",
		.owner = THIS_MODULE,
		.pm    = JZ4740_RTC_PM_OPS,
		.of_match_table = of_match_ptr(jz4740_rtc_of_match),
	},
};

module_platform_driver(jz4740_rtc_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RTC driver for the JZ4740 SoC\n");
MODULE_ALIAS("platform:jz4740-rtc");
