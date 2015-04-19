/*
 *  Copyright (C) 2009-2010, Lars-Peter Clausen <lars@metafoo.de>
 *  JZ4740 platform IRQ support
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General	 Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/timex.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <asm/io.h>
#include <asm/mipsregs.h>
#include <asm/mach-jz4740/jz4780-smp.h>

#include "irqchip.h"

static void __iomem *jz_intc_base;
static unsigned jz_num_chips;

#define JZ_REG_INTC_STATUS	0x00
#define JZ_REG_INTC_MASK	0x04
#define JZ_REG_INTC_SET_MASK	0x08
#define JZ_REG_INTC_CLEAR_MASK	0x0c
#define JZ_REG_INTC_PENDING	0x10
#define CHIP_SIZE		0x20
#define IRQ_BASE		8

asmlinkage void plat_irq_dispatch(void)
{
	uint32_t pending = read_c0_cause() & read_c0_status() & CAUSEF_IP;
	if (pending & CAUSEF_IP4) {
		/* from OS timer */
		do_IRQ(4);
#ifdef CONFIG_SMP
	} else if (pending & CAUSEF_IP3) {
		/* from a mailbox write */
		do_IRQ(3);
#endif
	} else if (pending & CAUSEF_IP2) {
		/* from interrupt controller */
		do_IRQ(2);
	} else {
		spurious_interrupt();
	}
}

static irqreturn_t jz4740_cascade(int irq, void *data)
{
	uint32_t irq_reg;
	unsigned i;

	for (i = 0; i < jz_num_chips; i++) {
		irq_reg = readl(jz_intc_base + (i * CHIP_SIZE) + JZ_REG_INTC_PENDING);
		if (!irq_reg)
			continue;

		generic_handle_irq(__fls(irq_reg) + (i * 32) + IRQ_BASE);
	}

	return IRQ_HANDLED;
}

static void jz4740_irq_set_mask(struct irq_chip_generic *gc, uint32_t mask)
{
	struct irq_chip_regs *regs = &gc->chip_types->regs;

	writel(mask, gc->reg_base + regs->enable);
	writel(~mask, gc->reg_base + regs->disable);
}

void jz4740_irq_suspend(struct irq_data *data)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(data);
	jz4740_irq_set_mask(gc, gc->wake_active);
}

void jz4740_irq_resume(struct irq_data *data)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(data);
	jz4740_irq_set_mask(gc, gc->mask_cache);
}

static struct irqaction jz4740_cascade_action = {
	.handler = jz4740_cascade,
	.name = "JZ4740 cascade interrupt",
};

static int __init jz47xx_intc_of_init(struct device_node *node, unsigned num_chips)
{
	struct irq_chip_generic *gc;
	struct irq_chip_type *ct;
	struct irq_domain *domain;
	struct resource res;
	int ret, parent_irq;
	unsigned i;

	parent_irq = irq_of_parse_and_map(node, 0);
	if (!parent_irq)
		return -EINVAL;

	ret = of_address_to_resource(node, 0, &res);
	if (ret < 0)
		return ret;

	jz_num_chips = num_chips;
	jz_intc_base = ioremap(res.start,
			       ((num_chips - 1) * CHIP_SIZE) + 0x14);

	for (i = 0; i < num_chips; i++) {
		/* Mask all irqs */
		writel(0xffffffff, jz_intc_base + (i * CHIP_SIZE) + JZ_REG_INTC_SET_MASK);

		gc = irq_alloc_generic_chip("INTC", 1, IRQ_BASE + (i * 32),
					    jz_intc_base + (i * CHIP_SIZE),
					    handle_level_irq);

		gc->wake_enabled = IRQ_MSK(32);

		ct = gc->chip_types;
		ct->regs.enable = JZ_REG_INTC_CLEAR_MASK;
		ct->regs.disable = JZ_REG_INTC_SET_MASK;
		ct->chip.irq_unmask = irq_gc_unmask_enable_reg;
		ct->chip.irq_mask = irq_gc_mask_disable_reg;
		ct->chip.irq_mask_ack = irq_gc_mask_disable_reg;
		ct->chip.irq_set_wake = irq_gc_set_wake;
		ct->chip.irq_suspend = jz4740_irq_suspend;
		ct->chip.irq_resume = jz4740_irq_resume;

		irq_setup_generic_chip(gc, IRQ_MSK(32), 0, 0, IRQ_NOPROBE | IRQ_LEVEL);
	}

	domain = irq_domain_add_legacy(node, num_chips * 32, IRQ_BASE, 0,
				       &irq_domain_simple_ops, NULL);
	if (!domain)
		pr_warn("unable to register IRQ domain\n");

	setup_irq(parent_irq, &jz4740_cascade_action);
	return 0;
}

static int __init jz4740_intc_of_init(struct device_node *node,
	struct device_node *parent)
{
	return jz47xx_intc_of_init(node, 1);
}
IRQCHIP_DECLARE(jz4740_intc, "ingenic,jz4740-intc", jz4740_intc_of_init);

static int __init jz4780_intc_of_init(struct device_node *node,
	struct device_node *parent)
{
	return jz47xx_intc_of_init(node, 2);
}
IRQCHIP_DECLARE(jz4780_intc, "ingenic,jz4780-intc", jz4780_intc_of_init);

#ifdef CONFIG_DEBUG_FS

static inline void intc_seq_reg(struct seq_file *s, const char *name,
	unsigned int reg)
{
	seq_printf(s, "%s:\t\t%08x\n", name, readl(jz_intc_base + reg));
}

static int intc_regs_show(struct seq_file *s, void *unused)
{
	intc_seq_reg(s, "Status", JZ_REG_INTC_STATUS);
	intc_seq_reg(s, "Mask", JZ_REG_INTC_MASK);
	intc_seq_reg(s, "Pending", JZ_REG_INTC_PENDING);

	return 0;
}

static int intc_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, intc_regs_show, NULL);
}

static const struct file_operations intc_regs_operations = {
	.open		= intc_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init intc_debugfs_init(void)
{
	(void) debugfs_create_file("jz_regs_intc", S_IFREG | S_IRUGO,
				NULL, NULL, &intc_regs_operations);
	return 0;
}
subsys_initcall(intc_debugfs_init);

#endif
