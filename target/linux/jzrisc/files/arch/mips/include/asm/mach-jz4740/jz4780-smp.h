/*
 * JZ4780 SMP
 *
 * Copyright (c) 2013 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __MIPS_ASM_MACH_JZ4780_JZ4780_SMP_H__
#define __MIPS_ASM_MACH_JZ4780_JZ4780_SMP_H__

#define read_c0_corectrl()		__read_32bit_c0_register($12, 2)
#define write_c0_corectrl(val)		__write_32bit_c0_register($12, 2, val)

#define read_c0_corestatus()		__read_32bit_c0_register($12, 3)
#define write_c0_corestatus(val)	__write_32bit_c0_register($12, 3, val)

#define read_c0_reim()			__read_32bit_c0_register($12, 4)
#define write_c0_reim(val)		__write_32bit_c0_register($12, 4, val)

#define read_c0_mailbox0()		__read_32bit_c0_register($20, 0)
#define write_c0_mailbox0(val)		__write_32bit_c0_register($20, 0, val)

#define read_c0_mailbox1()		__read_32bit_c0_register($20, 1)
#define write_c0_mailbox1(val)		__write_32bit_c0_register($20, 1, val)

/*
 * Core Control register
 */
#define CORECTRL_SLEEP1M_SHIFT	17
#define CORECTRL_SLEEP1M	(_ULCAST_(0x1) << CORECTRL_SLEEP1M_SHIFT)
#define CORECTRL_SLEEP0M_SHIFT	16
#define CORECTRL_SLEEP0M	(_ULCAST_(0x1) << CORECTRL_SLEEP0M_SHIFT)
#define CORECTRL_RPC1_SHIFT	9
#define CORECTRL_RPC1		(_ULCAST_(0x1) << CORECTRL_RPC1_SHIFT)
#define CORECTRL_RPC0_SHIFT	8
#define CORECTRL_RPC0		(_ULCAST_(0x1) << CORECTRL_RPC0_SHIFT)
#define CORECTRL_SWRST1_SHIFT	1
#define CORECTRL_SWRST1		(_ULCAST_(0x1) << CORECTRL_SWRST1_SHIFT)
#define CORECTRL_SWRST0_SHIFT	0
#define CORECTRL_SWRST0		(_ULCAST_(0x1) << CORECTRL_SWRST0_SHIFT)

/*
 * Core Status register
 */
#define CORESTATUS_SLEEP1_SHIFT	17
#define CORESTATUS_SLEEP1	(_ULCAST_(0x1) << CORESTATUS_SLEEP1_SHIFT)
#define CORESTATUS_SLEEP0_SHIFT	16
#define CORESTATUS_SLEEP0	(_ULCAST_(0x1) << CORESTATUS_SLEEP0_SHIFT)
#define CORESTATUS_IRQ1P_SHIFT	9
#define CORESTATUS_IRQ1P	(_ULCAST_(0x1) << CORESTATUS_IRQ1P_SHIFT)
#define CORESTATUS_IRQ0P_SHIFT	8
#define CORESTATUS_IRQ0P	(_ULCAST_(0x1) << CORESTATUS_IRQ8P_SHIFT)
#define CORESTATUS_MIRQ1P_SHIFT	1
#define CORESTATUS_MIRQ1P	(_ULCAST_(0x1) << CORESTATUS_MIRQ1P_SHIFT)
#define CORESTATUS_MIRQ0P_SHIFT	0
#define CORESTATUS_MIRQ0P	(_ULCAST_(0x1) << CORESTATUS_MIRQ0P_SHIFT)

/*
 * Reset Entry & IRQ Mask register
 */
#define REIM_ENTRY_SHIFT	16
#define REIM_ENTRY		(_ULCAST_(0xffff) << REIM_ENTRY_SHIFT)
#define REIM_IRQ1M_SHIFT	9
#define REIM_IRQ1M		(_ULCAST_(0x1) << REIM_IRQ1M_SHIFT)
#define REIM_IRQ0M_SHIFT	8
#define REIM_IRQ0M		(_ULCAST_(0x1) << REIM_IRQ0M_SHIFT)
#define REIM_MBOXIRQ1M_SHIFT	1
#define REIM_MBOXIRQ1M		(_ULCAST_(0x1) << REIM_MBOXIRQ1M_SHIFT)
#define REIM_MBOXIRQ0M_SHIFT	0
#define REIM_MBOXIRQ0M		(_ULCAST_(0x1) << REIM_MBOXIRQ0M_SHIFT)

#ifdef CONFIG_SMP

extern void jz4780_smp_init(void);
extern void jz4780_secondary_cpu_entry(void);
extern void jz4780_smp_irq(void);

#else /* !CONFIG_SMP */

static inline void jz4780_smp_init(void) { }

#endif /* !CONFIG_SMP */

#endif /* __MIPS_ASM_MACH_JZ4780_JZ4780_SMP_H__ */
