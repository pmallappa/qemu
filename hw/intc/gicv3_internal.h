/*
 * ARM GIC support - internal interfaces
 *
 * Copyright (c) 2012 Linaro Limited
 * Copyright (c) 2015 Huawei.
 * Written by Peter Maydell
 * Extended to 64 cores by Shlomo Pongratz
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef QEMU_ARM_GICV3_INTERNAL_H
#define QEMU_ARM_GICV3_INTERNAL_H

#include "hw/intc/arm_gicv3.h"

#define ALL_CPU_MASK ((uint64_t) (0xffffffffffffffff))

/* Keep this macro so it will be easy to compare the code to GICv2 code.
 * The compiler will optimize any +/- operation involving this macro
 */
#define GICV3_BASE_IRQ (0)

#define GIC_SET_ENABLED(irq, cm) s->irq_state[irq].enabled |= (cm)
#define GIC_CLEAR_ENABLED(irq, cm) s->irq_state[irq].enabled &= ~(cm)
#define GIC_TEST_ENABLED(irq, cm) ((s->irq_state[irq].enabled & (cm)) != 0)
#define GIC_SET_PENDING(irq, cm) s->irq_state[irq].pending |= (cm)
#define GIC_TEST_PENDING(irq, cm) ((s->irq_state[irq].pending & (cm)) != 0)
#define GIC_CLEAR_PENDING(irq, cm) s->irq_state[irq].pending &= ~(cm)
#define GIC_SET_ACTIVE(irq, cm) s->irq_state[irq].active |= (cm)
#define GIC_CLEAR_ACTIVE(irq, cm) s->irq_state[irq].active &= ~(cm)
#define GIC_TEST_ACTIVE(irq, cm) ((s->irq_state[irq].active & (cm)) != 0)
#define GIC_SET_LEVEL(irq, cm) s->irq_state[irq].level |= (cm)
#define GIC_CLEAR_LEVEL(irq, cm) s->irq_state[irq].level &= ~(cm)
#define GIC_TEST_LEVEL(irq, cm) ((s->irq_state[irq].level & (cm)) != 0)
#define GIC_SET_EDGE_TRIGGER(irq) s->irq_state[irq].edge_trigger = true
#define GIC_CLEAR_EDGE_TRIGGER(irq) s->irq_state[irq].edge_trigger = false
#define GIC_TEST_EDGE_TRIGGER(irq) (s->irq_state[irq].edge_trigger)
#define GIC_GET_PRIORITY(irq, cpu) (((irq) < GICV3_INTERNAL) ?          \
                                    s->priority1[irq][cpu] :            \
                                    s->priority2[(irq) - GICV3_INTERNAL])
#define GIC_TARGET(irq) s->irq_target[irq]
#define GIC_CLEAR_GROUP(irq, cm) (s->irq_state[irq].group &= ~(cm))
#define GIC_SET_GROUP(irq, cm) (s->irq_state[irq].group |= (cm))
#define GIC_TEST_GROUP(irq, cm) ((s->irq_state[irq].group & (cm)) != 0)

/* The special cases for the revision property: */
#define REV_V3 3

static inline bool gic_test_pending(GICv3State *s, int irq, uint64_t cm)
{
    /* Edge-triggered interrupts are marked pending on a rising edge, but
     * level-triggered interrupts are either considered pending when the
     * level is active or if software has explicitly written to
     * GICD_ISPENDR to set the state pending.
     */
    return (s->irq_state[irq].pending & cm) ||
        (!GIC_TEST_EDGE_TRIGGER(irq) && GIC_TEST_LEVEL(irq, cm));
}


#define GICD_CTLR            0x0000
#define GICD_TYPER           0x0004
#define GICD_IIDR            0x0008
#define GICD_STATUSR         0x0010
#define GICD_SETSPI_NSR      0x0040
#define GICD_CLRSPI_NSR      0x0048
#define GICD_SETSPI_SR       0x0050
#define GICD_CLRSPI_SR       0x0058
#define GICD_SEIR            0x0068
#define GICD_ISENABLER       0x0100
#define GICD_ICENABLER       0x0180
#define GICD_ISPENDR         0x0200
#define GICD_ICPENDR         0x0280
#define GICD_ISACTIVER       0x0300
#define GICD_ICACTIVER       0x0380
#define GICD_IPRIORITYR      0x0400
#define GICD_ICFGR           0x0C00
#define GICD_IROUTER         0x6000
#define GICD_PIDR2           0xFFE8

/* GICD_CTLR fields  */
#define GICD_CTLR_EN_GRP0           (1U << 0)
#define GICD_CTLR_EN_GRP1NS         (1U << 1) /* GICv3 5.3.20 */
#define GICD_CTLR_EN_GRP1S          (1U << 2)
#define GICD_CTLR_EN_GRP1_ALL       (GICD_CTLR_EN_GRP1NS | GICD_CTLR_EN_GRP1S)
#define GICD_CTLR_ARE               (1U << 4)
#define GICD_CTLR_ARE_S             (1U << 4)
#define GICD_CTLR_ARE_NS            (1U << 5)
#define GICD_CTLR_DS                (1U << 6)
#define GICD_CTLR_RWP               (1U << 31)


#define GICD_IROUTER_SPI_MODE_ONE    (0U << 31)
#define GICD_IROUTER_SPI_MODE_ANY    (1U << 31)

#define GIC_PIDR2_ARCH_MASK   0xf0
#define GIC_PIDR2_ARCH_GICv3  0x30
#define GIC_PIDR2_ARCH_GICv4  0x40

/*
 * Re-Distributor registers, offsets from RD_base
 */
#define GICR_CTLR             GICD_CTLR
#define GICR_IIDR             0x0004
#define GICR_TYPER            0x0008
#define GICR_STATUSR          GICD_STATUSR
#define GICR_WAKER            0x0014
#define GICR_SETLPIR          0x0040
#define GICR_CLRLPIR          0x0048
#define GICR_SEIR             GICD_SEIR
#define GICR_PROPBASER        0x0070
#define GICR_PENDBASER        0x0078
#define GICR_INVLPIR          0x00A0
#define GICR_INVALLR          0x00B0
#define GICR_SYNCR            0x00C0
#define GICR_MOVLPIR          0x0100
#define GICR_MOVALLR          0x0110
#define GICR_PIDR2            GICD_PIDR2

#define GICR_WAKER_ProcessorSleep    (1U << 1)
#define GICR_WAKER_ChildrenAsleep    (1U << 2)

/*
 * Re-Distributor registers, offsets from SGI_base
 */
#define GICR_ISENABLER0         GICD_ISENABLER
#define GICR_ICENABLER0         GICD_ICENABLER
#define GICR_ISPENDR0           GICD_ISPENDR
#define GICR_ICPENDR0           GICD_ICPENDR
#define GICR_ISACTIVER0         GICD_ISACTIVER
#define GICR_ICACTIVER0         GICD_ICACTIVER
#define GICR_IPRIORITYR0        GICD_IPRIORITYR
#define GICR_ICFGR0             GICD_ICFGR

#define GICR_TYPER_VLPIS        (1U << 1)
#define GICR_TYPER_LAST         (1U << 4)

/*
 * Simulated used system registers
 */
#define GICC_CTLR_EN_GRP0    (1U << 0)
#define GICC_CTLR_EN_GRP1    (1U << 1)
#define GICC_CTLR_ACK_CTL    (1U << 2)
#define GICC_CTLR_FIQ_EN     (1U << 3)
#define GICC_CTLR_CBPR       (1U << 4) /* GICv1: SBPR */
#define GICC_CTLR_EOIMODE    (1U << 9)
#define GICC_CTLR_EOIMODE_NS (1U << 10)

#define NUM_CPU(s) ((s)->num_cpu)

/* Return true if this GIC config has interrupt groups, which is
 * true if we're a GICv3. Keep just
 */
static inline bool gic_has_groups(GICv3State *s)
{
    return 1;
}

#undef DEBUG_GICV3

#ifdef DEBUG_GICV3
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "arm_gicv3::%s: " fmt , __func__, ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while(0)
#endif

#endif /* !QEMU_ARM_GIC_INTERNAL_H */
