/*
 * ARM Generic/Distributed Interrupt Controller
 *
 * Copyright (c) 2006-2007 CodeSourcery.
 * Copyright (c) 2015 Huawei.
 * Written by Shlomo Pongratz
 * Base on gic.c by Paul Brook
 *
 * This code is licensed under the GPL.
 */

/* This file contains implementation code for the GIC-500 interrupt
 * controller, which is an implementation of the GICv3 architecture.
 * Curently it supports up to 64 cores. Enhancmet to 128 cores requires
 * working with bitops.
 */

#include "hw/sysbus.h"
#include "gicv3_internal.h"
#include "qom/cpu.h"

#undef DEBUG_GICV3

#ifdef DEBUG_GICV3
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "arm_gicv3::%s: " fmt , __func__, ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while(0)
#endif

/* These routines are called from cpu64.c and are defined in target-arm/cpu.h
 * like armv7m_nvic_XXX routines.
 * I couldn't find how to include it without compilation errors
 */
void armv8_gicv3_set_sgi(void *opaque, int cpuindex, uint64_t value);
uint64_t armv8_gicv3_acknowledge_irq(void *opaque, int cpuindex,
                                     MemTxAttrs attrs);
void armv8_gicv3_complete_irq(void *opaque, int cpuindex, int irq,
                                     MemTxAttrs attrs);
uint64_t armv8_gicv3_get_priority_mask(void *opaque, int cpuindex);
void armv8_gicv3_set_priority_mask(void *opaque, int cpuindex, uint32_t mask);
uint64_t armv8_gicv3_get_sre(void *opaque);
void armv8_gicv3_set_sre(void *opaque, uint64_t sre);
uint64_t armv8_gicv3_get_igrpen1(void *opaque, int cpuindex);
void armv8_gicv3_set_igrpen1(void *opaque, int cpuindex, uint64_t igrpen1);


static uint32_t gicv3_acknowledge_irq(GICv3State *s, int cpu, MemTxAttrs attrs);
static void gicv3_complete_irq(GICv3State *s, int cpu, int irq,
                               MemTxAttrs attrs);
static void gicv3_update(GICv3State *s);
static void gicv3_set_priority(GICv3State *s, int cpu, int irq, uint8_t val,
                               MemTxAttrs attrs);

static const uint8_t gic_dist_ids[] = {
    0x44, 0x00, 0x00, 0x00, 0x092, 0xB4, 0x3B, 0x00, 0x0D, 0xF0, 0x05, 0xB1
};

static const uint8_t gic_redist_ids[] = {
    0x44, 0x00, 0x00, 0x00, 0x093, 0xB4, 0x3B, 0x00, 0x0D, 0xF0, 0x05, 0xB1
};

/* Cuurently no GICv2 backwards compatibility (no memory mapped regs)
 * Uses system registers mode.
 */
static const int no_gicv2_bc = 1;
static uint32_t gic_sre;

#define NUM_CPU(s) ((s)->num_cpu)

static inline int gic_get_current_cpu(GICv3State *s)
{
    if (s->num_cpu > 1) {
        return current_cpu->cpu_index;
    }
    return 0;
}

/* Return true if this GIC config has interrupt groups, which is
 * true if we're a GICv3. Keep just
 */
static inline bool gic_has_groups(GICv3State *s)
{
    return 1;
}

/* TODO: Many places that call this routine could be optimized.  */
/* Update interrupt status after enabled or pending bits have been changed.  */
void gicv3_update(GICv3State *s)
{
    int best_irq;
    int best_prio;
    int irq;
    int irq_level, fiq_level;
    int cpu;
    uint64_t cm;

    for (cpu = 0; cpu < NUM_CPU(s); cpu++) {
        cm = 1ll << cpu;
        s->current_pending[cpu] = 1023;
        if (!(s->ctlr & (GICD_CTLR_EN_GRP0 | GICD_CTLR_EN_GRP1_ALL))
            || !s->cpu_enabled[cpu]
            || !(s->cpu_ctlr[cpu] & (GICC_CTLR_EN_GRP0 | GICC_CTLR_EN_GRP1))) {
            qemu_irq_lower(s->parent_irq[cpu]);
            qemu_irq_lower(s->parent_fiq[cpu]);
            /* In original GICv2 there is a return here. But if status is
             * disabled then all parent IRQs need to be lowered
             * And assume CPU i is disabled then with the original GICv2
             * implementation CPU - 1 will be considered but not CPU + 1
             */
            continue;
        }
        best_prio = 0x100;
        best_irq = 1023;
        for (irq = 0; irq < s->num_irq; irq++) {
            if (GIC_TEST_ENABLED(irq, cm) && gic_test_pending(s, irq, cm) &&
                (irq < GICV3_INTERNAL || (GIC_TARGET(irq) & cm))) {
                if (GIC_GET_PRIORITY(irq, cpu) < best_prio) {
                    best_prio = GIC_GET_PRIORITY(irq, cpu);
                    best_irq = irq;
                }
            }
        }

        irq_level = fiq_level = 0;

        if (best_prio < s->priority_mask[cpu]) {
            s->current_pending[cpu] = best_irq;
            if (best_prio < s->running_priority[cpu]) {
                int group = GIC_TEST_GROUP(best_irq, cm);
                if (extract32(s->ctlr, group, 1) &&
                    extract32(s->cpu_ctlr[cpu], group, 1)) {
                    if (group == 0 && s->cpu_ctlr[cpu] & GICC_CTLR_FIQ_EN) {
                        DPRINTF("Raised pending FIQ %d (cpu %d)\n",
                                best_irq, cpu);
                        fiq_level = 1;
                    } else {
                        DPRINTF("Raised pending IRQ %d (cpu %d)\n",
                                best_irq, cpu);
                        irq_level = 1;
                    }
                }
            }
        }

        qemu_set_irq(s->parent_irq[cpu], irq_level);
        qemu_set_irq(s->parent_fiq[cpu], fiq_level);
    }
}

static void gicv3_set_irq_generic(GICv3State *s, int irq, int level,
                                  uint64_t cm, uint64_t target)
{
    if (level) {
        GIC_SET_LEVEL(irq, cm);
        DPRINTF("Set %d pending mask 0x%lx\n", irq, target);
        if (GIC_TEST_EDGE_TRIGGER(irq)) {
            GIC_SET_PENDING(irq, target);
        }
    } else {
        GIC_CLEAR_LEVEL(irq, cm);
    }
}

/* Process a change in an external IRQ input.  */
static void gic_set_irq(void *opaque, int irq, int level)
{
    /* Meaning of the 'irq' parameter:
     *  [0..N-1] : external interrupts
     *  [N..N+31] : PPI (internal) interrupts for CPU 0
     *  [N+32..N+63] : PPI (internal interrupts for CPU 1
     *  ...
     */
    GICv3State *s = (GICv3State *)opaque;
    uint64_t cm, target;

    if (irq < (s->num_irq - GICV3_INTERNAL)) {
        /* The first external input line is internal interrupt 32.  */
        cm = ALL_CPU_MASK;
        irq += GICV3_INTERNAL;
        target = GIC_TARGET(irq);
    } else {
        int cpu;
        irq -= (s->num_irq - GICV3_INTERNAL);
        cpu = irq / GICV3_INTERNAL;
        irq %= GICV3_INTERNAL;
        cm = 1ll << cpu;
        target = cm;
    }

    assert(irq >= GICV3_NR_SGIS);

    if (level == GIC_TEST_LEVEL(irq, cm)) {
        return;
    }

    gicv3_set_irq_generic(s, irq, level, cm, target);

    gicv3_update(s);
}

static uint16_t gic_get_current_pending_irq(GICv3State *s, int cpu,
                                            MemTxAttrs attrs)
{
    uint16_t pending_irq = s->current_pending[cpu];

    if (pending_irq < GICV3_MAXIRQ && gic_has_groups(s)) {
        /* GICv3 section 4.1.4
         * In systems that support a single security state,
         * there is no security distinction between Secure and Non -Secure
         * interrupt groups. That is, "Secure" and "Non-Secure" interrupts are
         * always accessible.
         */
        if (s->security_levels > 1) {
            int group = GIC_TEST_GROUP(pending_irq, (1ll << cpu));
            bool secure = attrs.secure;
            if (group == 0) {
                if (!secure) {
                    fprintf(stderr, "%s::%d\n", __func__, __LINE__);
                    /* Group0 interrupts hidden from Non-secure access */
                    return 1023;
                }
            } else {
                /* Note GICv3 5.6.18: AckCtl was removed in GICv3
                 * Also look at GICv3 5.3.20 for group 1 secure and non secure
                 */
                if (secure) {
                    if (s->ctlr & GICD_CTLR_EN_GRP1NS) {
                        /* Secure access to non secure group 1 interrupts */
                        fprintf(stderr, "%s::%d\n", __func__, __LINE__);
                        return 1022;
                    }
                } else {
                    if (s->ctlr & GICD_CTLR_EN_GRP1S) {
                        /* Non secure access to secure group 1 interrupts */
                        fprintf(stderr, "%s::%d\n", __func__, __LINE__);
                        return 1022;
                    }
                }
            }
        }
    }
    return pending_irq;
}

static void gic_set_running_irq(GICv3State *s, int cpu, int irq)
{
    s->running_irq[cpu] = irq;
    if (irq == 1023) {
        s->running_priority[cpu] = 0x100;
    } else {
        s->running_priority[cpu] = GIC_GET_PRIORITY(irq, cpu);
    }
    gicv3_update(s);
}

static uint32_t gicv3_acknowledge_irq(GICv3State *s, int cpu, MemTxAttrs attrs)
{
    int ret, irq, src;
    uint64_t cm = 1ll << cpu;

    /* gic_get_current_pending_irq() will return 1022 or 1023 appropriately
     * for the case where this GIC supports grouping and the pending interrupt
     * is in the wrong group.
     */
    irq = gic_get_current_pending_irq(s, cpu, attrs);

    if (irq >= GICV3_MAXIRQ) {
        //DPRINTF("ACK, no pending interrupt or it is hidden: %d\n", irq);
        return irq;
    }

    if (GIC_GET_PRIORITY(irq, cpu) >= s->running_priority[cpu]) {
        //DPRINTF("ACK, pending interrupt (%d) has insufficient priority\n", irq);
        return 1023;
    }

    s->last_active[irq][cpu] = s->running_irq[cpu];

    if (irq < GICV3_NR_SGIS) {
        /* Lookup the source CPU for the SGI and clear this in the
         * sgi_pending map.  Return the src and clear the overall pending
         * state on this CPU if the SGI is not pending from any CPUs.
         */
        assert(s->sgi_state[irq].pending[cpu] != 0);
        src = ctz64(s->sgi_state[irq].pending[cpu]);
        s->sgi_state[irq].pending[cpu] &= ~(1ll << src);
        if (s->sgi_state[irq].pending[cpu] == 0) {
            GIC_CLEAR_PENDING(irq, cm);
        }
        /* GICv3 kernel driver dosen't mask src bits like GICv2 driver
         * so don't add src i.e. ret = irq | ((src & 0x7) << 10);
         * Section 4.2.10 in GICv3 specification
         */
        ret = irq;
    } else {
        //DPRINTF("ACK irq(%d) cpu(%d) \n", irq, cpu);
        /* Clear pending state for both level and edge triggered
         * interrupts. (level triggered interrupts with an active line
         * remain pending, see gic_test_pending)
         */
        GIC_CLEAR_PENDING(irq, cm);
        ret = irq;
    }

    gic_set_running_irq(s, cpu, irq);
    DPRINTF("out ACK irq-ret(%d) cpu(%d) \n", ret, cpu);
    return ret;
}

static void gicv3_set_priority(GICv3State *s, int cpu, int irq, uint8_t val,
                        MemTxAttrs attrs)
{
    DPRINTF("%s cpu(%d) secure(%d)\n", __func__, cpu, attrs.secure);
    if (s->security_levels == 1 && !attrs.secure) {
        if (!GIC_TEST_GROUP(irq, (1ll << cpu))) {
            return; /* Ignore Non-secure access of Group0 IRQ */
        }
        val = 0x80 | (val >> 1); /* Non-secure view */
    }

    if (irq < GICV3_INTERNAL) {
        s->priority1[irq][cpu] = val;
    } else {
        s->priority2[irq - GICV3_INTERNAL] = val;
    }
}

static void gicv3_complete_irq(GICv3State *s, int cpu, int irq, MemTxAttrs attrs)
{
    uint64_t cm = 1ll << cpu;
    DPRINTF("EOI irq(%d) cpu (%d)\n", irq, cpu);
    if (irq >= s->num_irq) {
        /* This handles two cases:
         * 1. If software writes the ID of a spurious interrupt [ie 1023]
         * to the GICC_EOIR, the GIC ignores that write.
         * 2. If software writes the number of a non-existent interrupt
         * this must be a subcase of "value written does not match the last
         * valid interrupt value read from the Interrupt Acknowledge
         * register" and so this is UNPREDICTABLE. We choose to ignore it.
         */
        return;
    }

    if (s->running_irq[cpu] == 1023)
        return; /* No active IRQ.  */

    if (s->security_levels == 1 && !attrs.secure && !GIC_TEST_GROUP(irq, cm)) {
        DPRINTF("Non-secure EOI for Group0 interrupt %d ignored\n", irq);
        return;
    }

    /* Secure EOI with GICC_CTLR.AckCtl == 0 when the IRQ is a Group 1
     * interrupt is UNPREDICTABLE. We choose to handle it as if AckCtl == 1,
     * i.e. go ahead and complete the irq anyway.
     */

    if (irq != s->running_irq[cpu]) {
        /* Complete an IRQ that is not currently running.  */
        int tmp = s->running_irq[cpu];
        while (s->last_active[tmp][cpu] != 1023) {
            if (s->last_active[tmp][cpu] == irq) {
                s->last_active[tmp][cpu] = s->last_active[irq][cpu];
                break;
            }
            tmp = s->last_active[tmp][cpu];
        }
    } else {
        /* Complete the current running IRQ.  */
        gic_set_running_irq(s, cpu, s->last_active[s->running_irq[cpu]][cpu]);
    }
}

static uint64_t gic_dist_readb(void *opaque, hwaddr offset, MemTxAttrs attrs)
{
    GICv3State *s = (GICv3State *)opaque;
    uint64_t res;
    int irq;
    int i;
    int cpu;
    uint64_t cm;
    uint64_t mask;

    cpu = gic_get_current_cpu(s);
    cm = 1ll << cpu;
    if (offset < 0x100) {
        if (offset == 0) {/* GICD_CTLR */
            /* GICv3 5.3.20 without GICV2 backwards compatibility  */
            if (s->security_levels > 1) {
                /* Support TWO security states */
                if (attrs.secure) {
                    return s->ctlr;
                } else {
                    /* For non secure access bits [30:5] are reserved and ARE_NS
                     * is RAO/WI note that ARE_NS in this case is bit [4] same
                     * as ARE_S in the prvious secure case. Now since it is
                     * RAO/WI than bit [0] EnableGrp1 is reserved and we can
                     * only use Bit [1] EnableGrp1A to enable Non-secure Group1
                     * interrupts
                     */
                    return s->ctlr & (GICD_CTLR_ARE_NS |
                                      GICD_CTLR_EN_GRP1NS |
                                      GICD_CTLR_RWP);
                }
            } else {
                /* Support ONE security state without ARE is RAO/WI*/
                return s->ctlr & (GICD_CTLR_ARE |
                                  GICD_CTLR_EN_GRP0 | GICD_CTLR_EN_GRP1NS |
                                  GICD_CTLR_RWP);
            }
        }
        if (offset == 4) { /* GICD_TYPER */
            uint64_t num = NUM_CPU(s);
            /* the number of cores in the system, saturated to 8 minus one. */
            if (num > 8)
                num = 8;
            /* The num_irqs as given from virt machine via "num-irq"
             * includes the internal irqs, so subtract them
             */
            res = (s->num_irq - GICV3_INTERNAL) / 32;
            res |= (num - 1) << 5;
            res |= 0xF << 19;
            return res;
        }
        if (offset == 0x08)
            return 0x43B; /* GIC_IIDR */
        if (offset >= 0x80) {
            /* Interrupt Group Registers: these RAZ/WI if this is an NS
             * access to a GIC with the security extensions, or if the GIC
             * doesn't have groups at all.
             */
            res = 0;
            if (!attrs.secure) {
                /* GIC-500 comment 'f' */
                goto bad_reg;
            }
            /* Every byte offset holds 8 group status bits */
            irq = (offset - 0x080) * 8 + GICV3_BASE_IRQ;
            if (irq >= s->num_irq) {
                goto bad_reg;
            }
            for (i = 0; i < 8; i++) {
                if (GIC_TEST_GROUP(irq + i, cm)) {
                    res |= (1 << i);
                }
            }
            return res;
        }
        goto bad_reg;
    } else if (offset < 0x200) {
        /* Interrupt Set/Clear Enable.  */
        if (offset < 0x180)
            irq = (offset - 0x100) * 8;
        else
            irq = (offset - 0x180) * 8;
        irq += GICV3_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        res = 0;
        for (i = 0; i < 8; i++) {
            if (GIC_TEST_ENABLED(irq + i, cm)) {
                res |= (1 << i);
            }
        }
    } else if (offset < 0x300) {
        /* Interrupt Set/Clear Pending.  */
        if (offset < 0x280)
            irq = (offset - 0x200) * 8;
        else
            irq = (offset - 0x280) * 8;
        irq += GICV3_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        /* Comment 'o' GICv2 backwards competability support is not set...*/
        if (irq < GICV3_INTERNAL && no_gicv2_bc)
            goto bad_reg;
        res = 0;
        mask = (irq < GICV3_INTERNAL) ?  cm : ALL_CPU_MASK;
        for (i = 0; i < 8; i++) {
            if (gic_test_pending(s, irq + i, mask)) {
                res |= (1 << i);
            }
        }
    } else if (offset < 0x400) {
        /* Interrupt Set/Clear Active.  */
        if (offset < 0x380)
            irq = (offset - 0x300) * 8 + GICV3_BASE_IRQ;
        else
            irq = (offset - 0x380) * 8 + GICV3_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        /* Comment 'o' GICv2 backwards competability support is not set...*/
        if (irq < GICV3_INTERNAL && no_gicv2_bc)
            goto bad_reg;
        res = 0;
        mask = (irq < GICV3_INTERNAL) ?  cm : ALL_CPU_MASK;
        for (i = 0; i < 8; i++) {
            if (GIC_TEST_ACTIVE(irq + i, mask)) {
                res |= (1 << i);
            }
        }
    } else if (offset < 0x800) {
        /* Interrupt Priority.  */
        irq = (offset - 0x400) + GICV3_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        /* Comment 'p' GICv2 backwards competability support is not set...*/
        if (irq < GICV3_INTERNAL * 8 && no_gicv2_bc)
            goto bad_reg;
        res = GIC_GET_PRIORITY(irq, cpu);
    } else if (offset < 0xc00) {
        /* Interrupt CPU Target.  */
        if (s->num_cpu == 1) {
            /* For uniprocessor GICs these RAZ/WI */
            res = 0;
        } else {
            irq = (offset - 0x800) + GICV3_BASE_IRQ;
            if (irq >= s->num_irq) {
                goto bad_reg;
            }
            /* Comment 'p' GICv2 backwards competability support is not set...*/
            if (irq < GICV3_INTERNAL * 8 && no_gicv2_bc)
                goto bad_reg;
            if (irq >= 29 && irq <= 31) {
                res = cm;
            } else {
                res = GIC_TARGET(irq);
            }
        }
    } else if (offset < 0xd00) {
        /* Interrupt Configuration.  */
        irq = (offset - 0xc00) * 4 + GICV3_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        /* Comment 'l' GICv2 backwards competability support is not set...*/
        if (irq < GICV3_INTERNAL && no_gicv2_bc)
            goto bad_reg;
        res = 0;
        for (i = 0; i < 4; i++) {
            /* Even bits are reserved */
            if (GIC_TEST_EDGE_TRIGGER(irq + i))
                res |= (2 << (i * 2));
        }
    } else if (offset < 0xf10) {
        goto bad_reg;
    } else if (offset < 0xf30) {
        /* Comments 'e' & 't' GICv2 backwards competability support is not set...*/
        if (no_gicv2_bc)
            goto bad_reg;
        /* These are 32 bit registers, should not be used with 128 cores. */
        if (offset < 0xf20) {
            /* GICD_CPENDSGIRn */
            irq = (offset - 0xf10);
        } else {
            irq = (offset - 0xf20);
            /* GICD_SPENDSGIRn */
        }
        res = s->sgi_state[irq].pending[cpu];
    } else if (offset < 0xffd0) {
        goto bad_reg;
    } else /* offset >= 0xffd0 */ {
        if (offset & 3) {
            res = 0;
        } else {
            res = gic_dist_ids[(offset - 0xffd0) >> 2];
        }
    }
    return res;
bad_reg:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: Bad offset %x\n", __func__, (int)offset);
    return 0;
}

static uint64_t gic_dist_readll(void *opaque, hwaddr offset,
                                MemTxAttrs attrs)
{
    GICv3State *s = (GICv3State *)opaque;
    uint64_t value;

    if (offset >= 0x6100 && offset <= 0x7EF8) {
        int irq = (offset - 0x6100) / 8;
        /* GCID_IROUTERn [affinity-3:X:affinity-2:affinity-1:affininty-0]
         * See kernel code for fields
         * GIC 500 currently supports 32 clusters with 8 cores each,
         * but virtv2 fills the Aff0 before filling Aff1 so
         * 16 = 2 * 8 but not 4 x 4 nor 8 x 2 not 16 x 1
         * Note Linux kernel doesn't set bit 31 thus send to all is not needed
         */
        uint32_t cpu, Aff1, Aff0;
        cpu = ctz64(s->irq_target[irq]);
        Aff1 = cpu / 8;
        Aff0 = cpu % 8;
        value = (Aff1 << 8) | Aff0;
        return value;
    }

    value = gic_dist_readb(opaque, offset, attrs);
    value |= gic_dist_readb(opaque, offset + 1, attrs) << 8;
    value |= gic_dist_readb(opaque, offset + 2, attrs) << 16;
    value |= gic_dist_readb(opaque, offset + 3, attrs) << 24;
    value |= gic_dist_readb(opaque, offset + 4, attrs) << 32;
    value |= gic_dist_readb(opaque, offset + 5, attrs) << 40;
    value |= gic_dist_readb(opaque, offset + 6, attrs) << 48;
    value |= gic_dist_readb(opaque, offset + 7, attrs) << 56;

    return value;
}


static MemTxResult gic_dist_read(void *opaque, hwaddr offset, uint64_t *data,
                                 unsigned size, MemTxAttrs attrs)
{
    switch (size) {
    case 1:
        *data = gic_dist_readb(opaque, offset, attrs);
        return MEMTX_OK;
    case 2:
        *data = gic_dist_readb(opaque, offset, attrs);
        *data |= gic_dist_readb(opaque, offset + 1, attrs) << 8;
        return MEMTX_OK;
    case 4:
        *data = gic_dist_readb(opaque, offset, attrs);
        *data |= gic_dist_readb(opaque, offset + 1, attrs) << 8;
        *data |= gic_dist_readb(opaque, offset + 2, attrs) << 16;
        *data |= gic_dist_readb(opaque, offset + 3, attrs) << 24;
        return MEMTX_OK;
    case 8:
        *data = gic_dist_readll(opaque, offset, attrs);
        return MEMTX_OK;
    default:
        return MEMTX_ERROR;
    }
}

static void gic_dist_writeb(void *opaque, hwaddr offset,
                            uint64_t value, MemTxAttrs attrs)
{
    GICv3State *s = (GICv3State *)opaque;
    int irq;
    int i;
    int cpu;

    cpu = gic_get_current_cpu(s);

    if (offset < 0x100) {
        if (offset < 0x80) {
            /* Error! should be handeld in writel. */
            DPRINTF("Distributor: error should be handled in dist_writel \n");
            goto bad_reg;
        } else if (offset >= 0x80) {
            DPRINTF("Distributor: GICD_IGROUPRn\n");
            /* Interrupt Group Registers: RAZ/WI for NS access to secure
             * GIC, or for GICs without groups.
             */
            if (!attrs.secure) {
                /* GIC-500 comment 'f' */
                goto bad_reg;
            }
            /* Every byte offset holds 8 group status bits */
            irq = (offset - 0x80) * 8 + GICV3_BASE_IRQ;
            if (irq >= s->num_irq) {
                goto bad_reg;
            }
            for (i = 0; i < 8; i++) {
                /* Group bits are banked for private interrupts */
                uint64_t cm = (irq < GICV3_INTERNAL) ? (1ll << cpu) : ALL_CPU_MASK;
                if (value & (1 << i)) {
                    /* Group1 (Non-secure) */
                    GIC_SET_GROUP(irq + i, cm);
                } else {
                    /* Group0 (Secure) */
                    GIC_CLEAR_GROUP(irq + i, cm);
                }
            }
        } else {
            goto bad_reg;
        }
    } else if (offset < 0x180) {
        /* Interrupt Set Enable.  */
        irq = (offset - 0x100) * 8 + GICV3_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        if (irq < GICV3_NR_SGIS) {
            DPRINTF("ISENABLERn SGI should be only in redistributer %d\n", irq);
            /* Ignored according to comment 'g' in GIC-500 document.
             * The comment doen't say that the whole register is reservd
             */
            return;
        }

        for (i = 0; i < 8; i++) {
            if (value & (1 << i)) {
                uint64_t mask =
                    (irq < GICV3_INTERNAL) ? (1ll << cpu) : GIC_TARGET(irq + i);
                uint64_t cm = (irq < GICV3_INTERNAL) ? (1ll << cpu) : ALL_CPU_MASK;

                if (!GIC_TEST_ENABLED(irq + i, cm)) {
                    DPRINTF("Enabled IRQ %d\n", irq + i);
                }
                GIC_SET_ENABLED(irq + i, cm);
                /* If a raised level triggered IRQ enabled then mark
                   is as pending.  */
                if (GIC_TEST_LEVEL(irq + i, mask)
                        && !GIC_TEST_EDGE_TRIGGER(irq + i)) {
                    DPRINTF("Set %d pending mask %lx\n", irq + i, mask);
                    GIC_SET_PENDING(irq + i, mask);
                }
            }
        }
    } else if (offset < 0x200) {
        /* Interrupt Clear Enable.  */
        irq = (offset - 0x180) * 8 + GICV3_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        if (irq < GICV3_NR_SGIS) {
            DPRINTF("ICENABLERn SGI should be only in redistributer %d\n", irq);
            /* Ignored according to comment 'g' in GIC-500 document.
             * The comment doen't say that the whole register is reservd
             */
            return;
        }

        for (i = 0; i < 8; i++) {
            if (value & (1 << i)) {
                uint64_t cm = (irq < GICV3_INTERNAL) ? (1ll << cpu) : ALL_CPU_MASK;

                if (GIC_TEST_ENABLED(irq + i, cm)) {
                    DPRINTF("Disabled IRQ %d\n", irq + i);
                }
                GIC_CLEAR_ENABLED(irq + i, cm);
            }
        }
    } else if (offset < 0x280) {
        /* Interrupt Set Pending.  */
        irq = (offset - 0x200) * 8 + GICV3_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        /* Comment 'o' GICv2 backwards competability support is not set...*/
        if (irq < GICV3_INTERNAL && no_gicv2_bc)
            goto bad_reg;

        for (i = 0; i < 8; i++) {
            if (value & (1 << i)) {
                GIC_SET_PENDING(irq + i, GIC_TARGET(irq + i));
            }
        }
    } else if (offset < 0x300) {
        /* Interrupt Clear Pending.  */
        irq = (offset - 0x280) * 8 + GICV3_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        /* Comment 'o' GICv2 backwards competability support is not set...*/
        if (irq < GICV3_INTERNAL && no_gicv2_bc)
            goto bad_reg;

        for (i = 0; i < 8; i++) {
            /* ??? This currently clears the pending bit for all CPUs, even
               for per-CPU interrupts. It's unclear whether this is the
               correct behavior.  */
            if (value & (1 << i)) {
                GIC_CLEAR_PENDING(irq + i, ALL_CPU_MASK);
            }
        }
    } else if (offset < 0x400) {
        /* Interrupt Active.  */
        goto bad_reg;
    } else if (offset < 0x800) {
        /* Interrupt Priority.  */
        irq = (offset - 0x400) + GICV3_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        /* Comment 'p' GICv2 backwards competability support is not set...*/
        if (irq < GICV3_INTERNAL * 8 && no_gicv2_bc)
            goto bad_reg;
        gicv3_set_priority(s, cpu, irq, value, attrs);
    } else if (offset < 0xc00) {
        /* Interrupt CPU Target. RAZ/WI on uni-processor GICs */
        if (s->num_cpu != 1) {
            irq = (offset - 0x800) + GICV3_BASE_IRQ;
            if (irq >= s->num_irq) {
                goto bad_reg;
            }
            /* Comment 'p' GICv2 backwards competability support is not set...*/
            if (irq < GICV3_INTERNAL * 8 && no_gicv2_bc)
                goto bad_reg;
            if (irq < 29) {
                value = 0;
            } else if (irq < GICV3_INTERNAL) {
                value = ALL_CPU_MASK;
            }
            s->irq_target[irq] = value & ALL_CPU_MASK;
        }
    } else if (offset < 0xd00) {
        /* Interrupt Configuration.  */
        irq = (offset - 0xc00) * 4 + GICV3_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        /* Comment 'l' GICv2 backwards competability support is not set...*/
        if (irq < GICV3_INTERNAL && no_gicv2_bc)
            goto bad_reg;
        if (irq < GICV3_NR_SGIS)
            value |= 0xaa; /* 10101010 */
        /* Even bits are reserved */
        for (i = 0; i < 4; i++) {
            if (value & (2 << (i * 2))) {
                GIC_SET_EDGE_TRIGGER(irq + i);
            } else {
                GIC_CLEAR_EDGE_TRIGGER(irq + i);
            }
        }
    } else if (offset < 0xf30) {
        /* Comments 'e' & 't' GICv2 backwards competability support is not set */
        if (no_gicv2_bc)
            goto bad_reg;
        /* These are 32 bit registers, should not be used with 128 cores. */
        if (offset < 0xf20) {
            /* GICD_CPENDSGIRn */
            irq = (offset - 0xf10);
            DPRINTF("GICD_CPENDSGIRn irq(%d) %lu\n", irq, value);

            s->sgi_state[irq].pending[cpu] &= ~value;
            if (s->sgi_state[irq].pending[cpu] == 0) {
                GIC_CLEAR_PENDING(irq, 1ll << cpu);
            }
        } else {
            irq = (offset - 0xf20);
            /* GICD_SPENDSGIRn */
            DPRINTF("GICD_SPENDSGIRn irq(%d) %lu\n", irq, value);

            GIC_SET_PENDING(irq, 1ll << cpu);
            s->sgi_state[irq].pending[cpu] |= value;
        }
    }
    gicv3_update(s);
    return;
bad_reg:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: Bad offset %x\n", __func__, (int)offset);
}

static void gic_dist_writew(void *opaque, hwaddr offset,
                            uint64_t value, MemTxAttrs attrs)
{
    gic_dist_writeb(opaque, offset, value & 0xff, attrs);
    gic_dist_writeb(opaque, offset + 1, value >> 8, attrs);
}

static void gic_dist_writel(void *opaque, hwaddr offset,
                            uint64_t value, MemTxAttrs attrs)
{
    if (offset < 0x80) {
        if (offset == 0) {
            GICv3State *s = (GICv3State *)opaque;
            uint32_t ctlr;
            /* One day we may have MT QEMU */
            s->ctlr |= GICD_CTLR_RWP;
            /* GICv3 5.3.20 */
            if (s->security_levels > 1) {
                /* support TWO security states */
                if (attrs.secure) {
                    /* for secure access DS is RAZ/WI ARE_NS is RAO/WI and
                     * ARE_S is RAO/WI
                     * for non secure access bits [30:5] are reserved and ARE_NS is
                     * RAO/WI
                     *
                     * Can modify bits[2:0] Groups 0 & 1 NS & 1 S
                     */
                    ctlr = s->ctlr & ~(GICD_CTLR_EN_GRP0 | GICD_CTLR_EN_GRP1_ALL);
                    value &= (GICD_CTLR_EN_GRP0 | GICD_CTLR_EN_GRP1_ALL);
                    value |= GICD_CTLR_ARE_S | GICD_CTLR_ARE_NS;
                } else {
                    /* For non secure access bits [30:5] are reserved and ARE_NS is
                     * RAO/WI note that ARE_NS in this case is bit [4] same as
                     * ARE_S in the prvious secure case. Now since it is RAO/WI
                     * thane bit [0] EnableGrp1 is reserved and we can only use
                     * Bit [1] EnableGrp1A to enable Non-secure Group1 interrupts
                     */
                    ctlr = s->ctlr & ~GICD_CTLR_EN_GRP1NS;
                    value &= GICD_CTLR_EN_GRP1NS;
                    value |= GICD_CTLR_ARE_NS;
                }
            } else {
                /* support ONE security state */
                /* if GICv2 backwards cimpatibility is not implemented than ARE
                 * is RAO/WI
                 */
                ctlr = s->ctlr & ~(GICD_CTLR_EN_GRP0 | GICD_CTLR_EN_GRP1NS);
                value &= (GICD_CTLR_EN_GRP0 | GICD_CTLR_EN_GRP1NS);
                value |= GICD_CTLR_ARE;
            }
            s->ctlr = (ctlr | value) & ~GICD_CTLR_RWP;
            DPRINTF("Distributor: Group0 %sabled; Group 1S %sabled Group 1NS %sabled\n",
                    s->ctlr & GICD_CTLR_EN_GRP0 ? "En" : "Dis",
                    s->ctlr & GICD_CTLR_EN_GRP1S ? "En" : "Dis",
                    s->ctlr & GICD_CTLR_EN_GRP1NS ? "En" : "Dis");
            return;
        }
        if (offset < 0x40) {
            /* RO or reserved < 0x40 */
            return;
        }
        if (offset < 0x80) {
            /* SPI not supported < 0x80 */
            return;
        }
        return;
    }
    if (offset == 0xf00) {
        /* GICD_SGIR Software generated Interrupt register
         * This register should not be used if GICv2 backwards computability
         * support is not included. (comment t page 3-8 on GIC-500 doc)
         */
        int cpu;
        int irq;
        uint64_t mask, cm;
        int target_cpu;
        GICv3State *s = (GICv3State *)opaque;

        DPRINTF("GICv2 backwards computability is not supported\n");
        cpu = gic_get_current_cpu(s);
        irq = value & 0x3ff;
        switch ((value >> 24) & 3) {
        case 0:
            mask = (value >> 16) & ALL_CPU_MASK;
            break;
        case 1:
            mask = ALL_CPU_MASK ^ (1ll << cpu);
            break;
        case 2:
            mask = 1ll << cpu;
            break;
        default:
            DPRINTF("Bad Soft Int target filter\n");
            mask = ALL_CPU_MASK;
            break;
        }
        cm = (1ll << cpu);
        DPRINTF("irq(%d) mask(%lu)\n", irq, mask);
        GIC_SET_PENDING(irq, mask);
        target_cpu = ctz64(mask);
        while (target_cpu < GICV3_NCPU) {
            s->sgi_state[irq].pending[target_cpu] |= cm;
            mask &= ~(1ll << target_cpu);
            target_cpu = ctz64(mask);
        }
        gicv3_update(s);
        return;
    }
    gic_dist_writew(opaque, offset, value & 0xffff, attrs);
    gic_dist_writew(opaque, offset + 2, value >> 16, attrs);
}

static void gic_dist_writell(void *opaque, hwaddr offset,
                            uint64_t value, MemTxAttrs attrs)
{
    GICv3State *s = (GICv3State *)opaque;

    if (offset >= 0x6100 && offset <= 0x7EF8) {
        int irq = (offset - 0x6100) / 8;
        /* GCID_IROUTERn [affinity-3:X:affinity-2:affinity-1:affininty-0]
         * See kernel code for fields
         * GIC 500 currently supports 32 clusters with 8 cores each,
         * but virtv2 fills the Aff0 before filling Aff1 so
         * 16 = 2 * 8 but not 4 x 4 nor 8 x 2 not 16 x 1
         * Note Linux kernel doesn't set bit 31 thus send to all is not needed
         */
        uint32_t cpu, Aff1, Aff0;
        Aff1 = (value & 0xf00) >> (8 - 3); /* Shift by 8 multiply by 8 */
        Aff0 = value & 0x7;
        cpu = Aff1 + Aff0;
        s->irq_target[irq] = 1ll << cpu;
        gicv3_update(s);
        DPRINTF("irq(%d) cpu(%d)\n", irq, cpu);
        return;
    }

    gic_dist_writel(opaque, offset, value & 0xffffffff, attrs);
    gic_dist_writel(opaque, offset + 4, value >> 32, attrs);
}

static MemTxResult gic_dist_write(void *opaque, hwaddr addr, uint64_t data,
                                  unsigned size, MemTxAttrs attrs)
{
    DPRINTF("offset %p data %p secure(%d)\n", (void *) addr, (void *) data, attrs.secure);
    switch (size) {
    case 1:
        gic_dist_writeb(opaque, addr, data, attrs);
        return MEMTX_OK;
    case 2:
        gic_dist_writew(opaque, addr, data, attrs);
        return MEMTX_OK;
    case 4:
        gic_dist_writel(opaque, addr, data, attrs);
        return MEMTX_OK;
    case 8:
        gic_dist_writell(opaque, addr, data, attrs);
        return MEMTX_OK;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: size %u\n", __func__, size);
        return MEMTX_ERROR;
    }
}

/* ITS routines are stubs for future development */
static uint64_t gic_its_readb(void *opaque, hwaddr offset)
{
    return 0;
}

static uint64_t gic_its_readw(void *opaque, hwaddr offset)
{
    uint64_t val;
    val = gic_its_readb(opaque, offset);
    val |= gic_its_readb(opaque, offset + 1) << 8;
    return val;
}

static uint64_t gic_its_readl(void *opaque, hwaddr offset)
{
    uint64_t val;
    val = gic_its_readw(opaque, offset);
    val |= gic_its_readw(opaque, offset + 2) << 16;
    return val;
}

static uint64_t gic_its_readll(void *opaque, hwaddr offset)
{
    uint64_t val;
    val = gic_its_readl(opaque, offset);
    val |= gic_its_readl(opaque, offset + 4) << 32;
    return val;
}

static void gic_its_writeb(void *opaque, hwaddr offset,
                            uint64_t value)
{
    GICv3State *s = (GICv3State *)opaque;
    gicv3_update(s);
    return;
}

static void gic_its_writew(void *opaque, hwaddr offset,
                            uint64_t value)
{
    gic_its_writeb(opaque, offset, value & 0xff);
    gic_its_writeb(opaque, offset + 1, value >> 8);
}

static void gic_its_writel(void *opaque, hwaddr offset,
                            uint64_t value)
{
    gic_its_writel(opaque, offset, value & 0xffff);
    gic_its_writel(opaque, offset + 2, value >> 16);
}

static void gic_its_writell(void *opaque, hwaddr offset,
                            uint64_t value)
{
    gic_its_writell(opaque, offset, value & 0xffffffff);
    gic_its_writell(opaque, offset + 4, value >> 32);
}

static uint64_t gic_its_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t data;
    switch (size) {
    case 1:
        data = gic_its_readb(opaque, addr);
        break;
    case 2:
        data = gic_its_readw(opaque, addr);
        break;
    case 4:
        data = gic_its_readl(opaque, addr);
        break;
    case 8:
        data = gic_its_readll(opaque, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: size %u\n", __func__, size);
        assert(0);
        break;
    }
    DPRINTF("offset %p data %p\n", (void *) addr, (void *) data);
    return data;
}

static void gic_its_write(void *opaque, hwaddr addr, uint64_t data, unsigned size)
{
    DPRINTF("offset %p data %p\n", (void *) addr, (void *) data);
    switch (size) {
    case 1:
        gic_its_writew(opaque, addr, data);
        break;
    case 2:
        gic_its_writew(opaque, addr, data);
        break;
    case 4:
        gic_its_writel(opaque, addr, data);
        break;
    case 8:
        gic_its_writell(opaque, addr, data);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: size %u\n", __func__, size);
        assert(0);
        break;
    }
}

/* SPI routines are stubs for future development */
static uint64_t gic_spi_readb(void *opaque, hwaddr offset)
{
    return 0;
}

static uint64_t gic_spi_readw(void *opaque, hwaddr offset)
{
    uint64_t val;
    val = gic_spi_readb(opaque, offset);
    val |= gic_spi_readb(opaque, offset + 1) << 8;
    return val;
}

static uint64_t gic_spi_readl(void *opaque, hwaddr offset)
{
    uint64_t val;
    val = gic_spi_readw(opaque, offset);
    val |= gic_spi_readw(opaque, offset + 2) << 16;
    return val;
}

static uint64_t gic_spi_readll(void *opaque, hwaddr offset)
{
    uint64_t val;
    val = gic_spi_readl(opaque, offset);
    val |= gic_spi_readl(opaque, offset + 4) << 32;
    return val;
}

static void gic_spi_writeb(void *opaque, hwaddr offset,
                            uint64_t value)
{
    GICv3State *s = (GICv3State *)opaque;
    gicv3_update(s);
    return;
}

static void gic_spi_writew(void *opaque, hwaddr offset,
                            uint64_t value)
{
    gic_spi_writeb(opaque, offset, value & 0xff);
    gic_spi_writeb(opaque, offset + 1, value >> 8);
}

static void gic_spi_writel(void *opaque, hwaddr offset,
                            uint64_t value)
{
    gic_spi_writew(opaque, offset, value & 0xffff);
    gic_spi_writew(opaque, offset + 2, value >> 16);
}

static void gic_spi_writell(void *opaque, hwaddr offset,
                            uint64_t value)
{
    gic_spi_writel(opaque, offset, value & 0xffffffff);
    gic_spi_writel(opaque, offset + 4, value >> 32);
}

static uint64_t gic_spi_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t data;
    switch (size) {
    case 1:
        data = gic_spi_readb(opaque, addr);
        break;
    case 2:
        data = gic_spi_readw(opaque, addr);
        break;
    case 4:
        data = gic_spi_readl(opaque, addr);
        break;
    case 8:
        data = gic_spi_readll(opaque, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: size %u\n", __func__, size);
        assert(0);
        break;
    }
    DPRINTF("offset %p data %p\n", (void *) addr, (void *) data);
    return data;
}

static void gic_spi_write(void *opaque, hwaddr addr, uint64_t data, unsigned size)
{
    DPRINTF("offset %p data %p\n", (void *) addr, (void *) data);
    switch (size) {
    case 1:
        gic_spi_writeb(opaque, addr, data);
        break;
    case 2:
        gic_spi_writew(opaque, addr, data);
        break;
    case 4:
        gic_spi_writel(opaque, addr, data);
        break;
    case 8:
        gic_spi_writell(opaque, addr, data);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: size %u\n", __func__, size);
        assert(0);
        break;
    }
}

/* ITS control routines are stubs for future development */
static uint64_t gic_its_cntrl_readb(void *opaque, hwaddr offset)
{
    GICv3State *s = (GICv3State *)opaque;
    uint64_t res=0;

    if (offset < 0x100) {
          if (offset == 0)
            return 0;
          if (offset == 4)
              return 0;
          if (offset < 0x08)
            return s->num_cpu;
          if (offset >= 0x80) {
            return 0;
          }
          goto bad_reg;
      }
    return res;
bad_reg:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: Bad offset %x\n", __func__, (int)offset);
    return 0;
}

static uint64_t gic_its_cntrl_readw(void *opaque, hwaddr offset)
{
    uint64_t val;
    val = gic_its_cntrl_readb(opaque, offset);
    val |= gic_its_cntrl_readb(opaque, offset + 1) << 8;
    return val;
}

static uint64_t gic_its_cntrl_readl(void *opaque, hwaddr offset)
{
    uint64_t val;
    val = gic_its_cntrl_readw(opaque, offset);
    val |= gic_its_cntrl_readw(opaque, offset + 2) << 16;
    return val;
}

static uint64_t gic_its_cntrl_readll(void *opaque, hwaddr offset)
{
    uint64_t val;
    val = gic_its_cntrl_readl(opaque, offset);
    val |= gic_its_cntrl_readl(opaque, offset + 4) << 32;
    return val;
}

static void gic_its_cntrl_writeb(void *opaque, hwaddr offset,
                            uint64_t value)
{
    GICv3State *s = (GICv3State *)opaque;
    if (offset < 0x100) {
        if (offset < 0x08)
            s->num_cpu = value;
        else
            goto bad_reg;
    }
    gicv3_update(s);
    return;
bad_reg:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: Bad offset %x\n", __func__, (int)offset);
}

static void gic_its_cntrl_writew(void *opaque, hwaddr offset,
                            uint64_t value)
{
    gic_its_cntrl_writeb(opaque, offset, value & 0xff);
    gic_its_cntrl_writeb(opaque, offset + 1, value >> 8);
}

static void gic_its_cntrl_writel(void *opaque, hwaddr offset,
                            uint64_t value)
{
    gic_its_cntrl_writew(opaque, offset, value & 0xffff);
    gic_its_cntrl_writew(opaque, offset + 2, value >> 16);
}

static void gic_its_cntrl_writell(void *opaque, hwaddr offset,
                            uint64_t value)
{
    gic_its_cntrl_writel(opaque, offset, value & 0xffffffff);
    gic_its_cntrl_writel(opaque, offset + 4, value >> 32);
}

static uint64_t gic_its_cntrl_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t data;
    switch (size) {
    case 1:
        data = gic_its_cntrl_readb(opaque, addr);
        break;
    case 2:
        data = gic_its_cntrl_readw(opaque, addr);
        break;
    case 4:
        data = gic_its_cntrl_readl(opaque, addr);
        break;
    case 8:
        data = gic_its_cntrl_readll(opaque, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: size %u\n", __func__, size);
        assert(0);
        break;
    }
    DPRINTF("offset %p data %p\n", (void *) addr, (void *) data);
    return data;
}

static void gic_its_cntrl_write(void *opaque, hwaddr addr, uint64_t data, unsigned size)
{
    DPRINTF("offset %p data %p\n", (void *) addr, (void *) data);
    switch (size) {
    case 1:
        gic_its_cntrl_writeb(opaque, addr, data);
        break;
    case 2:
        gic_its_cntrl_writew(opaque, addr, data);
        break;
    case 4:
        gic_its_cntrl_writel(opaque, addr, data);
        break;
    case 8:
        gic_its_cntrl_writell(opaque, addr, data);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: size %u\n", __func__, size);
        assert(0);
        break;
    }
}

static uint64_t gic_redist_readb(void *opaque, hwaddr offset, MemTxAttrs attrs)
{
    GICv3State *s = (GICv3State *)opaque;
    uint64_t res = 0;
    uint64_t sgi_ppi, core, off;
    uint64_t cm, irq, i;

    /* GIC-500 Table 3-2 page 3-4 (Rev r0p0)
     * [          1<bits-for-core#>x<16-bits-offset>]
     * x = 0: LPIs
     * x = 1: SGIs & PPIs
     */
    off = offset;
    sgi_ppi = off & (1 << 16);
    core = (off >> 17) & s->cpu_mask;
    offset = off & 0xFFFF;
    cm = 1ll << core;

    if (sgi_ppi) {
        /* SGIs, PPIs */
        /* Interrupt Set/Clear Enable.  */
        if (offset < 0x100) {
            if (offset >= 0x80) {
                /* Interrupt Group Registers: these RAZ/WI if this is an NS
                 * access to a GIC with the security extensions, or if the GIC
                 * doesn't have groups at all.
                 */
                res = 0;
                if (!attrs.secure) {
                    /* GIC-500 comment 'a' */
                    goto bad_reg;
                }
                /* Every byte offset holds 8 group status bits */
                irq = (offset - 0x080) * 8 + GICV3_BASE_IRQ;
                if (irq >= s->num_irq) {
                    goto bad_reg;
                }
                if (irq >= GICV3_INTERNAL) {
                    DPRINTF("non Internal should be only in dist %lu\n", irq);
                    goto bad_reg;
                }
                for (i = 0; i < 8; i++) {
                    if (GIC_TEST_GROUP(irq + i, cm)) {
                        res |= (1 << i);
                    }
                }
                return res;
            }
        } else if (offset < 0x200) {
            /* Interrupt Set/Clear Enable.  */
            if (offset < 0x180)
                irq = (offset - 0x100) * 8;
            else
                irq = (offset - 0x180) * 8;
            irq += GICV3_BASE_IRQ;
            if (irq >= s->num_irq)
                goto bad_reg;
            if (irq >= GICV3_INTERNAL) {
                DPRINTF("non Internal should be only in dist %lu\n", irq);
                goto bad_reg;
            }
            res = 0;
            for (i = 0; i < 8; i++) {
                if (GIC_TEST_ENABLED(irq + i, cm)) {
                    res |= (1 << i);
                }
            }
        } else if (offset < 0x300) {
            /* Interrupt Set/Clear Pending.  */
            if (offset < 0x280)
                irq = (offset - 0x200) * 8;
            else
                irq = (offset - 0x280) * 8;
            irq += GICV3_BASE_IRQ;
            if (irq >= s->num_irq)
                goto bad_reg;
            if (irq >= GICV3_INTERNAL) {
                DPRINTF("non Internal should be only in dist %lu\n", irq);
                goto bad_reg;
            }
            res = 0;
            for (i = 0; i < 8; i++) {
                if (gic_test_pending(s, irq + i, cm)) {
                    res |= (1 << i);
                }
            }
        } else if (offset < 0x400) {
            /* Interrupt Set/Clear Active.  */
            if (offset < 0x380)
                irq = (offset - 0x300) * 8 + GICV3_BASE_IRQ;
            else
                irq = (offset - 0x380) * 8 + GICV3_BASE_IRQ;
            if (irq >= s->num_irq)
                goto bad_reg;
            if (irq >= GICV3_INTERNAL)
                goto bad_reg;
            res = 0;
            for (i = 0; i < 8; i++) {
                if (GIC_TEST_ACTIVE(irq + i, cm)) {
                    res |= (1 << i);
                }
            }
        } else if (offset < 0x800) {
            /* Interrupt Priority.  */
            irq = (offset - 0x400) + GICV3_BASE_IRQ;
            if (irq >= s->num_irq)
                goto bad_reg;
            if (irq >= GICV3_INTERNAL * 8) {
                DPRINTF("non Internal should be only in dist %lu\n", irq);
                goto bad_reg;
            }
            res = GIC_GET_PRIORITY(irq, core);
        } else if (offset < 0xc00) {
                goto bad_reg;
        } else if (offset < 0xd00) {
            /* Interrupt Configuration.  */
            irq = (offset - 0xc00) * 4 + GICV3_BASE_IRQ;
            if (irq >= s->num_irq)
                goto bad_reg;
            if (irq >= GICV3_INTERNAL) {
                DPRINTF("non Internal should be only in dist %lu\n", irq);
                goto bad_reg;
            }
            res = 0;
            for (i = 0; i < 4; i++) {
                /* Even bits are reserved */
                if (GIC_TEST_EDGE_TRIGGER(irq + i))
                    res |= (2 << (i * 2));
            }
        }
    } else {
        /* LPIs */
        if (offset & 3)
            return 0;
        if (offset < 0x100) {
            if (offset == 0) {/* GICR_CTLR */
                DPRINTF("Redist-GICR_CTLR-CPU caller cpu(%d) core(%lu)\n",
                    gic_get_current_cpu(s), core);
                return 0;
            }
            if (offset == 4)
                return 0x43B; /* GICR_IIDR */
            if (offset == 0x8) { /* GICR_TYPER */
                res = core << 8; /* Linear */
                /* Simple clustering */
                res |= (core % 8) << 32; /* Afinity 0 */
                res |= (core / 8) << 40; /* Afinity 1 */
                if (core == s->num_cpu - 1) {
                    /* Last redistributer */
                    res |= 1 << 4;
                }
                return res;
            }
            if (offset == 0xc) { /* GICR_TYPER */
                /* should write readll */
                return 0;
            }
            if (offset == 0x14) { /* GICR_WAKER */
                if (s->cpu_enabled[core])
                    return 0;
                else
                    return GICR_WAKER_ProcessorSleep;
                DPRINTF("Redist-CPU (%d) is enabled(%d)\n",
                        gic_get_current_cpu(s), s->cpu_enabled[core]);

            }
            if (offset >= 0x80 && offset < 0xFFD0)
                return 0;
            goto bad_reg;
        }
        if (offset < 0xffd0) {
            goto bad_reg;
        } else /* offset >= 0xffd0 */ {
            if (offset & 3) {
                res = 0;
            } else {
                res = gic_redist_ids[(offset - 0xffd0) >> 2];
            }
        }
    }
    return res;
bad_reg:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: Bad offset %x\n", __func__, (int)offset);
    return 0;
}

static MemTxResult gic_redist_read(void *opaque, hwaddr offset, uint64_t *data,
                                 unsigned size, MemTxAttrs attrs)
{
    switch (size) {
    case 1:
        *data = gic_redist_readb(opaque, offset, attrs);
        return MEMTX_OK;
    case 2:
        *data = gic_redist_readb(opaque, offset, attrs);
        *data |= gic_redist_readb(opaque, offset + 1, attrs) << 8;
        return MEMTX_OK;
    case 4:
        *data = gic_redist_readb(opaque, offset, attrs);
        *data |= gic_redist_readb(opaque, offset + 1, attrs) << 8;
        *data |= gic_redist_readb(opaque, offset + 2, attrs) << 16;
        *data |= gic_redist_readb(opaque, offset + 3, attrs) << 24;
        return MEMTX_OK;
    case 8:
        *data = gic_redist_readb(opaque, offset, attrs);
        *data |= gic_redist_readb(opaque, offset + 1, attrs) << 8;
        *data |= gic_redist_readb(opaque, offset + 2, attrs) << 16;
        *data |= gic_redist_readb(opaque, offset + 3, attrs) << 24;
        *data |= gic_redist_readb(opaque, offset + 4, attrs) << 32;
        *data |= gic_redist_readb(opaque, offset + 5, attrs) << 40;
        *data |= gic_redist_readb(opaque, offset + 6, attrs) << 48;
        *data |= gic_redist_readb(opaque, offset + 7, attrs) << 56;
        return MEMTX_OK;
    default:
        return MEMTX_ERROR;
    }
}

static void gic_redist_writeb(void *opaque, hwaddr offset,
                            uint64_t value, MemTxAttrs attrs)
{
    GICv3State *s = (GICv3State *)opaque;
    uint64_t sgi_ppi, core, off;
    uint64_t cm;
    int irq, i;
    /* GIC-500 Table 3-2 page 3-4 (Rev r0p0)
     * [          1<bits-for-core#>x<16-bits-offset>]
     * x = 0: LPIs
     * x = 1: SGIs & PPIs
     */
    off = offset;
    sgi_ppi = off & (1 << 16);
    core = (off >> 17) & s->cpu_mask;
    offset = off & 0xFFFF;
    cm = 1ll << core;

    if (sgi_ppi) {
        /* SGIs, PPIs */
        if (offset < 0x100) {
            if (offset >= 0x80) {
                /* Interrupt Group Registers: RAZ/WI for NS access to secure
                 * GIC, or for GICs without groups.
                 */
                if (!attrs.secure) {
                    /* GIC-500 comment 'f' */
                    goto bad_reg;
                }
                /* Every byte offset holds 8 group status bits */
                irq = (offset - 0x80) * 8 + GICV3_BASE_IRQ;
                if (irq >= s->num_irq) {
                    goto bad_reg;
                }
                if (irq >= GICV3_INTERNAL) {
                    DPRINTF("IGROUPR0 non Internal should be only in distributer %d\n", irq);
                    return;
                }
                for (i = 0; i < 8; i++) {
                    /* Group bits are banked for private interrupts */
                    if (value & (1 << i)) {
                        /* Group1 (Non-secure) */
                        GIC_SET_GROUP(irq + i, cm);
                    } else {
                        /* Group0 (Secure) */
                        GIC_CLEAR_GROUP(irq + i, cm);
                    }
                }
            }
        } else if (offset < 0x180) {
            /* Interrupt Set Enable.  */
            irq = (offset - 0x100) * 8 + GICV3_BASE_IRQ;
            if (irq >= s->num_irq)
                goto bad_reg;
            if (irq >= GICV3_INTERNAL) {
                DPRINTF("ISENABLERn non Internal should be only in distributer %d\n", irq);
                /* The registers after 0x100 are reserved */
                return;
            }
            if (irq < GICV3_NR_SGIS) {
                value = 0xff;
            }

            for (i = 0; i < 8; i++) {
                if (value & (1 << i)) {
                    /* This is redistributer ALL doesn't apply */
                    if (!GIC_TEST_ENABLED(irq + i, cm)) {
                        DPRINTF("Enabled IRQ %d\n", irq + i);
                    }
                    GIC_SET_ENABLED(irq + i, cm);
                    /* If a raised level triggered IRQ enabled then mark
                       is as pending.  */
                    if (GIC_TEST_LEVEL(irq + i, cm)
                            && !GIC_TEST_EDGE_TRIGGER(irq + i)) {
                        DPRINTF("Set %d pending mask %lx\n", irq + i, cm);
                        GIC_SET_PENDING(irq + i, cm);
                    }
                }
            }
        } else if (offset < 0x200) {
            /* Interrupt Clear Enable.  */
            irq = (offset - 0x180) * 8 + GICV3_BASE_IRQ;
            if (irq >= s->num_irq)
                goto bad_reg;
            if (irq >= GICV3_INTERNAL) {
                DPRINTF("ICENABLERn non Internal should be only in distributer %d\n", irq);
                /* The registers after 0x180 are reserved */
                return;
            }
            if (irq < GICV3_NR_SGIS) {
                value = 0;
            }

            for (i = 0; i < 8; i++) {
                if (value & (1 << i)) {
                    /* This is redistributer ALL doesn't apply */
                    if (GIC_TEST_ENABLED(irq + i, cm)) {
                        DPRINTF("Disabled IRQ %d\n", irq + i);
                    }
                    GIC_CLEAR_ENABLED(irq + i, cm);
                }
            }
            if (irq >= GICV3_INTERNAL) {
                DPRINTF("non Internal should be only in distributer %d\n", irq);
                goto bad_reg;
            }
        } else if (offset < 0x280) {
            /* Interrupt Set Pending.  */
            irq = (offset - 0x200) * 8 + GICV3_BASE_IRQ;
            if (irq >= s->num_irq)
                goto bad_reg;
            if (irq >= GICV3_INTERNAL) {
                DPRINTF("non Internal should be only in distributer %d\n", irq);
                goto bad_reg;
            }
            for (i = 0; i < 8; i++) {
                if (value & (1 << i)) {
                    GIC_SET_PENDING(irq + i, GIC_TARGET(irq + i));
                }
            }
        } else if (offset < 0x300) {
            /* Interrupt Clear Pending.  */
            irq = (offset - 0x280) * 8 + GICV3_BASE_IRQ;
            if (irq >= s->num_irq)
                goto bad_reg;
            if (irq >= GICV3_INTERNAL) {
                DPRINTF("non Internal should be only in distributer %d\n", irq);
                goto bad_reg;
            }
            for (i = 0; i < 8; i++) {
                if (value & (1 << i)) {
                    GIC_CLEAR_PENDING(irq + i, cm);
                }
            }
        } else if (offset < 0x400) {
            /* Interrupt Active.  */
            goto bad_reg;
        } else if (offset < 0x800) {
            /* Interrupt Priority. */
            irq = (offset - 0x400) + GICV3_BASE_IRQ;
            if (irq >= s->num_irq)
                goto bad_reg;
            if (irq >= GICV3_INTERNAL * 8) {
                DPRINTF("IPRIORITYRn non Internal should be only in distributer %d\n", irq);
                /* The registers after 0x180 are reserved */
                return;
            }
            gicv3_set_priority(s, core, irq, value, attrs);
        } else if (offset < 0xc00) {
            goto bad_reg;
        } else if (offset < 0xd00) {
            /* Interrupt Configuration.  */
            irq = (offset - 0xc00) * 4 + GICV3_BASE_IRQ;
            if (irq >= s->num_irq)
                goto bad_reg;
            if (irq >= GICV3_INTERNAL) {
                DPRINTF("non Internal should be only in distributer %d\n", irq);
                goto bad_reg;
            }
            if (irq < GICV3_NR_SGIS)
                value |= 0xaa; /* 10101010 */
            /* Even bits are reserved */
            for (i = 0; i < 4; i++) {
                if (value & (2 << (i * 2))) {
                    GIC_SET_EDGE_TRIGGER(irq + i);
                } else {
                    GIC_CLEAR_EDGE_TRIGGER(irq + i);
                }
            }
        }
    } else {
        /* LPIs */
        if (offset == 0x14) { /* GICR_WAKER */
            if (value & GICR_WAKER_ProcessorSleep)
                s->cpu_enabled[core] = 0;
            else
                s->cpu_enabled[core] = 1;
            DPRINTF("Redist-CPU (%d) core(%lu) set enabled(%d)\n",
                    gic_get_current_cpu(s), core, s->cpu_enabled[core]);
       }
    }
    gicv3_update(s);
    return;

bad_reg:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: Bad offset %x\n", __func__, (int)offset);
}

static void gic_redist_writew(void *opaque, hwaddr offset,
                            uint64_t value, MemTxAttrs attrs)
{
    gic_redist_writeb(opaque, offset, value & 0xff, attrs);
    gic_redist_writeb(opaque, offset + 1, value >> 8, attrs);
}

static void gic_redist_writel(void *opaque, hwaddr offset,
                            uint64_t value, MemTxAttrs attrs)
{
    gic_redist_writew(opaque, offset, value & 0xffff, attrs);
    gic_redist_writew(opaque, offset + 2, value >> 16, attrs);
}

static void gic_redist_writell(void *opaque, hwaddr offset,
                            uint64_t value, MemTxAttrs attrs)
{
    gic_redist_writel(opaque, offset, value & 0xffffffff, attrs);
    gic_redist_writel(opaque, offset + 4, value >> 32, attrs);
}

static MemTxResult gic_redist_write(void *opaque, hwaddr addr, uint64_t data,
                                 unsigned size, MemTxAttrs attrs)
{
    switch (size) {
    case 1:
        gic_redist_writeb(opaque, addr, data, attrs);
        return MEMTX_OK;
    case 2:
        gic_redist_writew(opaque, addr, data, attrs);
        return MEMTX_OK;
    case 4:
        gic_redist_writel(opaque, addr, data, attrs);
        return MEMTX_OK;
    case 8:
        gic_redist_writell(opaque, addr, data, attrs);
        return MEMTX_OK;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: size %u\n", __func__, size);
        return MEMTX_ERROR;
    }
}

static const MemoryRegionOps gic_ops[] = {
    {
        .read_with_attrs = gic_dist_read,
        .write_with_attrs = gic_dist_write,
        .impl = {
             .min_access_size = 4,
             .max_access_size = 8,
         },
        .endianness = DEVICE_NATIVE_ENDIAN,
    },
    {
        .read_with_attrs = gic_redist_read,
        .write_with_attrs = gic_redist_write,
        .impl = {
             .min_access_size = 4,
             .max_access_size = 8,
         },
        .endianness = DEVICE_NATIVE_ENDIAN,
    },
    {
        .read = gic_its_read,
        .write = gic_its_write,
        .impl = {
             .min_access_size = 4,
             .max_access_size = 8,
         },
        .endianness = DEVICE_NATIVE_ENDIAN,
    },
    {
        .read = gic_spi_read,
        .write = gic_spi_write,
        .impl = {
             .min_access_size = 4,
             .max_access_size = 8,
         },
        .endianness = DEVICE_NATIVE_ENDIAN,
    },
    {
        .read = gic_its_cntrl_read,
        .write = gic_its_cntrl_write,
        .impl = {
             .min_access_size = 4,
             .max_access_size = 8,
         },
        .endianness = DEVICE_NATIVE_ENDIAN,
    }
};

static void arm_gic_realize(DeviceState *dev, Error **errp)
{
    /* Device instance realize function for the GIC sysbus device */
    int i;
    GICv3State *s = ARM_GICV3(dev);
    ARMGICv3Class *agc = ARM_GICV3_GET_CLASS(s);
    Error *local_err = NULL;
    uint32_t power2;

    agc->parent_realize(dev, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    /* Cuurently no GICv2 backwards compatibility (e.g. no memory mapped regs)
     * Uses system registers mode
     */
    gic_sre = 1;

    /* Tell the common code we're a GICv3 */
    s->revision = REV_V3;

    gicv3_init_irqs_and_mmio(s, gic_set_irq, gic_ops);

    /* Compute mask for decoding the core number in redistributer */
    if (is_power_of_2(NUM_CPU(s)))
        power2 = NUM_CPU(s);
    else
        /* QEMU has only  pow2floor !!! */
        power2 = pow2floor(2 * NUM_CPU(s));
    s->cpu_mask = (power2 - 1);

    DPRINTF(" -- NUM_CPUS(%d) - cpu mask(0%x) -- \n", NUM_CPU(s), s->cpu_mask);

    for (i = 0; i < GICV3_NCPU; i++) {
        s->cpu_enabled[i] = 0;
    }
}

void armv8_gicv3_set_sgi(void *opaque, int cpuindex, uint64_t value)
{
    GICv3State *s = (GICv3State *) opaque;
    uint64_t cm = (1ll << cpuindex);
    uint32_t target;
    int irq, i;

    /* Page 2227 ICC_SGI1R_EL1 */

    irq = (value >> 24) & 0xf;

    /* The external routines use the hardware vector numbering, ie. the first
     * IRQ is #16.  The internal GIC routines use #32 as the first IRQ.
     */
    if (irq >= 16)
        irq += 16;

    /* IRM bit */
    if (value & (1ll << 40)) {
        /* Send to all the cores exclude self */
        for (i = 0; i < cpuindex; i++) {
            s->sgi_state[irq].pending[i] |= cm;
        }
        for (i = cpuindex + 1; i < s->num_cpu; i++) {
            s->sgi_state[irq].pending[i] |= cm;
        }
        GIC_SET_PENDING(irq, (ALL_CPU_MASK & ~cm));
        DPRINTF("cpu(%d) sends irq(%d) to ALL exclude self\n", cpuindex, irq);
    } else {
        /* Find linear of first core in cluster. See page 2227 ICC_SGI1R_EL1
         * With our GIC-500 implementation we can have 16 clusters of 8 cpu each
         */
#if 1
        target = (value & (0xfl << 16)) >> (16 - 3); /* shift 16 mult by 8 */
#else
        /* Prep for more advanced GIC */
        target  = (value & (0xffl << 16)) >> (16 - 8);
        target |= (value & (0xffl << 32)) >> (32 - 16);
        target |= (value & (0xffl << 48)) >> (48 - 24);
#endif

        /* Use 8 and not 16 since only 8 cores can be in a cluster of GIC-500 */
        assert((value & 0xff00) == 0);
        for (i = 0; i < 8; i++) {
            if (value & (1 << i)) {
                s->sgi_state[irq].pending[target + i] |= cm;
                GIC_SET_PENDING(irq, (1ll << (target + i)));
             }
         }
    }
    gicv3_update(s);
}

uint64_t armv8_gicv3_acknowledge_irq(void *opaque, int cpuindex,
                                     MemTxAttrs attrs)
{
    GICv3State *s = (GICv3State *) opaque;
    return gicv3_acknowledge_irq(s, cpuindex, attrs);
}

void armv8_gicv3_complete_irq(void *opaque, int cpuindex, int irq,
                              MemTxAttrs attrs)
{
    GICv3State *s = (GICv3State *) opaque;
    irq &= 0xffffff;
    gicv3_complete_irq(s, cpuindex, irq, attrs);
}

uint64_t armv8_gicv3_get_priority_mask(void *opaque, int cpuindex)
{
    GICv3State *s = (GICv3State *) opaque;
    return s->priority_mask[cpuindex];
}

void armv8_gicv3_set_priority_mask(void *opaque, int cpuindex, uint32_t mask)
{
    GICv3State *s = (GICv3State *) opaque;
    s->priority_mask[cpuindex] = mask & 0xff;
    DPRINTF("%s cpu(%d) priority mask 0x%x\n",
            __func__, cpuindex, s->priority_mask[cpuindex]);
    gicv3_update(s);
}

uint64_t armv8_gicv3_get_sre(void *opaque)
{
    /* Uses only system registers, no memory mapped access GICv2 mode */
    return gic_sre;
}

void armv8_gicv3_set_sre(void *opaque, uint64_t sre)
{
    if (!(sre & 1) && no_gicv2_bc) {
        /* Cuurently no GICv2 backwards compatibility (no memory mapped regs)
         * Uses system registers mode
         */
        DPRINTF("Try to use memory mapped interface sre(0x%lx)\n", sre);
        assert(0);
    }
    gic_sre = sre;
}

uint64_t armv8_gicv3_get_igrpen1(void *opaque, int cpuindex)
{
    GICv3State *s = (GICv3State *) opaque;
    return !!(s->cpu_ctlr[cpuindex] & GICC_CTLR_EN_GRP1);
}

void armv8_gicv3_set_igrpen1(void *opaque, int cpuindex, uint64_t igrpen1)
{
    GICv3State *s = (GICv3State *) opaque;
    if (igrpen1)
        s->cpu_ctlr[cpuindex] |= GICC_CTLR_EN_GRP1;
    else
        s->cpu_ctlr[cpuindex] &= ~GICC_CTLR_EN_GRP1;

    DPRINTF("CPU Interface %d: Group0 Interrupts %sabled, "
            "Group1 Interrupts %sabled\n", cpuindex,
            (s->cpu_ctlr[cpuindex] & GICC_CTLR_EN_GRP0) ? "En" : "Dis",
            (s->cpu_ctlr[cpuindex] & GICC_CTLR_EN_GRP1) ? "En" : "Dis");
}

static void arm_gicv3_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ARMGICv3Class *agc = ARM_GICV3_CLASS(klass);

    agc->parent_realize = dc->realize;
    dc->realize = arm_gic_realize;
}

static const TypeInfo arm_gicv3_info = {
    .name = TYPE_ARM_GICV3,
    .parent = TYPE_ARM_GICV3_COMMON,
    .instance_size = sizeof(GICv3State),
    .class_init = arm_gicv3_class_init,
    .class_size = sizeof(ARMGICv3Class),
};

static void arm_gicv3_register_types(void)
{
    type_register_static(&arm_gicv3_info);
}

type_init(arm_gicv3_register_types)
