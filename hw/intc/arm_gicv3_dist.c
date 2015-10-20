#include "gicv3_internal.h"
#include "qom/cpu.h"
#include "arm_gicv3_dist.h"
#include "arm_gicv3_interrupts.h"

static const uint8_t gic_dist_ids[] = {
    0x44, 0x00, 0x00, 0x00, 0x092, 0xB4, 0x3B, 0x00, 0x0D, 0xF0, 0x05, 0xB1
};

static uint64_t gic_dist_readb(void *opaque, hwaddr offset, MemTxAttrs attrs)
{
    GICv3State *s = (GICv3State *)opaque;
    uint64_t res;
    int irq;
    int i;
    int cpu;
    int cm;

    cpu = gic_get_current_cpu(s);
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
                if (GIC_TEST_GROUP(irq + i, cpu)) {
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
            if (GIC_TEST_ENABLED(irq + i, cpu)) {
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
        if (irq < GICV3_INTERNAL && gicv3_no_gicv2_bc)
            goto bad_reg;
        res = 0;
        cm = (irq < GICV3_INTERNAL) ?  cpu : ALL_CPU_MASK;
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
        /* Comment 'o' GICv2 backwards competability support is not set...*/
        if (irq < GICV3_INTERNAL && gicv3_no_gicv2_bc)
            goto bad_reg;
        res = 0;
        cm = (irq < GICV3_INTERNAL) ?  cpu : ALL_CPU_MASK;
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
        /* Comment 'p' GICv2 backwards competability support is not set...*/
        if (irq < GICV3_INTERNAL * 8 && gicv3_no_gicv2_bc)
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
            if (irq < GICV3_INTERNAL * 8 && gicv3_no_gicv2_bc)
                goto bad_reg;
            if (irq >= 29 && irq <= 31) {
                res = 1 << cpu;
            } else {
                /* Value is a small bitmap can do it directly */
                res = *GIC_TARGET(irq);
            }
        }
    } else if (offset < 0xd00) {
        /* Interrupt Configuration.  */
        irq = (offset - 0xc00) * 4 + GICV3_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        /* Comment 'l' GICv2 backwards competability support is not set...*/
        if (irq < GICV3_INTERNAL && gicv3_no_gicv2_bc)
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
        if (gicv3_no_gicv2_bc)
            goto bad_reg;
        /* These are 32 bit registers, should not be used with 128 cores. */
        if (offset < 0xf20) {
            /* GICD_CPENDSGIRn */
            irq = (offset - 0xf10);
        } else {
            irq = (offset - 0xf20);
            /* GICD_SPENDSGIRn */
        }
        /* Small bitmap */
        res = *s->sgi[irq].state[cpu].pending;
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
        cpu = find_first_bit(s->irq_state[irq].target, s->num_cpu);
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

MemTxResult gic_dist_read(void *opaque, hwaddr offset, uint64_t *data,
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
                int cm = (irq < GICV3_INTERNAL) ? cpu : ALL_CPU_MASK;
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
                int cm = (irq < GICV3_INTERNAL) ? cpu : ALL_CPU_MASK;

                if (!GIC_TEST_ENABLED(irq + i, cm)) {
                    DPRINTF("Enabled IRQ %d\n", irq + i);
                }
                GIC_SET_ENABLED(irq + i, cm);
                /* If a raised level triggered IRQ enabled then mark
                   is as pending.  */
                if (irq < GICV3_INTERNAL) {
                    if (GIC_TEST_LEVEL(irq + i, cpu)
                            && !GIC_TEST_EDGE_TRIGGER(irq + i)) {
                        DPRINTF("Set %d pending cpu %d\n", irq + i, cpu);
                        GIC_SET_PENDING(irq + i, cpu);
                    }
                } else {
                    unsigned long *mask = GIC_TARGET(irq + i);
                    if (GIC_TEST_LEVEL_MASK(irq + i, mask)
                            && !GIC_TEST_EDGE_TRIGGER(irq + i)) {
                        DPRINTF("Set %d pending target\n", irq + i);
                        GIC_SET_PENDING_MASK(irq + i, mask);
                    }
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
                int cm = (irq < GICV3_INTERNAL) ? cpu : ALL_CPU_MASK;

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
        if (irq < GICV3_INTERNAL && gicv3_no_gicv2_bc)
            goto bad_reg;

        for (i = 0; i < 8; i++) {
            if (value & (1 << i)) {
                GIC_SET_PENDING_MASK(irq + i, GIC_TARGET(irq + i));
            }
        }
    } else if (offset < 0x300) {
        /* Interrupt Clear Pending.  */
        irq = (offset - 0x280) * 8 + GICV3_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        /* Comment 'o' GICv2 backwards competability support is not set...*/
        if (irq < GICV3_INTERNAL && gicv3_no_gicv2_bc)
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
        if (irq < GICV3_INTERNAL * 8 && gicv3_no_gicv2_bc)
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
            if (irq < GICV3_INTERNAL * 8 && gicv3_no_gicv2_bc)
                goto bad_reg;
            if (irq < 29) {
                value = 0;
            } else if (irq < GICV3_INTERNAL) {
                value = ALL_CPU_MASK_COMPAT;
            }
            /* Small bitmap */
            *s->irq_state[irq].target = value & ALL_CPU_MASK_COMPAT;
        }
    } else if (offset < 0xd00) {
        /* Interrupt Configuration.  */
        irq = (offset - 0xc00) * 4 + GICV3_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        /* Comment 'l' GICv2 backwards competability support is not set...*/
        if (irq < GICV3_INTERNAL && gicv3_no_gicv2_bc)
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
        if (gicv3_no_gicv2_bc)
            goto bad_reg;
        /* These are 32 bit registers, should not be used with 128 cores. */
        if (offset < 0xf20) {
            /* GICD_CPENDSGIRn */
            irq = (offset - 0xf10);
            DPRINTF("GICD_CPENDSGIRn irq(%d) %lu\n", irq, value);
            /* Value is a small bitmap can do it directly */
            *s->sgi[irq].state[cpu].pending &= ~value;
            if (*s->sgi[irq].state[cpu].pending == 0) {
                GIC_CLEAR_PENDING(irq, cpu);
            }
        } else {
            irq = (offset - 0xf20);
            /* GICD_SPENDSGIRn */
            DPRINTF("GICD_SPENDSGIRn irq(%d) %lu\n", irq, value);
            GIC_SET_PENDING(irq, cpu);
            /* Value is a small bitmap can do it directly */
            *s->sgi[irq].state[cpu].pending |= value;
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
         * support is not included. (comment t page 3-8 on GIC-500 document)
         * Or table 6 on GICv3 documment
         */
        int cpu;
        int irq;
        uint32_t mask, cpu_mask;
        int target_cpu;
        GICv3State *s = (GICv3State *)opaque;

        if (gicv3_no_gicv2_bc) {
            DPRINTF("GICv2 backwards computability is not supported\n");
            return;
        }
        /* GICv2 competabiltu code only 8 CPU */
        cpu = gic_get_current_cpu(s);
        irq = value & 0x3ff;
        switch ((value >> 24) & 3) {
        case 0:
            mask = (value >> 16) & ALL_CPU_MASK_COMPAT;
            break;
        case 1:
            /* All cpus excpet this one (up to 7) */
            mask = ALL_CPU_MASK_COMPAT ^ (1ll << cpu);
            break;
        case 2:
            mask = 1ll << cpu;
            break;
        default:
            DPRINTF("Bad Soft Int target filter\n");
            /* All CPUs */
            mask = 0xff;
            break;
        }
        cpu_mask = (1ll << cpu);
        DPRINTF("irq(%d) mask(0x%x)\n", irq, mask);
        target_cpu = ctz32(mask);
        while (target_cpu < s->num_cpu) {
            GIC_SET_PENDING(irq, target_cpu);
            /* Small mask, can do direct assignment */
            *s->sgi[irq].state[target_cpu].pending |= cpu_mask;
            mask &= ~(1ll << target_cpu);
            target_cpu = ctz32(mask);
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
        GIC_SET_TARGET(irq, cpu);
        gicv3_update(s);
        DPRINTF("irq(%d) cpu(%d)\n", irq, cpu);
        return;
    }

    gic_dist_writel(opaque, offset, value & 0xffffffff, attrs);
    gic_dist_writel(opaque, offset + 4, value >> 32, attrs);
}

MemTxResult gic_dist_write(void *opaque, hwaddr addr, uint64_t data,
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
