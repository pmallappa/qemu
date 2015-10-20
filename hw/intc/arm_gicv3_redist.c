#include "gicv3_internal.h"
#include "qom/cpu.h"
#include "arm_gicv3_redist.h"
#include "arm_gicv3_interrupts.h"

static const uint8_t gic_redist_ids[] = {
    0x44, 0x00, 0x00, 0x00, 0x093, 0xB4, 0x3B, 0x00, 0x0D, 0xF0, 0x05, 0xB1
};

static uint64_t gic_redist_readb(void *opaque, hwaddr offset, MemTxAttrs attrs)
{
    GICv3State *s = (GICv3State *)opaque;
    uint64_t res = 0;
    uint64_t sgi_ppi, core, off;
    uint64_t irq, i;

    /* GIC-500 Table 3-2 page 3-4 (Rev r0p0)
     * [          1<bits-for-core#>x<16-bits-offset>]
     * x = 0: LPIs
     * x = 1: SGIs & PPIs
     */
    off = offset;
    sgi_ppi = off & (1 << 16);
    core = (off >> 17) & s->cpu_mask;
    offset = off & 0xFFFF;

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
                    if (GIC_TEST_GROUP(irq + i, core)) {
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
                if (GIC_TEST_ENABLED(irq + i, core)) {
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
                if (gic_test_pending(s, irq + i, core)) {
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
                if (GIC_TEST_ACTIVE(irq + i, core)) {
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
                uint64_t mp_affinity = s->mp_affinity[core];
                res = core << 8; /* Linear cpu number */
                /* See GICv3 section 5.4.8 */
                res |= (mp_affinity & ARM_AFF0_MASK) << (32 - ARM_AFF0_SHIFT);
                res |= (mp_affinity & ARM_AFF1_MASK) << (40 - ARM_AFF1_SHIFT);
                res |= (mp_affinity & ARM_AFF2_MASK) << (48 - ARM_AFF2_SHIFT);
                res |= (mp_affinity & ARM_AFF3_MASK) << (56 - ARM_AFF3_SHIFT);
                if (core == s->num_cpu - 1) {
                    /* Mark last redistributer */
                    res |= 1 << 4;
                }
                return res;
            }
            if (offset == 0xc) { /* GICR_TYPER */
                /* should write readll */
                return 0;
            }
            if (offset == 0x14) { /* GICR_WAKER */
                if (test_bit(core, s->cpu_enabled))
                    return 0;
                else
                    return GICR_WAKER_ProcessorSleep;
                DPRINTF("Redist-CPU (%d) is enabled(%lu)\n",
                        gic_get_current_cpu(s), test_bit(core, s->cpu_enabled));

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

MemTxResult gic_redist_read(void *opaque, hwaddr offset, uint64_t *data,
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
                        GIC_SET_GROUP(irq + i, core);
                    } else {
                        /* Group0 (Secure) */
                        GIC_CLEAR_GROUP(irq + i, core);
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
                    if (!GIC_TEST_ENABLED(irq + i, core)) {
                        DPRINTF("Enabled IRQ %d\n", irq + i);
                    }
                    GIC_SET_ENABLED(irq + i, core);
                    /* If a raised level triggered IRQ enabled then mark
                       is as pending.  */
                    if (GIC_TEST_LEVEL(irq + i, core)
                            && !GIC_TEST_EDGE_TRIGGER(irq + i)) {
                        DPRINTF("Set %d pending core %lx\n", irq + i, core);
                        GIC_SET_PENDING(irq + i, core);
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
                    if (GIC_TEST_ENABLED(irq + i, core)) {
                        DPRINTF("Disabled IRQ %d\n", irq + i);
                    }
                    GIC_CLEAR_ENABLED(irq + i, core);
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
                    GIC_SET_PENDING_MASK(irq + i, GIC_TARGET(irq + i));
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
                    GIC_CLEAR_PENDING(irq + i, core);
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
                clear_bit(core, s->cpu_enabled);
            else
                set_bit(core, s->cpu_enabled);
            DPRINTF("Redist-CPU (%d) core(%lu) set enabled(%d)\n",
                    gic_get_current_cpu(s), core, test_bit(core, s->cpu_enabled));
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

MemTxResult gic_redist_write(void *opaque, hwaddr addr, uint64_t data,
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
