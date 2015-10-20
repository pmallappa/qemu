#include "gicv3_internal.h"
#include "qom/cpu.h"
#include "arm_gicv3_cpu_interface.h"
#include "arm_gicv3_interrupts.h"

/* Cuurently no GICv2 backwards compatibility (no memory mapped regs)
 * Uses system registers mode.
 */
const int gicv3_no_gicv2_bc = 1;
uint32_t gicv3_sre;

void armv8_gicv3_set_sgi(void *opaque, int cpuindex, uint64_t value)
{
    GICv3State *s = (GICv3State *) opaque;
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
            set_bit(cpuindex, s->sgi[irq].state[i].pending);
        }
        for (i = cpuindex + 1; i < s->num_cpu; i++) {
            set_bit(cpuindex, s->sgi[irq].state[i].pending);
        }
        /* GIC_SET_PENDING(irq, (ALL_CPU_MASK & ~cm)); */
        bitmap_fill(s->irq_state[irq].pending, s->num_cpu);
        clear_bit(cpuindex, s->irq_state[irq].pending);
        DPRINTF("cpu(%d) sends irq(%d) to ALL exclude self\n", cpuindex, irq);
    } else {
        /* Find linear of first core in cluster. See page 2227 ICC_SGI1R_EL1
         * With our GIC-500 implementation we can have 16 clusters of 8 cpu each
         */
        uint64_t target_affinity;
        uint64_t target_list;
        target_affinity  = (value >> (16 - ARM_AFF1_SHIFT)) & ARM_AFF1_MASK;
        target_affinity |= (value >> (32 - ARM_AFF2_SHIFT)) & ARM_AFF2_MASK;
        target_affinity |= (value >> (48 - ARM_AFF3_SHIFT)) & ARM_AFF3_MASK;
        target_list = value & 0xff;

        for (i = 0; i < s->num_cpu; i++) {
            uint64_t cpu_aff0   = s->mp_affinity[i] & ARM_AFF0_MASK;
            uint64_t cpu_aff123 = s->mp_affinity[i] & ~ARM_AFF0_MASK;
            if (cpu_aff123 == target_affinity &&
                ((1 << cpu_aff0) & target_list)) {
                set_bit(cpuindex, s->sgi[irq].state[i].pending);
                GIC_SET_PENDING(irq, i);
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
    return gicv3_sre;
}

void armv8_gicv3_set_sre(void *opaque, uint64_t sre)
{
    if (!(sre & 1) && gicv3_no_gicv2_bc) {
        /* Cuurently no GICv2 backwards compatibility (no memory mapped regs)
         * Uses system registers mode
         */
        DPRINTF("Try to use memory mapped interface sre(0x%lx)\n", sre);
        assert(0);
    }
    gicv3_sre = sre;
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
