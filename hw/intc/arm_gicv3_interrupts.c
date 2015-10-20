#include "gicv3_internal.h"
#include "qom/cpu.h"
#include "arm_gicv3_interrupts.h"

/* TODO: Many places that call this routine could be optimized.  */
/* Update interrupt status after enabled or pending bits have been changed.  */
void gicv3_update(GICv3State *s)
{
    int best_irq;
    int best_prio;
    int irq;
    int irq_level, fiq_level;
    int cpu;

    for (cpu = 0; cpu < NUM_CPU(s); cpu++) {
        s->current_pending[cpu] = 1023;
        if (!(s->ctlr & (GICD_CTLR_EN_GRP0 | GICD_CTLR_EN_GRP1_ALL))
            || !test_bit(cpu, s->cpu_enabled)
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
            if (GIC_TEST_ENABLED(irq, cpu) && gic_test_pending(s, irq, cpu) &&
                (irq < GICV3_INTERNAL || GIC_TEST_TARGET(irq, cpu))) {
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
                int group = GIC_TEST_GROUP(best_irq, cpu);
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
                                  int cm, unsigned long *target)
{
    if (level) {
        /* if cm is -1 then the macro will set them all */
        GIC_SET_LEVEL(irq, cm);
        DPRINTF("Set %d pending cpu %d\n", irq, cm);
        if (GIC_TEST_EDGE_TRIGGER(irq)) {
            if (cm < 0)
                GIC_SET_PENDING_MASK(irq, target);
            else
                GIC_SET_PENDING(irq, cm);
        }
    } else {
        GIC_CLEAR_LEVEL(irq, cm);
    }
}

/* Process a change in an external IRQ input.  */
void gicv3_set_irq(void *opaque, int irq, int level)
{
    /* Meaning of the 'irq' parameter:
     *  [0..N-1] : external interrupts
     *  [N..N+31] : PPI (internal) interrupts for CPU 0
     *  [N+32..N+63] : PPI (internal interrupts for CPU 1
     *  ...
     */
    GICv3State *s = (GICv3State *)opaque;
    int cm;
    unsigned long *target;

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
        cm = cpu;
        target = NULL;
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
            int group = GIC_TEST_GROUP(pending_irq, cpu);
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

uint32_t gicv3_acknowledge_irq(GICv3State *s, int cpu, MemTxAttrs attrs)
{
    int ret, irq, src;

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

    s->irq_state[irq].last_active[cpu] = s->running_irq[cpu];

    if (irq < GICV3_NR_SGIS) {
        /* Lookup the source CPU for the SGI and clear this in the
         * sgi_pending map.  Return the src and clear the overall pending
         * state on this CPU if the SGI is not pending from any CPUs.
         */
        assert(s->sgi[irq].state[cpu].pending != 0);
        src = find_first_bit(s->sgi[irq].state[cpu].pending, s->num_cpu);
        if (src < s->num_cpu)
            clear_bit(src, s->sgi[irq].state[cpu].pending);
        if (bitmap_empty(s->sgi[irq].state[cpu].pending, s->num_cpu)) {
            GIC_CLEAR_PENDING(irq, cpu);
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
        GIC_CLEAR_PENDING(irq, cpu);
        ret = irq;
    }

    gic_set_running_irq(s, cpu, irq);
    DPRINTF("out ACK irq-ret(%d) cpu(%d) \n", ret, cpu);
    return ret;
}

void gicv3_set_priority(GICv3State *s, int cpu, int irq, uint8_t val,
                        MemTxAttrs attrs)
{
    DPRINTF("%s cpu(%d) secure(%d)\n", __func__, cpu, attrs.secure);
    if (s->security_levels == 1 && !attrs.secure) {
        if (!GIC_TEST_GROUP(irq, cpu)) {
            return; /* Ignore Non-secure access of Group0 IRQ */
        }
        val = 0x80 | (val >> 1); /* Non-secure view */
    }

    if (irq < GICV3_INTERNAL) {
        s->priority1[irq].p[cpu] = val;
    } else {
        s->priority2[irq - GICV3_INTERNAL] = val;
    }
}

void gicv3_complete_irq(GICv3State *s, int cpu, int irq, MemTxAttrs attrs)
{
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

    if (s->running_irq[cpu] == 1023) {
        DPRINTF("No active IRQ ignored cpu(%d) irq(%d)\n", irq, cpu);
        return; /* No active IRQ.  */
    }

    if (s->security_levels == 1 && !attrs.secure && !GIC_TEST_GROUP(irq, cpu)) {
        DPRINTF("Non-secure EOI for Group0 interrupt %d ignored\n", irq);
        fprintf(stderr, "Non-secure EOI for Group0 interrupt %d ignored cpu(%d)\n", irq, cpu);
        return;
    }

    /* Secure EOI with GICC_CTLR.AckCtl == 0 when the IRQ is a Group 1
     * interrupt is UNPREDICTABLE. We choose to handle it as if AckCtl == 1,
     * i.e. go ahead and complete the irq anyway.
     */

    if (irq != s->running_irq[cpu]) {
        /* Complete an IRQ that is not currently running.  */
        int tmp = s->running_irq[cpu];
        while (s->irq_state[tmp].last_active[cpu] != 1023) {
            if (s->irq_state[tmp].last_active[cpu] == irq) {
                s->irq_state[tmp].last_active[cpu] = s->irq_state[irq].last_active[cpu];
                break;
            }
            tmp = s->irq_state[tmp].last_active[cpu];
        }
    } else {
        /* Complete the current running IRQ.  */
        gic_set_running_irq(s, cpu, s->irq_state[s->running_irq[cpu]].last_active[cpu]);
    }
}
