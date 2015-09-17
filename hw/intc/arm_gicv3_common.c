/*
 * ARM GICv3 support - common bits of emulated and KVM kernel model
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

#include "hw/intc/arm_gicv3_common.h"
#include "gicv3_internal.h"

static void gicv3_pre_save(void *opaque)
{
    GICv3State *s = (GICv3State *)opaque;
    ARMGICv3CommonClass *c = ARM_GICV3_COMMON_GET_CLASS(s);

    if (c->pre_save) {
        c->pre_save(s);
    }
}

static int gicv3_post_load(void *opaque, int version_id)
{
    GICv3State *s = (GICv3State *)opaque;
    ARMGICv3CommonClass *c = ARM_GICV3_COMMON_GET_CLASS(s);

    if (c->post_load) {
        c->post_load(s);
    }
    return 0;
}

static const VMStateDescription vmstate_gicv3_irq_state = {
    .name = "arm_gicv3_irq_state",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT64(pending, gicv3_irq_state),
        VMSTATE_UINT64(active, gicv3_irq_state),
        VMSTATE_UINT64(level, gicv3_irq_state),
        VMSTATE_UINT64(group, gicv3_irq_state),
        VMSTATE_BOOL(edge_trigger, gicv3_irq_state),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_gicv3_sgi_state = {
    .name = "arm_gicv3_sgi_state",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT64_ARRAY(pending, gicv3_sgi_state, GICV3_NCPU),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_gicv3 = {
    .name = "arm_gicv3",
    .unmigratable = 1,
    .pre_save = gicv3_pre_save,
    .post_load = gicv3_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(ctlr, GICv3State),
        VMSTATE_UINT32_ARRAY(cpu_ctlr, GICv3State, GICV3_NCPU),
        VMSTATE_STRUCT_ARRAY(irq_state, GICv3State, GICV3_MAXIRQ, 1,
                             vmstate_gicv3_irq_state, gicv3_irq_state),
        VMSTATE_UINT64_ARRAY(irq_target, GICv3State, GICV3_MAXIRQ),
        VMSTATE_UINT8_2DARRAY(priority1, GICv3State, GICV3_INTERNAL, GICV3_NCPU),
        VMSTATE_UINT8_ARRAY(priority2, GICv3State, GICV3_MAXIRQ - GICV3_INTERNAL),
        VMSTATE_UINT16_2DARRAY(last_active, GICv3State, GICV3_MAXIRQ, GICV3_NCPU),
        VMSTATE_STRUCT_ARRAY(sgi_state, GICv3State, GICV3_NR_SGIS, 1,
                             vmstate_gicv3_sgi_state, gicv3_sgi_state),
        VMSTATE_UINT16_ARRAY(priority_mask, GICv3State, GICV3_NCPU),
        VMSTATE_UINT16_ARRAY(running_irq, GICv3State, GICV3_NCPU),
        VMSTATE_UINT16_ARRAY(running_priority, GICv3State, GICV3_NCPU),
        VMSTATE_UINT16_ARRAY(current_pending, GICv3State, GICV3_NCPU),
        VMSTATE_END_OF_LIST()
    }
};

void gicv3_init_irqs_and_mmio(GICv3State *s, qemu_irq_handler handler,
                              const MemoryRegionOps *ops)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(s);
    int i;

    /* For the GIC, also expose incoming GPIO lines for PPIs for each CPU.
     * GPIO array layout is thus:
     *  [0..N-1] spi
     *  [N..N+31] PPIs for CPU 0
     *  [N+32..N+63] PPIs for CPU 1
     *   ...
     */
    i = s->num_irq - GIC_INTERNAL + GIC_INTERNAL * s->num_cpu;
    qdev_init_gpio_in(DEVICE(s), handler, i);

    s->parent_irq = g_malloc(s->num_cpu * sizeof(qemu_irq));
    s->parent_fiq = g_malloc(s->num_cpu * sizeof(qemu_irq));

    for (i = 0; i < s->num_cpu; i++) {
        sysbus_init_irq(sbd, &s->parent_irq[i]);
    }
    for (i = 0; i < s->num_cpu; i++) {
        sysbus_init_irq(sbd, &s->parent_fiq[i]);
    }

    memory_region_init_io(&s->iomem_dist, OBJECT(s), ops, s,
                          "gicv3_dist", 0x10000);
    memory_region_init_io(&s->iomem_redist, OBJECT(s), ops ? &ops[1] : NULL, s,
                          "gicv3_redist", 0x20000 * s->num_cpu);

    sysbus_init_mmio(sbd, &s->iomem_dist);
    sysbus_init_mmio(sbd, &s->iomem_redist);
}

static void arm_gicv3_common_realize(DeviceState *dev, Error **errp)
{
    GICv3State *s = ARM_GICV3_COMMON(dev);
    int num_irq = s->num_irq;

    /* revision property is actually reserved and currently used only in order
     * to keep the interface compatible with GICv2 code, avoiding extra
     * conditions. However, in future it could be used, for example, if we
     * implement GICv4.
     */
    if (s->revision != 3) {
        error_setg(errp, "unsupported GIC revision %d", s->revision);
        return;
    }
    if (s->num_cpu > GICV3_NCPU) {
        error_setg(errp, "requested %u CPUs exceeds GIC maximum %d",
                   s->num_cpu, GICV3_NCPU);
        return;
    }
    s->num_irq += GICV3_BASE_IRQ;
    if (s->num_irq > GICV3_MAXIRQ) {
        error_setg(errp,
                   "requested %u interrupt lines exceeds GIC maximum %d",
                   num_irq, GICV3_MAXIRQ);
        return;
    }
    /* ITLinesNumber is represented as (N / 32) - 1 (see
     * gic_dist_readb) so this is an implementation imposed
     * restriction, not an architectural one:
     */
    if (s->num_irq < 32 || (s->num_irq % 32)) {
        error_setg(errp,
                   "%d interrupt lines unsupported: not divisible by 32",
                   num_irq);
        return;
    }
}

static void arm_gicv3_common_reset(DeviceState *dev)
{
    GICv3State *s = ARM_GICV3_COMMON(dev);
    int i;

    /* Note num_cpu and num_irq are properties */

    /* Don't reset anything assigned in arm_gic_realize or any property */

    /* No GICv2 backwards computability support */
    for (i = 0; i < s->num_cpu; i++) {
        s->priority_mask[i] = 0;
        s->current_pending[i] = 1023;
        s->running_irq[i] = 1023;
        s->running_priority[i] = 0x100;
        s->cpu_ctlr[i] = 0;
    }

    memset(s->irq_state, 0, sizeof(s->irq_state));
    /* GIC-500 comment 'j' SGI are always enabled */
    for (i = 0; i < GICV3_NR_SGIS; i++) {
        GIC_SET_ENABLED(i, ALL_CPU_MASK);
        GIC_SET_EDGE_TRIGGER(i);
    }
    memset(s->sgi_state, 0, sizeof(s->sgi_state));
    memset(s->irq_target, 0, sizeof(s->irq_target));
    if (s->num_cpu == 1) {
        /* For uniprocessor GICs all interrupts always target the sole CPU */
        for (i = 0; i < GICV3_MAXIRQ; i++) {
            s->irq_target[i] = 1;
        }
    }
    memset(s->priority1, 0, sizeof(s->priority1));
    memset(s->priority2, 0, sizeof(s->priority2));
    memset(s->last_active, 0, sizeof(s->last_active));

    /* With all configuration we don't  support GICv2 backwards computability */
    if (s->security_levels > 1) {
        /* GICv3 5.3.20 With two security So DS is RAZ/WI ARE_NS is RAO/WI
         * and ARE_S is RAO/WI
         */
         s->ctlr = GICD_CTLR_ARE_S | GICD_CTLR_ARE_NS;
    } else {
        /* GICv3 5.3.20 With one security So DS is RAO/WI ARE is RAO/WI
         */
        s->ctlr = GICD_CTLR_DS | GICD_CTLR_ARE;
    }
    /* Workaround!
     * Linux (drivers/irqchip/irq-gic-v3.c) is enabling only group one,
     * in gic_cpu_sys_reg_init it calls gic_write_grpen1(1);
     * but it doesn't conigure any interrupt to be in group one
     */
    for (i = 0; i < s->num_irq; i++)
        GIC_SET_GROUP(i, ALL_CPU_MASK);
}

static Property arm_gicv3_common_properties[] = {
    DEFINE_PROP_UINT32("num-cpu", GICv3State, num_cpu, 1),
    DEFINE_PROP_UINT32("num-irq", GICv3State, num_irq, 32),
    DEFINE_PROP_UINT32("revision", GICv3State, revision, 3),
    DEFINE_PROP_UINT8("security-levels", GICv3State, security_levels, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void arm_gicv3_common_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = arm_gicv3_common_reset;
    dc->realize = arm_gicv3_common_realize;
    dc->props = arm_gicv3_common_properties;
    dc->vmsd = &vmstate_gicv3;
}

static const TypeInfo arm_gicv3_common_type = {
    .name = TYPE_ARM_GICV3_COMMON,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GICv3State),
    .class_size = sizeof(ARMGICv3CommonClass),
    .class_init = arm_gicv3_common_class_init,
    .abstract = true,
};

static void register_types(void)
{
    type_register_static(&arm_gicv3_common_type);
}

type_init(register_types)
