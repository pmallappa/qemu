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
#include "arm_gicv3_cpu_interface.h"
#include "arm_gicv3_interrupts.h"
#include "arm_gicv3_dist.h"
#include "arm_gicv3_redist.h"
#include "arm_gicv3_spi_its.h"

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
     * Uses system gicv3_no_gicv2_bc mode
     */
    gicv3_sre = 1;

    /* Tell the common code we're a GICv3 */
    s->revision = REV_V3;

    gicv3_init_irqs_and_mmio(s, gicv3_set_irq, gic_ops);

    /* Compute mask for decoding the core number in redistributer */
    if (is_power_of_2(NUM_CPU(s)))
        power2 = NUM_CPU(s);
    else
        /* QEMU has only  pow2floor !!! */
        power2 = pow2floor(2 * NUM_CPU(s));
    s->cpu_mask = (power2 - 1);

    DPRINTF(" -- NUM_CPUS(%d) - cpu mask(0%x) -- \n", NUM_CPU(s), s->cpu_mask);

    bitmap_zero(s->cpu_enabled, s->num_cpu);
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
