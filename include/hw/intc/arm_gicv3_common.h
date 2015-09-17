/*
 * ARM GIC support
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

#ifndef HW_ARM_GICV3_COMMON_H
#define HW_ARM_GICV3_COMMON_H

#include "hw/sysbus.h"
#include "hw/intc/arm_gic_common.h"

/* Maximum number of possible interrupts, determined by the GIC architecture */
#define GICV3_MAXIRQ 1020
/* First 32 are private to each CPU (SGIs and PPIs). */
#define GICV3_INTERNAL 32
#define GICV3_NR_SGIS 16
#define GICV3_NCPU 64

#define MAX_NR_GROUP_PRIO 128

typedef struct gicv3_irq_state {
    /* The enable bits are only banked for per-cpu interrupts.  */
    uint64_t enabled;
    uint64_t pending;
    uint64_t active;
    uint64_t level;
    uint64_t group;
    bool edge_trigger; /* true: edge-triggered, false: level-triggered  */
} gicv3_irq_state;

typedef struct gicv3_sgi_state {
    uint64_t pending[GICV3_NCPU];
} gicv3_sgi_state;

typedef struct GICv3State {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    qemu_irq *parent_irq;
    qemu_irq *parent_fiq;
    /* GICD_CTLR; for a GIC with the security extensions the NS banked version
     * of this register is just an alias of bit 1 of the S banked version.
     */
    uint32_t ctlr;
    /* Sim GICC_CTLR; again, the NS banked version is just aliases of bits of
     * the S banked register, so our state only needs to store the S version.
     */
    uint32_t cpu_ctlr[GICV3_NCPU];
    bool cpu_enabled[GICV3_NCPU];

    gicv3_irq_state irq_state[GICV3_MAXIRQ];
    uint64_t irq_target[GICV3_MAXIRQ];
    uint8_t priority1[GICV3_INTERNAL][GICV3_NCPU];
    uint8_t priority2[GICV3_MAXIRQ - GICV3_INTERNAL];
    uint16_t last_active[GICV3_MAXIRQ][GICV3_NCPU];
    /* For each SGI on the target CPU, we store 64 bits
     * indicating which source CPUs have made this SGI
     * pending on the target CPU. These correspond to
     * the bytes in the GIC_SPENDSGIR* registers as
     * read by the target CPU.
     */
    gicv3_sgi_state sgi_state[GICV3_NR_SGIS];

    uint16_t priority_mask[GICV3_NCPU];
    uint16_t running_irq[GICV3_NCPU];
    uint16_t running_priority[GICV3_NCPU];
    uint16_t current_pending[GICV3_NCPU];

    MemoryRegion iomem_dist; /* Distributor */
    MemoryRegion iomem_spi;
    MemoryRegion iomem_its_cntrl;
    MemoryRegion iomem_its;
    MemoryRegion iomem_redist; /* Redistributors */

    uint32_t cpu_mask; /* For redistributer */
    uint32_t num_cpu;
    uint32_t num_irq;
    uint32_t revision;
    uint8_t security_levels; /* replace seurity extentions */

    int dev_fd; /* kvm device fd if backed by kvm vgic support */
} GICv3State;

#define TYPE_ARM_GICV3_COMMON "arm-gicv3-common"
#define ARM_GICV3_COMMON(obj) \
     OBJECT_CHECK(GICv3State, (obj), TYPE_ARM_GICV3_COMMON)
#define ARM_GICV3_COMMON_CLASS(klass) \
     OBJECT_CLASS_CHECK(ARMGICv3CommonClass, (klass), TYPE_ARM_GICV3_COMMON)
#define ARM_GICV3_COMMON_GET_CLASS(obj) \
     OBJECT_GET_CLASS(ARMGICv3CommonClass, (obj), TYPE_ARM_GICV3_COMMON)

typedef struct ARMGICv3CommonClass {
    /*< private >*/
    SysBusDeviceClass parent_class;
    /*< public >*/

    void (*pre_save)(GICv3State *s);
    void (*post_load)(GICv3State *s);
} ARMGICv3CommonClass;

void gicv3_init_irqs_and_mmio(GICv3State *s, qemu_irq_handler handler,
                              const MemoryRegionOps *ops);

#endif
