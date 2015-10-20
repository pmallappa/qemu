#ifndef QEMU_ARM_GICV3_CPU_INTERFACE_H
#define QEMU_ARM_GICV3_CPU_INTERFACE_H


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

#endif /* !QEMU_ARM_GIC_CPU_INTERFACE_H */
