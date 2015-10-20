#ifndef QEMU_ARM_GICV3_INTERRUPTS_H
#define QEMU_ARM_GICV3_INTERRUPTS_H

uint32_t gicv3_acknowledge_irq(GICv3State *s, int cpu, MemTxAttrs attrs);
void gicv3_complete_irq(GICv3State *s, int cpu, int irq, MemTxAttrs attrs);
void gicv3_update(GICv3State *s);
void gicv3_set_priority(GICv3State *s, int cpu, int irq, uint8_t val,
                        MemTxAttrs attrs);
void gicv3_set_irq(void *opaque, int irq, int level);

#endif /* !QEMU_ARM_GIC_INTERRUPTS_H */
