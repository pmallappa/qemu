#ifndef QEMU_ARM_GICV3_REDIST_H
#define QEMU_ARM_GICV3_REDIST_H

MemTxResult gic_redist_read(void *opaque, hwaddr offset, uint64_t *data,
                            unsigned size, MemTxAttrs attrs);
MemTxResult gic_redist_write(void *opaque, hwaddr addr, uint64_t data,
                             unsigned size, MemTxAttrs attrs);

#endif /* !QEMU_ARM_GIC_REDIST_H */
