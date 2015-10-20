#ifndef QEMU_ARM_GICV3_DIST_H
#define QEMU_ARM_GICV3_DIST_H

MemTxResult gic_dist_read(void *opaque, hwaddr offset, uint64_t *data,
                          unsigned size, MemTxAttrs attrs);
MemTxResult gic_dist_write(void *opaque, hwaddr addr, uint64_t data,
                           unsigned size, MemTxAttrs attrs);

#endif /* !QEMU_ARM_GIC_DIST_H */
