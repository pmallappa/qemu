#ifndef QEMU_ARM_GICV3_SPI_ITS_H
#define QEMU_ARM_GICV3_SPI_ITS_H

uint64_t gic_its_read(void *opaque, hwaddr addr, unsigned size);
void gic_its_write(void *opaque, hwaddr addr, uint64_t data, unsigned size);
uint64_t gic_spi_read(void *opaque, hwaddr addr, unsigned size);
void gic_spi_write(void *opaque, hwaddr addr, uint64_t data, unsigned size);
uint64_t gic_its_cntrl_read(void *opaque, hwaddr addr, unsigned size);
void gic_its_cntrl_write(void *opaque, hwaddr addr, uint64_t data, unsigned size);

#endif /* !QEMU_ARM_GIC_SPI_ITS_H */
