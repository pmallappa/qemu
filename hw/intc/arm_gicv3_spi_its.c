#include "gicv3_internal.h"
#include "qom/cpu.h"
#include "arm_gicv3_spi_its.h"
#include "arm_gicv3_interrupts.h"

/* ITS routines are stubs for future development */
static uint64_t gic_its_readb(void *opaque, hwaddr offset)
{
    return 0;
}

static uint64_t gic_its_readw(void *opaque, hwaddr offset)
{
    uint64_t val;
    val = gic_its_readb(opaque, offset);
    val |= gic_its_readb(opaque, offset + 1) << 8;
    return val;
}

static uint64_t gic_its_readl(void *opaque, hwaddr offset)
{
    uint64_t val;
    val = gic_its_readw(opaque, offset);
    val |= gic_its_readw(opaque, offset + 2) << 16;
    return val;
}

static uint64_t gic_its_readll(void *opaque, hwaddr offset)
{
    uint64_t val;
    val = gic_its_readl(opaque, offset);
    val |= gic_its_readl(opaque, offset + 4) << 32;
    return val;
}

static void gic_its_writeb(void *opaque, hwaddr offset,
                            uint64_t value)
{
    GICv3State *s = (GICv3State *)opaque;
    gicv3_update(s);
    return;
}

static void gic_its_writew(void *opaque, hwaddr offset,
                            uint64_t value)
{
    gic_its_writeb(opaque, offset, value & 0xff);
    gic_its_writeb(opaque, offset + 1, value >> 8);
}

static void gic_its_writel(void *opaque, hwaddr offset,
                            uint64_t value)
{
    gic_its_writel(opaque, offset, value & 0xffff);
    gic_its_writel(opaque, offset + 2, value >> 16);
}

static void gic_its_writell(void *opaque, hwaddr offset,
                            uint64_t value)
{
    gic_its_writell(opaque, offset, value & 0xffffffff);
    gic_its_writell(opaque, offset + 4, value >> 32);
}

uint64_t gic_its_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t data;
    switch (size) {
    case 1:
        data = gic_its_readb(opaque, addr);
        break;
    case 2:
        data = gic_its_readw(opaque, addr);
        break;
    case 4:
        data = gic_its_readl(opaque, addr);
        break;
    case 8:
        data = gic_its_readll(opaque, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: size %u\n", __func__, size);
        assert(0);
        break;
    }
    DPRINTF("offset %p data %p\n", (void *) addr, (void *) data);
    return data;
}

void gic_its_write(void *opaque, hwaddr addr, uint64_t data, unsigned size)
{
    DPRINTF("offset %p data %p\n", (void *) addr, (void *) data);
    switch (size) {
    case 1:
        gic_its_writew(opaque, addr, data);
        break;
    case 2:
        gic_its_writew(opaque, addr, data);
        break;
    case 4:
        gic_its_writel(opaque, addr, data);
        break;
    case 8:
        gic_its_writell(opaque, addr, data);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: size %u\n", __func__, size);
        assert(0);
        break;
    }
}

/* SPI routines are stubs for future development */
static uint64_t gic_spi_readb(void *opaque, hwaddr offset)
{
    return 0;
}

static uint64_t gic_spi_readw(void *opaque, hwaddr offset)
{
    uint64_t val;
    val = gic_spi_readb(opaque, offset);
    val |= gic_spi_readb(opaque, offset + 1) << 8;
    return val;
}

static uint64_t gic_spi_readl(void *opaque, hwaddr offset)
{
    uint64_t val;
    val = gic_spi_readw(opaque, offset);
    val |= gic_spi_readw(opaque, offset + 2) << 16;
    return val;
}

static uint64_t gic_spi_readll(void *opaque, hwaddr offset)
{
    uint64_t val;
    val = gic_spi_readl(opaque, offset);
    val |= gic_spi_readl(opaque, offset + 4) << 32;
    return val;
}

static void gic_spi_writeb(void *opaque, hwaddr offset,
                            uint64_t value)
{
    GICv3State *s = (GICv3State *)opaque;
    gicv3_update(s);
    return;
}

static void gic_spi_writew(void *opaque, hwaddr offset,
                            uint64_t value)
{
    gic_spi_writeb(opaque, offset, value & 0xff);
    gic_spi_writeb(opaque, offset + 1, value >> 8);
}

static void gic_spi_writel(void *opaque, hwaddr offset,
                            uint64_t value)
{
    gic_spi_writew(opaque, offset, value & 0xffff);
    gic_spi_writew(opaque, offset + 2, value >> 16);
}

static void gic_spi_writell(void *opaque, hwaddr offset,
                            uint64_t value)
{
    gic_spi_writel(opaque, offset, value & 0xffffffff);
    gic_spi_writel(opaque, offset + 4, value >> 32);
}

uint64_t gic_spi_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t data;
    switch (size) {
    case 1:
        data = gic_spi_readb(opaque, addr);
        break;
    case 2:
        data = gic_spi_readw(opaque, addr);
        break;
    case 4:
        data = gic_spi_readl(opaque, addr);
        break;
    case 8:
        data = gic_spi_readll(opaque, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: size %u\n", __func__, size);
        assert(0);
        break;
    }
    DPRINTF("offset %p data %p\n", (void *) addr, (void *) data);
    return data;
}

void gic_spi_write(void *opaque, hwaddr addr, uint64_t data, unsigned size)
{
    DPRINTF("offset %p data %p\n", (void *) addr, (void *) data);
    switch (size) {
    case 1:
        gic_spi_writeb(opaque, addr, data);
        break;
    case 2:
        gic_spi_writew(opaque, addr, data);
        break;
    case 4:
        gic_spi_writel(opaque, addr, data);
        break;
    case 8:
        gic_spi_writell(opaque, addr, data);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: size %u\n", __func__, size);
        assert(0);
        break;
    }
}

/* ITS control routines are stubs for future development */
static uint64_t gic_its_cntrl_readb(void *opaque, hwaddr offset)
{
    GICv3State *s = (GICv3State *)opaque;
    uint64_t res=0;

    if (offset < 0x100) {
          if (offset == 0)
            return 0;
          if (offset == 4)
              return 0;
          if (offset < 0x08)
            return s->num_cpu;
          if (offset >= 0x80) {
            return 0;
          }
          goto bad_reg;
      }
    return res;
bad_reg:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: Bad offset %x\n", __func__, (int)offset);
    return 0;
}

static uint64_t gic_its_cntrl_readw(void *opaque, hwaddr offset)
{
    uint64_t val;
    val = gic_its_cntrl_readb(opaque, offset);
    val |= gic_its_cntrl_readb(opaque, offset + 1) << 8;
    return val;
}

static uint64_t gic_its_cntrl_readl(void *opaque, hwaddr offset)
{
    uint64_t val;
    val = gic_its_cntrl_readw(opaque, offset);
    val |= gic_its_cntrl_readw(opaque, offset + 2) << 16;
    return val;
}

static uint64_t gic_its_cntrl_readll(void *opaque, hwaddr offset)
{
    uint64_t val;
    val = gic_its_cntrl_readl(opaque, offset);
    val |= gic_its_cntrl_readl(opaque, offset + 4) << 32;
    return val;
}

static void gic_its_cntrl_writeb(void *opaque, hwaddr offset,
                                 uint64_t value)
{
    GICv3State *s = (GICv3State *)opaque;
    if (offset < 0x100) {
        if (offset < 0x08)
            s->num_cpu = value;
        else
            goto bad_reg;
    }
    gicv3_update(s);
    return;
bad_reg:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: Bad offset %x\n", __func__, (int)offset);
}

static void gic_its_cntrl_writew(void *opaque, hwaddr offset,
                            uint64_t value)
{
    gic_its_cntrl_writeb(opaque, offset, value & 0xff);
    gic_its_cntrl_writeb(opaque, offset + 1, value >> 8);
}

static void gic_its_cntrl_writel(void *opaque, hwaddr offset,
                            uint64_t value)
{
    gic_its_cntrl_writew(opaque, offset, value & 0xffff);
    gic_its_cntrl_writew(opaque, offset + 2, value >> 16);
}

static void gic_its_cntrl_writell(void *opaque, hwaddr offset,
                            uint64_t value)
{
    gic_its_cntrl_writel(opaque, offset, value & 0xffffffff);
    gic_its_cntrl_writel(opaque, offset + 4, value >> 32);
}

uint64_t gic_its_cntrl_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t data;
    switch (size) {
    case 1:
        data = gic_its_cntrl_readb(opaque, addr);
        break;
    case 2:
        data = gic_its_cntrl_readw(opaque, addr);
        break;
    case 4:
        data = gic_its_cntrl_readl(opaque, addr);
        break;
    case 8:
        data = gic_its_cntrl_readll(opaque, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: size %u\n", __func__, size);
        assert(0);
        break;
    }
    DPRINTF("offset %p data %p\n", (void *) addr, (void *) data);
    return data;
}

void gic_its_cntrl_write(void *opaque, hwaddr addr, uint64_t data, unsigned size)
{
    DPRINTF("offset %p data %p\n", (void *) addr, (void *) data);
    switch (size) {
    case 1:
        gic_its_cntrl_writeb(opaque, addr, data);
        break;
    case 2:
        gic_its_cntrl_writew(opaque, addr, data);
        break;
    case 4:
        gic_its_cntrl_writel(opaque, addr, data);
        break;
    case 8:
        gic_its_cntrl_writell(opaque, addr, data);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: size %u\n", __func__, size);
        assert(0);
        break;
    }
}
