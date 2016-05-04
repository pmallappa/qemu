/*
 * QEMU PCI test device
 *
 * Copyright (c) 2012 Red Hat Inc.
 * Author: Michael S. Tsirkin <mst@redhat.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
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
#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "qemu/event_notifier.h"

/*
 * pci-testdev-smmu:
 *          Simple PCIe device, to enable read and write from memory.
 * Architecture:
 *          Following registers are supported.
 *          TST_COMMAND = 0x0
 *          TST_STATUS  = 0x4
 *          TST_SRC_ADDRESS = 0x8
 *          TST_SIZE        = 0x10
 *          TST_DST_ADDRESS = 0x18
 */

/*
 *  TST_COMMAND Register bits
 *      OP[0]
 *          READ = 0x0
 *          WRITE = 0x1
 */

enum reg {
    TST_REG_COMMAND  = 0x0,
    TST_REG_STATUS   = 0x4,
    TST_REG_SRC_ADDR = 0x8,
    TST_REG_SIZE     = 0x10,
    TST_REG_DST_ADDR = 0x18,

    TST_REG_LAST     = 0x30,
};

#define CMD_READ    0x100
#define CMD_WRITE   0x200
#define CMD_RW      0x300

#define STATUS_OK   (1 << 0)
#define STATUS_CMD_ERROR (1 << 1)
#define STATUS_CMD_INVALID (1 << 2)

typedef struct PCITestDevState {
    /*< private >*/
    PCIDevice dev;
    /*< public >*/

    MemoryRegion mmio;
    uint8_t regs[TST_REG_LAST];
} PCITestDevState;

#define TYPE_PCI_TEST_DEV "pci-testdev-smmu"

#define PCI_TEST_DEV(obj) \
    OBJECT_CHECK(PCITestDevState, (obj), TYPE_PCI_TEST_DEV)

static void
pci_testdev_smmu_reset(PCITestDevState *d)
{
    memset(d->regs, 0, sizeof(d->regs));
}


static void
pci_testdev_smmu_handle_cmd(PCITestDevState *d, hwaddr addr, uint64_t val,
                            unsigned _unused_size)
{
    hwaddr src, dst;
    src = *(uint64_t*)(&d->regs[TST_REG_SRC_ADDR]);
    dst = *(uint64_t*)(&d->regs[TST_REG_DST_ADDR]);
    uint32_t size = *(uint32_t*)(&d->regs[TST_REG_SIZE]);
    uint8_t buf[128];

    while (size) {
        int nbytes = (size < sizeof(buf)) ? size: sizeof(buf);
        int ret = 0;

        if (val & CMD_READ)
            ret = pci_dma_read(&d->dev, src, (void*)buf, nbytes);

        if (ret)
            return;

        if (val & CMD_WRITE)
            ret = pci_dma_write(&d->dev, dst, (void*)buf, nbytes);

        size -= nbytes;
    }
}

static void
pci_testdev_smmu_mmio_write(void *opaque, hwaddr addr,
                            uint64_t val, unsigned size)
{
    PCITestDevState *d = opaque;

    switch (addr) {
    case TST_REG_COMMAND:
        pci_testdev_smmu_handle_cmd(d, addr, val, size);
        break;
    case TST_REG_STATUS:        /* Read only reg */
        break;
    case TST_REG_SRC_ADDR:
    case TST_REG_DST_ADDR:
        d->regs[addr] = val;
        break;

    case TST_REG_SIZE:
        d->regs[addr] = (uint32_t)val;
        break;
    default:
        printf("Unkown register write\n");
    }
}

static uint64_t
pci_testdev_smmu_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    PCITestDevState *d = opaque;
    switch (addr) {
    case TST_REG_SRC_ADDR:
    case TST_REG_DST_ADDR:
    default:
        return d->regs[addr];
    }

    return (uint32_t)d->regs[addr];
}

static const MemoryRegionOps pci_testdev_mmio_ops = {
    .read = pci_testdev_smmu_mmio_read,
    .write = pci_testdev_smmu_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static void pci_testdev_smmu_realize(PCIDevice *pci_dev, Error **errp)
{
    PCITestDevState *d = PCI_TEST_DEV(pci_dev);
    uint8_t *pci_conf;

    printf("How are you\n");
    pci_conf = pci_dev->config;

    pci_conf[PCI_INTERRUPT_PIN] = 0; /* no interrupt pin */
    printf("a\n");
    memory_region_init_io(&d->mmio, OBJECT(d), &pci_testdev_mmio_ops, d,
                          "pci-testdev-smmu-mmio", 1 << 10);
    printf("b\n");
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->mmio);
    printf("c\n");
}

static void
pci_testdev_smmu_uninit(PCIDevice *dev)
{
    PCITestDevState *d = PCI_TEST_DEV(dev);

    pci_testdev_smmu_reset(d);
}

static void qdev_pci_testdev_smmu_reset(DeviceState *dev)
{
    PCITestDevState *d = PCI_TEST_DEV(dev);
    pci_testdev_smmu_reset(d);
}

static void pci_testdev_smmu_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    printf("Hello\n");
    k->realize = pci_testdev_smmu_realize;
    k->exit = pci_testdev_smmu_uninit;
    k->vendor_id = PCI_VENDOR_ID_REDHAT;
    k->device_id = PCI_DEVICE_ID_REDHAT_TEST;
    k->revision = 0x00;
    k->class_id = PCI_CLASS_OTHERS;
    dc->desc = "PCI Test Device - for smmu";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->reset = qdev_pci_testdev_smmu_reset;
}

static const TypeInfo pci_testdev_smmu_info = {
    .name          = TYPE_PCI_TEST_DEV,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PCITestDevState),
    .class_init    = pci_testdev_smmu_class_init,
};

static void pci_testdev_smmu_register_types(void)
{
    type_register_static(&pci_testdev_smmu_info);
}

type_init(pci_testdev_smmu_register_types)
