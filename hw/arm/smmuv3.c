/*
 * QEMU ARM SMMU3 Emulation
 *
 * Copyright (c) 2014-15 Prem Mallappa <pmallapp@broadcom.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/pci/pci.h"
#include "exec/address-spaces.h"
#include "qemu/event_notifier.h"
#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "trace.h"

#include "hw/arm/smmuv3.h"

#define SMMU_NREGS       0x200

#define ARM_SMMU_DEBUG
#ifdef ARM_SMMU_DEBUG

enum {SMMU_DBG_PANIC, SMMU_DBG_CRIT, SMMU_DBG_WARN, SMMU_DBG_INFO};

#define DBG_BIT(bit)    (1 << SMMU_DBG_##bit)

static unsigned char dbg_bits = DBG_BIT(PANIC) | DBG_BIT(CRIT);

#define SMMU_DPRINTF(lvl, fmt, ...)     do {            \
        if (dbg_bits & DBG_BIT(lvl))                    \
            fprintf(stderr, "(smmu)%s: " fmt "\n",      \
                    __func__, ## __VA_ARGS__);          \
    } while (0)

#define HERE() fprintf(stderr, "HERE ====> %s:%d\n", __func__, __LINE__)
#else
#define SMMU_DPRINTF(lvl, fmt, ...) {}
#endif

#define TYPE_SMMU_DEV "smmuv3"
#define SMMU_DEV(obj) OBJECT_CHECK(SMMUState, (obj), TYPE_SMMU_DEV)

#define SMMU_SYS_DEV(obj) OBJECT_CHECK(SMMUSysState, (obj), TYPE_SMMU_DEV)
#define SMMU_PCI_DEV(obj) OBJECT_CHECK(SMMUPciState, (obj), TYPE_SMMU_DEV)


typedef struct SMMUState {
    uint32_t      regs[SMMU_NREGS];
    uint32_t      cid[4];
    uint32_t      pid[8];
    uint32_t      mask[SMMU_NREGS];
    qemu_irq      irq;
    uint32_t      version;

    bool          virtual;
    bool          in_line;

    GHashTable   *iotlb;          /* IOTLB */
    MemoryRegion  iomem;

    MemoryRegion  iommu;
    AddressSpace  iommu_as;
} SMMUState;

typedef struct SMMUPciState{
    /* <private> */
    PCIDevice   dev;

    SMMUState   smmu_state;
} SMMUPciState;

typedef struct SMMUSysState {
    /* <private> */
    SysBusDevice  dev;
    SMMUState     smmu_state;
} SMMUSysState;


static IOMMUTLBEntry
smmu_translate_dev(DeviceState *dev, MemoryRegion *iommu,
		 hwaddr addr, bool is_write)
{
    IOMMUTLBEntry ret = {
        .target_as = &address_space_memory,
        .iova = addr,
        .translated_addr = addr,
        .addr_mask = TARGET_PAGE_MASK,
        .perm = IOMMU_NONE,
    };

    ret.perm = is_write ? IOMMU_WO: IOMMU_RO;

    //fprintf(stderr, "DeviceID is %x\n", );
    HERE();

    return ret;
}

/*
 * smmu_translate: this is called when device does a
 * pci_dma_{read,write}() or similar
 * Need to get the device's streamid which is nothing but PCI
 * requestor id (RID)
 *
 */
static IOMMUTLBEntry
smmu_translate(MemoryRegion *iommu, hwaddr addr, bool is_write)
{
    IOMMUTLBEntry ret = {
        .target_as = &address_space_memory,
        .iova = addr,
        .translated_addr = addr,
        .addr_mask = TARGET_PAGE_MASK,
        .perm = IOMMU_NONE,
    };

    ret.perm = is_write ? IOMMU_WO: IOMMU_RO;

    return ret;
}

static const MemoryRegionIOMMUOps smmu_ops = {
    .translate = smmu_translate,
    .translate_dev = smmu_translate_dev,
};


static AddressSpace *smmu_pci_iommu(PCIBus *bus, void *opaque, int devfn)
{
    SMMUState *s = opaque;
    HERE();
    return &s->iommu_as;
}

static uint64_t smmu_mem_read(void *opaque, hwaddr addr,
                              unsigned size)
{
    uint32_t ret;
    SMMUState *s = opaque;

    SMMU_DPRINTF(CRIT, "read %lx", addr);

    /* Primecell/Corelink ID registers */
    switch (addr) {
    case 0xFF0 ... 0xFFC:
        return (uint64_t)s->cid[(addr - 0xFF0)>>2];
    case 0xFDC ... 0xFE4:
        return (uint64_t)s->pid[(addr - 0xFDC)>>2];
    default:
        return s->regs[addr >> 2];
    }

    return ret;
}

static void smmu_mem_write(void *opaque, hwaddr addr,
                           uint64_t val, unsigned size)
{
    SMMUState *s = opaque;
    uint32_t val32 = (uint32_t)val;

    SMMU_DPRINTF(CRIT, "reg:%lx = %lx", addr, val);

    if (addr > 0xFDC) {
        SMMU_DPRINTF(CRIT, "Trying to write to read-only register %lx", addr);
        return;
    }

    s->regs[addr>>2] = val32;
}

static const MemoryRegionOps smmu_mem_ops = {
    .read = smmu_mem_read,
    .write = smmu_mem_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static const VMStateDescription vmstate_smmu = {
    .name ="smmu",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, SMMUState, SMMU_NREGS),
        VMSTATE_END_OF_LIST(),
    }
};

static void _smmu_populate_regs(SMMUState *s)
{
    int i;

    /* Primecell ID registers */
    s->cid[0] = 0x0D;
    s->cid[1] = 0xF0;
    s->cid[2] = 0x05;
    s->cid[3] = 0xB1;

    for (i = 0; i < sizeof(s->pid)/sizeof(s->pid[0]); i++)
        s->pid[i] = 0x1;

    /* Only IDR0-5 will show what features supported */
    s->regs[SMMU_REG_IDR0] =
        1 << 27 |                   /* 2 Level stream id */
        1 << 26 |                   /* Term Model  */
        1 << 24 |                   /* Stall model not supported */
        1 << 18 |                   /* VMID 16 bits */
        1 << 16 |                   /* PRI */
        1 << 12 |                   /* ASID 16 bits */
        1 << 10 |                   /* ATS */
        1 << 9 |                    /* HYP */
        2 << 6 |                    /* HTTU */
        1 << 4 |                    /* COHACC */
        2 << 2 |                    /* TTF=Arch64 */
        1 << 1 |                    /* Stage 1 */
        1 << 0;                     /* Stage 2 */

    s->regs[SMMU_REG_IDR1] =
        1 << 27 |                   /* Attr Types override */
        19 << 21|                   /* Cmd Q size */
        19 << 16|                   /* Event Q size */
        19 << 11|                   /* PRI Q size */
        16 << 0 ;                   /* SID size (SSID not supported) */

        s->regs[SMMU_REG_IDR5] =
        1 << 6 |                    /* Granule 16K */
        1 << 4 |                    /* Granule 4K */
        4 << 0;                     /* OAS = 44 bits */
}

static int smmu_init(SysBusDevice *dev)
{
    SMMUSysState *sys = SMMU_SYS_DEV(dev);
    SMMUState *s = &sys->smmu_state;
    PCIBus *pcibus = pci_find_primary_bus();

    HERE();
    /* Register Access */
    memory_region_init_io(&s->iomem, OBJECT(sys), &smmu_mem_ops,
                          s, "smmuv3", 0x1000);

    /* Host memory as seen from the PCI side, via the IOMMU.  */
    _smmu_populate_regs(s);

    sysbus_init_mmio(dev, &s->iomem);

    sysbus_init_irq(dev, &s->irq);

    /* Host Memory Access */
    memory_region_init_iommu(&s->iommu, OBJECT(sys), &smmu_ops,
                             "smmuv3", UINT64_MAX);

    address_space_init(&s->iommu_as, &s->iommu, "smmu-as");

    if (pcibus)
        pci_setup_iommu(pcibus, smmu_pci_iommu, s);

    return 0;
}

static void smmu_reset(DeviceState *dev)
{
    HERE();
}

static Property smmu_properties[] = {
    DEFINE_PROP_BOOL("inline", SMMUState, in_line, true),
    DEFINE_PROP_BOOL("virtual", SMMUState, virtual, true),
    DEFINE_PROP_END_OF_LIST(),
};

static void smmu_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    dc->reset = smmu_reset;
    k->init = smmu_init;
    dc->vmsd = &vmstate_smmu;
    dc->props = smmu_properties;

}

static const TypeInfo smmu_info = {
    .name          = TYPE_SMMU_DEV,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SMMUSysState),
    .class_init    = smmu_class_init,
};

#define PCI_DEVICE_ID_BRCM_SMMU         0x9005

static void smmu_pci_realize(PCIDevice *pci_dev, Error **errp)
{
    SMMUPciState *pci_smmu = SMMU_PCI_DEV(pci_dev);
    SMMUState *s = &pci_smmu->smmu_state;
    PCIBus *b = NULL;
    uint8_t *pci_conf;

    pci_conf = pci_dev->config;

    pci_conf[PCI_INTERRUPT_PIN] = 1; /* interrupt pin A */

    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->iomem);

    _smmu_populate_regs(&pci_smmu->smmu_state);

    pci_setup_iommu(b, smmu_pci_iommu, s);
}

static void smmu_pci_exit(PCIDevice *dev)
{
    //SMMUPciState *pci = DO_UPCAST(SMMUPciState, dev, dev);

}

static void smmu_pci_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(klass);

    pc->realize = smmu_pci_realize;
    pc->exit = smmu_pci_exit;
    pc->vendor_id = PCI_VENDOR_ID_BROADCOM;
    pc->device_id = PCI_DEVICE_ID_BRCM_SMMU;
    pc->revision = 1;
    pc->class_id = PCI_CLASS_COMMUNICATION_SERIAL;
    dc->vmsd = &vmstate_smmu;
    dc->props = smmu_properties;
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static const TypeInfo smmu_pci_info = {
    .name          = "smmuv3-pci",
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(SMMUPciState),
    .class_init    = smmu_pci_class_init,
};

static void smmu_register_types(void)
{
    type_register_static(&smmu_info);
    type_register_static(&smmu_pci_info);
}

type_init(smmu_register_types)
