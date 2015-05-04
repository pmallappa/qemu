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

/*  Details:
 *          - No PRI, due to no PCI PRI support
 *          - 44 Bit VA-IPA-OA support
 *          - Under Heavy development
 */

#define SMMU_NREGS       0x200

#define ARM_SMMU_DEBUG
#ifdef ARM_SMMU_DEBUG

enum {SMMU_DBG_PANIC, SMMU_DBG_CRIT, SMMU_DBG_WARN, SMMU_DBG_DEBUG, SMMU_DBG_INFO};

#define DBG_BIT(bit)    (1 << SMMU_DBG_##bit)

static unsigned char dbg_bits = DBG_BIT(PANIC) | DBG_BIT(CRIT) | DBG_BIT(DEBUG);

#define SMMU_DPRINTF(lvl, fmt, ...)     do {            \
        if (dbg_bits & DBG_BIT(lvl))                    \
            fprintf(stderr, "(smmu)%s: " fmt ,          \
                    __func__, ## __VA_ARGS__);          \
    } while (0)

#define HERE() fprintf(stderr, "HERE ====> %s:%d\n", __func__, __LINE__)
#else
#define SMMU_DPRINTF(lvl, fmt, ...)
#endif

#define TYPE_SMMU_DEV "smmuv3"
#define SMMU_DEV(obj) OBJECT_CHECK(SMMUState, (obj), TYPE_SMMU_DEV)

#define SMMU_SYS_DEV(obj) OBJECT_CHECK(SMMUSysState, (obj), TYPE_SMMU_DEV)
#define SMMU_PCI_DEV(obj) OBJECT_CHECK(SMMUPciState, (obj), TYPE_SMMU_DEV)

struct SMMUQueue {
    hwaddr    base;
    uint32_t  size;

    uint16_t  prod;
    uint16_t  cons;
};

typedef struct SMMUState {
    uint8_t          regs[SMMU_NREGS * sizeof(uint32_t)];
    uint32_t         cid[4];    /* Coresight registers */
    uint32_t         pid[8];    /* ---"---- */

    qemu_irq         irq;
    uint32_t         version;

    bool             virtual;
    bool             in_line;

    struct SMMUQueue cmdq, evtq;

#define SMMU_FEATURE_2LVL_STE   (1<<0)
    struct {
        uint32_t features;
        uint16_t sid_size;
        uint16_t sid_split;
        uint64_t strtab_base;
    };

    MemoryRegion     iomem;

    MemoryRegion     iommu;
    AddressSpace     iommu_as;
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

static inline int smmu_enabled(SMMUState *s)
{
    return s->regs[SMMU_REG_CR0] & SMMU_CR0_SMMU_ENABLE;
}

static inline int smmu_q_enabled(SMMUState *s, uint32_t q)
{
    return s->regs[SMMU_REG_CR0] & q;
}

#define smmu_cmd_q_enabled(s) smmu_q_enabled(s, SMMU_CR0_CMDQ_ENABLE)
#define smmu_evt_q_enabled(s) smmu_q_enabled(s, SMMU_CR0_EVTQ_ENABLE)

static uint32_t smmu_read32_reg(SMMUState *s, uint32_t reg)
{
    uint64_t *ptr;
    ptr = (uint64_t *)&s->regs[reg];
    return *ptr;
}

static void smmu_write32_reg(SMMUState *s, uint32_t reg, uint32_t val)
{
    uint32_t *ptr;
    ptr = (uint32_t *)&s->regs[reg];
    *ptr = val;
}

static uint64_t smmu_read64_reg(SMMUState *s, uint64_t reg)
{
    uint64_t *ptr;
    ptr = (uint64_t *)&s->regs[reg];
    return *ptr;
}
static void smmu_write64_reg(SMMUState *s, uint64_t reg, uint64_t val)
{
    uint64_t *ptr;
    ptr = (uint64_t *)&s->regs[reg];
    *ptr = val;
}

static uint16_t smmu_find_sid(DeviceState *dev)
{
    PCIDevice *pcidev = container_of(dev, PCIDevice, qdev);
    return pci_get_arid(pcidev);
}

static int smmu_walk_pgtable(SMMUState *s, ste_t *ste, cd_t *cd,
                             IOMMUTLBEntry *tlbe)
{
    uint64_t ttbr = 0;
    int retval = 0;

    switch(STE_CONFIG(ste)) {
    case STE_CONFIG_S1BY_S2BY:  /* No Change */
        return 0;
    case STE_CONFIG_S1TR_S2BY:
        ttbr = CD_TTB0(cd);
        ttbr = CD_TTB1(cd);
        break;
    default:
        ttbr = STE_S2TTB(ste);
        SMMU_DPRINTF(DEBUG, "Other PageTable Walks not supported\n");
        break;
    }

    if (ttbr) {

    }

    return retval;
}

static cd_t *smmu_get_cd(SMMUState *s, ste_t *ste, uint32_t ssid)
{
    cd_t *cd = NULL;

    if (STE_S1CDMAX(ste) != 0) {
        SMMU_DPRINTF(CRIT, "Multilevel Ctx Descriptor not supported yet\n");
    } else
        cd = (cd_t *)STE_CTXPTR(ste);

    return cd;
}

#define STM2U64(stm) ({                                 \
            uint64_t hi, lo;                            \
            hi = (stm)->word[1];                        \
            lo = (stm)->word[0] & ~(uint64_t)0x1f;      \
            hi << 32 | lo;                              \
        })

#define STMSPAN(stm) (stm)->word[0] & 0x1f;

static int smmu_find_ste(SMMUState *s, uint16_t sid, ste_t *ste)
{
    int l1_ste_offset = sid, l2_ste_offset;
    hwaddr addr;

    if (s->features & SMMU_FEATURE_2LVL_STE) {
        int span;

        l1_ste_offset = sid >> (s->sid_size - s->sid_split);
        l2_ste_offset = sid & ~(1 << s->sid_split);

        stm_t *stm = (stm_t *)(s->strtab_base + l1_ste_offset * sizeof(stm_t));

        span = STMSPAN(stm);
        if (l2_ste_offset > span)
            return SMMU_EVT_C_BAD_STE;

        addr = STM2U64(stm) + l2_ste_offset * sizeof(ste_t);
    } else
        addr = s->strtab_base + l1_ste_offset * sizeof(ste_t);

    if (smmu_read_sysmem(s, addr, ste, sizeof(ste_t)))
        return SMMU_EVT_F_UUT;

    return 0;
}

static IOMMUTLBEntry
smmu_translate_dev(DeviceState *dev, MemoryRegion *iommu,
                   hwaddr addr, bool is_write)
{
    int error = 0;
    uint16_t sid;
    ste_t ste;
    cd_t *cd;
    SMMUState *s = container_of(iommu, SMMUState, iommu);

    IOMMUTLBEntry ret = {
        .target_as = &address_space_memory,
        .iova = addr,
        .translated_addr = addr,
        .addr_mask = TARGET_PAGE_MASK,
        .perm = IOMMU_NONE,
    };

    /* We allow traffic through if SMMU is disabled */
    if (!smmu_enabled(s)) {
        SMMU_DPRINTF(CRIT, "SMMU Not enabled\n");
        goto bypass;
    }

    sid = smmu_find_sid(dev);
    SMMU_DPRINTF(CRIT, "SID:%x\n", sid);

    error = smmu_find_ste(s, sid, &ste);
    if (error | !STE_VALID(&ste)) {
        error = SMMU_EVT_C_BAD_STE;
        goto error_out;
    }

    switch(STE_CONFIG(&ste)) {
    case STE_CONFIG_S1BY_S2BY:
        goto bypass;
    case STE_CONFIG_S1TR_S2BY:
        cd = smmu_get_cd(s, &ste, 0); /* We dont have SSID, so 0 */
        cd = cd;
        break;
    case STE_CONFIG_S1BY_S2TR:
    case STE_CONFIG_S1TR_S2TR:
    default:
        SMMU_DPRINTF(CRIT, "Unsupported STE Configuration\n");
        goto out;
        break;
    }

    error = smmu_walk_pgtable(s, &ste, cd, &ret);

error_out:
    if (error) {        /* Post the Error using Event Q */
        SMMU_DPRINTF(CRIT, "Translation Error: %x\n", error);
        smmu_create_event(s, MIOMMUTLBEntry, error);
        goto out;
    }

bypass:
    /* After All check is done, we do allow Read/Write */
    ret.perm = is_write ? IOMMU_WO: IOMMU_RO;

out:
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

static uint64_t smmu_read_mmio(void *opaque, hwaddr addr,
                              unsigned size)
{
    SMMUState *s = opaque;
    uint64_t val;

    /* Primecell/Corelink ID registers */
    switch (addr) {
    case 0xFF0 ... 0xFFC:
        val = (uint64_t)s->cid[(addr - 0xFF0)>>2]; break;

    case 0xFDC ... 0xFE4:
        val = (uint64_t)s->pid[(addr - 0xFDC)>>2]; break;

    default:
    case SMMU_REG_IDR0 ... SMMU_REG_GERROR_IRQ_CFG1:
        val = (uint64_t)smmu_read32_reg(s, addr); break;

    case SMMU_REG_STRTAB_BASE ... SMMU_REG_CMDQ_BASE:
    case SMMU_REG_EVTQ_BASE:
    case SMMU_REG_PRIQ_BASE ... SMMU_REG_PRIQ_IRQ_CFG1:
        val = smmu_read64_reg(s, addr); break;
    }

    SMMU_DPRINTF(CRIT, "addr: %lx val:%lx\n",addr, val);
    return val;
}

static int smmu_read_sysmem(SMMUState *s, hwaddr addr,
                            void *buf, dma_addr_t len)
{
    return address_space_rw(&s->iommu_as, addr, buf,
                            len, false);

}

static int smmu_evtq_update(SMMUState *s)
{
    return 0;
}

#define SMMU_CMDQ_ERR(s) (smmu_read32_reg(s, SMMU_REG_GERROR) & \
                          SMMU_GERROR_CMDQ)

static int smmu_cmdq_consume(SMMUState *s)
{
    struct SMMUQueue *q = &s->cmdq;
    uint64_t head, tail;
    uint32_t error = SMMU_CMD_ERR_NONE;
    bool wrap = false;

    head = smmu_read32_reg(s, SMMU_REG_CMDQ_PROD);
    tail = smmu_read32_reg(s, SMMU_REG_CMDQ_CONS);

    head &= ~(1 << q->size);
    tail &= ~(1 << q->size);

    SMMU_DPRINTF(DEBUG, "q->size:%d head:%lx, tail:%lx\n", q->size, head, tail);

    while (!SMMU_CMDQ_ERR(s) && (tail != head)) {
        cmd_t cmd;
        hwaddr addr = q->base + (sizeof(cmd) * tail);

        if (smmu_read_sysmem(s, addr, &cmd, sizeof(cmd))) {
            error = SMMU_CMD_ERR_ABORT;
            goto out_while;
        }

        SMMU_DPRINTF(DEBUG, "Reading Command @idx:%lx\n", tail);
        dump_cmd(&cmd);

        switch(CMD_TYPE(&cmd)) {
        case SMMU_CMD_CFGI_STE:
        case SMMU_CMD_CFGI_STE_RANGE:
            break;
        case SMMU_CMD_TLBI_NSNH_ALL: /* TLB is not  */
        case SMMU_CMD_TLBI_EL2_ALL:  /* implemented */
        case SMMU_CMD_TLBI_EL3_ALL:
        case SMMU_CMD_TLBI_NH_ALL:
            break;
        case SMMU_CMD_SYNC:
            break;
        default:
            {                /* Check if we are coming here */
                int i;
                /*    Actually we should interrupt  */
                SMMU_DPRINTF(CRIT, "Unknown Command type: %lx, ignoring\n",
                             CMD_TYPE(&cmd));
                for (i = 0; i < ARRAY_SIZE(cmd.word); i++) {
                    SMMU_DPRINTF(CRIT, "CMD[%2d]: %#010x\t CMD[%2d]: %#010x\n",
                                 i, cmd.word[i], i+1, cmd.word[i+1]);
                    i++;
                }
            }
            error = SMMU_CMD_ERR_ILLEGAL;
            goto out_while;
        }

        tail++;
        if (tail == (1 << q->size)) {
            SMMU_DPRINTF(CRIT, "TAIL: wrapped\n");
            tail = 0;
            wrap = true;
        }
    }

out_while:

    if (error) {
        uint32_t gerror = smmu_read32_reg(s, SMMU_REG_GERROR);
        gerror |= SMMU_GERROR_CMDQ;
        smmu_write32_reg(s, SMMU_REG_CMDQ_CONS, tail);
        tail |= error << 24;
    }

    if (wrap)
        tail |= 1 << (q->size + 1);

    SMMU_DPRINTF(DEBUG, "head:%lx, tail:%lx\n", head, tail);

    /* Update tail pointer */
    smmu_write32_reg(s, SMMU_REG_CMDQ_CONS, tail);

    return 0;
}

static void smmu_update(SMMUState *s)
{
    /* Sanity check, do nothing if not enabled */
    if (!smmu_enabled(s)) {
        SMMU_DPRINTF(CRIT, "NOT ENABLED\n");
        return;
    }

    /* EVENT Q updates takes more priority */
    if (smmu_evt_q_enabled(s) && smmu_evtq_update(s)) {
    }

    if (smmu_cmd_q_enabled(s) && smmu_cmdq_consume(s)) {
        /* If Command can't be consumed, should be reported as an
         * Event on EVENT queue
         */
        //smmu_report_event(s, );
    }
}

static void smmu_write_mmio(void *opaque, hwaddr addr,
                           uint64_t val, unsigned size)
{
    SMMUState *s = opaque;
    struct SMMUQueue *q = NULL;
    bool check_queue = false;
    uint32_t val32 = (uint32_t)val;

    SMMU_DPRINTF(DEBUG, "reg:%lx cur: %lx new: %lx\n", addr,
                 smmu_read_mmio(opaque, addr, size), val);

    switch (addr) {
    case SMMU_REG_IRQ_CTRL:     /* Update the ACK as well */
    case SMMU_REG_CR0:
        smmu_write32_reg(s, addr, val32);
        smmu_write32_reg(s, addr + 4, val32);
        return;
    case SMMU_REG_GERRORN:
        if (smmu_read32_reg(s, SMMU_REG_GERROR) == val32)
            qemu_irq_lower(s->irq);

        smmu_write32_reg(s, SMMU_REG_GERROR, val32);
        break;
    case SMMU_REG_CMDQ_BASE ... SMMU_REG_CMDQ_CONS:
        q = &s->cmdq; break;
    case SMMU_REG_EVTQ_BASE ... SMMU_REG_EVTQ_IRQ_CFG1:
        q = &s->evtq; break;
    case SMMU_REG_STRTAB_BASE:
        break;
    case SMMU_REG_STRTAB_BASE_CFG:
        if (((val32 >> 16) & 0x3) == 0x1) {
            s->sid_split = (val32 >> 6) & 0x1f;
            s->features |= SMMU_FEATURE_2LVL_STE;
        }
        break;
    case SMMU_REG_PRIQ_BASE ... SMMU_REG_PRIQ_IRQ_CFG1:
        SMMU_DPRINTF(CRIT, "Trying to write to PRIQ, not implemented\n");
        break;
    default:
    case SMMU_REG_STATUSR:
    case 0xFDC ... 0xFFC:
    case SMMU_REG_IDR0 ... SMMU_REG_IDR5:
        SMMU_DPRINTF(CRIT, "Trying to write to read-only register %lx\n", addr);
        return;
    }

    switch (addr) {
    case SMMU_REG_STRTAB_BASE:
    case SMMU_REG_STRTAB_BASE_CFG:
    case SMMU_REG_CMDQ_BASE:
    case SMMU_REG_EVTQ_IRQ_CFG0:
    case SMMU_REG_EVTQ_IRQ_CFG1:
    case SMMU_REG_PRIQ_BASE:
        smmu_write64_reg(s, addr, val); break;
    default:
        smmu_write32_reg(s, addr, val32); break;
    }

    if (q) {
        switch (addr)  {
        case SMMU_REG_CMDQ_BASE:
        case SMMU_REG_EVTQ_BASE:
            q->base = val & ~0x1f;
            q->size = val & 0x1f;
            break;
        case SMMU_REG_CMDQ_PROD:
            check_queue = 1;
        case SMMU_REG_EVTQ_PROD:
            q->prod = val & ~(1 << q->size);
            break;
        case SMMU_REG_EVTQ_CONS:
            check_queue = 1;
        case SMMU_REG_CMDQ_CONS:
            q->cons = val & ~(1 << q->size);
            break;
        }
    }

    if (check_queue) {
        if (smmu_cmd_q_enabled(s) && smmu_cmdq_consume(s)) {
            /* Result in Error if not returned with 0 */
        }

        smmu_update(s);
    }
}

static const MemoryRegionOps smmu_mem_ops = {
    .read = smmu_read_mmio,
    .write = smmu_write_mmio,
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
        VMSTATE_UINT8_ARRAY(regs, SMMUState, SMMU_NREGS * sizeof(uint32_t)),
        VMSTATE_END_OF_LIST(),
    }
};

static void _smmu_populate_regs(SMMUState *s)
{
    int i;
    uint32_t val;

    /* Primecell ID registers */
    s->cid[0] = 0x0D;
    s->cid[1] = 0xF0;
    s->cid[2] = 0x05;
    s->cid[3] = 0xB1;

    for (i = 0; i < sizeof(s->pid)/sizeof(s->pid[0]); i++)
        s->pid[i] = 0x1;

    /* Only IDR0-5 will show what features supported */
    val =
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

    smmu_write32_reg(s, SMMU_REG_IDR0, val);

#define SMMU_QUEUE_SIZE_LOG2 19

    val =
        1 << 27 |                   /* Attr Types override */
        SMMU_QUEUE_SIZE_LOG2 << 21| /* Cmd Q size */
        SMMU_QUEUE_SIZE_LOG2 << 16| /* Event Q size */
        SMMU_QUEUE_SIZE_LOG2 << 11| /* PRI Q size */
        0  << 6 |                   /* SSID not supported */
        16 << 0 ;                   /* SID size  */

    smmu_write32_reg(s, SMMU_REG_IDR1, val);

    val =
        1 << 6 |                    /* Granule 16K */
        1 << 4 |                    /* Granule 4K */
        4 << 0;                     /* OAS = 44 bits */

    smmu_write32_reg(s, SMMU_REG_IDR5, val);

    s->cmdq.size = (smmu_read32_reg(s, SMMU_REG_IDR1) >> 21) & 0x1f;
    s->evtq.size = (smmu_read32_reg(s, SMMU_REG_IDR1) >> 16) & 0x1f;
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
