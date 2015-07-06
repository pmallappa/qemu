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

typedef struct {
    hwaddr   base;

    uint32_t prod;
    uint32_t cons;

    uint16_t entries;           /* Number of entries */
    uint8_t  ent_size;          /* Size of entry in bytes */
    uint8_t  shift;             /* Size in log2 */
} SMMUQueue;

#define Q_ENTRY(q, idx) (q->base + q->ent_size * idx)
#define Q_WRAP(q, pc) ((pc) >> (q)->shift)
#define Q_IDX(q, pc) ((pc) & __SMMU_MASK((q)->entries))

typedef struct SMMUState {
    uint8_t          regs[SMMU_NREGS * sizeof(uint32_t)];
    uint32_t         cid[4];    /* Coresight registers */
    uint32_t         pid[8];    /* ---"---- */

    qemu_irq         irq[4];
    uint32_t         version;

    bool             virtual;
    bool             in_line;

    SMMUQueue cmdq, evtq;

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

static inline int smmu_irq_enabled(SMMUState *s, uint32_t q)
{
    return s->regs[SMMU_REG_IRQ_CTRL] & q;
}
#define smmu_evt_irq_enabled(s) smmu_irq_enabled(s, SMMU_IRQ_CTRL_EVENT_EN)

static inline int smmu_q_enabled(SMMUState *s, uint32_t q)
{
    return s->regs[SMMU_REG_CR0] & q;
}

static inline int smmu_q_full(SMMUQueue *q)
{
    return (q->cons == q->prod) && (Q_WRAP(q, q->cons) != Q_WRAP(q, q->prod));
}

#define smmu_cmd_q_enabled(s) smmu_q_enabled(s, SMMU_CR0_CMDQ_ENABLE)
#define smmu_evt_q_enabled(s) smmu_q_enabled(s, SMMU_CR0_EVTQ_ENABLE)

#define smmu_evt_q_full(s) smmu_q_full(&s->evtq)
#define smmu_cmd_q_full(s) smmu_q_full(&s->cmdq)

#define smmu_gerror(s) (smmu_read32_reg(s, SMMU_REG_GERROR) ==  \
                        smmu_read32_reg(s, SMMU_REG_GERRORN))

static uint32_t smmu_read32_reg(SMMUState *s, uint32_t reg)
{
    return *(uint64_t *)&s->regs[reg];
}

static void smmu_write32_reg(SMMUState *s, uint32_t reg, uint32_t val)
{
    uint32_t *ptr;
    ptr = (uint32_t *)&s->regs[reg];
    *ptr = val;
}

static uint64_t smmu_read64_reg(SMMUState *s, uint64_t reg)
{
    return *(uint64_t *)&s->regs[reg];
}

static void smmu_write64_reg(SMMUState *s, uint64_t reg, uint64_t val)
{
    uint64_t *ptr;
    ptr = (uint64_t *)&s->regs[reg];
    *ptr = val;
}

static uint16_t smmu_get_sid(int busnum, int devfn)
{
    return  ((busnum & 0xff) << 8) | (devfn & 0x7);
}

/*
 * Heavily based on target-arm/helper.c get_phys_addr()
 * hold on to your cursings...
 */
static int smmu_walk_pgtable(SMMUState *s, ste_t *ste, cd_t *cd,
                             IOMMUTLBEntry *tlbe, bool is_write)
{
    uint64_t ttbr = 0;
    int retval = 0;
    uint32_t ste_cfg = STE_CONFIG(ste);
    hwaddr ipa, opa;          /* Input address, output address */

    SMMU_DPRINTF(DEBUG, "ste_cfg :%x\n", ste_cfg);
    /* Both Bypass, we dont need to do anything */
    if (ste_cfg == STE_CONFIG_S1BY_S2BY)
        return 0;

    ipa = tlbe->iova;
    SMMU_DPRINTF(DEBUG, "Before Stage1 Input addr: %lx\n", ipa);

    if (ste_cfg == STE_CONFIG_S1TR_S2BY || ste_cfg == STE_CONFIG_S1TR_S2TR) {
        if (CD_EPD0(cd))
            ttbr = CD_TTB1(cd);
        else
            ttbr = CD_TTB0(cd);
    }

    SMMU_DPRINTF(DEBUG, "Stage1 tanslated :%lx\n ", ttbr);

    ipa = opa;

    SMMU_DPRINTF(DEBUG, "Stage1 tanslated :%lx\n ", opa);

    if (ste_cfg == STE_CONFIG_S1BY_S2TR || ste_cfg == STE_CONFIG_S1TR_S2TR) {
        ttbr = STE_S2TTB(ste);
    }

    SMMU_DPRINTF(DEBUG, "Stage2 tanslated :%lx\n ", opa);

    tlbe->translated_addr = opa;

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

static int smmu_read_sysmem(SMMUState *s, hwaddr addr,
                            void *buf, dma_addr_t len)
{
    return address_space_read(&address_space_memory,
                              addr, buf, len);

}

static int smmu_write_sysmem(SMMUState *s, hwaddr addr,
                            void *buf, dma_addr_t len)
{
    return address_space_write(&address_space_memory,
                               addr, buf, len);

}

#define STM2U64(stm) ({                                 \
            uint64_t hi, lo;                            \
            hi = (stm)->word[1];                        \
            lo = (stm)->word[0] & ~(uint64_t)0x1f;      \
            hi << 32 | lo;                              \
        })

#define STMSPAN(stm) (1 << ((stm)->word[0] & 0x1f))

static int smmu_find_ste(SMMUState *s, uint16_t sid, ste_t *ste)
{
    hwaddr addr;

    /* Check SID range */
    if (sid > (1 << s->sid_size))
        return SMMU_EVT_C_BAD_SID;

    if (s->features & SMMU_FEATURE_2LVL_STE) {
        int span;
        hwaddr stm_addr;
        stm_t stm;
        int l1_ste_offset, l2_ste_offset;

        if (sid > (1 << s->sid_split))
            return SMMU_EVT_C_BAD_STE;

        l1_ste_offset = sid >> s->sid_split;
        l2_ste_offset = sid & ~(1 << s->sid_split);

        stm_addr = (hwaddr)(s->strtab_base + l1_ste_offset * sizeof(stm_t));
        smmu_read_sysmem(s, stm_addr, &stm, sizeof(stm));

        span = STMSPAN(&stm);

        if (l2_ste_offset > span)
            return SMMU_EVT_C_BAD_STE;

        addr = STM2U64(&stm) + l2_ste_offset * sizeof(ste_t);
    } else
        addr = s->strtab_base + sid * sizeof(ste_t);

    if (smmu_read_sysmem(s, addr, ste, sizeof(ste_t)))
        return SMMU_EVT_F_UUT;

    return 0;
}

/*
 * We notify the system using events
 */
static void smmu_create_event(SMMUState *s, hwaddr iova,
                              uint32_t sid, bool is_write, int error)
{
    SMMUQueue *q = &s->evtq;
    uint64_t head = Q_IDX(q, q->prod);
    bool overflow = true;
    evt_t evt;

    if (!smmu_evt_q_enabled(s))
        overflow = true; goto set_overflow;

    if (!smmu_evt_q_full(s))
        overflow = true; goto set_overflow;

    EVT_SET_TYPE(&evt, error);
    EVT_SET_SID(&evt, sid);

    switch (error) {
    case SMMU_EVT_F_UUT:
    case SMMU_EVT_C_BAD_STE:
    case SMMU_EVT_F_TRANS_FORBIDDEN:
    default:
        break;
    }

    EVT_SET_INPUT_ADDR(&evt, iova);

    smmu_write_sysmem(s, Q_ENTRY(q, head), &evt, sizeof(evt));

    head++;

set_overflow:
    if (overflow)
        head ^= 1 << 31;
    else if (smmu_evt_irq_enabled(s))
        qemu_irq_raise(s->irq[SMMU_IRQ_EVTQ]);

    q->prod = head;

    smmu_write32_reg(s, SMMU_REG_EVTQ_PROD, head);
}

/*
 * SMMU Spec differentiates, 3 types of transactions,
 * here we assume only ATS transaction. Others are used in an
 * inline SMMU where every transaction goes via SMMU.
 * pci_dma_read(write) only will call this function.
 */

/*
 * smmu_translate: this is called when device does a
 * pci_dma_{read,write}() or similar
 * Need to get the device's streamid which is equivalent of
 * PCI RID/ARID
 *
 */
typedef struct {
    int           busnum;
    int           devfn;
    SMMUState    *smmu;
    MemoryRegion  mr;
    AddressSpace  as;
} SMMUDevice;

static IOMMUTLBEntry
smmu_translate(MemoryRegion *iommu, hwaddr addr, bool is_write)
{
    SMMUDevice *sdev = container_of(iommu, SMMUDevice, mr);
    SMMUState *s = sdev->smmu;
    int error = 0;
    uint16_t sid = 0;
    ste_t ste;
    cd_t *cd;

    IOMMUTLBEntry ret = {
        .target_as = &address_space_memory,
        .iova = addr,
        .translated_addr = addr,
        .addr_mask = TARGET_PAGE_MASK,
        .perm = IOMMU_NONE,
    };

    SMMU_DPRINTF(INFO, "%s called for devfn:%d\n", __func__, sdev->devfn);

    /* We allow traffic through if SMMU is disabled */
    if (!smmu_enabled(s)) {
        SMMU_DPRINTF(CRIT, "SMMU Not enabled\n");
        goto bypass;
    }

    sid = smmu_get_sid(sdev->busnum, sdev->devfn);

    SMMU_DPRINTF(CRIT, "SID:%x\n", sid);

    error = smmu_find_ste(s, sid, &ste);
    if (error) goto error_out;  /* F_STE_FETCH or F_CFG_CONFLICT */

    if (!STE_VALID(&ste)) {
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

    error = smmu_walk_pgtable(s, &ste, cd, &ret, is_write);

error_out:
    if (error) {        /* Post the Error using Event Q */
        SMMU_DPRINTF(CRIT, "Translation Error: %x\n", error);
        smmu_create_event(s, ret.iova, sid, is_write, error);
        goto out;
    }

bypass:
    /* After All check is done, we do allow Read/Write */
    ret.perm = is_write ? IOMMU_WO: IOMMU_RO;

out:
    return ret;
}

static const MemoryRegionIOMMUOps smmu_ops = {
    .translate = smmu_translate,
};

/*
 * TODO: We need to keep track of all allocated struct SMMUDevice
 */
static AddressSpace *smmu_pci_iommu(PCIBus *bus, void *opaque, int devfn)
{
    SMMUState *s = opaque;
    SMMUDevice *sdev;

    HERE();

    sdev = g_malloc0(sizeof(*sdev));

    sdev->smmu = s;
    sdev->busnum = pci_bus_num(bus);

    memory_region_init_iommu(&sdev->mr, OBJECT(s),
                             &smmu_ops, "arm_smmuv3", UINT64_MAX);
    address_space_init(&sdev->as, &sdev->mr, "arm_smmuv3");

    return &sdev->as;
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

static int smmu_evtq_update(SMMUState *s)
{
    if (!smmu_enabled(s))
        return 0;

    return 1;
}

#define SMMU_CMDQ_ERR(s) (smmu_read32_reg(s, SMMU_REG_GERROR) & \
                          SMMU_GERROR_CMDQ)

static int smmu_cmdq_consume(SMMUState *s)
{
    SMMUQueue *q = &s->cmdq;
    uint64_t head, tail;
    uint32_t error = SMMU_CMD_ERR_NONE;
    bool wrap = false;

    head = smmu_read32_reg(s, SMMU_REG_CMDQ_PROD);
    //tail = smmu_read32_reg(s, SMMU_REG_CMDQ_CONS);
    q->prod = head;
    tail = Q_IDX(q, q->cons);

    SMMU_DPRINTF(DEBUG, "CMDQ size:%d head:%lx, tail:%lx\n", q->entries, head, tail);

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
                SMMU_DPRINTF(CRIT, "Unknown Command type: %x, ignoring\n",
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
        if (tail == q->entries) {
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
        tail ^= 1 << q->shift;

    SMMU_DPRINTF(DEBUG, "head:%lx, tail:%lx\n", head, tail);

    /* Update tail pointer */
    smmu_write32_reg(s, SMMU_REG_CMDQ_CONS, tail);
    q->cons = tail;

    return 0;
}

static void smmu_update(SMMUState *s)
{
    int error = 0;

    /* Sanity check, do nothing if not enabled */
    if (!smmu_enabled(s))
        SMMU_DPRINTF(CRIT, "NOT ENABLED\n");

    /* EVENT Q updates takes more priority */
    if ((smmu_evt_q_enabled(s)))
        error = smmu_evtq_update(s);

    if (error)
        smmu_create_event(s, 0, 0, 0, error);

    if (smmu_cmd_q_enabled(s)) {
        smmu_cmdq_consume(s);
    }
}

static inline void
smmu_update_q(SMMUState *s, SMMUQueue *q, uint32_t val)
{
    q->base = val & ~0x1f;
    q->shift = val & 0x1f;
    q->entries = 1 << (q->shift);
}

static void smmu_write_mmio(void *opaque, hwaddr addr,
                            uint64_t val, unsigned size)
{
    SMMUState *s = opaque;
    bool check_queue = false;
    uint32_t val32 = (uint32_t)val;
    bool is64 = false;
    int i;

    SMMU_DPRINTF(DEBUG, "reg:%lx cur: %lx new: %lx\n", addr,
                 smmu_read_mmio(opaque, addr, size), val);

    /*
     * We update the ACK registers, actual write takes place after
     * the switch-case
     */

    switch (addr) {
    case SMMU_REG_IRQ_CTRL:     /* Update the ACK as well */
        val &= 0xf;

        for (i = 0; i < 4; i++)
            if (!(val & (1 << i)))
                qemu_irq_lower(s->irq[i]);

        smmu_write32_reg(s, addr + 4, val32);
        break;

    case SMMU_REG_CR0:
        smmu_write32_reg(s, addr + 4, val32);
        smmu_update(s);         /* Start processing as soon as enabled */
        break;

    case SMMU_REG_GERRORN:
        if (smmu_read32_reg(s, SMMU_REG_GERROR) == val32)
            qemu_irq_lower(s->irq[SMMU_IRQ_GERROR]);

        smmu_write32_reg(s, SMMU_REG_GERROR, val32);
        break;

    case SMMU_REG_CMDQ_BASE:
        is64 = true;
        smmu_update_q(s, &s->cmdq, val); break;

    case SMMU_REG_EVTQ_BASE:
        is64 = true;
        smmu_update_q(s, &s->evtq, val); break;

    case SMMU_REG_CMDQ_PROD:    case SMMU_REG_EVTQ_PROD:
    case SMMU_REG_EVTQ_CONS:    case SMMU_REG_CMDQ_CONS:
        check_queue = 1;
        break;

    case SMMU_REG_STRTAB_BASE:
        is64 = true;
        break;

    case SMMU_REG_STRTAB_BASE_CFG:
        is64 = true;
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

    if (is64)
        smmu_write64_reg(s, addr, val);
    else
        smmu_write32_reg(s, addr, val32);

    if (check_queue)
        smmu_update(s);
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

#define SMMU_SID_SIZE    16
    s->sid_size = SMMU_SID_SIZE;

    val =
        1 << 27 |                   /* Attr Types override */
        SMMU_QUEUE_SIZE_LOG2 << 21| /* Cmd Q size */
        SMMU_QUEUE_SIZE_LOG2 << 16| /* Event Q size */
        SMMU_QUEUE_SIZE_LOG2 << 11| /* PRI Q size */
        0  << 6 |                   /* SSID not supported */
        SMMU_SID_SIZE << 0 ;        /* SID size  */

    smmu_write32_reg(s, SMMU_REG_IDR1, val);

    val =
        1 << 6 |                    /* Granule 16K */
        1 << 4 |                    /* Granule 4K */
        4 << 0;                     /* OAS = 44 bits */

    smmu_write32_reg(s, SMMU_REG_IDR5, val);

    s->cmdq.entries = (smmu_read32_reg(s, SMMU_REG_IDR1) >> 21) & 0x1f;
    s->cmdq.ent_size = sizeof(cmd_t);
    s->evtq.entries = (smmu_read32_reg(s, SMMU_REG_IDR1) >> 16) & 0x1f;
    s->evtq.ent_size = sizeof(evt_t);
}

static int smmu_init(SysBusDevice *dev)
{
    SMMUSysState *sys = SMMU_SYS_DEV(dev);
    SMMUState *s = &sys->smmu_state;
    PCIBus *pcibus = pci_find_primary_bus();
    int i;
    HERE();
    /* Register Access */
    memory_region_init_io(&s->iomem, OBJECT(sys), &smmu_mem_ops,
                          s, "smmuv3", 0x1000);

    /* Host memory as seen from the PCI side, via the IOMMU.  */
    _smmu_populate_regs(s);

    sysbus_init_mmio(dev, &s->iomem);

    for (i = 0; i < ARRAY_SIZE(s->irq); i++)
        sysbus_init_irq(dev, &s->irq[i]);
#if 0
    /* Host Memory Access */
    memory_region_init_iommu(&s->iommu, OBJECT(sys), &smmu_ops,
                             "smmuv3", UINT64_MAX);

    address_space_init(&s->iommu_as, &s->iommu, "smmu-as");
#endif

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
