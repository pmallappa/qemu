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

enum {SMMU_DBG_PANIC, SMMU_DBG_CRIT, SMMU_DBG_WARN,
      SMMU_DBG_DEBUG, SMMU_DBG_DEBUG1, SMMU_DBG_DEBUG2, SMMU_DBG_INFO};

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
#define SMMU_PCI_DEV(obj) OBJECT_CHECK(SMMUDevice, (obj), TYPE_SMMU_DEV)

typedef struct {
    hwaddr   base;

    uint32_t prod;
    uint32_t cons;

    union {
        struct {
            uint8_t prod:1;
            uint8_t cons:1;
        };
        uint8_t unused;
    }wrap;

    uint16_t entries;           /* Number of entries */
    uint8_t  ent_size;          /* Size of entry in bytes */
    uint8_t  shift;             /* Size in log2 */
} SMMUQueue;

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
} SMMUState;

typedef struct SMMUSysState {
    /* <private> */
    SysBusDevice  dev;
    SMMUState     smmu_state;
} SMMUSysState;

#define Q_ENTRY(q, idx) (q->base + q->ent_size * idx)
#define Q_WRAP(q, pc) ((pc) >> (q)->shift)
#define Q_IDX(q, pc) ((pc) & __SMMU_MASK((q)->shift))

typedef enum {
    CMD_Q_EMPTY,
    CMD_Q_FULL,
    CMD_Q_INUSE,
} SMMUQStatus;

static inline SMMUQStatus
__queue_status(SMMUState *s, SMMUQueue *q)
{
    if (q->prod == q->cons && q->wrap.prod != q->wrap.cons)
        return CMD_Q_FULL;
    else if (q->prod == q->cons && q->wrap.prod == q->wrap.cons)
        return CMD_Q_EMPTY;

    return CMD_Q_INUSE;
}
#define smmu_is_q_full(s, q) (__queue_status(s, q) == CMD_Q_FULL)
#define smmu_is_q_empty(s, q) (__queue_status(s, q) == CMD_Q_EMPTY)

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
#define smmu_cmd_q_enabled(s) smmu_q_enabled(s, SMMU_CR0_CMDQ_ENABLE)
#define smmu_evt_q_enabled(s) smmu_q_enabled(s, SMMU_CR0_EVTQ_ENABLE)

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

static MemTxResult smmu_read_sysmem(SMMUState *s, hwaddr addr,
                                    void *buf, dma_addr_t len)
{
    return address_space_rw(&address_space_memory, addr,
                            MEMTXATTRS_UNSPECIFIED, buf, len, false);
}

static MemTxResult smmu_write_sysmem(SMMUState *s, hwaddr addr,
                                     void *buf, dma_addr_t len)
{
    return address_space_rw(&address_space_memory, addr,
                            MEMTXATTRS_UNSPECIFIED, buf, len, true);

}

/*
 * Events created on the EventQ
 */
static void smmu_create_event(SMMUState *s, hwaddr iova,
                              uint32_t sid, bool is_write, int error)
{
    SMMUQueue *q = &s->evtq;
    uint64_t head = Q_IDX(q, q->prod);
    bool overflow = true;
    Evt evt;

    if (!smmu_evt_q_enabled(s))
        overflow = true; goto set_overflow;

    if (!smmu_is_q_full(s, &s->evtq))
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

    SMMU_DPRINTF(DEBUG2, "addr: %lx val:%lx\n",addr, val);
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
    uint64_t val = 0;
    uint32_t error = SMMU_CMD_ERR_NONE;

    while (!SMMU_CMDQ_ERR(s) && !smmu_is_q_empty(s, &s->cmdq)) {
        Cmd cmd;
        hwaddr addr;

        addr = q->base + (sizeof(cmd) * q->cons);

        if (smmu_read_sysmem(s, addr, &cmd, sizeof(cmd)) != MEMTX_OK) {
            error = SMMU_CMD_ERR_ABORT;
            goto out_while;
        }

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
        case SMMU_CMD_PREFETCH_CONFIG:
            break;
        case SMMU_CMD_TLBI_NH_ASID:
        case SMMU_CMD_TLBI_NH_VA:   /* too many of this is sent */
            break;
        default:
            { /* Actually we should interrupt  */
                SMMU_DPRINTF(CRIT, "Unknown Command type: %x, ignoring\n",
                             CMD_TYPE(&cmd));
                dump_cmd(&cmd);
                error = SMMU_CMD_ERR_ILLEGAL;
            }
            break; //goto out_while; // should be enabled at later stage
        }

        q->cons++;
        if (q->cons == q->entries) {
            q->cons = 0;
            q->wrap.cons++;     /* this will toggle */
        }
    }

out_while:
    if (error) {
        uint32_t gerror = smmu_read32_reg(s, SMMU_REG_GERROR);
        gerror |= SMMU_GERROR_CMDQ;
        smmu_write32_reg(s, SMMU_REG_GERROR, gerror);
        val |= error << 24;
    }
    val |= (q->wrap.cons << q->shift) | q->cons;

    SMMU_DPRINTF(DEBUG2, "prod_wrap:%d, prod:%x cons_wrap:%d cons:%x\n",
                 s->cmdq.wrap.prod, s->cmdq.prod,
                 s->cmdq.wrap.cons, s->cmdq.cons);

    /* Update consumer pointer */
    smmu_write32_reg(s, SMMU_REG_CMDQ_CONS, val);

    return 0;
}

static void smmu_update(SMMUState *s)
{
    int error = 0;

    /* SMMU starts processing commands even when not enabled */
    if (!smmu_enabled(s))
        goto check_cmdq;

    /* EVENT Q updates takes more priority */
    if ((smmu_evt_q_enabled(s)))
        error = smmu_evtq_update(s);

    if (error)
        smmu_create_event(s, 0, 0, 0, error);

check_cmdq:
    if (smmu_cmd_q_enabled(s) && !SMMU_CMDQ_ERR(s)) {
        smmu_cmdq_consume(s);
    }
}

static inline void
smmu_update_q(SMMUState *s, SMMUQueue *q, uint32_t val, uint32_t off)
{
    bool update = false;

    switch (off) {
    case SMMU_REG_CMDQ_BASE:
    case SMMU_REG_EVTQ_BASE:
        q->base = val & ~0x1f;
        q->shift = val & 0x1f;
        q->entries = 1 << (q->shift);
        break;
    case SMMU_REG_CMDQ_PROD:
        update = 1;
    case SMMU_REG_EVTQ_PROD:
        q->prod = Q_IDX(q, val);
        q->wrap.prod = val >> q->shift;
        break;
    case SMMU_REG_EVTQ_CONS:
    case SMMU_REG_CMDQ_CONS:
        q->cons = Q_IDX(q, val);
        q->wrap.cons = val >> q->shift;
        break;
    }

    if (update)
        smmu_update(s);
}

static void smmu_write_mmio(void *opaque, hwaddr addr,
                            uint64_t val, unsigned size)
{
    SMMUState *s = opaque;
    SMMUQueue *q = NULL;
    bool update_queue = false;
    uint32_t val32 = (uint32_t)val;
    bool is64 = false;
    int i;

    SMMU_DPRINTF(DEBUG2, "reg:%lx cur: %lx new: %lx\n", addr,
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
    case SMMU_REG_CMDQ_PROD:
    case SMMU_REG_CMDQ_CONS:
        q = &s->cmdq;
        update_queue = 1;
        break;
    case SMMU_REG_EVTQ_BASE:
        is64 = true;            /* fallthru */
    case SMMU_REG_EVTQ_CONS:
    case SMMU_REG_EVTQ_PROD:
        q = &s->evtq;
        update_queue = 1;
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

    if (update_queue)
        smmu_update_q(s, q, val, addr);
}

static const MemoryRegionOps smmu_mem_ops = {
    .read = smmu_read_mmio,
    .write = smmu_write_mmio,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
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
    s->cmdq.ent_size = sizeof(Cmd);
    s->evtq.entries = (smmu_read32_reg(s, SMMU_REG_IDR1) >> 16) & 0x1f;
    s->evtq.ent_size = sizeof(Evt);
}

static int smmu_init(SysBusDevice *dev)
{
    int i;
    SMMUSysState *sys = SMMU_SYS_DEV(dev);
    SMMUState *s = &sys->smmu_state;

    HERE();

    /* Register Access */
    memory_region_init_io(&s->iomem, OBJECT(sys), &smmu_mem_ops,
                          s, "smmuv3", 0x1000);

    sysbus_init_mmio(dev, &s->iomem);

    for (i = 0; i < ARRAY_SIZE(s->irq); i++)
        sysbus_init_irq(dev, &s->irq[i]);

    HERE();

    return 0;
}

static void smmu_reset(DeviceState *dev)
{
    SMMUSysState *sys = SMMU_SYS_DEV(dev);
    SMMUState *s = &sys->smmu_state;

    HERE();

    _smmu_populate_regs(s);
}

/* Dummy properties, doens't really mean anything */
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
    //dc->realize = smmu_realize;
    dc->vmsd = &vmstate_smmu;
    dc->props = smmu_properties;

}

static const TypeInfo smmu_info = {
    .name          = TYPE_SMMU_DEV,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SMMUSysState),
    .class_init    = smmu_class_init,
};

static void smmu_register_types(void)
{
    type_register_static(&smmu_info);
}

type_init(smmu_register_types)
