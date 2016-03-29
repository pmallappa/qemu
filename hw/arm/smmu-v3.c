/*
 * Copyright (C) 2014-2016 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Author: Prem Mallappa <pmallapp@broadcom.com>
 *
 */


#include "qemu/osdep.h"
#include "hw/boards.h"
#include "sysemu/sysemu.h"
#include "hw/sysbus.h"
#include "hw/pci/pci.h"
#include "exec/address-spaces.h"

#include "smmu.h"
#include "smmuv3-internal.h"

#define ARM_SMMU_DEBUG
#ifdef ARM_SMMU_DEBUG

#define HERE()  printf("%s:%d\n", __func__, __LINE__)

enum {
    SMMU_DBG_PANIC, SMMU_DBG_CRIT, SMMU_DBG_WARN, /* error level */
    SMMU_DBG_DBG1, SMMU_DBG_DBG2, SMMU_DBG_INFO, /* info level */
    SMMU_DBG_CMDQ,                               /* Just command queue */
    SMMU_DBG_STE, SMMU_DBG_CD,                   /* Specific parts STE/CD */
    SMMU_DBG_TT_1, SMMU_DBG_TT_2,                /* Translation Stage 1/2 */
    SMMU_DBG_IRQ,                                /* IRQ  */
};

#define DBG_BIT(bit)    (1 << SMMU_DBG_##bit)

#define IS_DBG_ENABLED(bit) (dbg_bits & (1 << SMMU_DBG_##bit))

#define DBG_DEFAULT  (DBG_BIT(PANIC) | DBG_BIT(CRIT) | DBG_BIT(IRQ))
#define DBG_EXTRA    (DBG_BIT(STE) | DBG_BIT(CD) | DBG_BIT(TT_1))
#define DBG_VERBOSE1 DBG_BIT(DBG1)
#define DBG_VERBOSE2 (DBG_VERBOSE1 | DBG_BIT(DBG1))
#define DBG_VERBOSE3 (DBG_VERBOSE2 | DBG_BIT(DBG2))
#define DBG_VERBOSE4 (DBG_VERBOSE3 | DBG_BIT(INFO))

static uint32_t  dbg_bits =                     \
    DBG_DEFAULT |                               \
    DBG_EXTRA |                                 \
    DBG_VERBOSE1;

#define SMMU_DPRINTF(lvl, fmt, ...)                     \
    do {                                                \
        if (dbg_bits & DBG_BIT(lvl)) {                  \
            fprintf(stderr, "(smmu)%s: " fmt ,          \
                    __func__, ## __VA_ARGS__);          \
        }                                               \
    } while (0)

#define dump_ste(...) do {} while (0)
#define dump_cd(...) do {} while (0)
#define dump_cmd(...) do {} while (0)
#else
#define IS_DBG_ENABLED(bit) false
#define SMMU_DPRINTF(lvl, fmt, ...)
#endif  /* SMMU_DEBUG */

#define SMMU_NREGS       0x200
#define PCI_DEVFN_MAX    256


typedef struct RegInfo RegInfo;

typedef void  (*post_write_t)(RegInfo *r, uint64_t addr, uint64_t val,
                              void *opaque);

struct RegInfo {
    uint64_t data;
    uint64_t rao_mask;               /* Reserved as One */
    uint64_t raz_mask;               /* Reserved as Zero */
    post_write_t post;
};

typedef struct SMMUQueue SMMUQueue;

struct SMMUQueue {
    hwaddr   base;
    uint32_t prod;
    uint32_t cons;
    union {
        struct {
            uint8_t prod:1;
            uint8_t cons:1;
        };
        uint8_t unused;
    } wrap;

    uint16_t entries;           /* Number of entries */
    uint8_t  ent_size;          /* Size of entry in bytes */
    uint8_t  shift;             /* Size in log2 */
};
#define Q_ENTRY(q, idx)  (q->base + q->ent_size * idx)
#define Q_WRAP(q, pc)    ((pc) >> (q)->shift)
#define Q_IDX(q, pc)     ((pc) & ((1 << (q)->shift) - 1))

typedef struct SMMUDevice SMMUDevice;

struct SMMUDevice {
    void         *smmu;
    PCIBus       *bus;
    int           devfn;
    MemoryRegion  iommu;
    AddressSpace  as;
};

typedef struct SMMUV3State SMMUV3State;

struct SMMUV3State {
    SMMUState     smmu_state;

#define SMMU_FEATURE_2LVL_STE (1 << 0)
    /* Local cache of most-frequently used register */
    uint32_t  features;
    uint16_t  sid_size;
    uint16_t  sid_split;
    uint64_t  strtab_base;

    RegInfo       regs[SMMU_NREGS];

    qemu_irq      irq[4];

    SMMUQueue     cmdq, evtq;

    /* IOMMU Address space */
    MemoryRegion  iommu;
    AddressSpace  iommu_as;

    SMMUDevice    pbdev[PCI_DEVFN_MAX];
};

#define SMMU_V3_DEV(obj) OBJECT_CHECK(SMMUV3State, (obj), TYPE_SMMU_V3_DEV)

static void smmu_write_reg(SMMUV3State *s, uint32_t addr, uint64_t val)
{
    RegInfo *reg = &s->regs[addr >> 2];

    reg->data = val;

    if (reg->post)
        reg->post(reg, addr, val, s);
}

#define smmu_write32_reg smmu_write_reg

static inline uint32_t smmu_read32_reg(SMMUV3State *s, uint32_t addr)
{
    RegInfo *reg = &s->regs[addr >> 2];

    return (uint32_t)reg->data;
}

static inline uint64_t smmu_read64_reg(SMMUV3State *s, uint32_t addr)
{
    RegInfo *reg = &s->regs[addr >> 2];

    return reg->data;
}

static inline MemTxResult smmu_read_sysmem(SMMUV3State *s, hwaddr addr,
                                           void *buf, int len)
{
    switch (len) {
    case 4:
        *(uint32_t *)buf = ldl_le_phys(&address_space_memory, addr);
        break;
    case 8:
        *(uint64_t *)buf = ldq_le_phys(&address_space_memory, addr);
        break;
    default:
        return address_space_rw(&address_space_memory, addr,
                                MEMTXATTRS_UNSPECIFIED, buf, len, false);
    }
    return MEMTX_OK;
}

static inline void
smmu_write_sysmem(SMMUV3State *s, hwaddr addr, void *buf, int len)
{
    switch (len) {
    case 4:
        stl_le_phys(&address_space_memory, addr, *(uint32_t *)buf);
        break;
    case 8:
        stq_le_phys(&address_space_memory, addr, *(uint64_t *)buf);
        break;
    default:
        address_space_rw(&address_space_memory, addr,
                         MEMTXATTRS_UNSPECIFIED, buf, len, true);
    }
}

static inline int smmu_enabled(SMMUV3State *s)
{
    return (smmu_read32_reg(s, SMMU_REG_CR0) & SMMU_CR0_SMMU_ENABLE) != 0;
}

typedef enum {
    CMD_Q_EMPTY,
    CMD_Q_FULL,
    CMD_Q_INUSE,
} SMMUQStatus;

static inline SMMUQStatus
__smmu_queue_status(SMMUV3State *s, SMMUQueue *q)
{
    if ((q->prod == q->cons) && (q->wrap.prod != q->wrap.cons)) {
        return CMD_Q_FULL;
    } else if ((q->prod == q->cons) && (q->wrap.prod == q->wrap.cons)) {
        return CMD_Q_EMPTY;
    }

    return CMD_Q_INUSE;
}
#define smmu_is_q_full(s, q) (__smmu_queue_status(s, q) == CMD_Q_FULL)
#define smmu_is_q_empty(s, q) (__smmu_queue_status(s, q) == CMD_Q_EMPTY)

static int __smmu_q_enabled(SMMUV3State *s, uint32_t q)
{
    return smmu_read32_reg(s, SMMU_REG_CR0) & q;
}
#define smmu_cmd_q_enabled(s) __smmu_q_enabled(s, SMMU_CR0_CMDQ_ENABLE)
#define smmu_evt_q_enabled(s) __smmu_q_enabled(s, SMMU_CR0_EVTQ_ENABLE)

static inline int __smmu_irq_enabled(SMMUV3State *s, uint32_t q)
{
    return smmu_read64_reg(s, SMMU_REG_IRQ_CTRL) & q;
}
#define smmu_evt_irq_enabled(s)                   \
    __smmu_irq_enabled(s, SMMU_IRQ_CTRL_EVENT_EN)
#define smmu_gerror_irq_enabled(s)                  \
    __smmu_irq_enabled(s, SMMU_IRQ_CTRL_GERROR_EN)
#define smmu_pri_irq_enabled(s)                 \
    __smmu_irq_enabled(s, SMMU_IRQ_CTRL_PRI_EN)

static void smmu_coresight_regs_init(SMMUV3State *sv3)
{
    SMMUState *s = SMMU_SYS_DEV(sv3);
    int i;

    /* Primecell ID registers */
    s->cid[0] = 0x0D;
    s->cid[1] = 0xF0;
    s->cid[2] = 0x05;
    s->cid[3] = 0xB1;

    for (i = 0; i < ARRAY_SIZE(s->pid); i++) {
        s->pid[i] = 0x1;
    }
}

/*
 * smmu_irq_update:
 * update corresponding register,
 * return > 0 when IRQ is supposed to be rased
 */
static int
smmu_irq_update(SMMUV3State *s, int irq, uint64_t data)
{
    uint32_t error = 0;

    switch (irq) {
    case SMMU_IRQ_EVTQ:
        if (smmu_evt_irq_enabled(s)) {
            error = SMMU_GERROR_EVENTQ;
        }
        break;
    case SMMU_IRQ_CMD_SYNC:
        if (smmu_gerror_irq_enabled(s)) {
            uint32_t err_type = (uint32_t)data;
            if (err_type) {
                uint32_t regval = smmu_read32_reg(s, SMMU_REG_CMDQ_CONS);
                smmu_write32_reg(s, SMMU_REG_CMDQ_CONS,
                                 regval | err_type << SMMU_CMD_CONS_ERR_SHIFT);
            }
            error = SMMU_GERROR_CMDQ;
        }
        break;
    case SMMU_IRQ_PRIQ:
        if (smmu_pri_irq_enabled(s)) {
            error = SMMU_GERROR_PRIQ;
        }
        break;
    }
    SMMU_DPRINTF(DBG2, "<< error:%x\n", error);
    if (error && smmu_gerror_irq_enabled(s)) {
        uint32_t val = smmu_read32_reg(s, SMMU_REG_GERROR);
        SMMU_DPRINTF(DBG2, "<<<< error:%x gerror:%x\n", error, val);
        smmu_write32_reg(s, SMMU_REG_GERROR, val ^ error);
    }
    return error;
}

static void smmu_irq_raise(SMMUV3State *s, int irq, uint64_t data)
{
    SMMU_DPRINTF(IRQ, "irq:%d\n", irq);
    if (smmu_irq_update(s, irq, data)) {
            qemu_irq_raise(s->irq[irq]);
    }
}

#define SMMU_CMDQ_ERR(s) ((smmu_read32_reg(s, SMMU_REG_GERROR) ^    \
                           smmu_read32_reg(s, SMMU_REG_GERRORN)) &  \
                          SMMU_GERROR_CMDQ)

static int smmu_cmdq_consume(SMMUV3State *s)
{
    SMMUQueue *q = &s->cmdq;
    uint64_t val = 0;
    uint32_t error = SMMU_CMD_ERR_NONE;

    SMMU_DPRINTF(CMDQ, "CMDQ_ERR: %d\n", SMMU_CMDQ_ERR(s));

    while (!SMMU_CMDQ_ERR(s) && !smmu_is_q_empty(s, &s->cmdq)) {
        Cmd cmd;
        hwaddr addr;

        addr = q->base + (sizeof(cmd) * q->cons);

        if (smmu_read_sysmem(s, addr, &cmd, sizeof(cmd)) != MEMTX_OK) {
            error = SMMU_CMD_ERR_ABORT;
            goto out_while;
        }

        switch (CMD_TYPE(&cmd)) {
        case SMMU_CMD_CFGI_STE:
        case SMMU_CMD_CFGI_STE_RANGE:
            break;
        case SMMU_CMD_TLBI_NSNH_ALL: /* TLB not implemented */
        case SMMU_CMD_TLBI_EL2_ALL:  /* Fallthrough */
        case SMMU_CMD_TLBI_EL3_ALL:
        case SMMU_CMD_TLBI_NH_ALL:
            break;
        case SMMU_CMD_SYNC:     /* Fallthrough */
            if (CMD_CS(&cmd) & CMD_SYNC_SIG_IRQ) {
                smmu_irq_raise(s, SMMU_IRQ_CMD_SYNC, SMMU_CMD_ERR_NONE);
            }
        case SMMU_CMD_PREFETCH_CONFIG:
            break;
        case SMMU_CMD_TLBI_NH_ASID:
        case SMMU_CMD_TLBI_NH_VA:   /* too many of this is sent */
            break;

        default:
            error = SMMU_CMD_ERR_ILLEGAL;
            SMMU_DPRINTF(CRIT, "Unknown Command type: %x, ignoring\n",
                         CMD_TYPE(&cmd));
            if (IS_DBG_ENABLED(CD)) {
                dump_cmd(&cmd);
            }
            break;
        }

        if (error) {
            SMMU_DPRINTF(INFO, "CMD Error\n");
            break;
        }

        q->cons++;
        if (q->cons == q->entries) {
            q->cons = 0;
            q->wrap.cons++;     /* this will toggle */
        }
    }

out_while:
    if (error) {
        smmu_irq_raise(s, SMMU_IRQ_GERROR, error);
    }
    val |= (q->wrap.cons << q->shift) | q->cons;

    SMMU_DPRINTF(CMDQ, "prod_wrap:%d, prod:%x cons_wrap:%d cons:%x\n",
                 s->cmdq.wrap.prod, s->cmdq.prod,
                 s->cmdq.wrap.cons, s->cmdq.cons);
    /* Update consumer pointer */
    smmu_write32_reg(s, SMMU_REG_CMDQ_CONS, val);

    return 0;
}

static inline bool
smmu_is_irq_pending(SMMUV3State *s, int irq)
{
    return (smmu_read32_reg(s, SMMU_REG_GERROR) ^
            smmu_read32_reg(s, SMMU_REG_GERRORN));
}

/*
 * GERROR is updated when rasing an interrupt, GERRORN will be updated
 * by s/w and should match GERROR before normal operation resumes.
 */
static void smmu_irq_clear(SMMUV3State *s, uint64_t gerrorn)
{
    int irq_new = SMMU_IRQ_GERROR;
    uint32_t toggled;

    toggled = smmu_read32_reg(s, SMMU_REG_GERRORN) ^ gerrorn;

    while (toggled) {
        //int intr = ctz32(toggled);
        qemu_irq_lower(s->irq[irq_new]);
        toggled &= toggled - 1;
    }
}

static int smmu_evtq_update(SMMUV3State *s)
{
    if (!smmu_enabled(s)) {
        return 0;
    }
    return 1;
}

static void smmu_update(SMMUV3State *s)
{
    int error = 0;

    /* SMMU starts processing commands even when not enabled */
    if (!smmu_enabled(s)) {
        goto check_cmdq;
    }
    /* EVENT Q updates takes more priority */
    if ((smmu_evt_q_enabled(s))) {
        error = smmu_evtq_update(s);
    }
    if (error) {                /* TODO: Attend this */
        //smmu_create_event(s, 0, 0, 0, error);
    }
check_cmdq:
    if (smmu_cmd_q_enabled(s) && !SMMU_CMDQ_ERR(s)) {
        smmu_cmdq_consume(s);
    }

}

static void __smmu_update_q(SMMUV3State *s, SMMUQueue *q, uint64_t val,
                            uint64_t addr)
{
    switch (addr) {
    case SMMU_REG_CMDQ_BASE:
    case SMMU_REG_EVTQ_BASE:
        q->shift = val & 0x1f;
        q->entries = 1 << (q->shift);
        break;
    case SMMU_REG_CMDQ_PROD:
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

    if (addr == SMMU_REG_CMDQ_PROD) { /* possibly new command present */
        smmu_update(s);
    }

}

static void smmu_update_q(RegInfo *r, uint64_t addr, uint64_t val,
                          void *opaque)
{
    SMMUQueue *q = NULL;
    SMMUV3State *s = opaque;

    switch (addr) {
    case SMMU_REG_CMDQ_BASE ... SMMU_REG_CMDQ_CONS:
        q = &s->cmdq;
        break;
    case SMMU_REG_EVTQ_BASE ... SMMU_REG_EVTQ_IRQ_CFG2:
        q = &s->evtq;
        break;
    default:
        SMMU_DPRINTF(CRIT, "Trying to write to not Q in %s\n", __func__);
        return;
    }

    __smmu_update_q(s, q, val, addr);
}

static void smmu_update_irq(RegInfo *r, uint64_t addr, uint64_t val,
                            void *opaque)
{
    SMMUV3State *s = opaque;

    smmu_irq_clear(s, val);

    smmu_write32_reg(s, SMMU_REG_GERRORN, val);

    SMMU_DPRINTF(IRQ, "irq pend: %d gerror:%x gerrorn:%x\n",
                 smmu_is_irq_pending(s, 0),
                 // smmu_read32_reg(s, SMMU_REG_INTERRUPT),
                 smmu_read32_reg(s, SMMU_REG_GERROR),
                 smmu_read32_reg(s, SMMU_REG_GERRORN));

    /* Clear only when no more left */
    if (!smmu_is_irq_pending(s, 0)) {
        qemu_irq_lower(s->irq[0]);
    }
}

static void smmu_update_base(RegInfo *r, uint64_t addr, uint64_t val,
                            void *opaque)
{
    SMMUV3State *s = opaque;
    uint64_t *base = NULL;

    switch (addr) {
    case SMMU_REG_STRTAB_BASE:
        base = &s->strtab_base;
        break;
    case SMMU_REG_EVTQ_BASE:
        base = &s->evtq.base;
        break;
    case SMMU_REG_CMDQ_BASE:
        base = &s->cmdq.base;
        break;
    }

    /* BIT[62], BIT[5:0] are ignored */
    *base = smmu_read64_reg(s, addr) & ~(SMMU_BASE_RA | 0x3fUL);
}

#define REG_TO_OFFSET(reg) (reg >> 2)

static RegInfo smmu_v3_regs[] = {
    [ REG_TO_OFFSET(SMMU_REG_CR0)]             = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_CR0_ACK)]         = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_CR1)]             = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_CR2)]             = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_STATUSR)]         = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_IRQ_CTRL)]        = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_IRQ_CTRL_ACK)]    = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_GERROR)]          = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_GERRORN)]         = {
        .data = 0x0,
        .post = smmu_update_irq,
    },
    [ REG_TO_OFFSET(SMMU_REG_GERROR_IRQ_CFG0)] = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_GERROR_IRQ_CFG1)] = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_GERROR_IRQ_CFG2)] = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_STRTAB_BASE)]     = {
        .post = smmu_update_base,
    },
    [ REG_TO_OFFSET(SMMU_REG_STRTAB_BASE_CFG)] = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_CMDQ_BASE)]       = {
        .post = smmu_update_base,
    },
    [ REG_TO_OFFSET(SMMU_REG_CMDQ_PROD)]       = {
        .post = smmu_update_q,
    },
    [ REG_TO_OFFSET(SMMU_REG_CMDQ_CONS)]       = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_EVTQ_BASE)]       = {
        .post = smmu_update_base,
    },
    [ REG_TO_OFFSET(SMMU_REG_EVTQ_PROD)]       = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_EVTQ_CONS)]       = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_EVTQ_IRQ_CFG0)]   = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_EVTQ_IRQ_CFG1)]   = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_EVTQ_IRQ_CFG2)]   = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_PRIQ_BASE)]       = {
        .post = smmu_update_base,
    },
    [ REG_TO_OFFSET(SMMU_REG_PRIQ_PROD)]       = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_PRIQ_CONS)]       = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_PRIQ_IRQ_CFG0)]   = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_PRIQ_IRQ_CFG1)]   = {
        .data = 0x0,
    },
    [ REG_TO_OFFSET(SMMU_REG_PRIQ_IRQ_CFG2)]   = {
        .data = 0x0,
    },
};

#define SMMU_ID_REG_INIT(s, reg, d) do {            \
        (s)->regs[REG_TO_OFFSET(reg)] = (RegInfo) { \
            .data = (d),                            \
            .rao_mask = (d),                        \
            .raz_mask = ~(d),                       \
            .post = NULL,                           \
        };                                          \
    } while(0)

static void __smmuv3_id_reg_init(SMMUV3State *s)
{
    uint32_t data =
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

    SMMU_ID_REG_INIT(s, SMMU_REG_IDR0, data);

#define SMMU_SID_SIZE         16
#define SMMU_QUEUE_SIZE_LOG2  19
    data =
        1 << 27 |                    /* Attr Types override */
        SMMU_QUEUE_SIZE_LOG2 << 21 | /* Cmd Q size */
        SMMU_QUEUE_SIZE_LOG2 << 16 | /* Event Q size */
        SMMU_QUEUE_SIZE_LOG2 << 11 | /* PRI Q size */
        0  << 6 |                    /* SSID not supported */
        SMMU_SID_SIZE << 0 ;         /* SID size  */

    SMMU_ID_REG_INIT(s, SMMU_REG_IDR1, data);

    data =
        1 << 6 |                    /* Granule 16K */
        1 << 4 |                    /* Granule 4K */
        4 << 0;                     /* OAS = 44 bits */

    SMMU_ID_REG_INIT(s, SMMU_REG_IDR5, data);

}

static void _smmuv3_regs_init(SMMUV3State *s)
{
    int i = ARRAY_SIZE(smmu_v3_regs);
    while(i--) {
        RegInfo *from = &smmu_v3_regs[i];
        RegInfo *to = &s->regs[i];
        *to = *from;
    }

    __smmuv3_id_reg_init(s);      /* Update ID regs alone */
}

static void smmuv3_regs_init(SMMUV3State *s)
{
    //RegInfo *reg = NULL;

    smmu_coresight_regs_init(s);

    _smmuv3_regs_init(s);

    s->sid_size = SMMU_SID_SIZE;

    s->cmdq.entries = (smmu_read32_reg(s, SMMU_REG_IDR1) >> 21) & 0x1f;
    s->cmdq.ent_size = sizeof(Cmd);
    s->evtq.entries = (smmu_read32_reg(s, SMMU_REG_IDR1) >> 16) & 0x1f;
    s->evtq.ent_size = sizeof(Evt);
}

typedef struct {
    SMMUBaseClass smmu_base_class;
} SMMUV3Class;

static const MemoryRegionIOMMUOps smmu_iommu_ops = {
    //.translate = smmu_translate,
};

static AddressSpace *smmu_init_pci_iommu(PCIBus *bus, void *opaque, int devfn)
{
    SMMUV3State *s = opaque;
    SMMUDevice *sdev = &s->pbdev[PCI_SLOT(devfn)];
    SMMUState *sys = SMMU_SYS_DEV(s);

    sdev->smmu = s;
    sdev->bus = bus;
    sdev->devfn = devfn;
    //sdev->as = &s->iommu_as;

    memory_region_init_iommu(&sdev->iommu, OBJECT(sys),
                             &smmu_iommu_ops, TYPE_SMMU_V3_DEV, UINT64_MAX);

    address_space_init(&sdev->as, &sdev->iommu, "smmu-as");
    //printf("returened as %p\n", sdev->as);

    //assert(sdev->as);

    // address_space_init(&sdev->as, &sdev->mr, "smmu-pci");
    return &sdev->as;
}

static void smmu_write_mmio(void *opaque, hwaddr addr,
                            uint64_t val, unsigned size)
{
    SMMUState *sys = opaque;
    SMMUV3State *s = SMMU_V3_DEV(sys);

    SMMU_DPRINTF(DBG2, "reg:%lx cur: %x new: %lx\n", addr,
                 smmu_read32_reg(s, addr), val);

    switch (addr) {
    /* Unlikely event */
    case SMMU_REG_CR0_ACK:
    case SMMU_REG_STATUSR:
    case SMMU_REG_GERROR:
    case SMMU_REG_IRQ_CTRL_ACK:
    case 0xFDC ... 0xFFC:
    case SMMU_REG_IDR0 ... SMMU_REG_IDR5:
        SMMU_DPRINTF(CRIT, "write to RO/Unimpl reg %lx val64:%lx\n",
                     addr, val);
        return;
        /* Some 64bit writes are done as if its 2 * 32-bit write */
    case SMMU_REG_STRTAB_BASE + 4:
    case SMMU_REG_EVTQ_BASE + 4:
    case SMMU_REG_CMDQ_BASE + 4: {
        uint64_t tmp;
        addr -= 4;
        tmp = smmu_read64_reg(s, addr);
        tmp &= 0xffffffff00000000ULL;
        tmp |= val;
        smmu_write_reg(s, addr, tmp);
    }
        break;
    default:
        smmu_write_reg(s, addr, val);
        break;
    }
}

static uint64_t smmu_read_mmio(void *opaque, hwaddr addr, unsigned size)
{
    SMMUState *sys = opaque;
    SMMUV3State *s = SMMU_V3_DEV(sys);
    uint64_t val;

    /* Primecell/Corelink ID registers */
    switch (addr) {
    case 0xFF0 ... 0xFFC:
        val = (uint64_t)sys->cid[(addr - 0xFF0) >> 2];
        break;

    case 0xFDC ... 0xFE4:
        val = (uint64_t)sys->pid[(addr - 0xFDC) >> 2];
        break;

    default:
    case SMMU_REG_IDR0 ... SMMU_REG_GERROR_IRQ_CFG1:
        val = (uint64_t)smmu_read32_reg(s, addr);
        break;

    case SMMU_REG_STRTAB_BASE ... SMMU_REG_CMDQ_BASE:
    case SMMU_REG_EVTQ_BASE:
    case SMMU_REG_PRIQ_BASE ... SMMU_REG_PRIQ_IRQ_CFG1:
        val = smmu_read64_reg(s, addr);
        break;
    }

    SMMU_DPRINTF(DBG2, "addr: %lx val:%lx\n", addr, val);
    return val;
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

static void smmu_init_irq(SMMUV3State *s, SysBusDevice *dev)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(s->irq); i++) {
        sysbus_init_irq(dev, &s->irq[i]);
    }
}

static void smmu_init_iommu_as(SMMUV3State *sys)
{
    SMMUState *s = SMMU_SYS_DEV(sys);
    PCIBus *pcibus = pci_find_primary_bus();

    if (pcibus) {
        SMMU_DPRINTF(CRIT, "Found PCI bus, setting up iommu\n");
        pci_setup_iommu(pcibus, smmu_init_pci_iommu, s);
    } else {
        SMMU_DPRINTF(CRIT, "Could'nt find PCI bus, SMMU is not registered\n");
    }
}

static int smmu_init(SysBusDevice *dev)
{
    SMMUState *sys = SMMU_SYS_DEV(dev);
    SMMUV3State *s = SMMU_V3_DEV(dev);
    //SMMUBaseClass *sbc = SMMU_DEVICE_GET_CLASS(s);

    //s->regs = g_malloc((SMMU_NREGS >> 2) * sizeof(struct RegInfo));

    /* Register Access */
    memory_region_init_io(&sys->iomem, OBJECT(s),
                          &smmu_mem_ops, sys, TYPE_SMMU_V3_DEV, 0x1000);

    sysbus_init_mmio(dev, &sys->iomem);

    smmu_init_irq(s, dev);

    smmu_init_iommu_as(s);

    return 0;
}

static void smmu_reset(DeviceState *dev)
{
    SMMUState *sys = SMMU_SYS_DEV(dev);
    SMMUV3State *s = SMMU_V3_DEV(dev);
    //SMMUState *s = &sys->smmu_state;
    printf("smmu_reset: %p\n", sys);
    smmuv3_regs_init(s);
}

static int smmuv3_get_reg_state(QEMUFile *f, void *pv, size_t size)
{
    RegInfo *r = pv;
    int i;
    for (i = 0; i < SMMU_NREGS; i++) {
        r[i].data = qemu_get_be64(f);
        r[i].rao_mask = qemu_get_be64(f);
        r[i].raz_mask = qemu_get_be64(f);
        r[i].post = (post_write_t)qemu_get_be64(f);
    }

    return 0;
}

static void smmuv3_put_reg_state(QEMUFile *f, void *pv, size_t size)
{
    RegInfo *r = pv;
    int i;
    for (i = 0; i < SMMU_NREGS; i++) {
        qemu_put_be64(f, r[i].data);
        qemu_put_be64(f, r[i].rao_mask);
        qemu_put_be64(f, r[i].raz_mask);
        qemu_put_be64(f, (uint64_t)r[i].post);
    }
}

static const VMStateInfo reg_state_info = {
    .name = "reg_state",
    .get = smmuv3_get_reg_state,
    .put = smmuv3_put_reg_state,
};

static int smmu_populate_internal_state(void *opaque, int version_id)
{
    /*
     * TODO: Need to restore state by re-reading registers
     */
    return 0;
}

static const VMStateDescription vmstate_smmu = {
    .name = "smmu",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = smmu_populate_internal_state,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(cid, SMMUState, 4),
        VMSTATE_UINT32_ARRAY(pid, SMMUState, 8),
        VMSTATE_ARRAY(regs, SMMUV3State, SMMU_NREGS,
                      0, reg_state_info, RegInfo),
        VMSTATE_END_OF_LIST(),
    },
};

static void smmu_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);
    //SMMUBaseClass *sbc = SMMU_DEVICE_CLASS(klass);
    //SMMUInfo *info = (SMMUInfo *)data;

    HERE();
    k->init = smmu_init;

    //sbc->info = info;

    //dc->desc = info->desc;
    dc->reset = smmu_reset;
    dc->vmsd = &vmstate_smmu;
    //dc->realize = smmu_realize;
}

static void smmu_instance_init(Object *obj)
{
    HERE();
}

static const TypeInfo smmu_base_info = {
    .name          = TYPE_SMMU_DEV_BASE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SMMUV3State),
    .class_size    = sizeof(SMMUBaseClass),
    .abstract      = true,
};

static void smmu_register_types(void)
{
    TypeInfo type_info = {
        .name = TYPE_SMMU_V3_DEV,
        .parent = TYPE_SMMU_DEV_BASE,
        .class_data = NULL,
        .class_init = smmu_class_init,
        .instance_init = smmu_instance_init,
    };

    type_register_static(&smmu_base_info);

    type_register(&type_info);
}

type_init(smmu_register_types)
