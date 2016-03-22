/*
 * QEMU emulation for ARM SMMUv3
 * Copyright (C) 2014-2015 Broadcom Corporation
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

/*
 * - Doesn't yet consider BE/LE for data structure and translation tables
 * - LPAE Pagetables only
 * - Works with SMMUv3 driver in Linux 4.5
 * - Save/Restore not yet supported
 */
#define NODEBUG

#include "qemu/osdep.h"
#include "hw/boards.h"
#include "sysemu/sysemu.h"
#include "hw/sysbus.h"
#include "hw/pci/pci.h"
#include "exec/address-spaces.h"

#include "hw/arm/smmuv3.h"
#include "smmuv3-internal.h"

#define ARM_SMMU_DEBUG
#ifdef ARM_SMMU_DEBUG

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

#define SMMU_NREGS       0xC000
#define PCI_DEVFN_MAX    32

typedef struct SMMUQueue {
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
} SMMUQueue;
#define Q_ENTRY(q, idx) (q->base + q->ent_size * idx)
#define Q_WRAP(q, pc) ((pc) >> (q)->shift)
#define Q_IDX(q, pc) ((pc) & ((1 << (q)->shift) - 1))

typedef struct SMMUDevice {
    void             *smmu;
    PCIBus           *bus;
    int              devfn;
    MemoryRegion     iommu;
    AddressSpace     as;
} SMMUDevice;

typedef struct SMMUInfo {
    const char *name;
    const char *desc;
    SMMUImpl    impl;           /* SMMU Implementations */
} SMMUInfo;

typedef struct SMMUState {
    SMMUInfo     *info;
    uint8_t       regs[SMMU_NREGS * sizeof(uint32_t)];
    uint32_t      cid[4];       /* Coresight registers */
    uint32_t      pid[8];       /* ---"---- */

    qemu_irq      irq[4];
    uint32_t      version;

    SMMUQueue     cmdq, evtq;

#define SMMU_FEATURE_2LVL_STE (1<<0)
    struct {                    /* Group; may move to different struct */
        uint32_t  features;
        uint16_t  sid_size;
        uint16_t  sid_split;
        uint64_t  strtab_base;
    };
    /* Register space */
    MemoryRegion  iomem;

    /* IOMMU Address space */
    MemoryRegion iommu;
    AddressSpace iommu_as;

    SMMUDevice    pbdev[PCI_DEVFN_MAX];
} SMMUState;

typedef enum {
    CMD_Q_EMPTY,
    CMD_Q_FULL,
    CMD_Q_INUSE,
} SMMUQStatus;

static inline SMMUQStatus
__smmu_queue_status(SMMUState *s, SMMUQueue *q)
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

static int __smmu_q_enabled(SMMUState *s, uint32_t q)
{
    return s->regs[SMMU_REG_CR0] & q;
}
#define smmu_cmd_q_enabled(s) __smmu_q_enabled(s, SMMU_CR0_CMDQ_ENABLE)
#define smmu_evt_q_enabled(s) __smmu_q_enabled(s, SMMU_CR0_EVTQ_ENABLE)

static inline int smmu_enabled(SMMUState *s)
{
    return (s->regs[SMMU_REG_CR0] & SMMU_CR0_SMMU_ENABLE) != 0;
}

static inline int __smmu_irq_enabled(SMMUState *s, uint32_t q)
{
    return s->regs[SMMU_REG_IRQ_CTRL] & q;
}
#define smmu_evt_irq_enabled(s)                   \
    __smmu_irq_enabled(s, SMMU_IRQ_CTRL_EVENT_EN)
#define smmu_gerror_irq_enabled(s)                  \
    __smmu_irq_enabled(s, SMMU_IRQ_CTRL_GERROR_EN)
#define smmu_pri_irq_enabled(s)                 \
    __smmu_irq_enabled(s, SMMU_IRQ_CTRL_PRI_EN)

static inline uint32_t smmu_read32_reg(SMMUState *s, uint32_t reg)
{
    return *(uint32_t *)&s->regs[reg];
}

static void smmu_write32_reg(SMMUState *s, uint32_t reg, uint32_t val)
{
    uint32_t *ptr = (uint32_t *)&s->regs[reg];
    *ptr = val;
}

static inline uint64_t smmu_read64_reg(SMMUState *s, uint64_t reg)
{
    return *(uint64_t *)&s->regs[reg];
}

static inline void smmu_write64_reg(SMMUState *s, uint64_t reg, uint64_t val)
{
    uint64_t *ptr = (uint64_t *)&s->regs[reg];
    *ptr = val;
}

static inline MemTxResult smmu_read_sysmem(SMMUState *s, hwaddr addr,
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
smmu_write_sysmem(SMMUState *s, hwaddr addr, void *buf, int len)
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

typedef struct SMMUSysState {
    /* <private> */
    SysBusDevice  dev;
    SMMUState     smmu_state;
} SMMUSysState;

#define SMMU_SYS_DEV(obj) OBJECT_CHECK(SMMUSysState, (obj), TYPE_SMMU_DEV_BASE)

static inline int is_cd_valid(SMMUState *s, Ste *ste, Cd *cd)
{
    return CD_VALID(cd);
}

static inline int is_ste_valid(SMMUState *s, Ste *ste)
{
    return STE_VALID(ste);
}

static inline int is_ste_bypass(SMMUState *s, Ste *ste)
{
    return STE_CONFIG(ste) == STE_CONFIG_S1BY_S2BY;
}

static uint16_t smmu_get_sid(PCIBus *bus, int devfn)
{
    return  ((pci_bus_num(bus) & 0xff) << 8) | devfn;
}

static inline MemTxResult
__smmu_read_aligned_sysmem(SMMUState *s, hwaddr addr, void *buf,
                           int size, int len)
{
    for ( ; len; len -= size, buf += size, addr += size) {
        MemTxResult ret = smmu_read_sysmem(s, addr, buf, size);
        if (ret != MEMTX_OK) {
            return ret;
        }
    }
    return MEMTX_OK;
}

/*
 * All SMMU data structures are little endian, and are aligned to 8 bytes
 *  L1STE/STE/L1CD/CD, Queue entries in CMDQ/EVTQ/
 */
static inline int smmu_get_ste(SMMUState *s, hwaddr addr, Ste *buf)
{
    return __smmu_read_aligned_sysmem(s, addr, buf, sizeof(buf->word[0]),
                                      ARRAY_SIZE(buf->word)) != MEMTX_OK;
}

/*
 * For now we only support CD with a single entry, 'ssid' is used to identify
 * otherwise
 */
static inline int smmu_get_cd(SMMUState *s, Ste *ste, uint32_t ssid, Cd *buf)
{
    hwaddr addr = STE_CTXPTR(ste);

    if (STE_S1CDMAX(ste) != 0) {
        SMMU_DPRINTF(CRIT, "Multilevel Ctx Descriptor not supported yet\n");
    }

    return __smmu_read_aligned_sysmem(s, addr, buf, sizeof(buf->word[0]),
                                      ARRAY_SIZE(buf->word)) != MEMTX_OK;
}

static inline void
smmu_write_event(SMMUState *s, hwaddr addr, Evt *evt)
{
    const int size = 4;
    int len =  sizeof(Evt);
    void *buf = evt;

    for (; len; len -= size, buf += size, addr += size) {
        smmu_write_sysmem(s, addr, buf, size);
    }
}

#define STM2U64(stm) ({                                 \
            uint64_t hi, lo;                            \
            hi = (stm)->word[1];                        \
            lo = (stm)->word[0] & ~(uint64_t)0x1f;      \
            hi << 32 | lo;                              \
        })

#define STMSPAN(stm) (1 << (extract32((stm)->word[0], 0, 4) - 1))

static int smmu_find_ste(SMMUState *s, uint16_t sid, Ste *ste)
{
    hwaddr addr;

    SMMU_DPRINTF(STE, "SID:%x\n", sid);
    /* Check SID range */
    if (sid > (1 << s->sid_size)) {
        return SMMU_EVT_C_BAD_SID;
    }

    if (s->features & SMMU_FEATURE_2LVL_STE) {
        int span;
        hwaddr stm_addr;
        STEDesc stm;
        int l1_ste_offset, l2_ste_offset;
        SMMU_DPRINTF(STE, "no. ste: %x\n", s->sid_split);

        l1_ste_offset = sid >> s->sid_split;
        l2_ste_offset = sid & ((1 << s->sid_split) - 1);

        stm_addr = (hwaddr)(s->strtab_base + l1_ste_offset * sizeof(stm));
        smmu_read_sysmem(s, stm_addr, &stm, sizeof(stm));

        SMMU_DPRINTF(STE, "strtab_base:%lx stm_addr:%lx\n"
                     "l1_ste_offset:%x l1(64):%#016lx\n",
                     s->strtab_base, stm_addr, l1_ste_offset, STM2U64(&stm));

        span = STMSPAN(&stm);
        SMMU_DPRINTF(STE, "l2_ste_offset:%x ~ span:%d\n", l2_ste_offset, span);
        if (l2_ste_offset > span) {
            return SMMU_EVT_C_BAD_STE;
        }
        addr = STM2U64(&stm) + l2_ste_offset * sizeof(*ste);
    } else {
        addr = s->strtab_base + sid * sizeof(*ste);
    }

    if (smmu_get_ste(s, addr, ste)) {
        return SMMU_EVT_F_UUT;
    }

    return 0;
}

/*
 * STE validity as per SMMUv3 11.0
 */
static int
is_ste_consistent(SMMUState *s, Ste *ste)
{
    uint32_t _config = STE_CONFIG(ste) & 0x7,
        idr0 = s->regs[SMMU_REG_IDR0],
        idr5 = s->regs[SMMU_REG_IDR5];

    uint32_t httu = extract32(idr0, 6, 2);
    bool config[] = {_config & 0x1,
                     _config & 0x2,
                     _config & 0x3};
    bool granule_supported = (1 << STE_S2TG(ste)) & (idr5 >> 4);

    bool s1p = idr0 & SMMU_IDR0_S1P,
        s2p = idr0 & SMMU_IDR0_S2P,
        hyp = idr0 & SMMU_IDR0_HYP,
        cd2l = idr0 & SMMU_IDR0_CD2L,
        idr0_vmid = idr0 & SMMU_IDR0_VMID16,
        ats = idr0 & SMMU_IDR0_ATS,
        ttf0 = (idr0 >> 2) & 0x1,
        ttf1 = (idr0 >> 3) & 0x1;

    int ssidsz = (s->regs[SMMU_REG_IDR1] >> 6) & 0x1f;

    uint32_t ste_vmid = STE_S2VMID(ste),
        ste_eats = STE_EATS(ste),
        ste_s2s = STE_S2S(ste),
        ste_s1fmt = STE_S1FMT(ste),
        aa64 = STE_S2AA64(ste),
        ste_s1cdmax = STE_S1CDMAX(ste);

    uint8_t ste_strw = STE_STRW(ste);
    uint64_t oas, max_pa;
    bool strw_ign;
    bool addr_out_of_range;

    if (!STE_VALID(ste)) {
        return false;
    }

    if (!config[2]) {
        if ((!s1p && config[0]) ||
            (!s2p && config[1]) ||
            (s2p && config[1])) {
            return false;
        }
        if (!ssidsz && ste_s1cdmax && config[0] && !cd2l &&
            (ste_s1fmt == 1 || ste_s1fmt == 2)) {
            return false;
        }
        if (ats && ((_config & 0x3) == 0) &&
            ((ste_eats == 2 && (_config != 0x7 || ste_s2s)) ||
             (ste_eats == 1 && !ste_s2s))) {
            return false;
        }
        if (config[0] && (ssidsz && (ste_s1cdmax > ssidsz))) {
            return false;
        }
    }

    oas = MIN(STE_S2PS(ste), idr5 & 0x7);

    if (oas == 3) {
        max_pa = ~(1UL << 42);
    } else {
        max_pa = ~(1UL << (32 + (oas * 4)));
    }

    strw_ign = (!s1p || !hyp || (_config == 4));

    addr_out_of_range = (int64_t)(max_pa - STE_S2TTB(ste)) < 0;

    if (config[1] &&
        (!granule_supported || addr_out_of_range ||
         (!aa64 && !ttf0) || (aa64 && ttf1)  ||
         ((STE_S2HA(ste) || STE_S2HD(ste)) && !aa64) ||
         ((STE_S2HA(ste) || STE_S2HD(ste)) && !httu) ||
         (STE_S2HD(ste) && httu))) {
        return false;
    }
    if (s2p && (config[0] == 0 && config[1]) &&
        (strw_ign || !ste_strw) && !idr0_vmid && !(ste_vmid>>8)) {
        return false;
    }

    return true;
}

static int tg2granule(int bits, bool tg1)
{
    switch (bits) {
    case 1:
        return tg1 ? 14 : 16;
    case 2:
        return tg1 ? 14 : 12;
    case 3:
        return tg1 ? 16 : 12;
    default:
        return 12;
    }
}

static inline int oas2bits(int oas)
{
    switch (oas) {
    case 2:
        return 40;
    case 3:
        return 42;
    case 4:
        return 44;
    case 5:
    default: return 48;
    }
}

typedef struct SMMUCfg {
    union {
        hwaddr va;              /* Input to S1 */
        hwaddr ipa;             /* Input to S2 */
    };
    uint32_t oas;
    uint32_t tsz;
    uint64_t ttbr;
    uint32_t granule;
    uint32_t va_size;
    uint32_t granule_sz;

    union {
        hwaddr opa;             /* Output from S2 */
        hwaddr pa;              /* Output from S1, Final PA */
    };

    struct SMMUCfg *s2cfg;
} SMMUCfg;

static void smmu_cfg_populate_s2(Ste *ste, SMMUCfg *cfg)
{                           /* stage 2 cfg */
    bool s2a64 = STE_S2AA64(ste);

    cfg->granule = STE_S2TG(ste);
    cfg->tsz = STE_S2T0SZ(ste);
    cfg->ttbr = STE_S2TTB(ste);
    cfg->oas = oas2bits(STE_S2PS(ste));

    if (s2a64) {
        cfg->tsz = MIN(cfg->tsz, 39);
        cfg->tsz = MAX(cfg->tsz, 16);
    }
    cfg->va_size = STE_S2AA64(ste) ? 64 : 32;
    cfg->granule_sz = tg2granule(cfg->granule, 0) - 3;
}

static void smmu_cfg_populate_s1(Cd *cd, SMMUCfg *cfg)
{                           /* stage 1 cfg */
    bool s1a64 = CD_AARCH64(cd);

    cfg->granule = (CD_EPD0(cd)) ? CD_TG1(cd) : CD_TG0(cd);
    cfg->tsz = (CD_EPD0(cd)) ? CD_T1SZ(cd) : CD_T0SZ(cd);
    cfg->ttbr = (CD_EPD0(cd)) ? CD_TTB1(cd) : CD_TTB0(cd);
    cfg->oas = oas2bits(CD_IPS(cd));

    if (s1a64) {
        cfg->tsz = MIN(cfg->tsz, 39);
        cfg->tsz = MAX(cfg->tsz, 16);
    }
    cfg->va_size = CD_AARCH64(cd) ? 64 : 32;
    cfg->granule_sz = tg2granule(cfg->granule, CD_EPD0(cd)) - 3;
}

static SMMUEvtErr
smmu_get_phys_addr(SMMUState *s, SMMUCfg *cfg, Ste *ste,
                   Cd *cd, uint32_t *pagesize, uint32_t *perm,
                   bool stage2, bool is_write)
{
    int     ret, level;
    int     granule_sz = cfg->granule_sz;
    int     va_size = cfg->va_size;
    hwaddr  va, addr, mask;
    hwaddr *outaddr;
    bool    s2needed   = false;

    va = addr = cfg->va;        /* or ipa in Stage2 */

    s2needed = !stage2 && (STE_CONFIG(ste) == STE_CONFIG_S1TR_S2TR);
    if (s2needed) {
        smmu_cfg_populate_s2(ste, cfg->s2cfg);
    }

    assert(va_size == 64);      /* We dont support 32-bit yet */

    outaddr = stage2 ? &cfg->opa : &cfg->pa; /* same location, for clearity */

    level = 4 - (va_size - cfg->tsz - 4) / granule_sz;

    mask = (1ULL << (granule_sz + 3)) - 1;

    addr = extract64(cfg->ttbr, 0, 48);
    addr &= ~((1ULL << (va_size - cfg->tsz - (granule_sz * (4 - level)))) - 1);

    for (;;) {
        uint64_t desc;

        addr |= (va >> (granule_sz * (4 - level))) & mask;
        addr &= ~7ULL;

        if (smmu_read_sysmem(s, addr, &desc, sizeof(desc))) {
            ret = SMMU_EVT_F_WALK_EXT_ABRT;
            SMMU_DPRINTF(CRIT, "Translation table read error lvl:%d\n", level);
            break;
        }
        SMMU_DPRINTF(TT_1,
                     "Level: %d granule_sz:%d mask:%lx addr:%lx desc:%lx\n",
                     level, granule_sz, mask, addr, desc);

        if (!(desc & 1) ||
            (!(desc & 2) & (level == 3))) {
            ret = SMMU_EVT_F_TRANS;
            break;
        }

        if (s2needed) {   /* We call again to resolve address at this 'level' */
            uint32_t unused1, unused2 ATTRIBUTE_UNUSED;
            SMMUCfg *s2cfg = cfg->s2cfg;
            s2cfg->ipa = desc;
            ret = smmu_get_phys_addr(s, s2cfg, ste, cd, &unused1,
                                     &unused2, true, is_write);
            if (ret) {
                break;
            }
            desc = (uint64_t)s2cfg->opa;
        }

        addr = desc & 0xfffffff000ULL;
        if ((desc & 2) && (level < 3)) {
            level++;
            continue;
        }
        *pagesize = (1ULL << ((granule_sz * (4 - level)) + 3));
        addr |= (va & (*pagesize - 1));
        SMMU_DPRINTF(TT_1, "addr:%lx pagesize:%x\n", addr, *pagesize);
        break;
    }

    if (ret == 0) {
        *outaddr = addr;
    }

    return ret;
}

static SMMUEvtErr smmu_walk_pgtable(SMMUState *s, Ste *ste, Cd *cd,
                                    IOMMUTLBEntry *tlbe, bool is_write)
{
    SMMUCfg cfg[2] = {{{0,} } };
    SMMUCfg *s1cfg = &cfg[0], *s2cfg = &cfg[1];
    SMMUEvtErr retval = 0;
    uint32_t ste_cfg = STE_CONFIG(ste);
    uint32_t page_size = 0, perm = 0;
    hwaddr pa;                 /* Input address, output address */

    SMMU_DPRINTF(DBG1, "ste_cfg :%x\n", ste_cfg);
    /* Both Bypass, we dont need to do anything */
    if (ste_cfg == STE_CONFIG_S1BY_S2BY) {
        return 0;
    }
    s1cfg->va = tlbe->iova;

    SMMU_DPRINTF(TT_1, "Input addr: %lx ste_config:%d\n",
                 s1cfg->va, ste_cfg);

    if (ste_cfg == STE_CONFIG_S1TR_S2BY || ste_cfg == STE_CONFIG_S1TR_S2TR) {
        smmu_cfg_populate_s1(cd, s1cfg);

        s1cfg->oas = MIN(oas2bits(smmu_read32_reg(s, SMMU_REG_IDR5) & 0xf),
                         s1cfg->oas);
        /* fix ttbr - make top bits zero*/
        cfg->ttbr = extract64(cfg->ttbr, 0, cfg->oas);
        s1cfg->s2cfg = s2cfg;

        retval = smmu_get_phys_addr(s, s1cfg, ste, cd, &page_size, &perm,
                                    false, is_write);
        if (retval != 0) {
            SMMU_DPRINTF(CRIT, "FAILED Stage1 translation\n");
            goto exit;
        }
        pa = cfg->pa;
        SMMU_DPRINTF(DBG1, "DONE: Stage1 tanslated :%lx\n ", pa);

    } else if (ste_cfg == STE_CONFIG_S1BY_S2TR) {
        /* Stage2 only configuratoin */
        smmu_cfg_populate_s2(ste, s2cfg);

        s2cfg->oas = MIN(oas2bits(smmu_read32_reg(s, SMMU_REG_IDR5) & 0xf),
                         s2cfg->oas);
        /* fix ttbr - make top bits zero*/
        cfg->ttbr = extract64(cfg->ttbr, 0, cfg->oas);

        retval = smmu_get_phys_addr(s, s2cfg, ste, cd, &page_size,
                                    &perm, true, is_write);
        if (retval != 0) {
            SMMU_DPRINTF(CRIT, "FAILED Stage2 translation\n");
            goto exit;
        }
        pa = s2cfg->opa;
        SMMU_DPRINTF(DBG1, "DONE: Stage2 tanslated :%lx\n ", pa);
    }

    SMMU_DPRINTF(TT_1, "DONE: translation o/p addr:%lx mask:%x is_write:%d\n ",
                 pa, page_size-1, is_write);
    tlbe->translated_addr = pa;
    tlbe->addr_mask = page_size - 1;
    tlbe->perm = perm;
exit:
    return retval;
}

/*
 * smmu_irq_update:
 * update corresponding register,
 * return > 0 when IRQ is supposed to be rased
 */
static int
smmu_irq_update(SMMUState *s, int irq, uint64_t data)
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

static void smmu_irq_raise(SMMUState *s, int irq, uint64_t data)
{
    SMMU_DPRINTF(IRQ, "irq:%d\n", irq);

    if (s->info->impl == SMMU_IMPL_BRCM) {
        uint32_t val = smmu_read32_reg(s, SMMU_REG_INTERRUPT);

        SMMU_DPRINTF(IRQ, "irq:%d reg_interrupt:%x\n", irq, val);

        switch (irq) {
        case SMMU_IRQ_EVTQ:
            val |= SMMU_INTR_EVENT;
            break;
        case SMMU_IRQ_PRIQ:
            val |= SMMU_INTR_PRI;
            break;
        case SMMU_IRQ_CMD_SYNC:
            val |= SMMU_INTR_CMD_SYNC;
            break;
        }

        smmu_write32_reg(s, SMMU_REG_INTERRUPT, val | SMMU_INTR_GERROR);
    }

    if (smmu_irq_update(s, irq, data)) {
        /*
         * Single interrupt pin in Broadcom implementation,
         * PRIq not supported
         */
        if (s->info->impl == SMMU_IMPL_BRCM) {
            qemu_irq_raise(s->irq[0]);
        } else {
            qemu_irq_raise(s->irq[irq]);
        }
    }
}

/*
 * Events created on the EventQ
 */
static void smmu_create_event(SMMUState *s, hwaddr iova,
                              uint32_t sid, bool is_write, int error)
{
    SMMUQueue *q = &s->evtq;
    uint64_t head = Q_IDX(q, q->prod);
    bool overflow = true, setva = false;
    Evt evt;

    if (!smmu_evt_q_enabled(s)) {
        overflow = true;
        goto set_overflow;
    }

    if (!smmu_is_q_full(s, &s->evtq)) {
        overflow = true;
        goto set_overflow;
    }

    EVT_SET_TYPE(&evt, error);
    EVT_SET_SID(&evt, sid);

    switch (error) {
    case SMMU_EVT_F_UUT:
    case SMMU_EVT_C_BAD_STE:
        break;
    case SMMU_EVT_C_BAD_CD:
    case SMMU_EVT_F_CD_FETCH:
        break;
    case SMMU_EVT_F_TRANS_FORBIDDEN:
    case SMMU_EVT_F_WALK_EXT_ABRT:
        setva = true;
    default:
        break;
    }
    if (setva) {
        EVT_SET_INPUT_ADDR(&evt, iova);
    }
    smmu_write_sysmem(s, Q_ENTRY(q, head), &evt, sizeof(evt));

    head++;

set_overflow:
    if (overflow) {
        head ^= 1 << 31;
    } else if (smmu_evt_irq_enabled(s)) {
        smmu_irq_raise(s, SMMU_IRQ_EVTQ, (uint64_t)&evt);
    }
    q->prod = head;

    smmu_write32_reg(s, SMMU_REG_EVTQ_PROD, head);
}

/*
 * TR - Translation Request
 * TT - Translated Tansaction
 * OT - Other Transaction
 */
static IOMMUTLBEntry
smmu_translate(MemoryRegion *mr, hwaddr addr, bool is_write)
{

    SMMUDevice *sdev = container_of(mr, SMMUDevice, iommu);
    SMMUState *s = sdev->smmu;
    uint16_t sid = 0, config;
    Ste ste;
    Cd cd;
    SMMUEvtErr error = 0;

    IOMMUTLBEntry ret = {
        .target_as = &address_space_memory,
        .iova = addr,
        .translated_addr = addr,
        .addr_mask = ~(hwaddr)0,
        .perm = IOMMU_NONE,
    };

    /* SMMU Bypass */
    /* We allow traffic through if SMMU is disabled */
    if (!smmu_enabled(s)) {
        SMMU_DPRINTF(CRIT, "SMMU Not enabled.. bypassing addr:%lx\n", addr);
        goto bypass;
    }

    sid = smmu_get_sid(sdev->bus, sdev->devfn);
    SMMU_DPRINTF(TT_1, "SID:%x bus:%d\n", sid, pci_bus_num(sdev->bus));

    /* Fetch & Check STE */
    error = smmu_find_ste(s, sid, &ste);
    if (error) {
        goto error_out;  /* F_STE_FETCH or F_CFG_CONFLICT */
    }

    if (IS_DBG_ENABLED(STE)) {
        dump_ste(&ste);
    }

    if (is_ste_valid(s, &ste) && is_ste_bypass(s, &ste)) {
        goto bypass;
    }

    SMMU_DPRINTF(STE, "STE is not bypass\n");
    if (!is_ste_consistent(s, &ste)) {
        error = SMMU_EVT_C_BAD_STE;
        goto error_out;
    }
    SMMU_DPRINTF(INFO, "Valid STE Found\n");

    /* Stream Bypass */
    config = STE_CONFIG(&ste);
    /*
     * Mostly we have S1-Translate and S2-Bypass, Others will be
     * implemented as we go
     */
    switch (config) {
    case STE_CONFIG_S1BY_S2BY:  /* S1-bypass, S2-bypass */
        goto bypass;

    case STE_CONFIG_S1TR_S2TR:  /* S1-trans, S2-trans, assume S1-Only */
        SMMU_DPRINTF(CRIT, "S1+S2 translation, not supported\n");
        break;
    case STE_CONFIG_S1TR_S2BY:        /* S1-Trans, S2-bypass */
        smmu_get_cd(s, &ste, 0, &cd); /* We dont have SSID yet, so 0 */

        if (IS_DBG_ENABLED(CD)) {
            dump_cd(&cd);
        }

        if (!is_cd_valid(s, &ste, &cd)) {
            error = SMMU_EVT_C_BAD_CD;
            goto error_out;
        }
        break;
    case STE_CONFIG_S1BY_S2TR:
        SMMU_DPRINTF(CRIT, "S2-only translation, not supported right now\n");
        goto out;
        break;
    default:
        SMMU_DPRINTF(CRIT, "Unknown config field in STE\n");
        goto out;
    }

    /* Walk Stage1, if S2 is enabled, S2 walked for Every access on S1 */
    error = smmu_walk_pgtable(s, &ste, &cd, &ret, is_write);

    SMMU_DPRINTF(INFO, "DONE walking tables(1)\n");
error_out:
    if (error) {        /* Post the Error using Event Q */
        SMMU_DPRINTF(CRIT, "Translation Error: %x\n", error);
        smmu_create_event(s, ret.iova, sid, is_write, error);
        goto out;
    }

bypass:
    ret.perm = is_write ? IOMMU_RW : IOMMU_RO;

out:
    return ret;
}

static const MemoryRegionIOMMUOps smmu_iommu_ops = {
    .translate = smmu_translate,
};


static bool
smmu_is_irq_pending(SMMUState *s, int irq)
{
    return (smmu_read32_reg(s, SMMU_REG_INTERRUPT) & 0xf) |
        (smmu_read32_reg(s, SMMU_REG_GERROR) ^
         smmu_read32_reg(s, SMMU_REG_GERRORN));
}

static void
smmu_irq_clear_brcm(SMMUState *s, int irq)
{
    uint32_t val = 0;

    val = smmu_read32_reg(s, SMMU_REG_INTERRUPT);
    SMMU_DPRINTF(IRQ, "Clearing IRQ:%d reg_interrupt:%x\n", irq, val);

    switch (irq) {
    case SMMU_GERROR_CMDQ:
        val ^= SMMU_INTR_CMD_SYNC;
        break;
    case SMMU_GERROR_EVENTQ:
        val ^= SMMU_INTR_EVENT;
        break;
    case SMMU_GERROR_PRIQ:
        val ^= SMMU_INTR_PRI;
        break;
    }
    val ^= SMMU_INTR_GERROR;

    smmu_write32_reg(s, SMMU_REG_INTERRUPT, val);
}

/*
 * GERROR is updated when rasing an interrupt, GERRORN will be updated
 * by s/w and should match GERROR before normal operation resumes.
 */
static void smmu_irq_clear(SMMUState *s, uint64_t gerrorn)
{
    int irq_new = SMMU_IRQ_GERROR;
    uint32_t toggled;

    toggled = smmu_read32_reg(s, SMMU_REG_GERRORN) ^ gerrorn;

    while (toggled) {
        int intr = ctz32(toggled);

        if (s->info->impl == SMMU_IMPL_BRCM) {
            smmu_irq_clear_brcm(s, intr);
        } else {
            qemu_irq_lower(s->irq[irq_new]);
        }

        toggled &= toggled - 1;
    }
}

static int smmu_evtq_update(SMMUState *s)
{
    if (!smmu_enabled(s)) {
        return 0;
    }
    return 1;
}

#define SMMU_CMDQ_ERR(s) ((smmu_read32_reg(s, SMMU_REG_GERROR) ^    \
                           smmu_read32_reg(s, SMMU_REG_GERRORN)) &  \
                          SMMU_GERROR_CMDQ)

static int smmu_cmdq_consume(SMMUState *s)
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

static void smmu_update(SMMUState *s)
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
    if (error) {
        smmu_create_event(s, 0, 0, 0, error);
    }
check_cmdq:
    if (smmu_cmd_q_enabled(s) && !SMMU_CMDQ_ERR(s)) {
        smmu_cmdq_consume(s);
    }
}

static inline void
smmu_update_base(SMMUState *s, uint32_t reg)
{
    uint64_t *base = NULL;

    switch (reg) {
    case SMMU_REG_STRTAB_BASE:
    case SMMU_REG_STRTAB_BASE + 4:
        base = &s->strtab_base;
        reg = SMMU_REG_STRTAB_BASE;
        break;
    case SMMU_REG_EVTQ_BASE + 4:
    case SMMU_REG_EVTQ_BASE:
        base = &s->evtq.base;
        reg = SMMU_REG_EVTQ_BASE;
        break;
    case SMMU_REG_CMDQ_BASE + 4:
    case SMMU_REG_CMDQ_BASE:
        base = &s->cmdq.base;
        reg = SMMU_REG_CMDQ_BASE;
        break;
    }

    /* BIT[62], BIT[5:0] are ignored */
    *base = smmu_read64_reg(s, reg) & ~(SMMU_BASE_RA | 0x3fUL);
}

static inline void
smmu_update_q(SMMUState *s, SMMUQueue *q, uint32_t val, uint32_t reg)
{
    bool update = false;

    switch (reg) {
    case SMMU_REG_CMDQ_BASE:
    case SMMU_REG_EVTQ_BASE:
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

    if (update) {
        smmu_update(s);
    }
}

static void smmu_write_mmio(void *opaque, hwaddr addr,
                            uint64_t val, unsigned size)
{
    SMMUState *s = opaque;
    SMMUQueue *q = NULL;
    int i;
    bool update_queue = false;
    bool is64 = false;
    bool base = false;
    uint32_t val32 = (uint32_t)val;
    SMMU_DPRINTF(DBG2, "reg:%lx cur: %x new: %lx\n", addr,
                 smmu_read32_reg(s, addr), val);

    /* We update the ACK registers, actual write happens towards end */

    switch (addr) {
    case SMMU_REG_IRQ_CTRL:     /* Update the ACK as well */
        val &= 0xf;

        for (i = 0; i < 4; i++)
            if (!(val & (1 << i))) {
                qemu_irq_lower(s->irq[i]);
            }

        smmu_write32_reg(s, addr + 4, val32);
        break;

    case SMMU_REG_CR0:
        smmu_write32_reg(s, addr + 4, val32);
        smmu_update(s);         /* Start processing as soon as enabled */
        break;

    case SMMU_REG_GERRORN:
        smmu_irq_clear(s, val32);
        smmu_write32_reg(s, SMMU_REG_GERRORN, val32);
        SMMU_DPRINTF(IRQ, "irq pend: %d reg_intr:%x gerror:%x gerrorn:%x\n",
                     smmu_is_irq_pending(s, 0),
                     smmu_read32_reg(s, SMMU_REG_INTERRUPT),
                     smmu_read32_reg(s, SMMU_REG_GERROR),
                     smmu_read32_reg(s, SMMU_REG_GERRORN));
        /* Clear only when no more left */
        if ((s->info->impl == SMMU_IMPL_BRCM) && !smmu_is_irq_pending(s, 0)) {
            qemu_irq_lower(s->irq[0]);
        }
        return;                 /* No further processing */

    case SMMU_REG_CMDQ_BASE:
        is64 = true;            /* fallthru */
    case SMMU_REG_CMDQ_BASE + 4:
        base = true;
    case SMMU_REG_CMDQ_PROD:
    case SMMU_REG_CMDQ_CONS:
        q = &s->cmdq;
        update_queue = true;
        break;
    case SMMU_REG_EVTQ_BASE:
        is64 = true;            /* fallthru */
    case SMMU_REG_EVTQ_BASE + 4:
        base = true;
    case SMMU_REG_EVTQ_CONS:
    case SMMU_REG_EVTQ_PROD:
        q = &s->evtq;
        update_queue = true;
        break;

    case SMMU_REG_STRTAB_BASE:
        is64 = true;
    case SMMU_REG_STRTAB_BASE + 4:
        base = true;
        break;

    case SMMU_REG_STRTAB_BASE_CFG:
        is64 = true;
        if (((val32 >> 16) & 0x3) == 0x1) {
            s->sid_split = (val32 >> 6) & 0x1f;
            s->features |= SMMU_FEATURE_2LVL_STE;
        }
        break;
    case SMMU_REG_INTERRUPT_EN: /* Valid in BRCM implementation */
        break;
    case SMMU_REG_PRIQ_BASE ... SMMU_REG_PRIQ_IRQ_CFG1:
        SMMU_DPRINTF(CRIT, "Trying to write to PRIQ, not implemented\n");
        break;

    case SMMU_REG_GERROR_IRQ_CFG0 ...  SMMU_REG_GERROR_IRQ_CFG2:
    case SMMU_REG_EVTQ_IRQ_CFG0 ... SMMU_REG_EVTQ_IRQ_CFG2:
        return; /* RAZ/WI */

    default:
    case SMMU_REG_STATUSR:
    case 0xFDC ... 0xFFC:
    case SMMU_REG_IDR0 ... SMMU_REG_IDR5:
        SMMU_DPRINTF(CRIT, "write to RO/Unimpl reg %lx val64:%lx val32:%x\n",
                     addr, val, val32);
        return;
    }

    if (is64) {
        SMMU_DPRINTF(CRIT, "64bit write, reg:%lx val:%lx\n", addr, val);
        smmu_write64_reg(s, addr, val);
    } else {
        smmu_write32_reg(s, addr, val32);
    }

    if (base) {
        smmu_update_base(s, addr);
    }
    if (update_queue) {
        smmu_update_q(s, q, val, addr);
    }
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

static AddressSpace *smmu_pci_iommu(PCIBus *bus, void *opaque, int devfn)
{
    SMMUState *s = opaque;
    SMMUDevice *sdev = &s->pbdev[PCI_SLOT(devfn)];
    SMMUSysState *sys = container_of(s, SMMUSysState, smmu_state);

    sdev->smmu = s;
    sdev->bus = bus;
    sdev->devfn = devfn;
    //sdev->as = &s->iommu_as;

    memory_region_init_iommu(&sdev->iommu, OBJECT(sys),
                             &smmu_iommu_ops, "smmuv3", UINT64_MAX);

    address_space_init(&sdev->as, &sdev->iommu, "smmu-as");
    //printf("returened as %p\n", sdev->as);

    //assert(sdev->as);

    // address_space_init(&sdev->as, &sdev->mr, "smmu-pci");
    return &sdev->as;
}

static void smmu_init_iommu_as(SMMUSysState *sys)
{
    SMMUState *s = &sys->smmu_state;
    PCIBus *pcibus = pci_find_primary_bus();

    if (pcibus) {
        SMMU_DPRINTF(CRIT, "Found PCI bus, setting up iommu\n");
        //memory_region_init_iommu(&s->iommu, OBJECT(sys),
        //&smmu_iommu_ops, "smmuv3-root", UINT64_MAX);

        //address_space_init(&s->iommu_as, &s->iommu, "smmu-as");

        pci_setup_iommu(pcibus, smmu_pci_iommu, s);

    } else {
        SMMU_DPRINTF(CRIT, "Could'nt find PCI bus, SMMU is not registered\n");
    }
}

typedef struct {
    /* <private> */
    SysBusDeviceClass parent_class;
    SMMUInfo *info;
} SMMUBaseClass;

#define SMMU_DEVICE_CLASS(klass)                                    \
    OBJECT_CLASS_CHECK(SMMUBaseClass, (klass), TYPE_SMMU_DEV_BASE)
#define SMMU_DEVICE_GET_CLASS(obj)                              \
    OBJECT_GET_CLASS(SMMUBaseClass, (obj), TYPE_SMMU_DEV_BASE)

static void smmu_configure(SMMUState *s, SysBusDevice *dev)
{
    int i;

    switch (s->info->impl) {
    default:
    case SMMU_IMPL_ARM:
        for (i = 0; i < ARRAY_SIZE(s->irq); i++) {
            sysbus_init_irq(dev, &s->irq[i]);
        }
        break;
    case SMMU_IMPL_BRCM:
        sysbus_init_irq(dev, &s->irq[0]);
        break;
    }
}

static int smmu_init(SysBusDevice *dev)
{
    SMMUSysState *sys = SMMU_SYS_DEV(dev);
    SMMUBaseClass *sbc = SMMU_DEVICE_GET_CLASS(sys);
    SMMUState *s = &sys->smmu_state;

    /* Register Access */
    memory_region_init_io(&s->iomem, OBJECT(sys),
                          &smmu_mem_ops, s, "smmuv3", 0x1000);

    sysbus_init_mmio(dev, &s->iomem);

    s->info = sbc->info;

    smmu_configure(s, dev);
    smmu_init_iommu_as(sys);

    return 0;
}

static void smmu_populate_regs(SMMUState *s)
{
    int i;
    uint32_t val;

    /* Primecell ID registers */
    s->cid[0] = 0x0D;
    s->cid[1] = 0xF0;
    s->cid[2] = 0x05;
    s->cid[3] = 0xB1;

    for (i = 0; i < ARRAY_SIZE(s->pid); i++) {
        s->pid[i] = 0x1;
    }
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

#define SMMU_SID_SIZE    16
    s->sid_size = SMMU_SID_SIZE;

#define SMMU_QUEUE_SIZE_LOG2 19
    val =
        1 << 27 |                    /* Attr Types override */
        SMMU_QUEUE_SIZE_LOG2 << 21 | /* Cmd Q size */
        SMMU_QUEUE_SIZE_LOG2 << 16 | /* Event Q size */
        SMMU_QUEUE_SIZE_LOG2 << 11 | /* PRI Q size */
        0  << 6 |                    /* SSID not supported */
        SMMU_SID_SIZE << 0 ;         /* SID size  */

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

static void smmu_reset(DeviceState *dev)
{
    SMMUSysState *sys = SMMU_SYS_DEV(dev);
    SMMUState *s = &sys->smmu_state;
    printf("smmu_reset: %p\n", sys);
    smmu_populate_regs(s);
}

#if 0
static void smmu_realize(DeviceState *dev, Error **errp)
{
    SMMUSysState *sys = SMMU_SYS_DEV(dev);
    printf("smmu_realize: %p\n", sys);
}
#endif

/*
 * DUMMY: Will get to this one day
 */
static const VMStateDescription vmstate_smmu = {
    .name = "smmu",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8_ARRAY(regs, SMMUState, SMMU_NREGS * sizeof(uint32_t)),
        VMSTATE_END_OF_LIST(),
    }
};

static void smmu_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);
    SMMUBaseClass *sbc = SMMU_DEVICE_CLASS(klass);
    SMMUInfo *info = (SMMUInfo *)data;

    k->init = smmu_init;

    sbc->info = info;

    dc->desc = info->desc;
    dc->reset = smmu_reset;
    dc->vmsd = &vmstate_smmu;
    //dc->realize = smmu_realize;
}

static void smmu_instance_init(Object *obj)
{
    /* Nothing so far */
}

static const SMMUInfo smmu_info[] = {
    {                           /* ARM Implementation */
        .name = TYPE_SMMU_DEV,
        .desc = "ARM SMMUv3",
        .impl = SMMU_IMPL_ARM,
    },
    {                           /* Broadcom version */
        .name = TYPE_SMMU_BRCM_DEV,
        .desc = "ARM SMMUv3 (Broadcom)",
        .impl = SMMU_IMPL_BRCM,
    },
};

static const TypeInfo smmu_base_info = {
    .name          = TYPE_SMMU_DEV_BASE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SMMUSysState),
    .instance_init = smmu_instance_init,
    .class_size    = sizeof(SMMUBaseClass),
    .abstract      = true,
};

static void smmu_register_types(void)
{
    int i;

    type_register_static(&smmu_base_info);

    for (i = 0; i < ARRAY_SIZE(smmu_info); i++) {
        const SMMUInfo *info = &smmu_info[i];
        TypeInfo type_info = {};

        type_info.name = info->name;
        type_info.parent = TYPE_SMMU_DEV_BASE;
        type_info.class_data = (void *)info;
        type_info.class_init = smmu_class_init;
        type_info.instance_init = smmu_instance_init;

        type_register(&type_info);
    }
}

type_init(smmu_register_types)
