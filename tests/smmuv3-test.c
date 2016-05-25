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
 *                       <prem.mallappa@gmail.com>
 */

#include "qemu/osdep.h"

#include <glib.h>
#include <glib/gstdio.h>

#include "libqtest.h"
#include "libqos/libqos.h"
#include "libqos/pci-generic.h"
#include "libqos/malloc-generic.h"

#include "qemu-common.h"
#include "hw/pci/pci_ids.h"
#include "hw/pci/pci_regs.h"

/* PCIe test device */
#include "hw/misc/pci-testdev-smmu.h"

/* SMMU */
#include "hw/arm/smmu-common.h"
#include "hw/arm/smmuv3-internal.h"


/*
 * STE/CD modification helpers
 */
#define ___SET(ste, off, start, len, val)                               \
    ({                                                                  \
        uint32_t *ptr = &(ste)->word[(off)];                            \
        *ptr = deposit32(*ptr, start, len, val);                        \
    })

#define STE_SET_VALID(ste, val)   ___SET(ste, 0, 0, 1, val)
#define STE_SET_CONFIG(ste, val)  ___SET(ste, 0, 1, 3, val)
#define STE_SET_S1FMT(ste, val)   ___SET(ste, 0, 4, 2, val)
#define STE_SET_S1CDMAX(ste, val) ___SET(ste, 1, 8, 2, val)
#define STE_SET_EATS(ste, val)    ___SET(ste, 2, 28, 2, val)
#define STE_SET_STRW(ste, val)    ___SET(ste, 2, 30, 2, val)
#define STE_SET_S2VMID(ste, val)  ___SET(ste, 4, 0, 16, val) /* 4 */
#define STE_SET_S2T0SZ(ste, val)  ___SET(ste, 5, 0, 6, val) /* 5 */
#define STE_SET_S2TG(ste, val)    ___SET(ste, 5, 14, 2, val)
#define STE_SET_S2PS(ste, val)    ___SET(ste, 5, 16, 3, val)
#define STE_SET_S2AA64(ste, val)  ___SET(ste, 5, 19, 1, val)
#define STE_SET_S2HD(ste, val)    ___SET(ste, 5, 24, 1, val)
#define STE_SET_S2HA(ste, val)    ___SET(ste, 5, 25, 1, val)
#define STE_SET_S2S(ste, val)     ___SET(ste, 5, 26, 1, val)
#define STE_SET_CTXPTR(ste, val)                \
    ({                                          \
        uint64_t __val = val;                   \
        __val >>= 6;                            \
        ___SET((ste), 0, 6, 26, __val);         \
        __val >>= 32;                           \
        ___SET((ste), 1, 0, 16, __val);         \
    })

#define STE_SET_S2TTB(ste, val)                   \
    ({                                            \
        uint64_t __val = val;                     \
        __val >>= 4;                              \
        ___SET(ste, 6, 4, 28, __val);             \
        __val >>= 32;                             \
        ___SET(ste, 7, 0, 16, __val);             \
    })

#define CD_SET_VALID(cd, val)   ___SET((cd), 0, 31, 1, val)
#define CD_SET_ASID(cd, val)    ___SET((cd), 1, 16, 16, val)
#define CD_SET_TTB(cd, sel, val)                               \
    ({                                                         \
        uint64_t __val = val;                                  \
        ___SET((cd), (sel)*2 + 2, 0, 32, __val & ~0xf);        \
        __val >>= 32;                                          \
        ___SET((cd), (sel)*2 + 3, 0, 16, __val & 0xffff);      \
    })

#define CD_SET_TSZ(cd, sel, val)   ___SET((cd), 0, (16*(sel)) + 0, 6, val)
#define CD_SET_TG(cd, sel, val)    ___SET((cd), 0, (16*(sel)) + 6, 2, val)
#define CD_SET_EPD(cd, sel, val)   ___SET((cd), 0, (16*(sel)) + 14, 1, val)

#define CD_SET_T0SZ(cd, val)    CD_SET_TSZ((cd), 0, val)
#define CD_SET_T1SZ(cd, val)    CD_SET_TSZ((cd), 1, val)
#define CD_SET_TG0(cd, val)     CD_SET_TG((cd), 0, val)
#define CD_SET_TG1(cd, val)     CD_SET_TG((cd), 1, val)
#define CD_SET_EPD0(cd, val)    CD_SET_EPD((cd), 0, val)
#define CD_SET_EPD1(cd, val)    CD_SET_EPD((cd), 1, val)
#define CD_SET_IPS(cd, val)     ___SET((cd), 1, 0, 3, val)
#define CD_SET_AARCH64(cd, val) ___SET((cd), 1, 9, 1, val)
#define CD_SET_TTB0(cd, val)    CD_SET_TTB((cd), 0, val)
#define CD_SET_TTB1(cd, val)    CD_SET_TTB((cd), 1, val)

/* SMMU Device local state */
struct SMMUDevState {
    /* SMMU related fields */
    void *reg_base;
    uint64_t strtab_base;
    QGuestAllocator *strtab_alloc;
    QGuestAllocator *cd_alloc;
    QGuestAllocator *pgtbl_alloc;
    SMMUQueue cmdq;
};
typedef struct SMMUDevState SMMUDevState;

static inline void smmu_write64_reg(SMMUDevState *s, uint32_t reg, uint64_t val)
{
    uint64_t addr = (uint64_t)(s->reg_base + reg);
    writel(addr, val);
}

static inline void smmu_write_reg(SMMUDevState *s, uint32_t reg, uint64_t val)
{
    uint64_t addr = (uint64_t)(s->reg_base + reg);
    writew(addr, val);
}

/* Our Test device */
struct SMMUTestDevState {
    QPCIDevice *dev;
    void *reg_base;
};
typedef struct SMMUTestDevState SMMUTestDevState;

typedef struct SMMUTestVMCfg {
    uint64_t ram_base;
    uint64_t ram_size;
    uint32_t page_size;

    QPCIBusGen virt_pci;
} SMMUTestVMCfg;

typedef struct SMMUTestCfg {
    uint8_t sid_size;
    uint8_t sid_split;
    uint32_t cmdq_shift;
} SMMUTestCfg;

/* SMMU Test state */
struct SMMUTestState {
    QPCIBus *pcibus;
    QTestState *qtest;

    QGuestAllocator *alloc;

    SMMUDevState sdev;

    SMMUTestDevState tdev;

    SMMUTestCfg  cfg;

    SMMUTestVMCfg vm_cfg;
};
typedef struct SMMUTestState SMMUTestState;

static void cleanup_vm(SMMUTestState *s)
{
    qpci_free_generic(s->pcibus);
}

static void test_smmu_cleanup(SMMUTestState *state)
{
    printf("Cleanup called\n");
    generic_alloc_uninit(state->alloc);
    qtest_quit(state->qtest);
    cleanup_vm(state);
}

static void abort_handler(void *data)
{
    SMMUTestState *state = (SMMUTestState *)data;
    g_test_message("abort handler called");
    test_smmu_cleanup(state);
}

static void save_fn(QPCIDevice *dev, int devfn, void *data)
{
    QPCIDevice **pdev = (QPCIDevice **) data;
    printf("dev->devfn:%d\n", devfn);
    *pdev = dev;
}

static QPCIDevice *get_device(QPCIBus *pcibus)
{
    QPCIDevice *dev;

    dev = NULL;
    qpci_device_foreach(pcibus, 0x1b36, 0x0005, save_fn, &dev);
    g_assert(dev != NULL);
    //while(1);
    return dev;
}

static void testdev_write64_reg(SMMUTestDevState *tdev, uint32_t reg,
                                uint64_t val)
{
    uint64_t addr = (uint64_t)(tdev->reg_base + reg);
    writel(addr, val);
}

static void testdev_write_reg(SMMUTestDevState *tdev, uint32_t reg,
                              uint64_t val)
{
    uint64_t addr = (uint64_t)(tdev->reg_base + reg);
    writew(addr, val);
}

static void testdev_dma(SMMUTestDevState *tdev, void *src,
                        void *dst, int nbytes)
{
    testdev_write64_reg(tdev, TST_REG_SRC_ADDR, (uint64_t)src);
    testdev_write64_reg(tdev, TST_REG_DST_ADDR, (uint64_t)dst);
    testdev_write_reg(tdev, TST_REG_SIZE, nbytes);

    testdev_write_reg(tdev, TST_REG_COMMAND, CMD_RW);
}

static void testdev_setup(SMMUTestState *s)
{
    SMMUTestDevState *tdev = &s->tdev;
    uint64_t barsize;

    tdev->dev = get_device(s->pcibus);
    g_assert_nonnull(tdev->dev);

    tdev->reg_base = qpci_iomap(tdev->dev, 0, &barsize);
    g_assert_nonnull(tdev->reg_base);

    qpci_device_enable(tdev->dev);
}

/* following values are taken from virt.c virt.h files */
#define MMIO_RAM_ADDR           0x40000000ULL
#define MMIO_RAM_SIZE           4096ULL /* in MB */

static void setup_vm_cmd(SMMUTestState *s, const char *cmd, bool msix)
{
    s->vm_cfg = (SMMUTestVMCfg) {
        .virt_pci = {
            .base = 0x3f000000,
            .pci_hole_start = 0x10000000,
            .pci_hole_size = 0x2eff0000,
            .pci_hole_alloc = 0,
        },
        .ram_base = MMIO_RAM_ADDR,
        .ram_size = MMIO_RAM_SIZE << 20,
        .page_size = s->vm_cfg.page_size,
    };

    QPCIBusGen *vpci = &s->vm_cfg.virt_pci;

    s->qtest = qtest_start(cmd);

    s->pcibus = qpci_init_generic(vpci);

    printf("VM setup with cmdline:%s\n", cmd);
}

static void setup_vm(SMMUTestState *s, bool is_gdb_start)
{
    const char *gdb = is_gdb_start? "-s -S": "";
    const char *mon = is_gdb_start? " -chardev socket,id=mon0,host=localhost,port=6001,server,telnet,nowait -monitor chardev:mon0 ": "";
    char *cmd = g_strdup_printf(
        " -cpu cortex-a57 -m %llu -machine virt " //-smp 4 "
        " -device i82801b11-bridge,multifunction=on,bus=pcie.0,addr=05,id=pcie.1 "
        " %s "
        " -device pci-testdev-smmu,bus=pcie.0,addr=04 "
        " -d iommu "
        " %s ", MMIO_RAM_SIZE, /* in MB */
        mon, gdb);

    setup_vm_cmd(s, cmd, false);

    g_free(cmd);
}

#define SIZE_MB(x)              ((x) << 20)
/* 20 is MB, 1/4th of the size we start the allocator */
#define TEST_ALLOCATOR_START    (SIZE_MB(MMIO_RAM_SIZE) >> 2)
#define TEST_ALLOCATOR_SIZE     SIZE_MB(32) /* 32MB */

#define STRTAB_ALLOCATOR_START  (TEST_ALLOCATOR_START + \
                                 TEST_ALLOCATOR_SIZE + SIZE_MB(32))
#define STRTAB_ALLOCATOR_SIZE   SIZE_MB(32)

#define CD_ALLOCATOR_START      (STRTAB_ALLOCATOR_START + \
                                 STRTAB_ALLOCATOR_SIZE + SIZE_MB(32))
#define CD_ALLOCATOR_SIZE       (SIZE_MB(32))

#define PGTABLE_ALLOCATOR_START (CD_ALLOCATOR_START + \
                                 CD_ALLOCATOR_SIZE + SIZE_MB(32))
#define PGTABLE_ALLOCATOR_SIZE  (SIZE_MB(32))


static int smmu_init_cmdq(SMMUTestState *state)
{
    SMMUDevState *smmu = &state->sdev;
    SMMUQueue *q = &smmu->cmdq;
    SMMUTestCfg *cfg = &state->cfg;

    q->shift = cfg->cmdq_shift;
    q->ent_size = sizeof(Cmd);
    q->entries = 1 << cfg->cmdq_shift;

    q->base = guest_alloc(state->alloc, q->entries * sizeof(Cmd));
    if (!q->base)
        return -ENOMEM;
    q->base &= ~0x1fULL;    /* last 5 bits are for size in log2 */
    smmu_write64_reg(smmu, SMMU_REG_CMDQ_BASE,
                     q->base | q->shift);
    smmu_write_reg(smmu, SMMU_REG_CMDQ_PROD, 0x0);
    smmu_write_reg(smmu, SMMU_REG_CMDQ_CONS, 0x0);

    return 0;
}

static uint64_t smmu_get_ste(SMMUDevState *smmu, int devfn)
{
    uint64_t stmp, stm, step;
    SMMUTestState *state = container_of(smmu, SMMUTestState, sdev);
    SMMUTestCfg *cfg = &state->cfg;
    int split = cfg->sid_split;
    int l1_off = devfn >> split,
        l2_off = devfn & ~(1 << split);
    uint64_t span_mask = 0x3fULL;

    printf("devfn:%x l1_off:%x l2_off:%x\n", devfn, l1_off, l2_off);
    stmp = smmu->strtab_base + (l1_off * sizeof(STEDesc));
    //stm = readl(stmp);
    qtest_memread(state->qtest, stmp, &stm, sizeof(stm));
    printf("stmp:%lx stm:%lx\n", stmp, stm);

    if (stm && (stm & span_mask)) {
        printf("already allocated ste \n");
    } else {
        uint64_t page;
        int size = sizeof(Ste) * (1 << split);

        page = guest_alloc(smmu->strtab_alloc, size);
        assert(!(page & span_mask));
        qmemset(page, 0, size);

        page &= ~span_mask;
        /*
         * 2^(span - 1) entries, should cover current devfn
         * if devfn is 0x20, devfn >> 2 = 8
         * this should cover 2^7 = 128 STE's, which covers devfn 0x20
         */
        page |= (devfn >> 2);

        printf("page:%lx size:%d qtest:%p global:%p\n",
               page, size, state->qtest, global_qtest);
        stm = page;
        //qtest_memwrite(state->qtest, stmp, &stm, sizeof(stmp));
        writeq(stmp, stm);

        //qtest_memread(state->qtest, stmp, &page, sizeof(stmp));
        page = readq(stmp);
        printf("stm:%lx page:%lx\n", stm, page);
        assert(stm == page);
    }

    stm &= ~span_mask;
    step = stm + (l2_off * sizeof(Ste));
    printf("step:%lx\n", step);
    return step;
}

static void smmu_strtab_initone(SMMUTestState *state, int devfn)
{

    SMMUDevState *smmu = &state->sdev;
    uint64_t step = smmu_get_ste(smmu, devfn);
    Ste ste;

    qtest_memread(state->qtest, step, &ste, sizeof(ste));
    STE_SET_CONFIG(&ste, 0x4);          /* bypass */
    STE_SET_VALID(&ste, 0x1);
    printf("%s: ste.word[0]:%x\n", __func__, ste.word[0]);
    qtest_memwrite(state->qtest, step, &ste, sizeof(ste));
}

static int smmu_strtab_init(SMMUTestState *state)
{
    uint32_t size;
    SMMUDevState *smmu = &state->sdev;
    SMMUTestCfg *cfg = &state->cfg;

    size = sizeof(STEDesc) * (1 << cfg->sid_split);
    smmu->strtab_base = guest_alloc(state->alloc, size);
    if (!smmu->strtab_base)
        return -ENOMEM;

    qmemset(smmu->strtab_base, 0, size);
    smmu_write64_reg(smmu, SMMU_REG_STRTAB_BASE, smmu->strtab_base);
    smmu_write64_reg(smmu, SMMU_REG_STRTAB_BASE_CFG, 0x10210);

    smmu->strtab_alloc = generic_alloc_init(MMIO_RAM_ADDR +
                                            STRTAB_ALLOCATOR_START,
                                            STRTAB_ALLOCATOR_SIZE,
                                            sizeof(Ste));

    smmu->cd_alloc = generic_alloc_init(MMIO_RAM_ADDR +
                                        CD_ALLOCATOR_START,
                                        CD_ALLOCATOR_SIZE,
                                        sizeof(Cd));

    smmu->pgtbl_alloc = generic_alloc_init(MMIO_RAM_ADDR +
                                           PGTABLE_ALLOCATOR_START,
                                           PGTABLE_ALLOCATOR_SIZE,
                                           state->vm_cfg.page_size);

    smmu_strtab_initone(state, state->tdev.dev->devfn);

    return 0;
}

static int smmu_pgtable_alloc(SMMUTestState *s)
{
    SMMUDevState *smmu = &s->sdev;
    uint64_t step = smmu_get_ste(smmu, s->tdev.dev->devfn);

    return step ? 0 : 1;
}

static int smmu_init(SMMUTestState *s)
{
    SMMUDevState *smmu = &s->sdev;
    int ret = 0;

    ret = smmu_init_cmdq(s);
    if (ret)
        return ret;

    ret = smmu_strtab_init(s);
    if (ret)
        return ret;

    ret = smmu_pgtable_alloc(s);
    if (ret)
        return ret;

    /* At last enable SMMU */
    smmu_write_reg(smmu, SMMU_REG_CR0, 0x1);
    return ret;
}

static void testdev_init(SMMUTestDevState *tdev)
{
    /* Nothing to be done at the moment */
}

static void smmu_setup(SMMUTestState *s)
{
    SMMUDevState *smmu = &s->sdev;
    /* Write command queue base, enable command queue and issue cmd_sync */
    /* This value is taken from hw/arm/virt.c, no portable way of getting it ? */
    smmu->reg_base = (void*)0x09050000;
}

static void test_smmu_setup(SMMUTestState *state)
{
    SMMUTestVMCfg *vm_cfg = &state->vm_cfg;

    qtest_add_abrt_handler(abort_handler, state);

    setup_vm(state, true);

    state->alloc = generic_alloc_init(vm_cfg->ram_base +
                                      TEST_ALLOCATOR_START,
                                      TEST_ALLOCATOR_SIZE,
                                      vm_cfg->page_size);

    smmu_setup(state);

    testdev_setup(state);

    if (smmu_init(state))
        return;

    testdev_init(&state->tdev);
}

static uint64_t
alloc_pgtable(SMMUDevState *smmu, SMMUTransCfg *cfg, bool s2needed)
{

    SMMUTestState *state = container_of(smmu, SMMUTestState, sdev);
    int stage = cfg->stage;
    int level, granule_sz = cfg->granule_sz[stage];
    int page_size = 1 << (cfg->granule_sz[stage] + 3);
    //hwaddr pagesize;
    hwaddr addr, mask, va = cfg->va, pa = cfg->pa;
    uint64_t ttbr;

    static const char *gap = "";

    printf("%s %s stage:%d va_size:%d va:%lx pa:%lx\n", gap,
           __func__, stage, cfg->va_size[stage], va, pa);

    if (!cfg->ttbr[stage]) {
        cfg->ttbr[stage] = guest_alloc(smmu->pgtbl_alloc, page_size);
        if (!cfg->ttbr[stage]) {
            printf("Unable to allocate guest memory for ttbr\n");
            return 0;
        }
        qmemset(cfg->ttbr[stage], 0, page_size);
    }

    ttbr = cfg->ttbr[stage];

    level = 4 - (cfg->va_size[stage] - cfg->tsz[stage] - 4) / cfg->granule_sz[stage];
    mask = (1ULL << (cfg->granule_sz[stage] + 3)) - 1;

    addr = extract64(ttbr, 0, 48);
    printf("%sTTBR:%lx va:%lx\n", gap, addr, va);
    addr &= ~((1ULL < (cfg->va_size[stage] - cfg->tsz[stage] -
                       (granule_sz * (4 - level)))) - 1);
    for (;;) {
        uint64_t desc;
        uint64_t ored = (va >> (granule_sz * (4 - level))) & mask;

        printf("%sstage%d LEVEL:%d addr:%lx ored:%lx\n", gap, stage, level, addr, ored);

        addr |= (va >> (granule_sz * (4 - level))) & mask;
        addr &= ~0x7ULL;

        qtest_memread(state->qtest, addr, &desc, sizeof(desc));

        if ((level < 3) && !desc) {
            desc = guest_alloc(smmu->pgtbl_alloc, page_size);
            if (!desc) {
                printf("Unable to allocate page table memory\n");
                break;
            }
            printf("%snew pgtable level@%d: %lx\n", gap, level, desc);

            qmemset(desc, 0, page_size); /* memset() */

            desc |= 3;
            qtest_memwrite(state->qtest, addr, &desc, sizeof(desc));
        }

        printf("%sLEVEL:%d gran_sz:%d mask:%lx addr:%lx desc:%lx\n",
               gap, level, granule_sz, mask, addr, desc);

        if (s2needed) {
            SMMUTransCfg s2cfg = *cfg;
            gap = "\t";
            if (level < 3)
<<<<<<< HEAD
<<<<<<< HEAD
                s2cfg->pa = s2cfg->va = desc & ~0x3UL;
=======
                s2cfg->pa = s2cfg->ipa = desc & ~0x3UL;
>>>>>>> a328a5f... [optional] tests: SMMUv3 unit tests
=======
                s2cfg.pa = s2cfg.va = desc & ~0x3UL;
>>>>>>> c6de33c... tests: SMMUv3 test to incorporate new changes in datastructure
            else
                s2cfg.pa = s2cfg.va = pa & ~mask;

            s2cfg.stage = 2;
            s2cfg.s2_needed = false;

            alloc_pgtable(smmu, &s2cfg, false);

            cfg->ttbr[2] = s2cfg.ttbr[2];

            gap = "";
        }

        if ((level < 3)) {
            //addr = desc & ~mask;
            addr = desc & 0xfffffffff000ULL;
            desc |= 0x2;
            level++;
            continue;
        }

        pa &= ~(page_size-1);
        pa |= 3;

        qtest_memwrite(state->qtest, addr, &pa, sizeof(pa));
        qtest_memread(state->qtest, addr, &desc, sizeof(va));
        assert(desc == pa);
        printf("LEVEL:%d final written value @:%lx is :%lx page_size:%x\n",
               level, addr, va, page_size);

        break;
    }

    return ttbr;
}

static inline void dump_ste1(Ste *ste)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(ste->word); i += 2) {
        printf("STE[%2d]: %#010x\t STE[%2d]: %#010x\n",
               i, ste->word[i], i + 1, ste->word[i + 1]);
    }
}

static inline void dump_cd1(Cd *cd)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(cd->word); i += 2) {
        printf("CD[%2d]: %#010x\t CD[%2d]: %#010x\n",
               i, cd->word[i], i + 1, cd->word[i + 1]);
    }
}

/*
 * This part is little complecated, we use the same page tables
 * 1-1 mapped page table, the resulting addr is same as original addr,
 * just that page table walk is done.
 */
static int
update_pgtable(SMMUDevState *smmu, bool s1needed,
               SMMUTestDevState *tdev, SMMUTransCfg *cfg, bool s2needed)
{
    uint64_t step = smmu_get_ste(smmu, tdev->dev->devfn);
    SMMUTestState *state = container_of(smmu, SMMUTestState, sdev);
    Cd cd = {0,};
    Ste ste = {0,};
    int stage = cfg->stage;

    if (!step) {
        printf("+++Could not find STE pointer\n");
        return -1;
    }
    qtest_memread(state->qtest, step, &ste, sizeof(ste));
    printf("===> step:%lx ste read config:%d stage:%d\n",
           step, STE_CONFIG(&ste), stage);

    if (s1needed) {           /* S1+S2 */
        uint64_t cdp = STE_CTXPTR(&ste);

        /* use single level CD pointer */
        STE_SET_S1FMT(&ste, 0);

        printf("==== setting up CD :%lx\n", cdp);
        if (!cdp) {
            cdp = guest_alloc(smmu->cd_alloc, sizeof(cd));
            qmemset(cdp, 0, sizeof(cd));
            printf("==== allocated cd:%lx\n", cdp);

            CD_SET_EPD1(&cd, 1);
            CD_SET_VALID(&cd, 1);

            CD_SET_T0SZ(&cd, cfg->tsz[stage]);
            CD_SET_TG0(&cd, cfg->granule[stage]);
            CD_SET_IPS(&cd, cfg->oas[stage]);
            CD_SET_AARCH64(&cd, 1);

            STE_SET_CTXPTR(&ste, cdp);

            STE_SET_CONFIG(&ste, STE_CONFIG(&ste) | 0x5);
            printf("=== cdp:%lx cd[0]:%x\n", cdp, cd.word[0]);
        } else
            qtest_memread(state->qtest, cdp, &cd, sizeof(cd));

        cfg->ttbr[stage] = CD_TTB0(&cd);
        cfg->pa = cfg->va;              /* 1-1 mapping */
        alloc_pgtable(smmu, cfg,  s2needed);

        CD_SET_TTB0(&cd, cfg->ttbr[stage]);
        dump_cd1(&cd);
        qtest_memwrite(state->qtest, cdp, &cd, sizeof(cd));
    }

    STE_SET_EATS(&ste, 0x1);
    dump_smmutranscfg(cfg);
    /* most values are what Linux fills */
    if (s2needed) {
        stage = 2;
        STE_SET_CONFIG(&ste, STE_CONFIG(&ste) | 0x6);

        if (!s1needed) {                /* S2 only  */
            cfg->ttbr[stage] = STE_S2TTB(&ste);
            cfg->pa = cfg->va;          /* 1-1 mapping */
            alloc_pgtable(smmu, cfg, false);
        }

        /* should we consider 16k case? then S2TG= 0x2 */
        STE_SET_S2TG(&ste, (cfg->granule[stage])? 1: 0);
        STE_SET_S2PS(&ste, 0x7);
        STE_SET_S2S(&ste, 0x1);
        STE_SET_S2AA64(&ste, 0x1);
        STE_SET_S2T0SZ(&ste, cfg->tsz[stage]);
        STE_SET_S2TTB(&ste, cfg->ttbr[stage]);
    }

    STE_SET_VALID(&ste, 0x1);

    dump_ste1(&ste);

    qtest_memwrite(state->qtest, step, &ste, sizeof(ste));

    return 0;
}

static void __do_dma(SMMUTestState *s, SMMUTransCfg *cfg,
                     bool s1needed, bool s2needed)
{
    int i;
    uint16_t *src, *dst;
    uint64_t g_src, g_dst;

    if (s2needed && !s1needed)
        cfg->stage = 2;
    else
        cfg->stage = 1;

#define TST_BUFFER_SIZE 0x200
    src = g_malloc(TST_BUFFER_SIZE);
    dst = g_malloc(TST_BUFFER_SIZE);
    g_src = guest_alloc(s->alloc, TST_BUFFER_SIZE);
    g_dst = guest_alloc(s->alloc, TST_BUFFER_SIZE);

    /* Fill array with integers */
    for (i = 0; i < TST_BUFFER_SIZE/sizeof(uint16_t); i++) {
        uint16_t *ptr = (uint16_t*)src + i;
        *ptr = i;
    }
    qtest_memwrite(s->qtest, g_src, src, TST_BUFFER_SIZE);

    /* Install tables for this device and src/dst addresses */

    cfg->va = g_src;
    update_pgtable(&s->sdev, s1needed, &s->tdev, cfg, s2needed);

    cfg->va = g_dst;
    update_pgtable(&s->sdev, s1needed, &s->tdev, cfg, s2needed);

    /* Start dma */
    testdev_dma(&s->tdev, (void*)g_src, (void*)g_dst,
                TST_BUFFER_SIZE);

    qtest_memread(s->qtest, g_dst, dst, TST_BUFFER_SIZE);
    for (i = 0; i < TST_BUFFER_SIZE/sizeof(uint16_t); i++) {
        if (src[i] != dst[i]) {
            printf("No match off:%d src:%x dst:%x\n",
                   i, src[i], dst[i]);
            printf("\n=========TEST FAILED=============\n");
            return;
        }
    }
    printf("\n=========TEST PASSED=============\n");
}

static void __test_smmu(SMMUTransCfg *cfg,
                        bool s1needed, bool s2needed)
{
    SMMUTestState state = {0,};
    int stage = cfg->stage;

    state.cfg = (SMMUTestCfg){
        .sid_size = 16,
        .sid_split = 8,
        .cmdq_shift = 3,                /* q size, 8 deep */
    };
    SMMUTestState *s = &state;

    s->vm_cfg.page_size = 1 << (cfg->granule_sz[stage] + 3);

    test_smmu_setup(s);

    __do_dma(s, cfg, s1needed, s2needed);

    printf("state->qtest:%p global:%p\n", s->qtest, global_qtest);

    test_smmu_cleanup(s);
}

static void __make_test(const char *pattern, const char *test,
                        void (*testfn)(void))
{
    char *_test =  g_strdup_printf("%s%s%s", pattern,
                                   !g_str_has_suffix(pattern, "/") ? "/": "",
                                   test);
    qtest_add_func(_test, testfn);
    g_free(_test);
}

static void test_smmu_cmdq(void)
{

    /*
     * First we setup command queue to be 64 entry deep and
     * test if it wraps around correctly
     */
    /* Actually it does, tested with Linux driver */

}

const SMMUTransCfg cfg4k = {
    .tsz = {0, 24, 24},
    .granule = {0, 0, 0},
    .va_size = {0, 64, 64},
    .granule_sz = {0, 9, 9},
};

const SMMUTransCfg cfg64k = {
    .tsz = {0, 16, 16},
    .granule = {0, 1, 1},
    .va_size = {0, 64, 64},
    .granule_sz = {0, 13, 13},
};

const SMMUTransCfg cfgs14k_s264k = {
    .tsz = {0, 24, 16},
    .granule = {0, 0, 1},
    .va_size = {0, 64, 64},
    .granule_sz = {0, 9, 13},
    .stage = 1,
    .s2_needed = true,
};

const SMMUTransCfg cfgs164k_s24k = {
    .tsz = {0, 16, 24},
    .granule = {0, 1, 0},
    .va_size = {0, 64, 64},
    .granule_sz = {0, 13, 9},
    .stage = 1,
    .s2_needed = true,
};


static void test_smmu_s1_4k(void)
{
    SMMUTransCfg cfg;

    memcpy(&cfg, &cfg4k, sizeof(cfg4k));

    __test_smmu(&cfg, true, false);
}

static void test_smmu_s1_64k(void)
{
    SMMUTransCfg cfg;

    memcpy(&cfg, &cfg64k, sizeof(cfg));

    __test_smmu(&cfg, true, false);
}

static void test_smmu_s1s2_4k(void)
{
    SMMUTransCfg cfg;

    memcpy(&cfg, &cfg4k, sizeof(cfg));
    cfg.stage = 1;
    cfg.s2_needed = true;
    __test_smmu(&cfg, true, true);
}

static void test_smmu_s1s2_64k(void)
{
    SMMUTransCfg cfg;

    memcpy(&cfg, &cfg64k, sizeof(cfg));

    __test_smmu(&cfg, true, true);

}

static void test_smmu_s14k_s264k(void)
{
    SMMUTransCfg cfg;

    memcpy(&cfg, &cfgs14k_s264k, sizeof(cfg));

    __test_smmu(&cfg, true, true);
}

static void test_smmu_s164k_s24k(void)
{
    SMMUTransCfg cfg;

    memcpy(&cfg, &cfgs164k_s24k, sizeof(cfg));

    __test_smmu(&cfg, true, true);
}

static void test_smmu_s2_4k(void)
{
    SMMUTransCfg cfg;

    memcpy(&cfg, &cfg4k, sizeof(cfg));
    cfg.stage = 2;

    __test_smmu(&cfg, false, true);

}

static void test_smmu_s2_64k(void)
{
    SMMUTransCfg cfg;

    memcpy(&cfg, &cfg64k, sizeof(cfg));
    cfg.stage = 2;

    __test_smmu(&cfg, false, true);
}

struct test_matrix {
    const char *pattern;
    const char *test;
    void (*testfn)(void);
} test_matrix[] = {
    {"/smmuv3/init", "cmdq", test_smmu_cmdq},
    {"/smmuv3/tt/s1", "4k", test_smmu_s1_4k},
    {"/smmuv3/tt/s1", "64k", test_smmu_s1_64k},
    {"/smmuv3/tt/s1s2", "4k", test_smmu_s1s2_4k},
    {"/smmuv3/tt/s1s2", "64k", test_smmu_s1s2_64k},
    {"/smmuv3/tt/s1s2", "4k/64k", test_smmu_s14k_s264k},
    {"/smmuv3/tt/s1s2", "64k/4k", test_smmu_s164k_s24k},
    {"/smmuv3/tt/s2", "4k", test_smmu_s2_4k},
    {"/smmuv3/tt/s2", "64k", test_smmu_s2_64k},
};

int main(int argc, char **argv)
{
    int ret = 0, i;

    if (strcmp(qtest_get_arch(), "aarch64") != 0) {
        return 0;
    }

    /* Run the tests */
    g_test_init(&argc, &argv, NULL);

    for (i = 0; i < ARRAY_SIZE(test_matrix); i++) {
        struct test_matrix *entry = &test_matrix[i];
        __make_test(entry->pattern, entry->test, entry->testfn);
    }

    ret = g_test_run();

    return ret;
}

