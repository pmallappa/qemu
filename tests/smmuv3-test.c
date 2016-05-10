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

struct SMMUDevState {
        /* SMMU related fields */
        void *reg_base;
        uint64_t strtab_base;
        QGuestAllocator *strtab_alloc;

        QGuestAllocator *pgtbl_alloc;
        SMMUQueue cmdq;
};
typedef struct SMMUDevState SMMUDevState;

struct SMMUTestDevState {
        QPCIDevice *dev;
        void *reg_base;
};
typedef struct SMMUTestDevState SMMUTestDevState;

struct SMMUTestState {
        QPCIBus *pcibus;
        QTestState *qtest;

        QGuestAllocator *alloc;

        SMMUDevState sdev;

        SMMUTestDevState tdev;
};
typedef struct SMMUTestState SMMUTestState;


const struct SMMUTestConfig {
        uint8_t sid_size;
        uint8_t sid_split;
        uint32_t cmdq_shift;
} smmu_config = {
        8,
        16,
        3,                 /* log2 of queue size, 3 means 8 entry deep */
};

#if 0
static void test_smmu_init_cmdqwrap(void)
{
}

static void test_smmu_ste_valid(void)
{
}

static void test_smmu_ste_bypass(void)
{
}
#endif

static void abort_handler(void *data)
{
        g_test_message("abort handler called");
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

static QPCIBusGen virt_pci = {
        .base = 0x3f000000,
        .pci_hole_start = 0x10000000,
        .pci_hole_size = 0x2eff0000,
        .pci_hole_alloc = 0,
};

static void setup_vm_cmd(SMMUTestState *s, const char *cmd, bool msix)
{
        s->qtest = qtest_start(cmd);

        s->pcibus = qpci_init_generic(&virt_pci);
}

static void setup_vm(SMMUTestState *s, bool is_gdb_start)
{
        const char *gdb = is_gdb_start? "-s -S": "";
        const char *mon = is_gdb_start? " -chardev socket,id=mon0,host=localhost,port=6001,server,telnet,nowait -monitor chardev:mon0 ": "";
        char *cmd = g_strdup_printf(
                " -cpu cortex-a57 -m 4096 -machine virt " //-smp 4 "
                " -device i82801b11-bridge,multifunction=on,bus=pcie.0,addr=05,id=pcie.1 "
                " %s "
                " -device pci-testdev-smmu,bus=pcie.0,addr=04 "
                " %s ",
                mon, gdb);

        setup_vm_cmd(s, cmd, false);

        g_free(cmd);
}

static void cleanup_vm(SMMUTestState *s)
{
        qpci_free_generic(s->pcibus);
}

static void smmu_write64_reg(SMMUDevState *s, uint32_t reg, uint64_t val)
{
        uint64_t addr = (uint64_t)(s->reg_base + reg);
        writel(addr, val);
}

static void smmu_write_reg(SMMUDevState *s, uint32_t reg, uint64_t val)
{
        uint64_t addr = (uint64_t)(s->reg_base + reg);
        writew(addr, val);
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

static void smmu_setup(SMMUTestState *s)
{
        SMMUDevState *smmu = &s->sdev;
        /* Write command queue base, enable command queue and issue cmd_sync */
        /* This value is taken from hw/arm/virt.c, no portable way of getting it ? */
        smmu->reg_base = (void*)0x09050000;
}

static int smmu_init_cmdq(SMMUTestState *state)
{
        SMMUDevState *smmu = &state->sdev;
        SMMUQueue *q = &smmu->cmdq;

        q->shift = smmu_config.cmdq_shift;
        q->ent_size = sizeof(Cmd);
        q->entries = 1 << smmu_config.cmdq_shift;

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
        int split = smmu_config.sid_split;
        int l1_off = devfn >> split,
                l2_off = devfn & ~(1 << split);;

        printf("devfn:%x l1_off:%x l2_off:%x\n", devfn, l1_off, l2_off);
        stmp = smmu->strtab_base + (l1_off * sizeof(STEDesc));
        stm = readl(stmp);
        printf("stmp:%lx stm:%lx\n", stmp, stm);
        stm &= ~0x1fUL;              /* get rid of span */
        if (stm) {
                printf("already allocated ste \n");
        } else {
                uint64_t page;
                page = guest_alloc(smmu->strtab_alloc, sizeof(Ste) * (1 << split));
                printf("page:%lx\n", page);
                writeq(stmp, (page & ~0x1fUL) | devfn>>2); /* span is 1 */
                stm = readq(stmp) & ~0x1fUL;
                printf("stm:%lx\n", stm);
        }
        step = stm + (l2_off * sizeof(Ste));
        printf("step:%lx\n", step);
        return step;
}

static void smmu_strtab_initone(SMMUTestState *state, int devfn)
{

        SMMUDevState *smmu = &state->sdev;
        uint64_t step = smmu_get_ste(smmu, devfn);

        writew(step, 0x9);      /* bypass, valid */
}

#define MMIO_RAM_ADDR 0x40000000ULL
#define MMIO_RAM_SIZE 0x10000000ULL /* 256 MB */
#define MMIO_PAGE_SIZE 0x1000    /* 4k */

static int smmu_strtab_init(SMMUTestState *state)
{
        uint32_t size;
        SMMUDevState *smmu = &state->sdev;

        size = sizeof(STEDesc) * (1 << smmu_config.sid_split);
        smmu->strtab_base = guest_alloc(state->alloc, size);
        if (!smmu->strtab_base)
                return -ENOMEM;

        qmemset(smmu->strtab_base, 0, size);
        smmu_write64_reg(smmu, SMMU_REG_STRTAB_BASE, smmu->strtab_base);
        smmu_write64_reg(smmu, SMMU_REG_STRTAB_BASE_CFG, 0x10210);
        /*
         * Tables are located after the first
         * MMIO_RAM_ADDR + MMIO_RAM_SIZE
         */
        smmu->strtab_alloc = generic_alloc_init(MMIO_RAM_ADDR +
                                                (MMIO_RAM_SIZE>>1),
                                                MMIO_RAM_SIZE,
                                                sizeof(Ste));

        smmu->pgtbl_alloc = generic_alloc_init(MMIO_RAM_ADDR +
                                               (MMIO_RAM_SIZE),
                                               MMIO_RAM_SIZE,
                                               MMIO_PAGE_SIZE);

        smmu_strtab_initone(state, state->tdev.dev->devfn);

        return 0;
}

static int smmu_pgtable_alloc(SMMUTestState *s)
{
        SMMUDevState *smmu = &s->sdev;
        uint64_t step = smmu_get_ste(smmu, s->tdev.dev->devfn);

        uint64_t pgd = guest_alloc(smmu->
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

static SMMUTestState *test_smmu_setup(void)
{
        SMMUTestState *state = g_malloc(sizeof(SMMUTestState));

        setup_vm(state, false);

        state->alloc = generic_alloc_init(MMIO_RAM_ADDR +
                                          (MMIO_RAM_SIZE>>2),
                                          MMIO_RAM_SIZE, MMIO_PAGE_SIZE);
        smmu_setup(state);

        testdev_setup(state);

        if (smmu_init(state))
                return NULL;

        testdev_init(&state->tdev);

        return state;
}

static void test_smmu_cmdq(void)
{

        /*
         * First we setup command queue to be 64 entry deep and
         * test if it wraps around correctly
         */
        /* Actually it does, tested with Linux driver */

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

static void test_smmu_tt_s1_s2(void)
{

}

SMMUTestState *state; // = test_smmu_setup();

static void update_pgtable(SMMUDevState *smmu, SMMUTestDevState *dev,
                           uint64_t src);
{
        
}
static void test_smmu_tt_s1(void)
{
        uint64_t src, dst;
        int i;

#define TST_BUFFER_SIZE 0x200
        src = guest_alloc(state->alloc, TST_BUFFER_SIZE);
        dst = guest_alloc(state->alloc, TST_BUFFER_SIZE);

        /* Fill array with integers */
        for (i = 0; i < TST_BUFFER_SIZE/sizeof(uint16_t); i++) {
                writew(src + i*sizeof(uint16_t), i);
        }

        /* Install tables for this device and src/dst addresses */
        update_pgtable(&state->sdev, &state->tdev, src);
        update_pgtable(&state->sdev, &state->tdev, dst);

        /* Start dma */
        testdev_dma(&state->tdev, (void*)src, (void*)dst,
                    TST_BUFFER_SIZE);

        for (i = 0; i < TST_BUFFER_SIZE/sizeof(uint16_t); i++) {
                hwaddr s_addr = src + i*(sizeof(uint16_t)),
                        d_addr = dst + i*sizeof(uint16_t);
                if (readw(s_addr) != readw(d_addr)) {
                        printf("No match off:%d src:%x dst:%x\n",
                               i, readw(s_addr), readw(d_addr));
                        return;
                }
        }
}

static void cleanup(SMMUTestState *state)
{
        printf("Cleanup called\n");
        generic_alloc_uninit(state->alloc);

        cleanup_vm(state);
}

int main(int argc, char **argv)
{
        int ret = 0;

        //SMMUTestState *state; // = g_malloc(sizeof(SMMUTestState));

        if (strcmp(qtest_get_arch(), "aarch64") != 0) {
                return 0;
        }

        /* Run the tests */
        g_test_init(&argc, &argv, NULL);

        qtest_add_abrt_handler(abort_handler, NULL);

        state = test_smmu_setup();

        if (!state)
                goto out;

        qtest_add_func("/smmuv3/init/cmdq", test_smmu_cmdq);
        /* qtest_add_func("/smmu/init/cmdqwrap", test_smmu_init_cmdqwrap); */
        /* qtest_add_func("/smmu/ste/validate", test_smmu_ste_valid); */
        /* qtest_add_func("/smmu/ste/bypass", test_smmu_ste_bypass); */
        qtest_add_func("/smmu/tt/s1", test_smmu_tt_s1);
        qtest_add_func("smmu/tt/s1s2", test_smmu_tt_s1_s2);

        ret = g_test_run();
out:
        cleanup(state);
        return ret;
}

#if 0

(smmu)smmu_write_mmio: reg:20 cur: 0 new: 0
(smmu)smmu_write_mmio: reg:28 cur: 0 new: d75
(smmu)smmu_write_mmio: reg:2c cur: 0 new: 7
(smmu)smmu_write_mmio: reg:80 cur: 0 new: e0070000
updated base:e0070000 val:e0070000 read_reg:e0070000 addr:80
(smmu)smmu_write_mmio: reg:84 cur: 0 new: 40000000
updated base:e0070000 val:40000000e0070000 read_reg:e0070000 addr:80
(smmu)smmu_write_mmio: reg:88 cur: 0 new: 10210
(smmu)smmu_write_mmio: reg:90 cur: 0 new: e0040008
updated base:e0040000 val:e0040008 read_reg:e0040000 addr:90
(smmu)smmu_write_mmio: reg:94 cur: 0 new: 40000000
updated base:e0040000 val:40000000e0040008 read_reg:e0040000 addr:90
(smmu)smmu_write_mmio: reg:98 cur: 0 new: 0
(smmu)smmu_write_mmio: reg:9c cur: 0 new: 0
(smmu)smmu_write_mmio: reg:20 cur: 0 new: 8
(smmu)smmu_read_mmio: addr: 24 val:8
(smmu)smmu_write_mmio: reg:98 cur: 0 new: 1
(smmu)smmu_cmdq_consume: CMDQ base: e0040000 cons:0 prod:1 val:4 wrap:0
(smmu)smmu_write_mmio: reg:98 cur: 1 new: 2
(smmu)smmu_cmdq_consume: CMDQ base: e0040000 cons:1 prod:2 val:2046 wrap:0
(smmu)smmu_read_mmio: addr: 9c val:2
(smmu)smmu_write_mmio: reg:98 cur: 2 new: 3
(smmu)smmu_cmdq_consume: CMDQ base: e0040000 cons:2 prod:3 val:20 wrap:0
(smmu)smmu_write_mmio: reg:98 cur: 3 new: 4
(smmu)smmu_cmdq_consume: CMDQ base: e0040000 cons:3 prod:4 val:30 wrap:0
(smmu)smmu_write_mmio: reg:98 cur: 4 new: 5
(smmu)smmu_cmdq_consume: CMDQ base: e0040000 cons:4 prod:5 val:2046 wrap:0
(smmu)smmu_read_mmio: addr: 9c val:5
(smmu)smmu_write_mmio: reg:a0 cur: 0 new: e0050007
updated base:e0050000 val:e0050007 read_reg:e0050000 addr:a0
(smmu)smmu_write_mmio: reg:a4 cur: 0 new: 40000000
updated base:e0050000 val:40000000e0050007 read_reg:e0050000 addr:a0
(smmu)smmu_write_mmio: reg:20 cur: 8 new: c
(smmu)smmu_read_mmio: addr: 24 val:c
(smmu)smmu_write_mmio: reg:c0 cur: 0 new: e0060008
(smmu)smmu_write_mmio: reg:c4 cur: 0 new: 40000000
(smmu)smmu_write_mmio: reg:20 cur: c new: e
(smmu)smmu_read_mmio: addr: 24 val:e
(smmu)smmu_write_mmio: reg:50 cur: 0 new: 0
(smmu)smmu_read_mmio: addr: 54 val:0
(smmu)smmu_write_mmio: reg:68 cur: 0 new: 0
(smmu)smmu_write_mmio: reg:6c cur: 0 new: 0
(smmu)smmu_write_mmio: reg:b0 cur: 0 new: 0
(smmu)smmu_write_mmio: reg:b4 cur: 0 new: 0
(smmu)smmu_write_mmio: reg:d0 cur: 0 new: 0
(smmu)smmu_write_mmio: reg:d4 cur: 0 new: 0
(smmu)smmu_write_mmio: reg:50 cur: 0 new: 7
(smmu)smmu_read_mmio: addr: 54 val:0
(smmu)smmu_read_mmio: addr: 54 val:0
(smmu)smmu_read_mmio: addr: 54 val:0
(smmu)smmu_write_mmio: reg:20 cur: e new: f
(smmu)smmu_update: An unfavourable condition
(smmu)smmu_read_mmio: addr: 24 val:f
(smmu)smmuv3_translate: SID:10 bus:0 ste_base:e0070000
(smmu)smmu_find_ste: SID:10
(smmu)smmu_find_ste: no. ste: 8
(smmu)smmu_find_ste: strtab_base:e0070000 stm_addr:e0070000
l1_ste_offset:0 l1(64):0x000000e0080000
(smmu)smmu_find_ste: l2_ste_offset:10 ~ span:256
(smmu)dump_ste: STE[ 0]: 0x00000009      STE[ 1]: 0000000000
(smmu)dump_ste: STE[ 2]: 0000000000      STE[ 3]: 0x00001000
(smmu)dump_ste: STE[ 4]: 0xe6dcce20      STE[ 5]: 0x00007f18

#endif
