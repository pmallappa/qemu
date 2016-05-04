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

/* IVSHMEM */

/* SMMU */
#include "hw/arm/smmu-common.h"
#include "hw/arm/smmuv3-internal.h"


/* 1. Create a PCI device ("edu" in this case) for data transfer */
/* 2. Initialize SMMU */
/* 3. SMMU table created for data transfer */
/* 4. */

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

static void test_smmu_tt_s1(void)
{
}

static void test_smmu_tt_s1_s2(void)
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
        printf("a\n");
        dev = NULL;
        qpci_device_foreach(pcibus, 0x1b36, 0x0005, save_fn, &dev);
        printf("b\n");
        //while(1);
        g_assert(dev != NULL);
        printf("c\n");
        return dev;
}

struct SMMUTestState {
        QPCIBus *pcibus;
        QPCIDevice *dev;
        QTestState *qtest;
        void *reg_base;

        /* SMMU related fields */
        void *smmu_reg_base;
};
typedef struct SMMUTestState SMMUTestState;

static QPCIBusGen virt_pci = {
        .base = 0x3f000000,
        .pci_hole_start = 0x10000000,
        .pci_hole_size = 0x2eff0000,
        .pci_hole_alloc = 0,
};

static void setup_vm_cmd(SMMUTestState *s, const char *cmd, bool msix)
{
        uint64_t barsize;
        printf("Hello 0 \n");
        s->qtest = qtest_start(cmd);
        printf("Hello 1 \n");
        s->pcibus = qpci_init_generic(&virt_pci);
        printf("Hello 2 \n");
        s->dev = get_device(s->pcibus);

        g_assert_nonnull(s->dev);
        printf("Hello 3 \n");
        s->reg_base = qpci_iomap(s->dev, 0, &barsize);
        g_assert_nonnull(s->reg_base);
        qpci_device_enable(s->dev);
}

static void setup_vm(SMMUTestState *s, bool is_gdb_start)
{
        const char *gdb = is_gdb_start? "-s -S": "";
        const char *mon = is_gdb_start? " -chardev socket,id=mon0,host=localhost,port=6001,server,telnet,nowait -monitor chardev:mon0 ": "";
        char *cmd = g_strdup_printf(
                " -cpu cortex-a57 -m 4192 -machine virt " //-smp 4 "
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

static void smmu_write_reg(SMMUTestState *s, uint32_t reg, uint64_t val)
{
        uint64_t addr = (uint64_t)(s->smmu_reg_base + reg);
        writew(addr, val);
}


static void test_smmu_init(SMMUTestState *s)
{
        /* Write command queue base, enable command queue and issue cmd_sync */
}

static SMMUTestState *test_smmu_setup(void)
{
        SMMUTestState *state = g_malloc(sizeof(SMMUTestState));

        setup_vm(state, false);

        /* This value is taken from hw/arm/virt.c, no portable way of getting it ? */
        state->smmu_reg_base = (void*)0x09050000;

        return state;
}

static void test_smmu_cmdq(void)
{
        SMMUTestState *state; // = g_malloc(sizeof(SMMUTestState));

        state = test_smmu_setup();

        /*
         * First we setup command queue to be 64 entry deep and
         * test if it wraps around correctly
         */
        /* Actually it does, tested with Linux driver */

        cleanup_vm(state);
}


static void test_smmu_tt_s1(void)
{

}

static void test_smmu_tt_s1(void)
{

}

int main(int argc, char **argv)
{
        int ret = 0;

        if (strcmp(qtest_get_arch(), "aarch64") != 0) {
                return 0;
        }

        /* Run the tests */
        g_test_init(&argc, &argv, NULL);

        qtest_add_abrt_handler(abort_handler, NULL);

        qtest_add_func("/smmuv3/init/cmdq", test_smmu_cmdq);
        /* qtest_add_func("/smmu/init/cmdqwrap", test_smmu_init_cmdqwrap); */
        /* qtest_add_func("/smmu/ste/validate", test_smmu_ste_valid); */
        /* qtest_add_func("/smmu/ste/bypass", test_smmu_ste_bypass); */
        qtest_add_func("/smmu/tt/s1", test_smmu_tt_s1);
        qtest_add_func("smmu/tt/s1s2", test_smmu_tt_s1_s2);

        ret = g_test_run();

        return ret;
}
