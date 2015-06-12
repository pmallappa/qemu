/*
 * support for vsmmu interface to use it with vfio devices
 *
 * Copyright (C) 2015 - Virtual Open Systems
 *
 * Author: Baptiste Reynal <b.reynal at virtualopensystems.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

#include <sys/ioctl.h>

#include "hw/vfio/vfio-smmu.h"
#include "sysemu/kvm.h"
#include "sysemu/sysemu.h"
#include "qemu/error-report.h"
#include "hw/platform-bus.h"
#include "hw/vfio/vfio.h"
#include "hw/vfio/vfio-common.h"

static void vsmmu_notify(Notifier *notifier, void *data)
{
    SmmuNotifierParams *p = DO_UPCAST(SmmuNotifierParams,
                                                notifier, notifier);

    DeviceState *dev;
    PlatformBusDevice *pbus;
    SysBusDevice *sbdev = SYS_BUS_DEVICE(p->vsmmu);

    dev = qdev_find_recursive(sysbus_get_default(), TYPE_PLATFORM_BUS_DEVICE);
    pbus = PLATFORM_BUS_DEVICE(dev);
    assert(pbus->done_gathering);

    hwaddr base = platform_bus_get_mmio_addr(pbus, sbdev, 0);
    base += pbus->base_address;

    struct kvm_device_attr attr = {
        .group = KVM_DEV_ARM_SMMU_V2_CFG,
        .attr = KVM_DEV_ARM_SMMU_V2_CFG_INIT,
        .addr = (uint64_t)(unsigned long) &base,
    };

    if (ioctl(p->vsmmu->kvm_device, KVM_SET_DEVICE_ATTR, &attr)) {
        error_report("Error during vSMMU initialization: %m.\n");
    } else {
        p->vsmmu->base = base;
    }
}

static void smmu_realize(DeviceState *dev, Error **errp)
{
    VFIOSmmuDevice *vsmmu = VFIO_SMMU_DEVICE(dev);
    SysBusDevice *sbdev = SYS_BUS_DEVICE(dev);
    VFIOGroup *group;
    int ret;
    const char *name = "smmu memory";

    group = vfio_get_group(vsmmu->group, &address_space_memory);
    if (!group) {
        error_report("vfio: failed to get group %d", vsmmu->group);
        goto fail;
    }

    /* Create ARM_SMMU_V2 */
    struct kvm_create_device cd = {
        .type = KVM_DEV_TYPE_ARM_SMMU_V2,
    };

    ret = kvm_vm_ioctl(kvm_state, KVM_CREATE_DEVICE, &cd);
    if (ret < 0) {
        error_report("vfio: error creating vSMMU: %m");
        goto fail;
    };
    vsmmu->kvm_device = cd.fd;

    /* Add group */
    struct arm_smmu_v2_vfio_group_sid group_sid = {
        .fd = group->fd,
        .sid = group->groupid,
    };

    struct kvm_device_attr attr = {
        .group = KVM_DEV_ARM_SMMU_V2_VFIO,
        .attr = KVM_DEV_ARM_SMMU_V2_VFIO_GROUP_ADD,
        .addr = (uint64_t)(unsigned long) &group_sid,
    };

    if (ioctl(cd.fd, KVM_SET_DEVICE_ATTR, &attr)) {
        error_report("Failed to add group %d to vSMMU: %m",
                 group->groupid);
        goto fail;
    }

    /* Initialize the virtual device */
    /* Get vSMMU size for allocation */
    int size;

    attr.group = KVM_DEV_ARM_SMMU_V2_CFG;
    attr.attr = KVM_DEV_ARM_SMMU_V2_CFG_SIZE;
    attr.addr = (uint64_t)(unsigned long) &size;

    ret = ioctl(cd.fd, KVM_GET_DEVICE_ATTR, &attr);

    if (ret) {
        error_report("Failed to get vSMMU size: %m");
        goto fail;
    }

    vsmmu->size = size;

    memory_region_init_reservation(&vsmmu->mem, OBJECT(vsmmu), name, size);
    sysbus_init_mmio(sbdev, &vsmmu->mem);

    /* Register a machine init done notifier to initialize the
     * vSMMU at the right address */
    SmmuNotifierParams *p = g_new(SmmuNotifierParams, 1);
    p->vsmmu = vsmmu;
    p->notifier.notify = vsmmu_notify;
    qemu_add_platform_bus_link_done_notifier(&p->notifier);
fail:
    return;
}

static const VMStateDescription vfio_platform_vmstate = {
    .name = TYPE_VFIO_SMMU,
    .unmigratable = 1,
};

static Property vfio_smmu_dev_properties[] = {
    DEFINE_PROP_UINT32("x-group", VFIOSmmuDevice,
                       group, -1),
    DEFINE_PROP_END_OF_LIST(),
};

static void vfio_smmu_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = smmu_realize;
    dc->desc = "VFIO SMMU";
    dc->props = vfio_smmu_dev_properties;
}

static const TypeInfo vfio_smmu_dev_info = {
    .name = TYPE_VFIO_SMMU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(VFIOSmmuDevice),
    .class_init = vfio_smmu_class_init,
    .class_size = sizeof(VFIOSmmuDeviceClass),
};

static void register_smmu_dev_type(void)
{
    type_register_static(&vfio_smmu_dev_info);
}

type_init(register_smmu_dev_type)
