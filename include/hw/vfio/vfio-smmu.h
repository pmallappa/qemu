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

#ifndef HW_VFIO_SMMU_H
#define HW_VFIO_SMMU_H

#include "hw/sysbus.h"

#define TYPE_VFIO_SMMU "vfio-smmu"

typedef struct VFIOSmmuDevice {
    SysBusDevice sbdev;
    int kvm_device;
    int size;
    hwaddr base;

    MemoryRegion mem;

    uint32_t group;
} VFIOSmmuDevice;

typedef struct SmmuNotifierParams {
    Notifier notifier;
    VFIOSmmuDevice *vsmmu;
} SmmuNotifierParams;

typedef struct VFIOSmmuDeviceClass {
    /*< private >*/
    SysBusDeviceClass parent_class;
    /*< public >*/
} VFIOSmmuDeviceClass;

#define VFIO_SMMU_DEVICE(obj) \
     OBJECT_CHECK(VFIOSmmuDevice, (obj), TYPE_VFIO_SMMU)
#define VFIO_SMMU_DEVICE_CLASS(klass) \
     OBJECT_CLASS_CHECK(VFIOSmmuDeviceClass, (klass), \
                        TYPE_VFIO_SMMU)
#define VFIO_SMMU_DEVICE_GET_CLASS(obj) \
     OBJECT_GET_CLASS(VFIOSmmuDeviceClass, (obj), \
                        TYPE_VFIO_SMMU)

#endif
