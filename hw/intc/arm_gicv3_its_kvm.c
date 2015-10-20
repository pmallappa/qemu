/*
 * KVM-based ITS implementation for a GICv3-based system
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd.
 * Written by Pavel Fedin <p.fedin@samsung.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "hw/intc/arm_gicv3_its_common.h"
#include "sysemu/kvm.h"
#include "kvm_arm.h"

#define TYPE_KVM_ARM_ITS "arm-its-kvm"
#define KVM_ARM_ITS(obj) OBJECT_CHECK(GICv3ITSState, (obj), TYPE_KVM_ARM_ITS)

static int kvm_its_send_msi(GICv3ITSState *s, uint32_t value, uint16_t devid)
{
    struct kvm_msi msi;

    msi.address_lo = 0x0040;
    msi.address_hi = 0;
    msi.data = value;
    msi.flags = KVM_MSI_VALID_DEVID;
    msi.devid = devid;
    memset(msi.pad, 0, sizeof(msi.pad));

    return kvm_vm_ioctl(kvm_state, KVM_SIGNAL_MSI, &msi);
}

static void kvm_arm_its_realize(DeviceState *dev, Error **errp)
{
    GICv3ITSState *s = ARM_GICV3_ITS_COMMON(dev);

    gicv3_its_init_mmio(s, NULL);
    kvm_arm_register_device(&s->iomem_its_cntrl, -1, KVM_DEV_ARM_VGIC_GRP_ADDR,
                            KVM_VGIC_V3_ADDR_TYPE_ITS, s->gicv3->dev_fd);

    kvm_arm_msi_use_devid = true;
    kvm_gsi_routing_allowed = kvm_has_gsi_routing();
    kvm_msi_via_irqfd_allowed = kvm_gsi_routing_allowed;
}

static void kvm_arm_its_init(Object *obj)
{
    GICv3ITSState *s = KVM_ARM_ITS(obj);

    object_property_add_link(obj, "parent-gicv3",
                             "kvm-arm-gicv3", (Object **)&s->gicv3,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_UNREF_ON_RELEASE,
                             &error_abort);
}

static void kvm_arm_its_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    GICv3ITSCommonClass *icc = ARM_GICV3_ITS_COMMON_CLASS(klass);

    dc->realize = kvm_arm_its_realize;
    icc->send_msi = kvm_its_send_msi;
}

static const TypeInfo kvm_arm_its_info = {
    .name = TYPE_KVM_ARM_ITS,
    .parent = TYPE_ARM_GICV3_ITS_COMMON,
    .instance_size = sizeof(GICv3ITSState),
    .instance_init = kvm_arm_its_init,
    .class_init = kvm_arm_its_class_init,
};

static void kvm_arm_its_register_types(void)
{
    type_register_static(&kvm_arm_its_info);
}

type_init(kvm_arm_its_register_types)
