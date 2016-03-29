/*
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
 * Copyright (C) 2015-2016 Broadcom Corporation
 *
 * Author: Prem Mallappa <pmallapp@broadcom.com>
 *
 */
#ifndef HW_ARM_SMMU_H
#define HW_ARM_SMMU_H

#define TYPE_SMMU_DEV_BASE "smmu-base"
#define TYPE_SMMU_V3_DEV   "smmuv3"
//#define TYPE_SMMU_500_DEV "smmu-500"

typedef struct SMMUState {
    /* <private> */
    SysBusDevice  dev;

    uint32_t cid[4];            /* Coresight registers */
    uint32_t pid[8];

    MemoryRegion iomem;
} SMMUState;

#define SMMU_SYS_DEV(obj) OBJECT_CHECK(SMMUState, (obj), TYPE_SMMU_DEV_BASE)

typedef struct {
    /* <private> */
    SysBusDeviceClass parent_class;
} SMMUBaseClass;
#define SMMU_DEVICE_GET_CLASS(obj)                              \
    OBJECT_GET_CLASS(SMMUBaseClass, (obj), TYPE_SMMU_DEV_BASE)

/*
 * SMMUV3 related
 */
typedef enum {
    SMMU_IRQ_GERROR,
    SMMU_IRQ_PRIQ,
    SMMU_IRQ_EVTQ,
    SMMU_IRQ_CMD_SYNC,
} SMMUIrq;

#endif
