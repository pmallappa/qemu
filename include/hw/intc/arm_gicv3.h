/*
 * ARM GIC support
 *
 * Copyright (c) 2012 Linaro Limited
 * Copyright (c) 2015 Huawei.
 * Written by Peter Maydell
 * Extended to 64 cores by Shlomo Pongratz
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_ARM_GICV3_H
#define HW_ARM_GICV3_H

#include "arm_gicv3_common.h"

#define TYPE_ARM_GICV3 "arm_gicv3"
#define ARM_GICV3(obj) \
     OBJECT_CHECK(GICv3State, (obj), TYPE_ARM_GICV3)
#define ARM_GICV3_CLASS(klass) \
     OBJECT_CLASS_CHECK(ARMGICv3Class, (klass), TYPE_ARM_GICV3)
#define ARM_GICV3_GET_CLASS(obj) \
     OBJECT_GET_CLASS(ARMGICv3Class, (obj), TYPE_ARM_GICV3)

typedef struct ARMGICv3Class {
    /*< private >*/
    ARMGICv3CommonClass parent_class;
    /*< public >*/

    DeviceRealize parent_realize;
} ARMGICv3Class;

#endif
