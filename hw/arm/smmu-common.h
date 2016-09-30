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
#ifndef HW_ARM_SMMU_COMMON_H
#define HW_ARM_SMMU_COMMON_H

#include <qemu/log.h>
#include <hw/sysbus.h>

#define TYPE_SMMU_DEV_BASE "smmu-base"
#define TYPE_SMMU_V3_DEV   "smmuv3"

typedef struct SMMUState {
    /* <private> */
    SysBusDevice  dev;

    uint32_t cid[4];            /* Coresight registers */
    uint32_t pid[8];

    MemoryRegion iomem;
} SMMUState;

#define SMMU_SYS_DEV(obj) OBJECT_CHECK(SMMUState, (obj), TYPE_SMMU_DEV_BASE)

typedef enum {
    SMMU_TRANS_ERR_WALK_EXT_ABRT = 0x1,  /* Translation walk external abort */
    SMMU_TRANS_ERR_TRANS         = 0x10, /* Translation fault */
    SMMU_TRANS_ERR_ADDR_SZ,              /* Address Size fault */
    SMMU_TRANS_ERR_ACCESS,               /* Access fault */
    SMMU_TRANS_ERR_PERM,                 /* Permission fault */
    SMMU_TRANS_ERR_TLB_CONFLICT  = 0x20, /* TLB Conflict */
} SMMUTransErr;


/*
 * This needs to be populated by SMMUv2 and SMMUv3
 * each do it in their own way
 * translate functions use it to call translations
 */
typedef struct SMMUTransCfg {
    hwaddr   va;                        /* Input to S1 */
    int      stage;
    uint32_t oas[3];
    uint32_t tsz[3];
    uint64_t ttbr[3];
    uint32_t granule[3];
    uint32_t va_size[3];
    uint32_t granule_sz[3];

    hwaddr pa;                          /* Output from S1, Final PA */
    bool    s2_needed;
} SMMUTransCfg;

struct SMMUTransReq {
    uint32_t stage;
    SMMUTransCfg cfg[2];
};

typedef struct {
    /* <private> */
    SysBusDeviceClass parent_class;

    /* public */
    SMMUTransErr (*translate_32)(SMMUTransCfg *cfg, uint32_t *pagesize,
                                 uint32_t *perm, bool is_write);
    SMMUTransErr (*translate_64)(SMMUTransCfg *cfg, uint32_t *pagesize,
                                 uint32_t *perm, bool is_write);
} SMMUBaseClass;

#define SMMU_DEVICE_GET_CLASS(obj)                              \
    OBJECT_GET_CLASS(SMMUBaseClass, (obj), TYPE_SMMU_DEV_BASE)

#define ARM_SMMU_DEBUG
#ifdef ARM_SMMU_DEBUG

extern uint32_t  dbg_bits;

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

#define SMMU_DPRINTF(lvl, fmt, ...)             \
    do {                                        \
        if (dbg_bits & DBG_BIT(lvl)) {          \
            qemu_log_mask(LOG_GUEST_IOMMU,      \
                          "(smmu)%s: " fmt ,    \
                          __func__,             \
                          ## __VA_ARGS__);      \
        }                                       \
    } while (0)

#else
#define IS_DBG_ENABLED(bit) false
#define SMMU_DPRINTF(lvl, fmt, ...)

#endif  /* SMMU_DEBUG */

SMMUTransErr smmu_translate_64(SMMUTransCfg *cfg, uint32_t *pagesize,
                               uint32_t *perm, bool is_write);

SMMUTransErr smmu_translate_32(SMMUTransCfg *cfg, uint32_t *pagesize,
                               uint32_t *perm, bool is_write);

MemTxResult smmu_read_sysmem(hwaddr addr, void *buf, int len, bool secure);
void smmu_write_sysmem(hwaddr addr, void *buf, int len, bool secure);

#endif  /* HW_ARM_SMMU_COMMON */
