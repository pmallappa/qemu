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
 * Copyright (C) 2014-2015 Broadcom Corporation
 *
 * Author: Prem Mallappa <pmallapp@broadcom.com>
 *
 */
#ifndef __ARM_SMMU_V3_H__
#define __ARM_SMMU_V3_H__

#include "exec/memory.h"
#include "hw/qdev.h"
#include "sysemu/dma.h"

enum {
    SMMU_IRQ_GERROR,
    SMMU_IRQ_PRIQ,
    SMMU_IRQ_EVTQ,
    SMMU_IRQ_CMD_SYNC,
};

#define GENMASK(h, l)                                           \
    (((~0UL) << (l)) & (~0UL >> (BITS_PER_LONG - 1 - (h))))

#define __SMMU_MASK(x)	GENMASK(((x)-1), 0)

/*****************************
 * MMIO Register
 *****************************/
enum {
    SMMU_REG_IDR0	     = 0x0,
    SMMU_REG_IDR1	     = 0x4,
    SMMU_REG_IDR2	     = 0x8,
    SMMU_REG_IDR3	     = 0xc,
    SMMU_REG_IDR4	     = 0x10,
    SMMU_REG_IDR5	     = 0x14,
    SMMU_REG_IIDR	     = 0x1c,
    SMMU_REG_CR0	     = 0x20,
    SMMU_REG_CR0_ACK         = 0x24,
    SMMU_REG_CR1	     = 0x28,
    SMMU_REG_CR2	     = 0x2c,

    SMMU_REG_STATUSR         = 0x40,

    SMMU_REG_IRQ_CTRL        = 0x50,
    SMMU_REG_IRQ_CTRL_ACK    = 0x54,

    SMMU_REG_GERROR	     = 0x60,
    SMMU_REG_GERRORN         = 0x64,
    SMMU_REG_GERROR_IRQ_CFG0 = 0x68,
    SMMU_REG_GERROR_IRQ_CFG1 = 0x70,

    SMMU_REG_STRTAB_BASE     = 0x80,
    SMMU_REG_STRTAB_BASE_CFG = 0x88,

    SMMU_REG_CMDQ_BASE       = 0x90,
    SMMU_REG_CMDQ_PROD       = 0x98,
    SMMU_REG_CMDQ_CONS       = 0x9c,

    SMMU_REG_EVTQ_BASE       = 0xa0,
    SMMU_REG_EVTQ_PROD       = 0xa8,
    SMMU_REG_EVTQ_CONS       = 0xac,
    SMMU_REG_EVTQ_IRQ_CFG0   = 0xb0,
    SMMU_REG_EVTQ_IRQ_CFG1   = 0xb8,

    SMMU_REG_PRIQ_BASE       = 0xc0,
    SMMU_REG_PRIQ_PROD       = 0xc8,
    SMMU_REG_PRIQ_CONS       = 0xcc,
    SMMU_REG_PRIQ_IRQ_CFG0   = 0xd0,
    SMMU_REG_PRIQ_IRQ_CFG1   = 0xd8,

    SMMU_ID_REGS_OFFSET      = 0xfd0,

    /* Secure registers are not used for now */
    SMMU_SECURE_OFFSET       = 0x8000,
};

/* IRQ Bits */
#define SMMU_IRQ_CTRL_GERROR_EN (1 << 0)
#define SMMU_IRQ_CTRL_EVENT_EN  (1 << 1)
#define SMMU_IRQ_CTRL_PRI_EN    (1 << 2)

/* CR0 Bits */
#define SMMU_CR0_SMMU_ENABLE (1 << 0)
#define SMMU_CR0_PRIQ_ENABLE (1 << 1)
#define SMMU_CR0_EVTQ_ENABLE (1 << 2)
#define SMMU_CR0_CMDQ_ENABLE (1 << 3)
#define SMMU_CR0_ATS_CHECK   (1 << 4)


/*****************************
 * Commands
 *****************************/
enum {
    SMMU_CMD_PREFETCH_CONFIG = 0x01,
    SMMU_CMD_PREFETCH_ADDR,
    SMMU_CMD_CFGI_STE,
    SMMU_CMD_CFGI_STE_RANGE,
    SMMU_CMD_CFGI_CD,
    SMMU_CMD_CFGI_CD_ALL,
    SMMU_CMD_TLBI_NH_ALL     = 0x10,
    SMMU_CMD_TLBI_NH_ASID,
    SMMU_CMD_TLBI_NH_VA,
    SMMU_CMD_TLBI_NH_VAA,
    SMMU_CMD_TLBI_EL3_ALL    = 0x18,
    SMMU_CMD_TLBI_EL3_VA     = 0x1a,
    SMMU_CMD_TLBI_EL2_ALL    = 0x20,
    SMMU_CMD_TLBI_EL2_ASID,
    SMMU_CMD_TLBI_EL2_VA,
    SMMU_CMD_TLBI_EL2_VAA,  /* 0x23 */
    SMMU_CMD_TLBI_S12_VMALL  = 0x28,
    SMMU_CMD_TLBI_S2_IPA     = 0x2a,
    SMMU_CMD_TLBI_NSNH_ALL   = 0x30,
    SMMU_CMD_ATC_INV         = 0x40,
    SMMU_CMD_PRI_RESP,
    SMMU_CMD_RESUME	     = 0x44,
    SMMU_CMD_STALL_TERM,
    SMMU_CMD_SYNC,          /* 0x46 */
};

/* CMD Consumer (CONS) */
#define SMMU_CMD_CONS_ERR_SHIFT        24
#define SMMU_CMD_CONS_ERR_BITS         7

/* GERROR/GERRORN Bits */
#define SMMU_GERROR_CMDQ       (1 << 0)
#define SMMU_GERROR_EVENTQ     (1 << 1)
#define SMMU_GERROR_PRIQ       (1 << 2)
#define SMMU_GERROR_MSI_CMDQ   (1 << 3)
#define SMMU_GERROR_MSI_EVENTQ (1 << 4)
#define SMMU_GERROR_MSI_PRIQ   (1 << 5)
#define SMMU_GERROR_MSI_GERROR (1 << 6)
#define SMMU_GERROR_SFM_ERR    (1 << 7)


/* Command Errors */
enum {
    SMMU_CMD_ERR_NONE	= 0,
    SMMU_CMD_ERR_ILLEGAL,
    SMMU_CMD_ERR_ABORT
};

enum {
    SMMU_CMD_PRI_RESP_DENY	= 0,
    SMMU_CMD_PRI_RESP_FAIL,
    SMMU_CMD_PRI_RESP_SUCCESS,
};

/* Command Entry Fields, some are overlapping */
#define CMD_TYPE_SHIFT    0	/* WORD 0 */
#define CMD_TYPE_BITS     8
#define CMD_SEC_SHIFT     9
#define CMD_SEC_BITS      1
#define CMD_SEV_SHIFT     10
#define CMD_SEV_BITS      1
#define CMD_SSV_SHIFT     11
#define CMD_SSV_BITS      1
#define CMD_AC_SHIFT      12
#define CMD_AC_BITS       1
#define CMD_AB_SHIFT      13
#define CMD_AB_BITS       1
#define CMD_SSID_SHIFT    16
#define CMD_SSID_BITS     20
#define CMD_VMID_SHIFT    0
#define CMD_VMID_BITS     16
#define CMD_ASID_SHIFT    16
#define CMD_ASID_BITS     16
#define CMD_STAG_SHIFT    0	/* WORD 2 */
#define CMD_STAG_BITS     16
#define CMD_RESP_SHIFT    11
#define CMD_RESP_BITS     2
#define CMD_GRPID_SHIFT   0	/* WORD 3 */
#define CMD_GRPID_BITS    8
#define CMD_SIZE_SHIFT    0
#define CMD_SIZE_BITS     16
#define CMD_LEAF_SHIFT    0
#define CMD_LEAF_BITS     1
#define CMD_SPAN_SHIFT    0
#define CMD_SPAN_BITS     5
#define CMD_ADDR_HI_SHIFT 0	/* WORD 4 */
#define CMD_ADDR_HI_BITS  32
#define CMD_ADDR_LO_SHIFT 12
#define CMD_ADDR_LO_BITS  20

#define __SET_FLD(var, mask, shift, val)        \
    ({ var |= ((val) & (mask)) << (shift); })

#define __GET_FLD(val, mask, shift)		\
    (((val) >> (shift)) & (mask))

#define __CMD_GET_FLD(x, idx, val)                              \
    (__GET_FLD((x)->word[(idx)], __SMMU_MASK(CMD_##val##_BITS),	\
               CMD_##val##_SHIFT))

#define CMD_SET_TYPE(cmd, type) ((cmd)->word[0] |= (type))

#define CMD_TYPE(x)  __CMD_GET_FLD((x), 0, TYPE)
#define CMD_SEC(x)   __CMD_GET_FLD((x), 0, SEC)
#define CMD_SEV(x)   __CMD_GET_FLD((x), 0, SEV)
#define CMD_AC(x)    __CMD_GET_FLD((x), 0, AC)
#define CMD_AB(x)    __CMD_GET_FLD((x), 0, AB)
#define CMD_SSID(x)  __CMD_GET_FLD((x), 0, SSID)
#define CMD_SID(x)   ((x)->word[1])
#define CMD_VMID(x)  __CMD_GET_FLD((x), 1, VMID)
#define CMD_ASID(x)  __CMD_GET_FLD((x), 1, ASID)
#define CMD_STAG(x)  __CMD_GET_FLD((x), 2, STAG)
#define CMD_RESP(x)  __CMD_GET_FLD((x), 2, RESP)
#define CMD_GRPID(x) __CMD_GET_FLD((x), 2, GRPID)
#define CMD_SIZE(x)  __CMD_GET_FLD((x), 2, SIZE)
#define CMD_LEAF(x)  __CMD_GET_FLD((x), 2, LEAF)
#define CMD_SPAN(x)  __CMD_GET_FLD((x), 2, SPAN)
#define CMD_ADDR(x) ({                                  \
            uint64_t addr = (uint64_t)(x)->word[3];               \
            addr <<= 32;                                \
            addr |=  __CMD_GET_FLD((x), 0x3, ADDR_LO);	\
            addr;                                       \
        })


/*****************************
 * Events
 *****************************/
#define EVT_Q_OVERFLOW        (1<<31)
enum {
    SMMU_EVT_F_UUT		= 0x1,
    SMMU_EVT_C_BAD_SID,
    SMMU_EVT_F_STE_FETCH,
    SMMU_EVT_C_BAD_STE,
    SMMU_EVT_F_BAD_ATS_REQ,
    SMMU_EVT_F_STREAM_DISABLED,
    SMMU_EVT_F_TRANS_FORBIDDEN,
    SMMU_EVT_C_BAD_SSID,
    SMMU_EVT_F_CD_FETCH,
    SMMU_EVT_C_BAD_CD,
    SMMU_EVT_F_WALK_EXT_ABRT,
    SMMU_EVT_F_TRANS	= 0x10,
    SMMU_EVT_F_ADDR_SZ,
    SMMU_EVT_F_ACCESS,
    SMMU_EVT_F_PERM,
    SMMU_EVT_F_TLB_CONFLICT = 0x20,
    SMMU_EVT_F_CFG_CONFLICT = 0x21,
    SMMU_EVT_E_PAGE_REQ	= 0x24,
};

#define EVT_EVT_BITS	8
#define EVT_EVT_SHIFT	0
#define EVT_ME_BITS	1
#define EVT_ME_SHIFT	10
#define EVT_SSV_BITS	1
#define EVT_SSV_SHIFT	11
#define EVT_SSID_BITS	16
#define EVT_SSID_SHIFT	(EVT_SSV_SHIFT + EVT_SSV_BITS)
#define EVT_SPAN_BITS	4
#define EVT_SPAN_SHIFT	0
#define EVT_P_BITS	1
#define EVT_P_SHIFT	28
#define EVT_X_BITS	1
#define EVT_X_SHIFT	29	/*  */
#define EVT_W_BITS	1
#define EVT_W_SHIFT	30
#define EVT_R_BITS	1
#define EVT_R_SHIFT	31
#define EVT_PNU_BITS	1
#define EVT_PNU_SHIFT	1
#define EVT_IND_BITS	1
#define EVT_IND_SHIFT	2
#define EVT_RNW_BITS	1
#define EVT_RNW_SHIFT	3
#define EVT_S2_BITS	1
#define EVT_S2_SHIFT	7
#define EVT_CLS_BITS	2
#define EVT_CLS_SHIFT	8
#define __EVT_SET_FLD(x, off, name, val)                         \
    (__SET_FLD((x)->word[(off)], __SMMU_MASK(EVT_##name##_BITS), \
               EVT_##name##_SHIFT, val))

#define __EVT_GET_FLD(x, off, val)                               \
    (__GET_FLD((x)->word[(off)], __SMMU_MASK(EVT_##val##_BITS),  \
               EVT_##val##_SHIFT))

#define EVT_SET_TYPE(x, t)    __EVT_SET_FLD((x), 0, EVT, t)
#define EVT_SET_ME(x, m)      __EVT_SET_FLD((x), 0, ME, m)
#define EVT_SET_SSV(x, s)     __EVT_SET_FLD((x), 0, SSV, s)
#define EVT_SET_SSID(x, s)    __EVT_SET_FLD((x), 0, SSID, s)
#define EVT_SET_SID(x, s)     ((x)->word[1] =  s)
#define EVT_SET_SPAN(x, s)    __EVT_SET_FLD((x), 2, SPAN, s)
#define EVT_SET_P(x, s)       __EVT_SET_FLD((x), 2, P, s)
#define EVT_SET_X(x, s)       __EVT_SET_FLD((x), 2, X, s)
#define EVT_SET_W(x, s)       __EVT_SET_FLD((x), 2, W, s)
#define EVT_SET_R(x, s)       __EVT_SET_FLD((x), 2, R, s)
#define EVT_SET_PNU(x, s)     __EVT_SET_FLD((x), 3, PNU, s)
#define EVT_SET_IND(x, s)     __EVT_SET_FLD((x), 3, IND, s)
#define EVT_SET_RNW(x, s)     __EVT_SET_FLD((x), 3, RNW, s)
#define EVT_SET_S2(x, s)      __EVT_SET_FLD((x), 3, S2, s)
#define EVT_SET_CLS(x, s)     __EVT_SET_FLD((x), 3, CLS, s)
#define EVT_SET_EFF_IND(x, s) __SET_FLD((x)->word[3], __SMMU_MASK(1), 12, s)
#define EVT_SET_EFF_RNW(x, s) __SET_FLD((x)->word[3], __SMMU_MASK(1), 13, s)
#define EVT_SET_INPUT_ADDR(x, addr) ({                               \
            (x)->word[5] = (uint32_t)(addr >> 32);                   \
            (x)->word[4] = (uint32_t)(addr & 0xffffffff);            \
            addr;                                                    \
        })

#define EVT_SET_ADDR_FETCH2(x, addr) ({                     \
            (x)->word[7] = (uint32_t)(addr >> 32);          \
            (x)->word[6] = (uint32_t)(addr & 0xffffffff);   \
        })

#define EVT_SET_IPA(x, addr) EVT_SET_ADDR_FETCH(x, addr)

#define EVT_TYPE(x)      __EVT_GET_FLD((x), 0, EVT)
#define EVT_ME(x)        __EVT_GET_FLD((x), 0, ME)
#define EVT_SSV(x)	 __EVT_GET_FLD((x), 0, SSV)
#define EVT_SSID(x)	 __EVT_GET_FLD((x), 0, SSID)
#define EVT_SID(x)	 ((x)->word[1])
#define EVT_SPAN(x)	 __EVT_GET_FLD((x), 2, SPAN)
#define EVT_P(x)	 __EVT_GET_FLD((x), 2, P)
#define EVT_X(x)	 __EVT_GET_FLD((x), 2, X)
#define EVT_W(x)	 __EVT_GET_FLD((x), 2, W)
#define EVT_R(x)	 __EVT_GET_FLD((x), 2, R)
#define EVT_PNU(x)	 __EVT_GET_FLD((x), 3, PNU)
#define EVT_IND(x)	 __EVT_GET_FLD((x), 3, IND)
#define EVT_RNW(x)	 __EVT_GET_FLD((x), 3, RNW)
#define EVT_S2(x)	 __EVT_GET_FLD((x), 3, S2)
#define EVT_CLS(x)	 __EVT_GET_FLD((x), 3, CLS)
#define EVT_EFF_IND(x)	 __GET_FLD((x)->word[3], __SMMU_MASK(1), 12)
#define EVT_EFF_RNW(x)	 __GET_FLD((x)->word[3], __SMMU_MASK(1), 13)
#define EVT_INPUT_ADDR(x) ({                     \
            uint64_t addr = (uint64_t)(x)->word[5] << 32;  \
            addr |= (x)->word[4];                \
            addr;                                \
        })
#define EVT_ADDR_FETCH1(x) ({                                   \
            uint64_t addr = __GET_FLD((x)->word[5], 0xffff, 0);	\
            addr <<= 32;					\
            addr |= ((x)->word[4] & ~(0x1f));                   \
            addr;						\
        })
#define EVT_IPA(x) ({                                           \
            uint64_t addr = __GET_FLD((x)->word[7], 0xffff, 0);	\
            addr <<= 32;					\
            addr |= ((x)->word[6] & ~(0xfff));                  \
            addr ;						\
        })
#define EVT_ADDR_FETCH2(x) ({                                   \
            uint64_t addr = __GET_FLD((x)->word[7], 0xffff, 0);	\
            addr <<= 32;					\
            addr |= ((x)->word[6] & ~(0xfff));                  \
            addr;						\
        })

/*****************************
 * DATA STRUCTURES
 *****************************/
#define STE_L1_SPAN_BITS        5
#define STE_L1_SPAN_SHIFT       0
#define STE_L1_SPAN_MASK        GENMASK((STE_L1_SPAN_BITS - 1), 0)

/*
 * Bits for Context Descriptor
 */
#define CD_TSZ_BITS   6         /* WORD 0 */
#define CD_T0SZ_SHIFT 0
#define CD_T1SZ_SHIFT (CD_T0SZ_SHIFT + 16)
#define CD_TG_BITS    2
#define CD_TG0_SHIFT  6
#define CD_TG1_SHIFT  (CD_TG0_SHIFT + 16)
#define CD_IR_BITS    2
#define CD_IR0_SHIFT  8
#define CD_IR1_SHIFT  (CD_IR0_SHIFT + 16)
#define CD_OR_BITS    2
#define CD_OR0_SHIFT  10
#define CD_OR1_SHIFT  (CD_OR1_SHIFT + 16)
#define CD_SH_BITS    2
#define CD_SH0_SHIFT  12
#define CD_SH1_SHIFT  (CD_SH0_SHIFT + 16)
#define CD_EPD_BITS   1
#define CD_EPD0_SHIFT 14
#define CD_EPD1_SHIFT (CD_EPD0_SHIFT + 16)

#define _CD_GET_FLD(x, off, val)                                \
    (__GET_FLD((x)->word[(off)], __SMMU_MASK(CD_##val##_BITS),	\
               CD_##val##_SHIFT))

#define CD_VALID_BITS	1
#define CD_VALID_SHIFT 31
#define CD_VALID(x)   _CD_GET_FLD((x), 0, VALID)

#define CD_ASID_BITS  16
#define CD_ASID_SHIFT 16
#define CD_ASID(x)    _CD_GET_FLD((x), 1, VALID)

#define CD_TTB_HI_BITS 16
#define CD_TTB_HI_SHIFT 0
#define CD_TTB_LO_BITS 28
#define CD_TTB_LO_SHIFT 4
#define CD_TTB(x, sel)                                          \
    ({								\
        uint64_t addr = _CD_GET_FLD((x), (sel)*2 + 2, TTB_HI);	\
        addr <<= 32;						\
        addr |= (uint64_t)_CD_GET_FLD((x), (sel)*2 + 3, TTB_LO);	\
        addr;							\
    })
#define CD_TTB0(x)	CD_TTB((x), 0)
#define CD_TTB1(x)	CD_TTB((x), 1)

#define CDM_VALID(x)    ((x)->word[0] & 0x1)

/*
 * Bits Required by STE and CD
 */
enum {
    STE_CONFIG_NONE      = 0,
    STE_CONFIG_S1BY_S2BY = 4, /* S1 Bypass, S2 Bypass */
    STE_CONFIG_S1TR_S2BY,	  /* S1 Translate, S2 Bypass */
    STE_CONFIG_S1BY_S2TR,	  /* S1 Bypass, S2 Translate */
    STE_CONFIG_S1TR_S2TR,	  /* S1 Translate, S2 Translate */
};

enum {                          /* S1ContextPtr points to 1 / 2 or more */
    STE_S1FMT_CDMAX = 0,
    STE_S1FMT_2LVL_4K,
    STE_S1FMT_2LVL_64K,
    STE_S1FMT_RESERVED,
};

/* Translation Request without a Substream ID */
enum {
    STE_S1DSS_TERMINATE = 0,
    STE_S1DSS_BYPASS,
    STE_S1DSS_TRANSLATE,
    STE_S1DSS_RESERVED,
};

#define STE_VALID_BITS      1	/* WORD 0 */
#define STE_VALID_SHIFT     0
#define STE_CONFIG_BITS     3
#define STE_CONFIG_SHIFT    1
#define STE_S1FMT_BITS      2
#define STE_S1FMT_SHIFT     (STE_CONFIG_SHIFT + STE_CONFIG_BITS)
#define STE_CTXPTR_LO_BITS  26
#define STE_CTXPTR_LO_SHIFT (STE_S1FMT_SHIFT + STE_S1FMT_BITS)
#define STE_CTXPTR_HI_BITS  16
#define STE_CTXPTR_HI_SHIFT 0	/* WORD 1 */
#define STE_S1DSS_BITS      2
#define STE_S1DSS_SHIFT     0	/* WORD 2 */
#define STE_S1CDMAX_BITS    2
#define STE_S1CDMAX_SHIFT   8
#define STE_S2VMID_BITS     16
#define STE_S2VMID_SHIFT    0	/* WORD 4 */
#define STE_S2T0SZ_BITS     6
#define STE_S2T0SZ_SHIFT    0	/* WORD 5 */
#define STE_S2SL0_BITS      2
#define STE_S2SL0_SHIFT     (STE_S2T0SZ_SHIFT + STE_S2T0SZ_BITS)
#define STE_S2IR0_BITS      2
#define STE_S2IR0_SHIFT     (STE_S2SL0_SHIFT + STE_S2SL0_BITS)
#define STE_S2OR0_BITS      2
#define STE_S2OR0_SHIFT     (STE_S2IR0_SHIFT + STE_S2IR0_BITS)
#define STE_S2SH0_BITS      2
#define STE_S2SH0_SHIFT     (STE_S2OR0_SHIFT + STE_S2OR0_BITS)
#define STE_S2TG_BITS       2
#define STE_S2TG_SHIFT      (STE_S2SH0_SHIFT + STE_S2SH0_BITS)
#define STE_S2PS_BITS       3
#define STE_S2PS_SHIFT      (STE_S2TG_SHIFT + STE_S2TG_BITS)
#define STE_S2AA64_BITS     1
#define STE_S2AA64_SHIFT    (STE_S2PS_SHIFT + STE_S2PS_BITS)
#define STE_S2TTB_LO_BITS   28
#define STE_S2TTB_LO_SHIFT  4
#define STE_S2TTB_HI_BITS   16
#define STE_S2TTB_HI_SHIFT  0

#define _STE_GET_FLD(x, off, val)                               \
    (__GET_FLD((x)->word[(off)], __SMMU_MASK(STE_##val##_BITS),	\
               STE_##val##_SHIFT))

#define STE_VALID(x)     _STE_GET_FLD((x), 0, VALID) /* 0 */
#define STE_CONFIG(x)    _STE_GET_FLD((x), 0, CONFIG)
#define STE_S1FMT(x)     _STE_GET_FLD((x), 0, S1FMT)
#define STE_S1CDMAX(x)   _STE_GET_FLD((x), 2, S1CDMAX)
#define STE_S2VMID(x)    _STE_GET_FLD((x), 4, S2VMID) /* 4 */
#define STE_S2T0SZ(x)    _STE_GET_FLD((x), 5, S2T0SZ)  /* 5 */
#define STE_S2SL0(x)     _STE_GET_FLD((x), 5, S2SL0)
#define STE_S2IR0(x)     _STE_GET_FLD((x), 5, S2IR0)
#define STE_S2OR0(x)     _STE_GET_FLD((x), 5, S2OR0)
#define STE_S2SH0(x)     _STE_GET_FLD((x), 5, S2SH0)
#define STE_S2TG(x)      _STE_GET_FLD((x), 5, S2TG)
#define STE_S2PS(x)      _STE_GET_FLD((x), 5, S2PS)
#define STE_S2AA64(x)    _STE_GET_FLD((x), 5, S2AA64)
#define STE_CTXPTR(x)                                           \
    ({								\
        unsigned long addr;					\
        addr = (uint64_t)_STE_GET_FLD((x), 1, CTXPTR_HI) << 32;	\
        addr |= (uint64_t)_STE_GET_FLD((x), 0, CTXPTR_LO);		\
        addr;							\
    })

#define STE_S2TTB(x)                                            \
    ({								\
        unsigned long addr;					\
        addr = (uint64_t)_STE_GET_FLD((x), 7, S2TTB_HI) << 32;	\
        addr |= (uint64_t)_STE_GET_FLD((x), 6, S2TTB_LO);		\
        addr;							\
    })


#define ARM_SMMU_FEAT_PASSID_SUPPORT	(1 << 24) /* Some random bits for now */
#define ARM_SMMU_FEAT_CD_2LVL		(1 << 25)

struct __smmu_data2 {
    uint32_t word[2];
};

struct __smmu_data8 {
    uint32_t word[8];
};

struct __smmu_data4 {
    uint32_t word[4];
};

typedef struct __smmu_data2	stm_t; /* STE Level 1 Descriptor */
typedef struct __smmu_data8	ste_t; /* Stream Table Entry(STE) */
typedef struct __smmu_data2	cdm_t; /* CD Level 1 Descriptor */
typedef struct __smmu_data8	cd_t;  /* Context Descriptor(CD) */

typedef struct __smmu_data4	cmd_t; /* Command Entry */
typedef struct __smmu_data8	evt_t; /* Event Entry */
typedef struct __smmu_data4	pri_t; /* PRI entry */

void dump_cmd(cmd_t *);
void dump_ste(ste_t *);
void dump_evt(evt_t *);

#endif
