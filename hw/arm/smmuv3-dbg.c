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
 * Copyright (C) 2014-15 Broadcom Corporation
 *
 * Author: Prem Mallappa <pmallapp@broadcom.com>
 *
 */

#include "hw/arm/smmuv3.h"

#define pr_crit(fmt, ...) do {                  \
        fprintf(stderr, "(smmu)%s: " fmt,  \
                __func__, ##__VA_ARGS__);       \
    } while(0)

#define STE_S1FMT_BITS      2
#define STE_S1FMT_SHIFT     (STE_CONFIG_SHIFT + STE_CONFIG_BITS)
#define STE_S1CIR_BITS      2
#define STE_S1CIR_SHIFT     (STE_S1DSS_SHIFT + STE_S1DSS_BITS)
#define STE_S1COR_BITS      2
#define STE_S1COR_SHIFT     (STE_S1CIR_SHIFT + STE_S1CIR_BITS)
#define STE_S1CSH_BITS      2
#define STE_S1CSH_SHIFT     (STE_S1COR_SHIFT + STE_S1COR_BITS)
#define STE_S1CDMAX_BITS    2
//#define STE_S1CDMAX_SHIFT   (STE_S1CSH_SHIFT + STE_S1CSH_BITS)
#define STE_CONT_BITS       4
#define STE_CONT_SHIFT      (STE_S1CDMAX_SHIFT + STE_S1CDMAX_BITS)
#define STE_PPAR_BITS       1
#define STE_PPAR_SHIFT      (STE_CONT_SHIFT + STE_CONT_BITS + 1)
#define STE_MEV_BITS        1
#define STE_MEV_SHIFT       (STE_PPAR_SHIFT + STE_PPAR_BITS)
#define STE_S1STALLD_BITS   1
#define STE_S1STALLD_SHIFT  (STE_MEV_SHIFT + STE_MEV_BITS + 7)
#define STE_EATS_BITS       2
#define STE_EATS_SHIFT      (STE_S1STALLD_SHIFT + STE_S1STALLD_BITS)
#define STE_STRW_BITS       2
#define STE_STRW_SHIFT      (STE_EATS_SHIFT + STE_EATS_BITS)
#define STE_MEMATTR_BITS    4	/* WORD 3 */
#define STE_MEMATTR_SHIFT   0
#define STE_MTCFG_BITS     1
#define STE_MTCFG_SHIFT    (STE_MEMATTR_SHIFT + STE_MEMATTR_BITS)
#define STE_ALLOCCFG_BITS   4
#define STE_ALLOCCFG_SHIFT (STE_MTCFG_SHIFT + STE_MTCFG_BITS)
#define STE_SHCFG_BITS     2
#define STE_SHCFG_SHIFT    (STE_ALLOCCFG_SHIFT + STE_ALLOCCFG_BITS + 3)
#define STE_NSCFG_BITS     2
#define STE_NSCFG_SHIFT    (STE_SHCFG_SHIFT + STE_SHCFG_BITS)
#define STE_PRIVCFG_BITS   2
#define STE_PRIVCFG_SHIFT  (STE_NSCFG_SHIFT + STE_NSCFG_BITS)
#define STE_INSTCFG_BITS   2
#define STE_INSTCFG_SHIFT  (STE_PRIVCFG_SHIFT + STE_PRIVCFG_BITS)
#define STE_S2ENDIAN_BITS   1
#define STE_S2ENDIAN_SHIFT  (STE_S2AA64_SHIFT + STE_S2AA64_BITS)
#define STE_S2AFFD_BITS     1
#define STE_S2AFFD_SHIFT    (STE_S2ENDIAN_SHIFT + STE_S2ENDIAN_BITS)
#define STE_S2PTW_BITS      1
#define STE_S2PTW_SHIFT     (STE_S2AFFD_SHIFT + STE_S2AFFD_BITS)
#define STE_S2HD_BITS       1
#define STE_S2HD_SHIFT      (STE_S2PTW_SHIFT + STE_S2PTW_BITS)
#define STE_S2HA_BITS       1
#define STE_S2HA_SHIFT      (STE_S2HD_SHIFT + STE_S2HD_BITS)
#define STE_S2S_BITS        1
#define STE_S2S_SHIFT       (STE_S2HA_SHIFT + STE_S2HA_BITS)
#define STE_S2R_BITS        1
#define STE_S2R_SHIFT       (STE_S2S_SHIFT + STE_S2S_BITS)

#define STE_S1FMT(x)    _STE_GET_FLD((x), 0, S1FMT)
#define STE_S1DSS(x)    _STE_GET_FLD((x), 2, S1DSS) /* 2 */
#define STE_S1CIR(x)    _STE_GET_FLD((x), 2, S1CIR)
#define STE_S1COR(x)    _STE_GET_FLD((x), 2, S1COR)
#define STE_S1CSH(x)    _STE_GET_FLD((x), 2, S1CSH)
#define STE_S1CDMAX(x)  _STE_GET_FLD((x), 2, S1CDMAX)
#define STE_CONT(x)     _STE_GET_FLD((x), 2, CONT)
#define STE_PPAR(x)     _STE_GET_FLD((x), 2, PPAR)
#define STE_MEV(x)      _STE_GET_FLD((x), 2, MEV)
#define STE_S1STALLD(x) _STE_GET_FLD((x), 2, S1STALLD)
#define STE_EATS(x)     _STE_GET_FLD((x), 2, EATS)
#define STE_STRW(x)     _STE_GET_FLD((x), 2, STRW)
#define STE_MEMATTR(x)  _STE_GET_FLD((x), 3, MEMATTR) /* 3 */
#define STE_MTCFG(x)    _STE_GET_FLD((x), 3, MTCFG)
#define STE_ALLOCCFG(x) _STE_GET_FLD((x), 3, ALLOCCFG)
#define STE_SHCFG(x)    _STE_GET_FLD((x), 3, SHCFG)
#define STE_NSCFG(x)    _STE_GET_FLD((x), 3, NSCFG)
#define STE_PRIVCFG(x)  _STE_GET_FLD((x), 3, PRIVCFG)
#define STE_INSTCFG(x)  _STE_GET_FLD((x), 3, INSTCFG)
#define STE_S2ENDIAN(x) _STE_GET_FLD((x), 5, S2ENDIAN)
#define STE_S2AFFD(x)   _STE_GET_FLD((x), 5, S2AFFD)
#define STE_S2PTW(x)    _STE_GET_FLD((x), 5, S2PTW)
#define STE_S2HD(x)     _STE_GET_FLD((x), 5, S2HD)
#define STE_S2HA(x)     _STE_GET_FLD((x), 5, S2HA)
#define STE_S2S(x)      _STE_GET_FLD((x), 5, S2S)
#define STE_S2R(x)      _STE_GET_FLD((x), 5, S2R)

void dump_ste(ste_t *ste)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(ste->word); i++) {
        pr_crit("STE[%2d]: %#010x\t STE[%2d]: %#010x\n",
                i, ste->word[i], i+1, ste->word[i+1]);
        i++;
    }

    pr_crit("***************STE**************\n"
            "s1_ctx:%lx s1_fmt:%ld config:%ld valid:%ld\n"
            "strw:%ld eats:%ld s1_stalld:%ld mev:%ld ppar:%ld cont:%ld"
            "s1_cdmax:%ld s1_csh:%ld s1_cor:%ld s1_cir:%ld s1_dss:%ld\n"
            "instcfg:%ld privcfg:%ld nscfg:%ld shcfg:%ld alloccfg:%ld"
            "mtcft:%ld memattr:%ld\n s2vmid:%x\n"
            "s2_s2r:%ld s2_s2s:%ld s2_ha:%ld s2_hd:%ld s2_ptw:%ld s2_affd:%ld\n"
            "s2_endian:%ld s2_aa64:%ld s2_ps:%ld s2_tg:%ld s2_sh0:%ld s2_or0:%ld\n"
            "s2_ir0:%ld s2_sl0:%ld s2t0sz:%ld\n"
            "s2_ttb:%lx\n"
            "***************STE**************\n",
            STE_CTXPTR(ste), STE_S1FMT(ste), STE_CONFIG(ste),
            STE_VALID(ste), STE_STRW(ste), STE_EATS(ste),
            STE_S1STALLD(ste), STE_MEV(ste), STE_PPAR(ste),
            STE_CONT(ste), STE_S1CDMAX(ste), STE_S1CSH(ste), STE_S1COR(ste),
            STE_S1CIR(ste), STE_S1DSS(ste), STE_INSTCFG(ste),
            STE_PRIVCFG(ste), STE_NSCFG(ste), STE_SHCFG(ste),
            STE_ALLOCCFG(ste), STE_MTCFG(ste), STE_MEMATTR(ste),
            (unsigned)STE_S2VMID(ste),
            STE_S2R(ste), STE_S2S(ste), STE_S2HA(ste), STE_S2HD(ste),
            STE_S2PTW(ste), STE_S2AFFD(ste), STE_S2ENDIAN(ste),
            STE_S2AA64(ste), STE_S2PS(ste), STE_S2TG(ste),
            STE_S2SH0(ste), STE_S2OR0(ste), STE_S2IR0(ste),
            STE_S2SL0(ste), STE_S2T0SZ(ste), STE_S2TTB(ste));
}

static const char *__cmd_str_arr[] = {
    [ SMMU_CMD_PREFETCH_CONFIG ] = "CMD_PREFETCH_CONFIG",
    [ SMMU_CMD_PREFETCH_ADDR ]   = "CMD_PREFETCH_ADDR",
    [ SMMU_CMD_CFGI_STE ]        = "CMD_CFGI_STE",
    //   [ SMMU_CMD_CFGI_ALL ]        = "CMD_CFGI_ALL",
    [ SMMU_CMD_CFGI_STE_RANGE ]  = "CMD_CFGI_STE_RANGE",
    [ SMMU_CMD_CFGI_CD ]         = "CMD_CFGI_CD",
    [ SMMU_CMD_CFGI_CD_ALL ]     = "CMD_CFGI_CD_ALL",
    [ SMMU_CMD_TLBI_NH_ALL ]     = "CMD_TLBI_NH_ALL",
    [ SMMU_CMD_TLBI_NH_ASID ]    = "CMD_TLBI_NH_ASID",
    [ SMMU_CMD_TLBI_NH_VA ]      = "CMD_TLBI_NH_VA",
    [ SMMU_CMD_TLBI_NH_VAA ]     = "CMD_TLBI_NH_VAA",
    [ SMMU_CMD_TLBI_EL3_ALL ]    = "CMD_TLBI_EL3_ALL",
    [ SMMU_CMD_TLBI_EL3_VA ]     = "CMD_TLBI_EL3_VA",
    [ SMMU_CMD_TLBI_EL2_ALL ]    = "CMD_TLBI_EL2_ALL",
    [ SMMU_CMD_TLBI_EL2_ASID ]   = "CMD_TLBI_EL2_ASID",
    [ SMMU_CMD_TLBI_EL2_VA ]     = "CMD_TLBI_EL2_VA",
    [ SMMU_CMD_TLBI_EL2_VAA ]    = "CMD_TLBI_EL2_VAA",
    [ SMMU_CMD_TLBI_S12_VMALL ]  = "CMD_TLBI_S12_VMALL",
    [ SMMU_CMD_TLBI_S2_IPA ]     = "CMD_TLBI_S2_IPA",
    [ SMMU_CMD_TLBI_NSNH_ALL ]   = "CMD_TLBI_NSNH_ALL",
    [ SMMU_CMD_ATC_INV ]         = "CMD_ATC_INV",
    [ SMMU_CMD_PRI_RESP ]        = "CMD_PRI_RESP",
    [ SMMU_CMD_RESUME ]          = "CMD_RESUME",
    [ SMMU_CMD_STALL_TERM ]      = "CMD_STALL_TERM",
    [ SMMU_CMD_SYNC ]            = "CMD_SYNC",
};


static const char *__evt_str_arr[] = {
    [SMMU_EVT_F_UUT]             = "SMMU_EVT_F_UUT",
    [SMMU_EVT_C_BAD_SID]         = "SMMU_EVT_C_BAD_SID",
    [SMMU_EVT_F_STE_FETCH]       = "SMMU_EVT_F_STE_FETCH",
    [SMMU_EVT_C_BAD_STE]         = "SMMU_EVT_C_BAD_STE",
    [SMMU_EVT_F_BAD_ATS_REQ]     = "SMMU_EVT_F_BAD_ATS_REQ",
    [SMMU_EVT_F_STREAM_DISABLED] = "SMMU_EVT_F_STREAM_DISABLED",
    [SMMU_EVT_F_TRANS_FORBIDDEN] = "SMMU_EVT_F_TRANS_FORBIDDEN",
    [SMMU_EVT_C_BAD_SSID]        = "SMMU_EVT_C_BAD_SSID",
    [SMMU_EVT_F_CD_FETCH]        = "SMMU_EVT_F_CD_FETCH",
    [SMMU_EVT_C_BAD_CD]          = "SMMU_EVT_C_BAD_CD",
    [SMMU_EVT_F_WALK_EXT_ABRT]   = "SMMU_EVT_F_WALK_EXT_ABRT",
    [SMMU_EVT_F_TRANS]           = "SMMU_EVT_F_TRANS",
    [SMMU_EVT_F_ADDR_SZ]         = "SMMU_EVT_F_ADDR_SZ",
    [SMMU_EVT_F_ACCESS]          = "SMMU_EVT_F_ACCESS",
    [SMMU_EVT_F_PERM]            = "SMMU_EVT_F_PERM",
    [SMMU_EVT_F_TLB_CONFLICT]    = "SMMU_EVT_F_TLB_CONFLICT",
    [SMMU_EVT_F_CFG_CONFLICT]    = "SMMU_EVT_F_CFG_CONFLICT",
    [SMMU_EVT_E_PAGE_REQ]        = "SMMU_EVT_E_PAGE_REQ",
};

void dump_evt(evt_t *evt)
{
    int i;

    pr_crit(" %s\n", __evt_str_arr[EVT_TYPE(evt)]);

    for (i = 0; i < ARRAY_SIZE(evt->word); i++) {
        pr_crit("EVT[%2d]: %#010x\t EVT[%2d]: %#010x\n",
                i, evt->word[i], i+1, evt->word[i+1]);
        i++;
    }

    pr_crit("**************EVENT****************\n"
            "ssid:%ld ssv:%ld me:%ld event:%ld\n"
            "r:%ld w:%ld x:%ld p:%ld span:%ld\n"
            "eff_RnW:%ld eff_InD:%ld class:%ld s2:%ld RnW:%ld InD:%ld PnU:%ld\n"
            "va:%lx ipa:%lx\n"
            "****************EVENT**************\n",
            EVT_SSID(evt), EVT_SSV(evt), EVT_ME(evt), EVT_TYPE(evt),
            EVT_R(evt), EVT_W(evt), EVT_X(evt), EVT_P(evt), EVT_SPAN(evt),
            EVT_EFF_RNW(evt), EVT_EFF_IND(evt), EVT_CLS(evt), EVT_S2(evt),
            EVT_RNW(evt), EVT_IND(evt), EVT_PNU(evt),
            EVT_INPUT_ADDR(evt), EVT_IPA(evt));
}

struct cmd_flags {
    uint32_t sec:1;
    uint32_t sev:1;
    uint32_t ssv:1;
    uint32_t ab:1;
    uint32_t ac:1;
    uint32_t ssid:1;
    uint32_t sid:1;
    uint32_t vmid:1;
    uint32_t asid:1;
    uint32_t leaf:1;
    uint32_t span:1;
    uint32_t stag:1;
    uint32_t addr:1;
    uint32_t size:1;
};

static void _smmu_build_tlb_cmd_fields(uint8_t cmd, bool *f_addr, bool *f_vmid,
                                       bool *f_asid, bool *f_leaf)
{
    switch (cmd) {
    case SMMU_CMD_TLBI_EL3_VA:
    case SMMU_CMD_TLBI_NH_VA: *f_addr = 1;
    case SMMU_CMD_TLBI_NH_VAA: *f_leaf = 1;
    case SMMU_CMD_TLBI_EL2_ASID:
    case SMMU_CMD_TLBI_NH_ASID: *f_asid = 1;
    case SMMU_CMD_TLBI_S12_VMALL:
    case SMMU_CMD_TLBI_NH_ALL: *f_vmid = 1;
    case SMMU_CMD_TLBI_EL2_ALL:
    case SMMU_CMD_TLBI_EL3_ALL:
    case SMMU_CMD_TLBI_NSNH_ALL: break;
    case SMMU_CMD_TLBI_EL2_VA: *f_asid = 1;
    case SMMU_CMD_TLBI_EL2_VAA: *f_addr = *f_leaf = 1; break;
    case SMMU_CMD_TLBI_S2_IPA: *f_vmid = *f_addr = *f_leaf = 1; break;
    default:
        return;
    }
}

static int __smmu_build_cmd_fields(uint8_t cmd, struct cmd_flags *f)
{
    bool f_addr = 0, f_vmid = 0, f_asid = 0, f_leaf = 0;

    _smmu_build_tlb_cmd_fields(cmd, &f_addr, &f_vmid, &f_asid, &f_leaf);

    switch(cmd) {
    case SMMU_CMD_PREFETCH_ADDR: f->size = f->addr = 1;/* fallthrough */
    case SMMU_CMD_PREFETCH_CONFIG: f->ssid = f->sec = f->sev = f->sid = 1; break;
    case SMMU_CMD_CFGI_CD: f->ssid = 1; /* fallthrough */
    case SMMU_CMD_CFGI_CD_ALL: f->sec = f->sid = 1; break;
        //case SMMU_CMD_CFGI_ALL:	/* fallthrough */
    case SMMU_CMD_CFGI_STE_RANGE: f->sec = f->sid = f->span = 1; break;
    case SMMU_CMD_CFGI_STE: f->sec = f->sid = 1; break;
    case SMMU_CMD_ATC_INV:	f->size = 1; /* fallthrough */
    case SMMU_CMD_PRI_RESP: f->ssv = f->ssid = f->sid = f->addr = 1; break;
    case SMMU_CMD_RESUME:f->ab = f->ac = f->sec = f->sid = f->stag = 1; break;
    default:
        return -1;
        break;
    }
    f->addr = f_addr;
    f->vmid = f_vmid;
    f->asid = f_asid;
    f->leaf = f_leaf;

    return 0;
}

void dump_cmd(cmd_t *cmde)
{
    /* Considering 100 chars per line, 4 lines */
    char buf[512];
    int len = ARRAY_SIZE(buf), count = 0, ret = 0, tmpcnt;
    struct cmd_flags flg;
    uint8_t cmd = CMD_TYPE(cmde);
    int i;

    pr_crit(" %s \n", __cmd_str_arr[cmd]);
    for (i = 0; i < ARRAY_SIZE(cmde->word); i++) {
        pr_crit("CMD[%2d]: %#010x\t CMD[%2d]: %#010x\n",
                i, cmde->word[i], i+1, cmde->word[i+1]);
        i++;
    }

    switch (cmd) {
    case SMMU_CMD_PRI_RESP:	/* fallthrough */
    default:
        ret = __smmu_build_cmd_fields(cmd, &flg);
        goto out; break;

        /* THE ODD ONE */
    case SMMU_CMD_SYNC:
        count += snprintf(&buf[count], len - count,
                          "MSIAttr:%x MSH:%x CS:%d\n"
                          "MSIDATA:%x MSIADDR:%lx\n",
                          __GET_FLD(cmde->word[0], 0x3, 12),
                          __GET_FLD(cmde->word[0], 0x3, 22),
                          __GET_FLD(cmde->word[0], 0xf, 22),
                          cmde->word[1], CMD_ADDR(cmde));
        break;
    }

    if (ret < 0) {
        count += snprintf(&buf[count], len - count, "INVALID COMMAND: %x\n",
                          cmd);
        goto out;
    }
    tmpcnt = count;
    count += flg.sec ? snprintf(&buf[count], len - count, "SEC:%lx ",
				CMD_SEC(cmde)): 0;
    count += flg.sev ? snprintf(&buf[count], len - count, "SEV=%lx ",
				CMD_SEV(cmde)): 0;
    count += flg.ab ? snprintf(&buf[count], len - count, "AB=%lx ",
			       CMD_AB(cmde)): 0;
    count += flg.ac ? snprintf(&buf[count], len - count, "AC=%lx ",
			       CMD_AC(cmde)): 0;
    count += flg.sec ? snprintf(&buf[count], len - count, "SSID:%lx ",
				CMD_SSID(cmde)): 0;
    count += tmpcnt != count? snprintf(&buf[count], len - count, "\n"): 0;
    tmpcnt = count;

    count += flg.vmid ? snprintf(&buf[count], len - count, "VMID=%lx ",
				 CMD_VMID(cmde)): 0;
    count += flg.asid ? snprintf(&buf[count], len - count, "ASID=%lx ",
				 CMD_ASID(cmde)): 0;
    count += flg.sid ? snprintf(&buf[count], len - count, "SID=%x ",
				CMD_SID(cmde)): 0;
    count += tmpcnt != count? snprintf(&buf[count], len - count, "\n"): 0;
    tmpcnt = count;

    count += flg.size ? snprintf(&buf[count], len - count, "SIZE=%lx ",
				 CMD_SIZE(cmde)): 0;
    count += flg.leaf ? snprintf(&buf[count], len - count, "LEAF=%lx ",
				 CMD_LEAF(cmde)): 0;
    count += flg.span ? snprintf(&buf[count], len - count, "SPAN=%lx ",
				 CMD_SPAN(cmde)): 0;
    count += flg.addr ? snprintf(&buf[count], len - count, "ADDR=%lx ",
				 CMD_ADDR(cmde)): 0;
    count += snprintf(&buf[count], len - count, "GROUP:%lx RESP=%lx ",
                      CMD_GRPID(cmde), CMD_RESP(cmde));
    count += tmpcnt != count? snprintf(&buf[count], len - count, "\n"): 0;
    tmpcnt = count;

 out:
    /* All we could accomodate */
    buf[count] = '\0';
    pr_crit("%s\n", &buf[0]);

}

