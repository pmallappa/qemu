#ifndef __libqos_ahci_h
#define __libqos_ahci_h

/*
 * AHCI qtest library functions and definitions
 *
 * Copyright (c) 2014 John Snow <jsnow@redhat.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "libqos/libqos.h"
#include "libqos/pci.h"
#include "libqos/malloc-pc.h"

/*** Supplementary PCI Config Space IDs & Masks ***/
#define PCI_DEVICE_ID_INTEL_Q35_AHCI   (0x2922)
#define PCI_MSI_FLAGS_RESERVED         (0xFF00)
#define PCI_PM_CTRL_RESERVED             (0xFC)
#define PCI_BCC(REG32)          ((REG32) >> 24)
#define PCI_PI(REG32)   (((REG32) >> 8) & 0xFF)
#define PCI_SCC(REG32) (((REG32) >> 16) & 0xFF)

/*** Recognized AHCI Device Types ***/
#define AHCI_INTEL_ICH9 (PCI_DEVICE_ID_INTEL_Q35_AHCI << 16 | \
                         PCI_VENDOR_ID_INTEL)

/*** AHCI/HBA Register Offsets and Bitmasks ***/
#define AHCI_CAP                          (0)
#define AHCI_CAP_NP                    (0x1F)
#define AHCI_CAP_SXS                   (0x20)
#define AHCI_CAP_EMS                   (0x40)
#define AHCI_CAP_CCCS                  (0x80)
#define AHCI_CAP_NCS                 (0x1F00)
#define AHCI_CAP_PSC                 (0x2000)
#define AHCI_CAP_SSC                 (0x4000)
#define AHCI_CAP_PMD                 (0x8000)
#define AHCI_CAP_FBSS               (0x10000)
#define AHCI_CAP_SPM                (0x20000)
#define AHCI_CAP_SAM                (0x40000)
#define AHCI_CAP_RESERVED           (0x80000)
#define AHCI_CAP_ISS               (0xF00000)
#define AHCI_CAP_SCLO             (0x1000000)
#define AHCI_CAP_SAL              (0x2000000)
#define AHCI_CAP_SALP             (0x4000000)
#define AHCI_CAP_SSS              (0x8000000)
#define AHCI_CAP_SMPS            (0x10000000)
#define AHCI_CAP_SSNTF           (0x20000000)
#define AHCI_CAP_SNCQ            (0x40000000)
#define AHCI_CAP_S64A            (0x80000000)

#define AHCI_GHC                          (1)
#define AHCI_GHC_HR                    (0x01)
#define AHCI_GHC_IE                    (0x02)
#define AHCI_GHC_MRSM                  (0x04)
#define AHCI_GHC_RESERVED        (0x7FFFFFF8)
#define AHCI_GHC_AE              (0x80000000)

#define AHCI_IS                           (2)
#define AHCI_PI                           (3)
#define AHCI_VS                           (4)

#define AHCI_CCCCTL                       (5)
#define AHCI_CCCCTL_EN                 (0x01)
#define AHCI_CCCCTL_RESERVED           (0x06)
#define AHCI_CCCCTL_CC               (0xFF00)
#define AHCI_CCCCTL_TV           (0xFFFF0000)

#define AHCI_CCCPORTS                     (6)
#define AHCI_EMLOC                        (7)

#define AHCI_EMCTL                        (8)
#define AHCI_EMCTL_STSMR               (0x01)
#define AHCI_EMCTL_CTLTM              (0x100)
#define AHCI_EMCTL_CTLRST             (0x200)
#define AHCI_EMCTL_RESERVED      (0xF0F0FCFE)

#define AHCI_CAP2                         (9)
#define AHCI_CAP2_BOH                  (0x01)
#define AHCI_CAP2_NVMP                 (0x02)
#define AHCI_CAP2_APST                 (0x04)
#define AHCI_CAP2_RESERVED       (0xFFFFFFF8)

#define AHCI_BOHC                        (10)
#define AHCI_RESERVED                    (11)
#define AHCI_NVMHCI                      (24)
#define AHCI_VENDOR                      (40)
#define AHCI_PORTS                       (64)

/*** Port Memory Offsets & Bitmasks ***/
#define AHCI_PX_CLB                       (0)
#define AHCI_PX_CLB_RESERVED          (0x1FF)

#define AHCI_PX_CLBU                      (1)

#define AHCI_PX_FB                        (2)
#define AHCI_PX_FB_RESERVED            (0xFF)

#define AHCI_PX_FBU                       (3)

#define AHCI_PX_IS                        (4)
#define AHCI_PX_IS_DHRS                 (0x1)
#define AHCI_PX_IS_PSS                  (0x2)
#define AHCI_PX_IS_DSS                  (0x4)
#define AHCI_PX_IS_SDBS                 (0x8)
#define AHCI_PX_IS_UFS                 (0x10)
#define AHCI_PX_IS_DPS                 (0x20)
#define AHCI_PX_IS_PCS                 (0x40)
#define AHCI_PX_IS_DMPS                (0x80)
#define AHCI_PX_IS_RESERVED       (0x23FFF00)
#define AHCI_PX_IS_PRCS            (0x400000)
#define AHCI_PX_IS_IPMS            (0x800000)
#define AHCI_PX_IS_OFS            (0x1000000)
#define AHCI_PX_IS_INFS           (0x4000000)
#define AHCI_PX_IS_IFS            (0x8000000)
#define AHCI_PX_IS_HBDS          (0x10000000)
#define AHCI_PX_IS_HBFS          (0x20000000)
#define AHCI_PX_IS_TFES          (0x40000000)
#define AHCI_PX_IS_CPDS          (0x80000000)

#define AHCI_PX_IE                        (5)
#define AHCI_PX_IE_DHRE                 (0x1)
#define AHCI_PX_IE_PSE                  (0x2)
#define AHCI_PX_IE_DSE                  (0x4)
#define AHCI_PX_IE_SDBE                 (0x8)
#define AHCI_PX_IE_UFE                 (0x10)
#define AHCI_PX_IE_DPE                 (0x20)
#define AHCI_PX_IE_PCE                 (0x40)
#define AHCI_PX_IE_DMPE                (0x80)
#define AHCI_PX_IE_RESERVED       (0x23FFF00)
#define AHCI_PX_IE_PRCE            (0x400000)
#define AHCI_PX_IE_IPME            (0x800000)
#define AHCI_PX_IE_OFE            (0x1000000)
#define AHCI_PX_IE_INFE           (0x4000000)
#define AHCI_PX_IE_IFE            (0x8000000)
#define AHCI_PX_IE_HBDE          (0x10000000)
#define AHCI_PX_IE_HBFE          (0x20000000)
#define AHCI_PX_IE_TFEE          (0x40000000)
#define AHCI_PX_IE_CPDE          (0x80000000)

#define AHCI_PX_CMD                       (6)
#define AHCI_PX_CMD_ST                  (0x1)
#define AHCI_PX_CMD_SUD                 (0x2)
#define AHCI_PX_CMD_POD                 (0x4)
#define AHCI_PX_CMD_CLO                 (0x8)
#define AHCI_PX_CMD_FRE                (0x10)
#define AHCI_PX_CMD_RESERVED           (0xE0)
#define AHCI_PX_CMD_CCS              (0x1F00)
#define AHCI_PX_CMD_MPSS             (0x2000)
#define AHCI_PX_CMD_FR               (0x4000)
#define AHCI_PX_CMD_CR               (0x8000)
#define AHCI_PX_CMD_CPS             (0x10000)
#define AHCI_PX_CMD_PMA             (0x20000)
#define AHCI_PX_CMD_HPCP            (0x40000)
#define AHCI_PX_CMD_MPSP            (0x80000)
#define AHCI_PX_CMD_CPD            (0x100000)
#define AHCI_PX_CMD_ESP            (0x200000)
#define AHCI_PX_CMD_FBSCP          (0x400000)
#define AHCI_PX_CMD_APSTE          (0x800000)
#define AHCI_PX_CMD_ATAPI         (0x1000000)
#define AHCI_PX_CMD_DLAE          (0x2000000)
#define AHCI_PX_CMD_ALPE          (0x4000000)
#define AHCI_PX_CMD_ASP           (0x8000000)
#define AHCI_PX_CMD_ICC          (0xF0000000)

#define AHCI_PX_RES1                      (7)

#define AHCI_PX_TFD                       (8)
#define AHCI_PX_TFD_STS                (0xFF)
#define AHCI_PX_TFD_STS_ERR            (0x01)
#define AHCI_PX_TFD_STS_CS1            (0x06)
#define AHCI_PX_TFD_STS_DRQ            (0x08)
#define AHCI_PX_TFD_STS_CS2            (0x70)
#define AHCI_PX_TFD_STS_BSY            (0x80)
#define AHCI_PX_TFD_ERR              (0xFF00)
#define AHCI_PX_TFD_RESERVED     (0xFFFF0000)

#define AHCI_PX_SIG                       (9)
#define AHCI_PX_SIG_SECTOR_COUNT       (0xFF)
#define AHCI_PX_SIG_LBA_LOW          (0xFF00)
#define AHCI_PX_SIG_LBA_MID        (0xFF0000)
#define AHCI_PX_SIG_LBA_HIGH     (0xFF000000)

#define AHCI_PX_SSTS                     (10)
#define AHCI_PX_SSTS_DET               (0x0F)
#define AHCI_PX_SSTS_SPD               (0xF0)
#define AHCI_PX_SSTS_IPM              (0xF00)
#define AHCI_PX_SSTS_RESERVED    (0xFFFFF000)
#define SSTS_DET_NO_DEVICE             (0x00)
#define SSTS_DET_PRESENT               (0x01)
#define SSTS_DET_ESTABLISHED           (0x03)
#define SSTS_DET_OFFLINE               (0x04)

#define AHCI_PX_SCTL                     (11)

#define AHCI_PX_SERR                     (12)
#define AHCI_PX_SERR_ERR             (0xFFFF)
#define AHCI_PX_SERR_DIAG        (0xFFFF0000)
#define AHCI_PX_SERR_DIAG_X      (0x04000000)

#define AHCI_PX_SACT                     (13)
#define AHCI_PX_CI                       (14)
#define AHCI_PX_SNTF                     (15)

#define AHCI_PX_FBS                      (16)
#define AHCI_PX_FBS_EN                  (0x1)
#define AHCI_PX_FBS_DEC                 (0x2)
#define AHCI_PX_FBS_SDE                 (0x4)
#define AHCI_PX_FBS_DEV               (0xF00)
#define AHCI_PX_FBS_ADO              (0xF000)
#define AHCI_PX_FBS_DWE             (0xF0000)
#define AHCI_PX_FBS_RESERVED     (0xFFF000F8)

#define AHCI_PX_RES2                     (17)
#define AHCI_PX_VS                       (28)

#define HBA_DATA_REGION_SIZE            (256)
#define HBA_PORT_DATA_SIZE              (128)
#define HBA_PORT_NUM_REG (HBA_PORT_DATA_SIZE/4)

#define AHCI_VERSION_0_95        (0x00000905)
#define AHCI_VERSION_1_0         (0x00010000)
#define AHCI_VERSION_1_1         (0x00010100)
#define AHCI_VERSION_1_2         (0x00010200)
#define AHCI_VERSION_1_3         (0x00010300)

/*** Structures ***/

typedef struct AHCIPortQState {
    uint64_t fb;
    uint64_t clb;
} AHCIPortQState;

typedef struct AHCIQState {
    QOSState *parent;
    QPCIDevice *dev;
    void *hba_base;
    uint64_t barsize;
    uint32_t fingerprint;
    uint32_t cap;
    uint32_t cap2;
    AHCIPortQState port[32];
} AHCIQState;

/**
 * Generic FIS structure.
 */
typedef struct FIS {
    uint8_t fis_type;
    uint8_t flags;
    char data[0];
} __attribute__((__packed__)) FIS;

/**
 * Register device-to-host FIS structure.
 */
typedef struct RegD2HFIS {
    /* DW0 */
    uint8_t fis_type;
    uint8_t flags;
    uint8_t status;
    uint8_t error;
    /* DW1 */
    uint8_t lba_low;
    uint8_t lba_mid;
    uint8_t lba_high;
    uint8_t device;
    /* DW2 */
    uint8_t lba3;
    uint8_t lba4;
    uint8_t lba5;
    uint8_t res1;
    /* DW3 */
    uint16_t count;
    uint8_t res2;
    uint8_t res3;
    /* DW4 */
    uint16_t res4;
    uint16_t res5;
} __attribute__((__packed__)) RegD2HFIS;

/**
 * Register host-to-device FIS structure.
 */
typedef struct RegH2DFIS {
    /* DW0 */
    uint8_t fis_type;
    uint8_t flags;
    uint8_t command;
    uint8_t feature_low;
    /* DW1 */
    uint8_t lba_low;
    uint8_t lba_mid;
    uint8_t lba_high;
    uint8_t device;
    /* DW2 */
    uint8_t lba3;
    uint8_t lba4;
    uint8_t lba5;
    uint8_t feature_high;
    /* DW3 */
    uint16_t count;
    uint8_t icc;
    uint8_t control;
    /* DW4 */
    uint32_t aux;
} __attribute__((__packed__)) RegH2DFIS;

/**
 * Command List entry structure.
 * The command list contains between 1-32 of these structures.
 */
typedef struct AHCICommand {
    uint8_t b1;
    uint8_t b2;
    uint16_t prdtl; /* Phys Region Desc. Table Length */
    uint32_t prdbc; /* Phys Region Desc. Byte Count */
    uint32_t ctba;  /* Command Table Descriptor Base Address */
    uint32_t ctbau; /*                                    '' Upper */
    uint32_t res[4];
} __attribute__((__packed__)) AHCICommand;

/**
 * Physical Region Descriptor; pointed to by the Command List Header,
 * struct ahci_command.
 */
typedef struct PRD {
    uint32_t dba;  /* Data Base Address */
    uint32_t dbau; /* Data Base Address Upper */
    uint32_t res;  /* Reserved */
    uint32_t dbc;  /* Data Byte Count (0-indexed) & Interrupt Flag (bit 2^31) */
} PRD;

/*** Macro Utilities ***/
#define BITANY(data, mask) (((data) & (mask)) != 0)
#define BITSET(data, mask) (((data) & (mask)) == (mask))
#define BITCLR(data, mask) (((data) & (mask)) == 0)
#define ASSERT_BIT_SET(data, mask) g_assert_cmphex((data) & (mask), ==, (mask))
#define ASSERT_BIT_CLEAR(data, mask) g_assert_cmphex((data) & (mask), ==, 0)

/* For calculating how big the PRD table needs to be: */
#define CMD_TBL_SIZ(n) ((0x80 + ((n) * sizeof(PRD)) + 0x7F) & ~0x7F)

/* Helpers for reading/writing AHCI HBA register values */

static inline uint32_t ahci_mread(AHCIQState *ahci, size_t offset)
{
    return qpci_io_readl(ahci->dev, ahci->hba_base + offset);
}

static inline void ahci_mwrite(AHCIQState *ahci, size_t offset, uint32_t value)
{
    qpci_io_writel(ahci->dev, ahci->hba_base + offset, value);
}

static inline uint32_t ahci_rreg(AHCIQState *ahci, uint32_t reg_num)
{
    return ahci_mread(ahci, 4 * reg_num);
}

static inline void ahci_wreg(AHCIQState *ahci, uint32_t reg_num, uint32_t value)
{
    ahci_mwrite(ahci, 4 * reg_num, value);
}

static inline void ahci_set(AHCIQState *ahci, uint32_t reg_num, uint32_t mask)
{
    ahci_wreg(ahci, reg_num, ahci_rreg(ahci, reg_num) | mask);
}

static inline void ahci_clr(AHCIQState *ahci, uint32_t reg_num, uint32_t mask)
{
    ahci_wreg(ahci, reg_num, ahci_rreg(ahci, reg_num) & ~mask);
}

static inline size_t ahci_px_offset(uint8_t port, uint32_t reg_num)
{
    return AHCI_PORTS + (HBA_PORT_NUM_REG * port) + reg_num;
}

static inline uint32_t ahci_px_rreg(AHCIQState *ahci, uint8_t port,
                                    uint32_t reg_num)
{
    return ahci_rreg(ahci, ahci_px_offset(port, reg_num));
}

static inline void ahci_px_wreg(AHCIQState *ahci, uint8_t port,
                                uint32_t reg_num, uint32_t value)
{
    ahci_wreg(ahci, ahci_px_offset(port, reg_num), value);
}

static inline void ahci_px_set(AHCIQState *ahci, uint8_t port,
                               uint32_t reg_num, uint32_t mask)
{
    ahci_px_wreg(ahci, port, reg_num,
                 ahci_px_rreg(ahci, port, reg_num) | mask);
}

static inline void ahci_px_clr(AHCIQState *ahci, uint8_t port,
                               uint32_t reg_num, uint32_t mask)
{
    ahci_px_wreg(ahci, port, reg_num,
                 ahci_px_rreg(ahci, port, reg_num) & ~mask);
}

/*** Prototypes ***/
uint64_t ahci_alloc(AHCIQState *ahci, size_t bytes);
void ahci_free(AHCIQState *ahci, uint64_t addr);
QPCIDevice *get_ahci_device(uint32_t *fingerprint);
void free_ahci_device(QPCIDevice *dev);
void ahci_pci_enable(AHCIQState *ahci);
void start_ahci_device(AHCIQState *ahci);
void ahci_hba_enable(AHCIQState *ahci);
unsigned ahci_port_select(AHCIQState *ahci);
void ahci_port_clear(AHCIQState *ahci, uint8_t port);

#endif
