/*!
	\file    sdcard.h
	\brief   the header file of SD card driver

	\version 2017-02-10, V1.0.0, firmware for GD32F30x
	\version 2018-10-10, V1.1.0, firmware for GD32F30x
	\version 2018-12-25, V2.0.0, firmware for GD32F30x
	\version 2020-09-30, V2.1.0, firmware for GD32F30x
*/

/*
	Copyright (c) 2020, GigaDevice Semiconductor Inc.

	Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice, this 
	   list of conditions and the following disclaimer.
	2. Redistributions in binary form must reproduce the above copyright notice, 
	   this list of conditions and the following disclaimer in the documentation 
	   and/or other materials provided with the distribution.
	3. Neither the name of the copyright holder nor the names of its contributors 
	   may be used to endorse or promote products derived from this software without 
	   specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#ifndef SDCARD_H
#define SDCARD_H

#include <gd32_def.h>

#ifdef __cplusplus
extern "C" {
#endif

//#define GD32F30X_HD

/* sd memory card bus commands index */
#define SD_CMD_GO_IDLE_STATE                  ((uint8_t)0)   /* CMD0, GO_IDLE_STATE */
#define SD_CMD_ALL_SEND_CID                   ((uint8_t)2)   /* CMD2, ALL_SEND_CID */
#define SD_CMD_SEND_RELATIVE_ADDR             ((uint8_t)3)   /* CMD3, SEND_RELATIVE_ADDR */
#define SD_CMD_SET_DSR                        ((uint8_t)4)   /* CMD4, SET_DSR */
#define SD_CMD_SWITCH_FUNC                    ((uint8_t)6)   /* CMD6, SWITCH_FUNC */
#define SD_CMD_SELECT_DESELECT_CARD           ((uint8_t)7)   /* CMD7, SELECT_DESELECT_CARD */
#define SD_CMD_SEND_IF_COND                   ((uint8_t)8)   /* CMD8, SEND_IF_COND */
#define SD_CMD_SEND_CSD                       ((uint8_t)9)   /* CMD9, SEND_CSD */
#define SD_CMD_SEND_CID                       ((uint8_t)10)  /* CMD10, SEND_CID */
#define SD_CMD_STOP_TRANSMISSION              ((uint8_t)12)  /* CMD12, STOP_TRANSMISSION */
#define SD_CMD_SEND_STATUS                    ((uint8_t)13)  /* CMD13, SEND_STATUS */
#define SD_CMD_GO_INACTIVE_STATE              ((uint8_t)15)  /* CMD15, GO_INACTIVE_STATE */
#define SD_CMD_SET_BLOCKLEN                   ((uint8_t)16)  /* CMD16, SET_BLOCKLEN */
#define SD_CMD_READ_SINGLE_BLOCK              ((uint8_t)17)  /* CMD17, READ_SINGLE_BLOCK */
#define SD_CMD_READ_MULTIPLE_BLOCK            ((uint8_t)18)  /* CMD18, READ_MULTIPLE_BLOCK */
#define SD_CMD_WRITE_BLOCK                    ((uint8_t)24)  /* CMD24, WRITE_BLOCK */
#define SD_CMD_WRITE_MULTIPLE_BLOCK           ((uint8_t)25)  /* CMD25, WRITE_MULTIPLE_BLOCK */
#define SD_CMD_PROG_CSD                       ((uint8_t)27)  /* CMD27, PROG_CSD */
#define SD_CMD_SET_WRITE_PROT                 ((uint8_t)28)  /* CMD28, SET_WRITE_PROT */
#define SD_CMD_CLR_WRITE_PROT                 ((uint8_t)29)  /* CMD29, CLR_WRITE_PROT */
#define SD_CMD_SEND_WRITE_PROT                ((uint8_t)30)  /* CMD30, SEND_WRITE_PROT */
#define SD_CMD_ERASE_WR_BLK_START             ((uint8_t)32)  /* CMD32, ERASE_WR_BLK_START */
#define SD_CMD_ERASE_WR_BLK_END               ((uint8_t)33)  /* CMD33, ERASE_WR_BLK_END */
#define SD_CMD_ERASE                          ((uint8_t)38)  /* CMD38, ERASE */
#define SD_CMD_LOCK_UNLOCK                    ((uint8_t)42)  /* CMD42, LOCK_UNLOCK */
#define SD_CMD_APP_CMD                        ((uint8_t)55)  /* CMD55, APP_CMD */
#define SD_CMD_GEN_CMD                        ((uint8_t)56)  /* CMD56, GEN_CMD */

/* SD memory card application specific commands index */
#define SD_ACMD_SET_BUS_WIDTH               	((uint8_t)6)   /* ACMD6, SET_BUS_WIDTH */
#define SD_ACMD_SD_STATUS                   	((uint8_t)13)  /* ACMD13, SD_STATUS */
#define SD_ACMD_SEND_NUM_WR_BLOCKS          	((uint8_t)22)  /* ACMD22, SEND_NUM_WR_BLOCKS */
#define SD_ACMD_SET_WR_BLK_ERASE_COUNT      	((uint8_t)23)  /* ACMD23, SET_WR_BLK_ERASE_COUNT */
#define SD_ACMD_SD_SEND_OP_COND             	((uint8_t)41)  /* ACMD41, SD_SEND_OP_COND */
#define SD_ACMD_SET_CLR_CARD_DETECT         	((uint8_t)42)  /* ACMD42, SET_CLR_CARD_DETECT */
#define SD_ACMD_SEND_SCR                    	((uint8_t)51)  /* ACMD51, SEND_SCR */

/* card command class */
#define SD_CCC_SWITCH                          BIT(10)       /* class 10 */
#define SD_CCC_IO_MODE                         BIT(9)        /* class 9 */
#define SD_CCC_APPLICATION_SPECIFIC            BIT(8)        /* class 8 */
#define SD_CCC_LOCK_CARD                       BIT(7)        /* class 7 */
#define SD_CCC_WRITE_PROTECTION                BIT(6)        /* class 6 */
#define SD_CCC_ERASE                           BIT(5)        /* class 5 */
#define SD_CCC_BLOCK_WRITE                     BIT(4)        /* class 4 */
#define SD_CCC_BLOCK_READ                      BIT(2)        /* class 2 */
#define SD_CCC_BASIC                           BIT(0)        /* class 0 */

/* sd card data transmission mode */
#define SD_DMA_MODE                           ((uint32_t)0x00000000U) /* DMA mode */
#define SD_POLLING_MODE                       ((uint32_t)0x00000001U) /* polling mode */

/* lock unlock status */
#define SD_LOCK                               ((uint8_t)0x05)        /* lock the SD card */
#define SD_UNLOCK                             ((uint8_t)0x02)        /* unlock the SD card */

/* supported memory cards types */
typedef enum {
	SDIO_STD_CAPACITY_SD_CARD_V1_1 = 0,   /* standard capacity SD card version 1.1 */
	SDIO_STD_CAPACITY_SD_CARD_V2_0,       /* standard capacity SD card version 2.0 */
	SDIO_HIGH_CAPACITY_SD_CARD,           /* high capacity SD card */
	SDIO_SECURE_DIGITAL_IO_CARD,          /* secure digital IO card */
	SDIO_SECURE_DIGITAL_IO_COMBO_CARD,    /* secure digital IO combo card */
	SDIO_MULTIMEDIA_CARD,                 /* multimedia card */
	SDIO_HIGH_CAPACITY_MULTIMEDIA_CARD,   /* high capacity multimedia card */
	SDIO_HIGH_SPEED_MULTIMEDIA_CARD       /* high speed multimedia card */
} sdio_card_t;

/* card identification (CID) register */
typedef struct {
	__IO uint8_t mid;                     /* manufacturer ID */
	__IO uint16_t oid;                    /* OEM/application ID */
	__IO uint32_t pnm0;                   /* product name */
	__IO uint8_t pnm1;                    /* product name */
	__IO uint8_t prv;                     /* product revision */
	__IO uint32_t psn;                    /* product serial number */
	__IO uint16_t mdt;                    /* manufacturing date */
	__IO uint8_t cid_crc;                 /* CID CRC7 checksum */
} sd_cid_struct;

/* CSD register (CSD version 1.0 and 2.0) */
typedef struct {
	__IO uint8_t csd_struct;              /* CSD struct */
	__IO uint8_t taac;                    /* data read access-time */
	__IO uint8_t nsac;                    /* data read access-time in CLK cycles */
	__IO uint8_t tran_speed;              /* max. data transfer rate */
	__IO uint16_t ccc;                    /* card command classes */
	__IO uint8_t read_bl_len;             /* max. read data block length */
	__IO uint8_t read_bl_partial;         /* partial blocks for read allowed */
	__IO uint8_t write_blk_misalign;      /* write block misalignment */
	__IO uint8_t read_blk_misalign;       /* read block misalignment */
	__IO uint8_t dsp_imp;                 /* DSR implemented */
	__IO uint32_t c_size;                 /* device size, 12 bits in CSD version 1.0, 22 bits in CSD version 2.0 */
	__IO uint8_t vdd_r_curr_min;          /* max. read current @VDD min, CSD version 1.0 */
	__IO uint8_t vdd_r_curr_max;          /* max. read current @VDD max, CSD version 1.0 */
	__IO uint8_t vdd_w_curr_min;          /* max. write current @VDD min, CSD version 1.0 */
	__IO uint8_t vdd_w_curr_max;          /* max. write current @VDD max, CSD version 1.0 */
	__IO uint8_t c_size_mult;             /* device size multiplier, CSD version 1.0 */
	__IO uint8_t erase_blk_en;            /* erase single block enable */
	__IO uint8_t sector_size;             /* erase sector size */
	__IO uint8_t wp_grp_size;             /* write protect group size */
	__IO uint8_t wp_grp_enable;           /* write protect group enable */
	__IO uint8_t r2w_factor;              /* write speed factor */
	__IO uint8_t write_bl_len;            /* max. write data block length */
	__IO uint8_t write_bl_partial;        /* partial blocks for write allowed */
	__IO uint8_t file_format_grp;         /* file format group */
	__IO uint8_t copy_flag;               /* copy flag (OTP) */
	__IO uint8_t perm_write_protect;      /* permanent write protection */
	__IO uint8_t tmp_write_protect;       /* temporary write protection */
	__IO uint8_t file_format;             /* file format */
	__IO uint8_t csd_crc;                 /* CSD CRC checksum */
} sd_csd_struct;

/* card info */
typedef struct {
	sd_cid_struct card_cid;        /* CID register */
	sd_csd_struct card_csd;        /* CSD register */
	sdio_card_t card_type; 				/* card tpye */
	uint32_t card_capacity;        /* card capacity */
	uint32_t card_blocksize;       /* card block size */
	uint16_t card_rca;             /* card relative card address */
} sd_card_info_struct;

/* sd error flags */
typedef enum sdcard_error {
	SD_OUT_OF_RANGE = 0,           /* command's argument was out of range */
	SD_ADDRESS_ERROR,              /* misaligned address which did not match the block length */
	SD_BLOCK_LEN_ERROR,            /* transferred block length is not allowed for the card or the number of transferred bytes does not match the block length */
	SD_ERASE_SEQ_ERROR,            /* an error in the sequence of erase command occurs */
	SD_ERASE_PARAM,                /* an invalid selection of write-blocks for erase occurred */
	SD_WP_VIOLATION,               /* attempt to program a write protect block or permanent write protected card */
	SD_LOCK_UNLOCK_FAILED,         /* sequence or password error has been detected in lock/unlock card command */
	SD_COM_CRC_ERROR,              /* CRC check of the previous command failed */
	SD_ILLEGAL_COMMAND,            /* command not legal for the card state */
	SD_CARD_ECC_FAILED,            /* card internal ECC was applied but failed to correct the data */
	SD_CC_ERROR,                   /* internal card controller error */
	SD_GENERAL_UNKNOWN_ERROR,      /* general or unknown error occurred during the operation */
	SD_CSD_OVERWRITE,              /* read only section of the CSD does not match the card content or an attempt to reverse the copy or permanent WP bits was made */
	SD_WP_ERASE_SKIP,              /* only partial address space was erased or the temporary or permanent write protected card was erased */
	SD_CARD_ECC_DISABLED,          /* command has been executed without using internal ECC */
	SD_ERASE_RESET,                /* erase sequence was cleared before executing because an out of erase sequence command was received */
	SD_AKE_SEQ_ERROR,              /* error in the sequence of the authentication process */
	SD_CMD_CRC_ERROR,              /* command response received (CRC check failed) */
	SD_DATA_CRC_ERROR,             /* data block sent/received (CRC check failed) */
	SD_CMD_RESP_TIMEOUT,           /* command response timeout */
	SD_DATA_TIMEOUT,               /* data timeout */
	SD_TX_UNDERRUN_ERROR,          /* transmit FIFO underrun error occurs */
	SD_RX_OVERRUN_ERROR,           /* received FIFO overrun error occurs */
	SD_START_BIT_ERROR,            /* start bit error in the bus */
	SD_VOLTRANGE_INVALID,          /* the voltage range is invalid */
	SD_PARAMETER_INVALID,          /* the parameter is invalid */
	SD_OPERATION_IMPROPER,         /* the operation is improper */
	SD_FUNCTION_UNSUPPORTED,       /* the function is unsupported */
	SD_ERROR,                      /* an error occurred */
	SD_OK,                         /* no error occurred */
} sdcard_error_t;

typedef enum {
  SD_NO_TRANSFER = 0,           /* no data transfer is acting */
  SD_TRANSFER_IN_PROGRESS       /* data transfer is in progress */
} sd_transfer_state_enum;

/* card status of R1 definitions */
#define SD_R1_OUT_OF_RANGE                  BIT(31)                   /* command's argument was out of the allowed range */
#define SD_R1_ADDRESS_ERROR                 BIT(30)                   /* misaligned address which did not match the block length */
#define SD_R1_BLOCK_LEN_ERROR               BIT(29)                   /* transferred block length is not allowed */
#define SD_R1_ERASE_SEQ_ERROR               BIT(28)                   /* an error in the sequence of erase commands occurred */
#define SD_R1_ERASE_PARAM                   BIT(27)                   /* an invalid selection of write-blocks for erase occurred */
#define SD_R1_WP_VIOLATION                  BIT(26)                   /* the host attempts to write to a protected block or to the temporary or permanent write protected card */
#define SD_R1_CARD_IS_LOCKED                BIT(25)                   /* the card is locked by the host */
#define SD_R1_LOCK_UNLOCK_FAILED            BIT(24)                   /* a sequence or password error has been detected in lock/unlock card command */
#define SD_R1_COM_CRC_ERROR                 BIT(23)                   /* CRC check of the previous command failed */
#define SD_R1_ILLEGAL_COMMAND               BIT(22)                   /* command not legal for the card state */
#define SD_R1_CARD_ECC_FAILED               BIT(21)                   /* card internal ECC was applied but failed to correct the data */
#define SD_R1_CC_ERROR                      BIT(20)                   /* internal card controller error */
#define SD_R1_GENERAL_UNKNOWN_ERROR         BIT(19)                   /* a general or an unknown error occurred during the operation */
#define SD_R1_CSD_OVERWRITE                 BIT(16)                   /* read only section of the CSD does not match or attempt to reverse the copy or permanent WP bits */
#define SD_R1_WP_ERASE_SKIP                 BIT(15)                   /* partial address space was erased */
#define SD_R1_CARD_ECC_DISABLED             BIT(14)                   /* command has been executed without using the internal ECC */
#define SD_R1_ERASE_RESET                   BIT(13)                   /* an erase sequence was cleared before executing */
#define SD_R1_READY_FOR_DATA                BIT(8)                    /* correspond to buffer empty signaling on the bus */
#define SD_R1_APP_CMD                       BIT(5)                    /* card will expect ACMD */
#define SD_R1_AKE_SEQ_ERROR                 BIT(3)                    /* error in the sequence of the authentication process */
#define SD_R1_ERROR_BITS                    ((uint32_t)0xFDF9E008U)   /* all the R1 error bits */

/* card status of R6 definitions */
#define SD_R6_COM_CRC_ERROR                 BIT(15)                   /* CRC check of the previous command failed */
#define SD_R6_ILLEGAL_COMMAND               BIT(14)                   /* command not legal for the card state */
#define SD_R6_ERROR         								BIT(13)                   /* a general or an unknown error occurred during the operation */

/* card state */
#define SD_CARDSTATE_IDLE                   ((uint8_t)0x00)           /* card is in idle state */
#define SD_CARDSTATE_READY                  ((uint8_t)0x01)           /* card is in ready state */
#define SD_CARDSTATE_IDENTIFICAT            ((uint8_t)0x02)           /* card is in identificat state */
#define SD_CARDSTATE_STANDBY                ((uint8_t)0x03)           /* card is in standby state */
#define SD_CARDSTATE_TRANSFER               ((uint8_t)0x04)           /* card is in transfer state */
#define SD_CARDSTATE_DATA                   ((uint8_t)0x05)           /* card is in data sending state */
#define SD_CARDSTATE_RECEIVING              ((uint8_t)0x06)           /* card is in receiving state */
#define SD_CARDSTATE_PROGRAMMING            ((uint8_t)0x07)           /* card is in programming state */
#define SD_CARDSTATE_DISCONNECT             ((uint8_t)0x08)           /* card is in disconnect state */
#define SD_CARDSTATE_LOCKED                 ((uint32_t)0x02000000U)    /* card is in locked state */

#define SD_CHECK_PATTERN                    ((uint32_t)0x000001AAU)    /* check pattern for CMD8 */
#define SD_VOLTAGE_WINDOW                   ((uint32_t)0x80100000U)    /* host 3.3V request in ACMD41 */

/* parameters for ACMD41(voltage validation) */
#define SD_HIGH_CAPACITY                    ((uint32_t)0x40000000U)    /* high capacity SD memory card */
#define SD_STD_CAPACITY                     ((uint32_t)0x00000000U)    /* standard capacity SD memory card */
#define SD_SWITCH_1_8V_CAPACITY             ((uint32_t)0x01000000U)

/* SD bus width, check SCR register */
#define SD_BUS_WIDTH_4BIT                   ((uint32_t)0x00040000U)    /* 4-bit width bus mode */
#define SD_BUS_WIDTH_1BIT                   ((uint32_t)0x00010000U)    /* 1-bit width bus mode */

/* masks for SCR register */
#define SD_MASK_0_7BITS                     ((uint32_t)0x000000FFU)    /* mask [7:0] bits */
#define SD_MASK_8_15BITS                    ((uint32_t)0x0000FF00U)    /* mask [15:8] bits */
#define SD_MASK_16_23BITS                   ((uint32_t)0x00FF0000U)    /* mask [23:16] bits */
#define SD_MASK_24_31BITS                   ((uint32_t)0xFF000000U)    /* mask [31:24] bits */

#define SDIO_FIFO_ADDR                      ((uint32_t)SDIO + 0x80U)  /* address of SDIO_FIFO */
#define SD_FIFOHALF_WORDS                   ((uint32_t)0x00000008U)    /* words of FIFO half full/empty */
#define SD_FIFOHALF_BYTES                   ((uint32_t)0x00000020U)    /* bytes of FIFO half full/empty */

#define SD_DATATIMEOUT                      ((uint32_t)0xFFFFFFFFU)    /* DSM data timeout */
#define SD_MAX_VOLT_VALIDATION              ((uint32_t)0x0000FFFFU)    /* the maximum times of voltage validation */
#define SD_MAX_DATA_LENGTH                  ((uint32_t)0x01FFFFFFU)    /* the maximum length of data */
#define SD_ALLZERO                          ((uint32_t)0x00000000U)    /* all zero */
#define SD_RCA_SHIFT                        ((uint8_t)0x10)           /* RCA shift bits */
#define SD_CLK_DIV_INIT                     ((uint16_t)0x012A)        /* SD clock division in initilization phase */
#define SD_CLK_DIV_TRANS                    ((uint16_t)0x0009)        /* SD clock division in transmission phase */

#define SDIO_MASK_INTC_FLAGS                ((uint32_t)0x00C007FFU)    /* mask flags of SDIO_INTC */

extern uint32_t sd_scr[2];      /* SD card SCR */

/* function declarations */
/* initialize the SD card and put in standby mode */
sdcard_error_t sd_init(uint32_t clock_edge, uint32_t clock_bypass, uint32_t clock_powersave, uint16_t clock_division, uint32_t bus_mode);
/* initialize the card and get CID and CSD of the card */
sdcard_error_t sd_card_init(void);
/* configure the clock and the work voltage, and get the card type */
sdcard_error_t sd_power_on(void);
/* close the power of SDIO */
sdcard_error_t sd_power_off(void);

/* configure the bus mode */
sdcard_error_t sd_bus_mode_config(uint32_t bmode);
/* configure the mode of transmission */
sdcard_error_t sd_transfer_mode_config(uint32_t txmode);

/* read a block data into a buffer from the specified address of a card */
sdcard_error_t sd_block_read(uint32_t *buf, uint32_t addr, uint16_t blksize);
/* read multiple blocks data into a buffer from the specified address of a card */
sdcard_error_t sd_multiblocks_read(uint32_t *buf, uint32_t addr, uint16_t blksize, uint32_t blksnum);
/* write a block data to the specified address of a card */
sdcard_error_t sd_block_write(uint32_t *buf, uint32_t addr, uint16_t blksize);
/* write multiple blocks data to the specified address of a card */
sdcard_error_t sd_multiblocks_write(uint32_t *buf, uint32_t addr, uint16_t blksize, uint32_t blksnum);
/* erase a continuous area of a card */
sdcard_error_t sd_erase(uint32_t startaddr, uint32_t endaddr);
/* process all the interrupts which the corresponding flags are set */
sdcard_error_t sd_interrupts_process(void);

sdcard_error_t sd_card_state_get(uint8_t *pcardstate);
/* select or deselect a card */
sdcard_error_t sd_card_select_deselect(uint16_t card);
/* get the card status whose response format R1 contains a 32-bit field */
sdcard_error_t sd_cardstatus_get(uint32_t *cardstatus);
/* get the SD status, the size of the SD status is one data block of 512 bit */
sdcard_error_t sd_sdstatus_get(uint32_t *sdstatus);
/* stop an ongoing data transfer */
sdcard_error_t sd_transfer_stop(void);
/* lock or unlock a card */
sdcard_error_t sd_lock_unlock(uint8_t lockstate);

/* get the data transfer state */
sd_transfer_state_enum sd_transfer_state_get(void);
/* get SD card capacity(KB) */
uint32_t sd_card_capacity_get(void);
/* get the detailed information of the SD card based on received CID and CSD */
sdcard_error_t sd_card_information_get(sd_card_info_struct *cardinfo);

#ifdef __cplusplus
}
#endif

#endif /* SDCARD_H */
