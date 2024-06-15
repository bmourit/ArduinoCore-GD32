/*!
	\file    sdcard.c
	\brief   SD card driver 

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

#ifdef ARDUINO_ARCH_GD32

//#if ENABLED(ONBOARD_SDIO)

#include <stddef.h>
#include "sdcard.h"

#include <gd32f30x_sdio.h>
#include <gd32f30x_dma.h>
#include <gd32f30x_rcu.h>


uint32_t sd_scr[2] = {0,0};										/* content of SCR register */

static sdio_card_t cardtype = SDIO_STD_CAPACITY_SD_CARD_V2_0; 	/* SD card type */
static uint32_t sd_csd[4] = {0,0,0,0};							/* content of CSD register */
static uint32_t sd_cid[4] = {0,0,0,0};							/* content of CID register */
static uint16_t sd_rca = 0;										/* RCA of SD card */
static uint32_t transmode = SD_DMA_MODE;
static uint32_t totalnumber_bytes = 0, stopcondition = 0;
static __IO sdcard_error_t transerror = SD_OK;
static __IO uint32_t transend = 0, number_bytes = 0;
static uint32_t sd_class = 0;

/* check if the command sent error occurs */
static sdcard_error_t cmdsent_error_check(void);
/* check if error occurs for R1 response */
static sdcard_error_t r1_error_check(uint8_t cmdindex);
/* check if error type for R1 response */
static sdcard_error_t r1_error_type_check(uint32_t resp);
/* check if error occurs for R2 response */
static sdcard_error_t r2_error_check(void);
/* check if error occurs for R3 response */
static sdcard_error_t r3_error_check(void);
/* check if error occurs for R6 response */
static sdcard_error_t r6_error_check(uint8_t cmdindex, uint16_t *prca);
/* check if error occurs for R7 response */
static sdcard_error_t r7_error_check(void);

/* configure the bus width mode */
static sdcard_error_t sd_bus_width_config(uint32_t buswidth);
/* get the SCR of corresponding card */
static sdcard_error_t sd_scr_get(uint16_t rca, uint32_t *pscr);
/* get the data block size */
static uint32_t sd_datablocksize_get(uint16_t bytesnumber);

/* configure the GPIO of SDIO interface */
//static void gpio_config(void);
/* configure the RCU of SDIO and DMA */
//static void rcu_config(void);
/* configure the DMA for SDIO transfer request */
static void dma_transfer_config(uint32_t *srcbuf, uint32_t bufsize);
/* configure the DMA for SDIO reveive request */
static void dma_receive_config(uint32_t *dstbuf, uint32_t bufsize);


sdcard_error_t sd_init(uint32_t clock_edge, uint32_t clock_bypass, uint32_t clock_powersave, uint16_t clock_division, uint32_t bus_mode)
{
	sdio_clock_config(clock_edge, clock_bypass, clock_powersave, clock_division);
  	sdio_bus_mode_set(bus_mode);
  	sdio_hardware_clock_disable();
  	return SD_OK;
}

/**
 * initialize the card and get CID and CSD of the card
 * returns sdcard_error_t status
 */
sdcard_error_t sd_card_init(void)
{
	sdcard_error_t state = SD_OK;
	uint16_t temp_rca = 1U;

	if (sdio_power_state_get() == SDIO_POWER_OFF) {
		return SD_OPERATION_IMPROPER;
	}

	/* the card is not I/O only card */
	if (cardtype != SDIO_SECURE_DIGITAL_IO_CARD) {
		/* send CMD2(SD_CMD_ALL_SEND_CID) to get the CID numbers */
		sdio_command_response_config(SD_CMD_ALL_SEND_CID, (uint32_t)0x0, SDIO_RESPONSETYPE_LONG);
		sdio_wait_type_set(SDIO_WAITTYPE_NO);
		sdio_csm_enable();

		state = r2_error_check();
		if (state != SD_OK) {
			return state;
		} else {
			/* get Card ID (CID) numbers */
			sd_cid[0] = sdio_response_get(SDIO_RESPONSE0);
			sd_cid[1] = sdio_response_get(SDIO_RESPONSE1);
			sd_cid[2] = sdio_response_get(SDIO_RESPONSE2);
			sd_cid[3] = sdio_response_get(SDIO_RESPONSE3);
		}
	}

	if (cardtype != SDIO_SECURE_DIGITAL_IO_CARD) {
		/* send CMD3(SEND_RELATIVE_ADDR) to ask the card to publish a new relative address (RCA) */
		sdio_command_response_config(SD_CMD_SEND_RELATIVE_ADDR, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
		sdio_wait_type_set(SDIO_WAITTYPE_NO);
		sdio_csm_enable();

		state = r6_error_check(SD_CMD_SEND_RELATIVE_ADDR, &temp_rca);
		if (SD_OK != state) {
			return state;
		}
	}
	if (cardtype != SDIO_SECURE_DIGITAL_IO_CARD) {
		/* get the sd card RCA */
		sd_rca = temp_rca;

		/* send CMD9(SEND_CSD) to get the addressed card's card-specific data (CSD) */
		sdio_command_response_config(SD_CMD_SEND_CSD, (uint32_t)(temp_rca << SD_RCA_SHIFT), SDIO_RESPONSETYPE_LONG);
		sdio_wait_type_set(SDIO_WAITTYPE_NO);
		sdio_csm_enable();

		state = r2_error_check();
		if (state != SD_OK) {
			return state;
		} else {
			/* get the Card Specific Data (CSD) */
			sd_csd[0] = sdio_response_get(SDIO_RESPONSE0);
			sd_csd[1] = sdio_response_get(SDIO_RESPONSE1);
			sd_csd[2] = sdio_response_get(SDIO_RESPONSE2);
			sd_csd[3] = sdio_response_get(SDIO_RESPONSE3);
		}
	}
	sd_class = (sdio_response_get(SDIO_RESPONSE1) >> 20U);

	state = sd_card_select_deselect(sd_rca);

	if (state != SD_OK) {
		return state;
	}

	/**
	 * now that we know about the card we are using,
	 * init default params once more to refresh
	 */
	(void)sd_init(SDIO_SDIOCLKEDGE_RISING, SDIO_CLOCKBYPASS_DISABLE, SDIO_CLOCKPWRSAVE_DISABLE, SD_CLK_DIV_INIT, SDIO_BUSMODE_1BIT);

	return SD_OK;
}

/**
 * configure the clock, working voltage, and card type
 * returns sdcard_error_t status
 */
sdcard_error_t sd_power_on(void)
{
	__IO uint32_t count = 0U;
	uint32_t sdcardtype = SD_STD_CAPACITY;
	uint32_t response = 0U, voltage_valid = 0U;
	sdcard_error_t status = SD_OK;

	/* CMD0: GO_IDLE_STATE */
	sdio_command_response_config(SD_CMD_GO_IDLE_STATE, (uint32_t)0x0, SDIO_RESPONSETYPE_NO);
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	sdio_csm_enable();

	status = cmdsent_error_check();
	if (status != SD_OK) {
		return status;
	}

	/* CMD8: SEND_IF_COND */
	sdio_command_response_config(SD_CMD_SEND_IF_COND, SD_CHECK_PATTERN, SDIO_RESPONSETYPE_SHORT);
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	sdio_csm_enable();

	if (r7_error_check() != SD_OK) {
		cardtype = SDIO_STD_CAPACITY_SD_CARD_V1_1;
		sdcardtype = SD_HIGH_CAPACITY;
		/* CMD0: GO_IDLE_STATE */
		sdio_command_response_config(SD_CMD_GO_IDLE_STATE, (uint32_t)0x0, SDIO_RESPONSETYPE_NO);
		sdio_wait_type_set(SDIO_WAITTYPE_NO);
		sdio_csm_enable();

		status = cmdsent_error_check();
		if (status != SD_OK) {
			return status;
		}
	} else {
		cardtype = SDIO_STD_CAPACITY_SD_CARD_V2_0;
		sdcardtype = SD_HIGH_CAPACITY;
	}
	if (cardtype == SDIO_STD_CAPACITY_SD_CARD_V2_0) {
		/* CMD55: APP_CMD */
		sdio_command_response_config(SD_CMD_APP_CMD, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
		sdio_wait_type_set(SDIO_WAITTYPE_NO);
		sdio_csm_enable();

		status = r1_error_check(SD_CMD_APP_CMD);
		if (status != SD_OK) {
			return status;
		}
	}
	/* ACMD42: SD_APP_OP_COND with arg 0x80100000 */
	while ((count < SD_MAX_VOLT_VALIDATION) && (voltage_valid == 0U)) {
		/* CMD55 APP_CMD */
		sdio_command_response_config(SD_CMD_APP_CMD, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
		sdio_wait_type_set(SDIO_WAITTYPE_NO);
		sdio_csm_enable();

		status = r1_error_check(SD_CMD_APP_CMD);
		if (status != SD_OK) {
			return status;
		}

		/* CMD41 */
		sdio_command_response_config(SD_ACMD_SD_SEND_OP_COND, (SD_VOLTAGE_WINDOW | sdcardtype | SD_SWITCH_1_8V_CAPACITY), SDIO_RESPONSETYPE_SHORT);
		sdio_wait_type_set(SDIO_WAITTYPE_NO);
		sdio_csm_enable();

		status = r3_error_check();
		if (status != SD_OK) {
			return status;
		}

		response = sdio_response_get(SDIO_RESPONSE0);
		voltage_valid = (((response >> 31U) == 1U) ? 1U : 0U);

		count++;
	}

	if (count >= SD_MAX_VOLT_VALIDATION) {
		return SD_VOLTRANGE_INVALID;
	}

	if ((response & SD_HIGH_CAPACITY) == SD_HIGH_CAPACITY) {
		cardtype = SDIO_HIGH_CAPACITY_SD_CARD;
	} else {
		sdcardtype = SD_STD_CAPACITY;
	}

	return SD_OK;
}

/*!
	\brief      close the power of SDIO
	\param[in]  none
	\param[out] none
	\retval     sdcard_error_t
*/
sdcard_error_t sd_power_off(void)
{
	sdcard_error_t status = SD_OK;
	sdio_power_state_set(SDIO_POWER_OFF);
	return status;
}

/*!
	\brief      configure the bus mode
	\param[in]  busmode: the bus mode
	  \arg        SDIO_BUSMODE_1BIT: 1-bit SDIO card bus mode
	  \arg        SDIO_BUSMODE_4BIT: 4-bit SDIO card bus mode
	  \arg        SDIO_BUSMODE_8BIT: 8-bit SDIO card bus mode (MMC only)
	\param[out] none
	\retval     sdcard_error_t
*/
sdcard_error_t sd_bus_mode_config(uint32_t busmode)
{
	sdcard_error_t status = SD_OK;
	if (SDIO_MULTIMEDIA_CARD == cardtype) {
		/* mmc card doesn't support this function */
		status = SD_FUNCTION_UNSUPPORTED;
		return status;
	} else if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == cardtype) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == cardtype) || 
			 (SDIO_HIGH_CAPACITY_SD_CARD == cardtype)) {
		if (SDIO_BUSMODE_8BIT == busmode) {
			/* 8 bit bus mode doesn't support */
			status = SD_FUNCTION_UNSUPPORTED;
			return status;
		} else if (SDIO_BUSMODE_4BIT == busmode) {
			/* configure sd bus width and the SDIO */
			status = sd_bus_width_config(SD_BUS_WIDTH_4BIT);
			if (SD_OK == status) {
				sdio_clock_config(SDIO_SDIOCLKEDGE_RISING, SDIO_CLOCKBYPASS_DISABLE,
									SDIO_CLOCKPWRSAVE_DISABLE, SD_CLK_DIV_TRANS);
				sdio_bus_mode_set(busmode);
				sdio_hardware_clock_disable();
			}
		} else if (SDIO_BUSMODE_1BIT == busmode) {
			/* configure sd bus width and the SDIO */
			status = sd_bus_width_config(SD_BUS_WIDTH_1BIT);
			if (SD_OK == status) {
				sdio_clock_config(SDIO_SDIOCLKEDGE_RISING, SDIO_CLOCKBYPASS_DISABLE, 
									SDIO_CLOCKPWRSAVE_DISABLE, SD_CLK_DIV_TRANS);
				sdio_bus_mode_set(busmode);
				sdio_hardware_clock_disable();
			}
		} else {
			status = SD_PARAMETER_INVALID;
		}
	}

	return status;
}

/*!
	\brief      configure the mode of transmission
	\param[in]  txmode: transfer mode
	  \arg        SD_DMA_MODE: DMA mode
	  \arg        SD_POLLING_MODE: polling mode
	\param[out] none
	\retval     sdcard_error_t
*/
sdcard_error_t sd_transfer_mode_config(uint32_t txmode)
{
	sdcard_error_t status = SD_OK;

	/* set the transfer mode */
	if ((SD_DMA_MODE == txmode) || (SD_POLLING_MODE == txmode)) {
		transmode = txmode;
	} else {
		status = SD_PARAMETER_INVALID;
	}

	return status;
}

/*!
	\brief      read a block data into a buffer from the specified address of a card
	\param[out] preadbuffer: a pointer that store a block read data
	\param[in]  readaddr: the read data address
	\param[in]  blocksize: the data block size
	\retval     sdcard_error_t
*/
sdcard_error_t sd_block_read(uint32_t *preadbuffer, uint32_t readaddr, uint16_t blocksize)
{
    sdcard_error_t status = SD_OK;
    
    if (NULL == preadbuffer) {
        return SD_PARAMETER_INVALID;
    }
    
    if (sdio_response_get(SDIO_RESPONSE0) & SD_CARDSTATE_LOCKED) {
        return SD_LOCK_UNLOCK_FAILED;
    }
    
    if (SDIO_HIGH_CAPACITY_SD_CARD == cardtype) {
        blocksize = 512;
        readaddr /= 512;
    }
    
    if ((blocksize > 0) && (blocksize <= 2048) && ((blocksize & (blocksize - 1)) == 0)) {
        uint32_t datablksize = sd_datablocksize_get(blocksize);

        sdio_data_config(0, 0, datablksize);
        sdio_data_transfer_config(SDIO_TRANSMODE_BLOCK, SDIO_TRANSDIRECTION_TOCARD);
        sdio_dsm_disable();
        sdio_dma_disable();

        sdio_command_response_config(SD_CMD_SET_BLOCKLEN, (uint32_t)blocksize, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        sdio_csm_enable();

        status = r1_error_check(SD_CMD_SET_BLOCKLEN);
        if (SD_OK != status) {
            return status;
        }

        totalnumber_bytes = blocksize;

        sdio_data_config(SD_DATATIMEOUT, totalnumber_bytes, datablksize);
        sdio_data_transfer_config(SDIO_TRANSMODE_BLOCK, SDIO_TRANSDIRECTION_TOSDIO);
        sdio_dsm_enable();

        sdio_command_response_config(SD_CMD_READ_SINGLE_BLOCK, (uint32_t)readaddr, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        sdio_csm_enable();

        status = r1_error_check(SD_CMD_READ_SINGLE_BLOCK);
        if (SD_OK != status) {
            return status;
        }

        if (SD_POLLING_MODE == transmode) {
            // Polling mode
            while (!sdio_flag_get(SDIO_FLAG_DTCRCERR | SDIO_FLAG_DTTMOUT | SDIO_FLAG_RXORE | SDIO_FLAG_DTBLKEND | SDIO_FLAG_STBITE)) {
                if (RESET != sdio_flag_get(SDIO_FLAG_RFH)) {
                    for (uint32_t count = 0; count < SD_FIFOHALF_WORDS; count++) {
                        *(preadbuffer + count) = sdio_data_read();
                    }
                    preadbuffer += SD_FIFOHALF_WORDS;
                }
            }

            if (RESET != sdio_flag_get(SDIO_FLAG_DTCRCERR)) {
                return SD_DATA_CRC_ERROR;
            } else if (RESET != sdio_flag_get(SDIO_FLAG_DTTMOUT)) {
                return SD_DATA_TIMEOUT;
            } else if (RESET != sdio_flag_get(SDIO_FLAG_RXORE)) {
                return SD_RX_OVERRUN_ERROR;
            } else if (RESET != sdio_flag_get(SDIO_FLAG_STBITE)) {
                return SD_START_BIT_ERROR;
            }

            while (RESET != sdio_flag_get(SDIO_FLAG_RXDTVAL)) {
                *preadbuffer = sdio_data_read();
                ++preadbuffer;
            }

            sdio_flag_clear(SDIO_MASK_INTC_FLAGS);
        } else if (SD_DMA_MODE == transmode) {
            // DMA mode
            sdio_interrupt_enable(SDIO_INT_CCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_RXORE | SDIO_INT_DTEND | SDIO_INT_STBITE);
            sdio_dma_enable();
            dma_receive_config(preadbuffer, blocksize);

            uint32_t timeout = 100000;
            while ((RESET == dma_flag_get(DMA1, DMA_CH3, DMA_FLAG_FTF)) && (timeout > 0)) {
                timeout--;
                if (timeout == 0) {
                    return SD_ERROR;
                }
            }
        } else {
            return SD_PARAMETER_INVALID;
        }
    } else {
        return SD_PARAMETER_INVALID;
    }

    return status;
}

/*!
	\brief      read multiple blocks data into a buffer from the specified address of a card
	\param[out] preadbuffer: a pointer that store multiple blocks read data
	\param[in]  readaddr: the read data address
	\param[in]  blocksize: the data block size
	\param[in]  blocksnumber: number of blocks that will be read
	\retval     sdcard_error_t
*/
sdcard_error_t sd_multiblocks_read(uint32_t *preadbuffer, uint32_t readaddr, uint16_t blocksize, uint32_t blocksnumber)
{
    sdcard_error_t status = SD_OK;

    if (NULL == preadbuffer) {
        return SD_PARAMETER_INVALID;
    }

    if (sdio_response_get(SDIO_RESPONSE0) & SD_CARDSTATE_LOCKED) {
        return SD_LOCK_UNLOCK_FAILED;
    }

    if (SDIO_HIGH_CAPACITY_SD_CARD == cardtype) {
        blocksize = 512;
        readaddr /= 512;
    }

    if ((blocksize > 0) && (blocksize <= 2048) && ((blocksize & (blocksize - 1)) == 0)) {
        uint32_t datablksize = sd_datablocksize_get(blocksize);

        sdio_data_config(0, 0, datablksize);
        sdio_data_transfer_config(SDIO_TRANSMODE_BLOCK, SDIO_TRANSDIRECTION_TOCARD);
        sdio_dsm_disable();
        sdio_dma_disable();

        sdio_command_response_config(SD_CMD_SET_BLOCKLEN, (uint32_t)blocksize, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        sdio_csm_enable();

        status = r1_error_check(SD_CMD_SET_BLOCKLEN);
        if (SD_OK != status) {
            return status;
        }

        if (blocksnumber > 1) {
            if (blocksnumber * blocksize > SD_MAX_DATA_LENGTH) {
                return SD_PARAMETER_INVALID;
            }

            totalnumber_bytes = blocksnumber * blocksize;

            sdio_data_config(SD_DATATIMEOUT, totalnumber_bytes, datablksize);
            sdio_data_transfer_config(SDIO_TRANSMODE_BLOCK, SDIO_TRANSDIRECTION_TOSDIO);
            sdio_dsm_enable();

            sdio_command_response_config(SD_CMD_READ_MULTIPLE_BLOCK, readaddr, SDIO_RESPONSETYPE_SHORT);
            sdio_wait_type_set(SDIO_WAITTYPE_NO);
            sdio_csm_enable();

            status = r1_error_check(SD_CMD_READ_MULTIPLE_BLOCK);
            if (SD_OK != status) {
                return status;
            }

            if (SD_POLLING_MODE == transmode) {
                // Polling mode
                while (!sdio_flag_get(SDIO_FLAG_DTCRCERR | SDIO_FLAG_DTTMOUT | SDIO_FLAG_RXORE | SDIO_FLAG_DTEND | SDIO_FLAG_STBITE)) {
                    if (RESET != sdio_flag_get(SDIO_FLAG_RFH)) {
                        for (uint32_t count = 0; count < SD_FIFOHALF_WORDS; count++) {
                            *(preadbuffer + count) = sdio_data_read();
                        }
                        preadbuffer += SD_FIFOHALF_WORDS;
                    }
                }

                if (RESET != sdio_flag_get(SDIO_FLAG_DTCRCERR)) {
                    return SD_DATA_CRC_ERROR;
                } else if (RESET != sdio_flag_get(SDIO_FLAG_DTTMOUT)) {
                    return SD_DATA_TIMEOUT;
                } else if (RESET != sdio_flag_get(SDIO_FLAG_RXORE)) {
                    return SD_RX_OVERRUN_ERROR;
                } else if (RESET != sdio_flag_get(SDIO_FLAG_STBITE)) {
                    return SD_START_BIT_ERROR;
                }

                while (RESET != sdio_flag_get(SDIO_FLAG_RXDTVAL)) {
                    *preadbuffer = sdio_data_read();
                    ++preadbuffer;
                }

                if (RESET != sdio_flag_get(SDIO_FLAG_DTEND)) {
                    if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == cardtype) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == cardtype) || 
                        (SDIO_HIGH_CAPACITY_SD_CARD == cardtype)) {
                        sdio_command_response_config(SD_CMD_STOP_TRANSMISSION, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
                        sdio_wait_type_set(SDIO_WAITTYPE_NO);
                        sdio_csm_enable();
                        status = r1_error_check(SD_CMD_STOP_TRANSMISSION);
                        if (SD_OK != status) {
                            return status;
                        }
                    }
                }

                sdio_flag_clear(SDIO_MASK_INTC_FLAGS);
            } else if (SD_DMA_MODE == transmode) {
                // DMA mode
                sdio_interrupt_enable(SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_RXORE | SDIO_INT_DTEND | SDIO_INT_STBITE);
                sdio_dma_enable();
                dma_receive_config(preadbuffer, totalnumber_bytes);
                
                uint32_t timeout = 100000;
                while ((RESET == dma_flag_get(DMA1, DMA_CH3, DMA_FLAG_FTF)) && (timeout > 0)) {
                    timeout--;
                    if (timeout == 0) {
                        return SD_ERROR;
                    }
                }

                while ((0 == transend) && (SD_OK == transerror)) {
                }

                if (SD_OK != transerror) {
                    return transerror;
                }
            } else {
                return SD_PARAMETER_INVALID;
            }
        }
    } else {
        return SD_PARAMETER_INVALID;
    }

    return status;
}

/*!
	\brief      write a block data to the specified address of a card
	\param[in]  pwritebuffer: a pointer that store a block data to be transferred
	\param[in]  writeaddr: the read data address
	\param[in]  blocksize: the data block size
	\param[out] none
	\retval     sdcard_error_t
*/
sdcard_error_t sd_block_write(uint32_t *pwritebuffer, uint32_t writeaddr, uint16_t blocksize)
{
	/* initialize the variables */
	sdcard_error_t status = SD_OK;
	uint8_t cardstate = 0;
	uint32_t count = 0, align = 0, datablksize = SDIO_DATABLOCKSIZE_1BYTE, *ptempbuff = pwritebuffer;
	uint32_t transbytes = 0, restwords = 0, response = 0;
	__IO uint32_t timeout = 0;

	if (NULL == pwritebuffer) {
		status = SD_PARAMETER_INVALID;
		return status;
	}

	transerror = SD_OK;
	transend = 0;
	totalnumber_bytes = 0;

	/* clear all DSM configuration */
	sdio_data_config(0, 0, SDIO_DATABLOCKSIZE_1BYTE);
	sdio_data_transfer_config(SDIO_TRANSMODE_BLOCK, SDIO_TRANSDIRECTION_TOCARD);
	sdio_dsm_disable();
	sdio_dma_disable();

	/* check whether the card is locked */
	if (sdio_response_get(SDIO_RESPONSE0) & SD_CARDSTATE_LOCKED) {
		status = SD_LOCK_UNLOCK_FAILED;
		return status;
	}

	/* blocksize is fixed in 512B for SDHC card */
	if (SDIO_HIGH_CAPACITY_SD_CARD == cardtype) {
		blocksize = 512;
		writeaddr /= 512;
	}

	align = blocksize & (blocksize - 1);
	if ((blocksize > 0) && (blocksize <= 2048) && (0 == align)) {
		datablksize = sd_datablocksize_get(blocksize);
		/* send CMD16(SET_BLOCKLEN) to set the block length */
		sdio_command_response_config(SD_CMD_SET_BLOCKLEN, (uint32_t)blocksize, SDIO_RESPONSETYPE_SHORT);
		sdio_wait_type_set(SDIO_WAITTYPE_NO);
		sdio_csm_enable();
	
		/* r1 error check */
		status = r1_error_check(SD_CMD_SET_BLOCKLEN);
		if (SD_OK != status) {
			return status;
		}
	} else {
		status = SD_PARAMETER_INVALID;
		return status;
	}

	/* send CMD13(SEND_STATUS), addressed card sends its status registers */
	sdio_command_response_config(SD_CMD_SEND_STATUS, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	sdio_csm_enable();

	/* r1 error check */
	status = r1_error_check(SD_CMD_SEND_STATUS);
	if (SD_OK != status) {
		return status;
	}

	response = sdio_response_get(SDIO_RESPONSE0);
	timeout = 100000;

	while ((0 == (response & SD_R1_READY_FOR_DATA)) && (timeout > 0)) {
		/* continue to send CMD13 to polling the state of card until buffer empty or timeout */
		--timeout;

		/* send CMD13(SEND_STATUS), addressed card sends its status registers */
		sdio_command_response_config(SD_CMD_SEND_STATUS, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
		sdio_wait_type_set(SDIO_WAITTYPE_NO);
		sdio_csm_enable();

		/* r1 error check */
		status = r1_error_check(SD_CMD_SEND_STATUS);
		if (SD_OK != status) {
			return status;
		}
		response = sdio_response_get(SDIO_RESPONSE0);
	}

	if (0 == timeout) {
		return SD_ERROR;
	}

	/* send CMD24(WRITE_BLOCK) to write a block */
	sdio_command_response_config(SD_CMD_WRITE_BLOCK, writeaddr, SDIO_RESPONSETYPE_SHORT);
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	sdio_csm_enable();

	/* r1 error check */
	status = r1_error_check(SD_CMD_WRITE_BLOCK);
	if (SD_OK != status) {
		return status;
	}

	stopcondition = 0;
	totalnumber_bytes = blocksize;

	/* configure the SDIO data transmission */
	sdio_data_config(SD_DATATIMEOUT, totalnumber_bytes, datablksize);
	sdio_data_transfer_config(SDIO_TRANSMODE_BLOCK, SDIO_TRANSDIRECTION_TOCARD);
	sdio_dsm_enable();

	if (SD_POLLING_MODE == transmode) {
		/* polling mode */
		while (!sdio_flag_get(SDIO_FLAG_DTCRCERR | SDIO_FLAG_DTTMOUT | SDIO_FLAG_TXURE | SDIO_FLAG_DTBLKEND | SDIO_FLAG_STBITE)) {
			if (RESET != sdio_flag_get(SDIO_FLAG_TFH)) {
				/* at least 8 words can be written into the FIFO */
				if ((totalnumber_bytes - transbytes) < SD_FIFOHALF_BYTES) {
					restwords = (totalnumber_bytes - transbytes) / 4 + (((totalnumber_bytes - transbytes) % 4 == 0) ? 0 : 1);
					for (count = 0; count < restwords; count++) {
						sdio_data_write(*ptempbuff);
						++ptempbuff;
						transbytes += 4;
					}
				} else {
					for (count = 0; count < SD_FIFOHALF_WORDS; count++) {
						sdio_data_write(*(ptempbuff + count));
					}
					/* 8 words(32 bytes) has been transferred */
					ptempbuff += SD_FIFOHALF_WORDS;
					transbytes += SD_FIFOHALF_BYTES;
				}
			}
		}

		/* error check */
		if (RESET != sdio_flag_get(SDIO_FLAG_DTCRCERR)) {
			status = SD_DATA_CRC_ERROR;
			sdio_flag_clear(SDIO_FLAG_DTCRCERR);
			return status;
		} else if (RESET != sdio_flag_get(SDIO_FLAG_DTTMOUT)) {
			status = SD_DATA_TIMEOUT;
			sdio_flag_clear(SDIO_FLAG_DTTMOUT);
			return status;
		} else if (RESET != sdio_flag_get(SDIO_FLAG_TXURE)) {
			status = SD_TX_UNDERRUN_ERROR;
			sdio_flag_clear(SDIO_FLAG_TXURE);
			return status;
		} else if (RESET != sdio_flag_get(SDIO_FLAG_STBITE)) {
			status = SD_START_BIT_ERROR;
			sdio_flag_clear(SDIO_FLAG_STBITE);
			return status;
		}
	} else if (SD_DMA_MODE == transmode) {
		/* DMA mode */
		/* enable the SDIO corresponding interrupts and DMA */
		sdio_interrupt_enable(SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_TXURE | SDIO_INT_DTEND | SDIO_INT_STBITE);
		dma_transfer_config(pwritebuffer, blocksize);
		sdio_dma_enable();
		timeout = 100000;

		while ((RESET == dma_flag_get(DMA1, DMA_CH3, DMA_FLAG_FTF)) && (timeout > 0)) {
			timeout--;
			if (0 == timeout) {
				return SD_ERROR;
			}
		}
		while ((0 == transend) && (SD_OK == transerror)) {
		}

		if (SD_OK != transerror) {
			return transerror;
		}
	} else {
		status = SD_PARAMETER_INVALID;
		return status;
	}

	/* clear the SDIO_INTC flags */
	sdio_flag_clear(SDIO_MASK_INTC_FLAGS);

	/* get the card state and wait the card is out of programming and receiving state */
	status = sd_card_state_get(&cardstate);
	while ((SD_OK == status) && ((SD_CARDSTATE_PROGRAMMING == cardstate) || (SD_CARDSTATE_RECEIVING == cardstate))) {
		status = sd_card_state_get(&cardstate);
	}

	return status;
}

/*!
	\brief      write multiple blocks data to the specified address of a card
	\param[in]  pwritebuffer: a pointer that store multiple blocks data to be transferred
	\param[in]  writeaddr: the read data address
	\param[in]  blocksize: the data block size
	\param[in]  blocksnumber: number of blocks that will be written
	\param[out] none
	\retval     sdcard_error_t
*/
sdcard_error_t sd_multiblocks_write(uint32_t *pwritebuffer, uint32_t writeaddr, uint16_t blocksize, uint32_t blocksnumber)
{
	/* initialize the variables */
	sdcard_error_t status = SD_OK;
	uint8_t cardstate = 0;
	uint32_t count = 0, align = 0, datablksize = SDIO_DATABLOCKSIZE_1BYTE, *ptempbuff = pwritebuffer;
	uint32_t transbytes = 0, restwords = 0;
	__IO uint32_t timeout = 0;

	if (NULL == pwritebuffer) {
		status = SD_PARAMETER_INVALID;
		return status;
	}

	transerror = SD_OK;
	transend = 0;
	totalnumber_bytes = 0;

	/* clear all DSM configuration */
	sdio_data_config(0, 0, SDIO_DATABLOCKSIZE_1BYTE);
	sdio_data_transfer_config(SDIO_TRANSMODE_BLOCK, SDIO_TRANSDIRECTION_TOCARD);
	sdio_dsm_disable();
	sdio_dma_disable();

	/* check whether the card is locked */
	if (sdio_response_get(SDIO_RESPONSE0) & SD_CARDSTATE_LOCKED) {
		status = SD_LOCK_UNLOCK_FAILED;
		return status;
	}

	/* blocksize is fixed in 512B for SDHC card */
	if (SDIO_HIGH_CAPACITY_SD_CARD == cardtype) {
		blocksize = 512;
		writeaddr /= 512;
	}

	align = blocksize & (blocksize - 1);
	if ((blocksize > 0) && (blocksize <= 2048) && (0 == align)) {
		datablksize = sd_datablocksize_get(blocksize);
		/* send CMD16(SET_BLOCKLEN) to set the block length */
		sdio_command_response_config(SD_CMD_SET_BLOCKLEN, (uint32_t)blocksize, SDIO_RESPONSETYPE_SHORT);
		sdio_wait_type_set(SDIO_WAITTYPE_NO);
		sdio_csm_enable();

		/* r1 error check */
		status = r1_error_check(SD_CMD_SET_BLOCKLEN);
		if (SD_OK != status) {
			return status;
		}
	} else {
		status = SD_PARAMETER_INVALID;
		return status;
	}

	/* send CMD13(SEND_STATUS), addressed card sends its status registers */
	sdio_command_response_config(SD_CMD_SEND_STATUS, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	sdio_csm_enable();

	/* r1 error check */
	status = r1_error_check(SD_CMD_SEND_STATUS);
	if (SD_OK != status) {
		return status;
	}

	if (blocksnumber > 1) {
		if (blocksnumber * blocksize > SD_MAX_DATA_LENGTH) {
			status = SD_PARAMETER_INVALID;
			return status;
		}

		if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == cardtype) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == cardtype) || 
			(SDIO_HIGH_CAPACITY_SD_CARD == cardtype)) {
			/* send CMD55(APP_CMD) to indicate next command is application specific command */
			sdio_command_response_config(SD_CMD_APP_CMD, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
			sdio_wait_type_set(SDIO_WAITTYPE_NO);
			sdio_csm_enable();

			/* r1 error check */
			status = r1_error_check(SD_CMD_APP_CMD);
			if (SD_OK != status) {
				return status;
			}

			/* send ACMD23(SET_WR_BLK_ERASE_COUNT) to set the number of write blocks to be preerased before writing */
			sdio_command_response_config(SD_ACMD_SET_WR_BLK_ERASE_COUNT, blocksnumber, SDIO_RESPONSETYPE_SHORT);
			sdio_wait_type_set(SDIO_WAITTYPE_NO);
			sdio_csm_enable();

			/* r1 error check */
			status = r1_error_check(SD_ACMD_SET_WR_BLK_ERASE_COUNT);
			if (SD_OK != status) {
				return status;
			}
		}

		/* send CMD25(WRITE_MULTIPLE_BLOCK) to continuously write blocks of data */
		sdio_command_response_config(SD_CMD_WRITE_MULTIPLE_BLOCK, writeaddr, SDIO_RESPONSETYPE_SHORT);
		sdio_wait_type_set(SDIO_WAITTYPE_NO);
		sdio_csm_enable();

		/* r1 error check */
		status = r1_error_check(SD_CMD_WRITE_MULTIPLE_BLOCK);
		if (SD_OK != status) {
			return status;
		}

		stopcondition = 1;
		totalnumber_bytes = blocksnumber * blocksize;

		/* configure the SDIO data transmission */
		sdio_data_config(SD_DATATIMEOUT, totalnumber_bytes, datablksize);
		sdio_data_transfer_config(SDIO_TRANSMODE_BLOCK, SDIO_TRANSDIRECTION_TOCARD);
		sdio_dsm_enable();

		if (SD_POLLING_MODE == transmode) {
			/* polling mode */
			while (!sdio_flag_get(SDIO_FLAG_DTCRCERR | SDIO_FLAG_DTTMOUT | SDIO_FLAG_TXURE | SDIO_FLAG_DTEND | SDIO_FLAG_STBITE)) {
				if (RESET != sdio_flag_get(SDIO_FLAG_TFH)) {
					/* at least 8 words can be written into the FIFO */
					if (!((totalnumber_bytes - transbytes) < SD_FIFOHALF_BYTES)) {
						for (count = 0; count < SD_FIFOHALF_WORDS; count++) {
							sdio_data_write(*(ptempbuff + count));
						}
						/* 8 words(32 bytes) has been transferred */
						ptempbuff += SD_FIFOHALF_WORDS;
						transbytes += SD_FIFOHALF_BYTES;
					} else {
						restwords = (totalnumber_bytes - transbytes) / 4 + (((totalnumber_bytes - transbytes) % 4 == 0) ? 0 : 1);
						for (count = 0; count < restwords; count++) {
							sdio_data_write(*ptempbuff);
							++ptempbuff;
							transbytes += 4;
						}
					}
				}
			}

			/* error check */
			if (RESET != sdio_flag_get(SDIO_FLAG_DTCRCERR)) {
				status = SD_DATA_CRC_ERROR;
				sdio_flag_clear(SDIO_FLAG_DTCRCERR);
				return status;
			} else if (RESET != sdio_flag_get(SDIO_FLAG_DTTMOUT)) {
				status = SD_DATA_TIMEOUT;
				sdio_flag_clear(SDIO_FLAG_DTTMOUT);
				return status;
			} else if (RESET != sdio_flag_get(SDIO_FLAG_TXURE)) {
				status = SD_TX_UNDERRUN_ERROR;
				sdio_flag_clear(SDIO_FLAG_TXURE);
				return status;
			} else if (RESET != sdio_flag_get(SDIO_FLAG_STBITE)) {
				status = SD_START_BIT_ERROR;
				sdio_flag_clear(SDIO_FLAG_STBITE);
				return status;
			}

			if (RESET != sdio_flag_get(SDIO_FLAG_DTEND)) {
				if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == cardtype) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == cardtype) || 
					(SDIO_HIGH_CAPACITY_SD_CARD == cardtype)) {
					/* send CMD12(STOP_TRANSMISSION) to stop transmission */
					sdio_command_response_config(SD_CMD_STOP_TRANSMISSION, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
					sdio_wait_type_set(SDIO_WAITTYPE_NO);
					sdio_csm_enable();
					/* check if some error occurs */
					status = r1_error_check(SD_CMD_STOP_TRANSMISSION);
					if (SD_OK != status) {
						return status;
					}
				}
			}
			sdio_flag_clear(SDIO_MASK_INTC_FLAGS);
		} else if (SD_DMA_MODE == transmode) {
			/* DMA mode */
			/* enable SDIO corresponding interrupts and DMA */
			sdio_interrupt_enable(SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_TXURE | SDIO_INT_DTEND | SDIO_INT_STBITE);
			sdio_dma_enable();
			dma_transfer_config(pwritebuffer, totalnumber_bytes);
			timeout = 100000;

			while ((RESET == dma_flag_get(DMA1, DMA_CH3, DMA_FLAG_FTF) && (timeout > 0))) {
				timeout--;
				if (0 == timeout) {
					return SD_ERROR;
				}
			}
			while ((0 == transend) && (SD_OK == transerror)) {
			}

			if (SD_OK != transerror) {
				return transerror;
			}
		} else {
			status = SD_PARAMETER_INVALID;
			return status;
		}
	}

	/* clear the SDIO_INTC flags */
	sdio_flag_clear(SDIO_MASK_INTC_FLAGS);

	/* get the card state and wait the card is out of programming and receiving state */
	status = sd_card_state_get(&cardstate);
	while ((SD_OK == status) && ((SD_CARDSTATE_PROGRAMMING == cardstate) || (SD_CARDSTATE_RECEIVING == cardstate))) {
		status = sd_card_state_get(&cardstate);
	}

	return status;
}

/*!
	\brief      erase a continuous area of a card
	\param[in]  startaddr: the start address
	\param[in]  endaddr: the end address
	\param[out] none
	\retval     sdcard_error_t
*/
sdcard_error_t sd_erase(uint32_t startaddr, uint32_t endaddr)
{
	/* initialize the variables */
	sdcard_error_t status = SD_OK;
	uint32_t count = 0, clkdiv = 0;
	__IO uint32_t delay = 0;
	uint8_t cardstate = 0, tempbyte = 0;
	uint16_t tempccc = 0;

	/* get the card command classes from CSD */
	tempbyte = (uint8_t)((sd_csd[1] & SD_MASK_24_31BITS) >> 24);
	tempccc = (uint16_t)((uint16_t)tempbyte << 4);
	tempbyte = (uint8_t)((sd_csd[1] & SD_MASK_16_23BITS) >> 16);
	tempccc |= (uint16_t)((uint16_t)(tempbyte & 0xF0) >> 4);
	if (0 == (tempccc & SD_CCC_ERASE)) {
		/* don't support the erase command */
		status = SD_FUNCTION_UNSUPPORTED;
		return status;
	}
	clkdiv = (SDIO_CLKCTL & SDIO_CLKCTL_DIV);
	clkdiv += ((SDIO_CLKCTL & SDIO_CLKCTL_DIV8)>>31)*256;
	clkdiv += 2;
	delay = 120000 / clkdiv;

	/* check whether the card is locked */
	if (sdio_response_get(SDIO_RESPONSE0) & SD_CARDSTATE_LOCKED) {
		status = SD_LOCK_UNLOCK_FAILED;
		return(status);
	}

	/* blocksize is fixed in 512B for SDHC card */
	if (SDIO_HIGH_CAPACITY_SD_CARD == cardtype) {
		startaddr /= 512;
		endaddr /= 512;
	}

	if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == cardtype) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == cardtype) || 
		(SDIO_HIGH_CAPACITY_SD_CARD == cardtype)) {
		/* send CMD32(ERASE_WR_BLK_START) to set the address of the first write block to be erased */
		sdio_command_response_config(SD_CMD_ERASE_WR_BLK_START, startaddr, SDIO_RESPONSETYPE_SHORT);
		sdio_wait_type_set(SDIO_WAITTYPE_NO);
		sdio_csm_enable();

		/* r1 error check */
		status = r1_error_check(SD_CMD_ERASE_WR_BLK_START);
		if (SD_OK != status) {
			return status;
		}

		/* send CMD33(ERASE_WR_BLK_END) to set the address of the last write block of the continuous range to be erased */
		sdio_command_response_config(SD_CMD_ERASE_WR_BLK_END, endaddr, SDIO_RESPONSETYPE_SHORT);
		sdio_wait_type_set(SDIO_WAITTYPE_NO);
		sdio_csm_enable();

		/* r1 error check */
		status = r1_error_check(SD_CMD_ERASE_WR_BLK_END);
		if (SD_OK != status) {
			return status;
		}
	}

	/* send CMD38(ERASE) to set the address of the first write block to be erased */
	sdio_command_response_config(SD_CMD_ERASE, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	sdio_csm_enable();

	/* error check */
	status = r1_error_check(SD_CMD_ERASE);
	if (SD_OK != status) {
		return status;
	}

	/* loop until the counter is reach to the calculated time */
	for (count = 0; count < delay; count++) {
	}

	/* get the card state and wait the card is out of programming and receiving state */
	status = sd_card_state_get(&cardstate);
	while ((SD_OK == status) && ((SD_CARDSTATE_PROGRAMMING == cardstate) || (SD_CARDSTATE_RECEIVING == cardstate))) {
		status = sd_card_state_get(&cardstate);
	}

	return status;
}

/*!
	\brief      process all the interrupts when their corresponding flags are set
	\param[in]  none
	\param[out] none
	\retval     sdcard_error_t
*/
sdcard_error_t sd_interrupts_process(void)
{
	transerror = SD_OK;
	if (RESET != sdio_interrupt_flag_get(SDIO_INT_FLAG_DTEND)) {
		/* send CMD12 to stop data transfer in multipule blocks operation */
		if (1 == stopcondition) {
			transerror = sd_transfer_stop();
		} else {
			transerror = SD_OK;
		}
		sdio_interrupt_flag_clear(SDIO_INT_DTEND);

		/* disable all the interrupts */
		sdio_interrupt_disable(SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_DTEND | SDIO_INT_STBITE | 
								SDIO_INT_TFH | SDIO_INT_RFH | SDIO_INT_TXURE | SDIO_INT_RXORE);
		transend = 1;
		number_bytes = 0;
		return transerror;
	}

	if (RESET != sdio_interrupt_flag_get(SDIO_INT_FLAG_DTCRCERR)) {
		sdio_interrupt_flag_clear(SDIO_INT_DTCRCERR);
		/* disable all the interrupts */
		sdio_interrupt_disable(SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_DTEND | SDIO_INT_STBITE | 
								SDIO_INT_TFH | SDIO_INT_RFH | SDIO_INT_TXURE | SDIO_INT_RXORE);
		number_bytes = 0;
		transerror = SD_DATA_CRC_ERROR;
		return transerror;
	}

	if (RESET != sdio_interrupt_flag_get(SDIO_INT_FLAG_DTTMOUT)) {
		sdio_interrupt_flag_clear(SDIO_INT_DTTMOUT);
		/* disable all the interrupts */
		sdio_interrupt_disable(SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_DTEND | SDIO_INT_STBITE | 
								SDIO_INT_TFH | SDIO_INT_RFH | SDIO_INT_TXURE | SDIO_INT_RXORE);
		number_bytes = 0;
		transerror = SD_DATA_TIMEOUT;
		return transerror;
	}

	if (RESET != sdio_interrupt_flag_get(SDIO_INT_FLAG_STBITE)) {
		sdio_interrupt_flag_clear(SDIO_INT_STBITE);
		/* disable all the interrupts */
		sdio_interrupt_disable(SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_DTEND | SDIO_INT_STBITE | 
								SDIO_INT_TFH | SDIO_INT_RFH | SDIO_INT_TXURE | SDIO_INT_RXORE);
		number_bytes = 0;
		transerror = SD_START_BIT_ERROR;
		return transerror;
	}

	if (RESET != sdio_interrupt_flag_get(SDIO_INT_FLAG_TXURE)) {
		sdio_interrupt_flag_clear(SDIO_INT_TXURE);
		/* disable all the interrupts */
		sdio_interrupt_disable(SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_DTEND | SDIO_INT_STBITE | 
								SDIO_INT_TFH | SDIO_INT_RFH | SDIO_INT_TXURE | SDIO_INT_RXORE);
		number_bytes = 0;
		transerror = SD_TX_UNDERRUN_ERROR;
		return transerror;
	}

	if (RESET != sdio_interrupt_flag_get(SDIO_INT_FLAG_RXORE)) {
		sdio_interrupt_flag_clear(SDIO_INT_RXORE);
		/* disable all the interrupts */
		sdio_interrupt_disable(SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_DTEND | SDIO_INT_STBITE | 
								SDIO_INT_TFH | SDIO_INT_RFH | SDIO_INT_TXURE | SDIO_INT_RXORE);
		number_bytes = 0;
		transerror = SD_RX_OVERRUN_ERROR;
		return transerror;
	}

	return transerror;
}

/*!
	\brief      select or deselect a card
	\param[in]  cardrca: the RCA of a card
	\param[out] none
	\retval     sdcard_error_t
*/
sdcard_error_t sd_card_select_deselect(uint16_t cardrca)
{
	sdcard_error_t status = SD_OK;

	/* send CMD7(SELECT/DESELECT_CARD) to select or deselect the card */
	sdio_command_response_config(SD_CMD_SELECT_DESELECT_CARD, (uint32_t)(cardrca << SD_RCA_SHIFT), SDIO_RESPONSETYPE_SHORT);
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	sdio_csm_enable();

	status = r1_error_check(SD_CMD_SELECT_DESELECT_CARD);

	return status;
}

/*!
	\brief      get the card status whose response format R1 contains a 32-bit field
	\param[in]  none
	\param[out] pcardstatus: a pointer that store card status
	\retval     sdcard_error_t
*/
sdcard_error_t sd_cardstatus_get(uint32_t *pcardstatus)
{
	sdcard_error_t status = SD_OK;
	if (NULL == pcardstatus) {
		status = SD_PARAMETER_INVALID;
		return status;
	}

	/* send CMD13(SEND_STATUS), addressed card sends its status register */
	sdio_command_response_config(SD_CMD_SEND_STATUS, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	sdio_csm_enable();

	/* r1 error check */
	status = r1_error_check(SD_CMD_SEND_STATUS);
	if (SD_OK != status) {
		return status;
	}

	*pcardstatus = sdio_response_get(SDIO_RESPONSE0);

	return status;
}

/*!
	\brief      get the SD status, the size of the SD status is one data block of 512 bit
	\param[in]  none
	\param[out] psdstatus: a pointer that store SD card status
	\retval     sdcard_error_t
*/
sdcard_error_t sd_sdstatus_get(uint32_t *psdstatus)
{
	sdcard_error_t status = SD_OK;
	uint32_t count = 0;

	/* check whether the card is locked */
	if (sdio_response_get(SDIO_RESPONSE0) & SD_CARDSTATE_LOCKED) {
		status = SD_LOCK_UNLOCK_FAILED;
		return(status);
	}

	/* send CMD16(SET_BLOCKLEN) to set the block length */
	sdio_command_response_config(SD_CMD_SET_BLOCKLEN, (uint32_t)64, SDIO_RESPONSETYPE_SHORT);
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	sdio_csm_enable();

	/* r12 error check */
	status = r1_error_check(SD_CMD_SET_BLOCKLEN);
	if (SD_OK != status) {
		return status;
	}

	/* send CMD55(APP_CMD) to indicate next command is application specific command */
	sdio_command_response_config(SD_CMD_APP_CMD, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	sdio_csm_enable();

	/* r1 error check */
	status = r1_error_check(SD_CMD_APP_CMD);
	if (SD_OK != status) {
		return status;
	}

	/* configure the SDIO data transmission */
	sdio_data_config(SD_DATATIMEOUT, (uint32_t)64, SDIO_DATABLOCKSIZE_64BYTES);
	sdio_data_transfer_config(SDIO_TRANSMODE_BLOCK, SDIO_TRANSDIRECTION_TOSDIO);
	sdio_dsm_enable();

	/* send ACMD13(SD_STATUS) to get the SD status */
	sdio_command_response_config(SD_ACMD_SD_STATUS, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	sdio_csm_enable();

	/* r1 error check */
	status = r1_error_check(SD_ACMD_SD_STATUS);
	if (SD_OK != status) {
		return status;
	}

	while (!sdio_flag_get(SDIO_FLAG_DTCRCERR | SDIO_FLAG_DTTMOUT | SDIO_FLAG_RXORE | SDIO_FLAG_DTBLKEND | SDIO_FLAG_STBITE)) {
		if (RESET != sdio_flag_get(SDIO_FLAG_RFH)) {
			for (count = 0; count < SD_FIFOHALF_WORDS; count++) {
				*(psdstatus + count) = sdio_data_read();
			}
			psdstatus += SD_FIFOHALF_WORDS;
		}
	}

	/* error check */
	if (RESET != sdio_flag_get(SDIO_FLAG_DTCRCERR)) {
		status = SD_DATA_CRC_ERROR;
		sdio_flag_clear(SDIO_FLAG_DTCRCERR);
		return status;
	} else if (RESET != sdio_flag_get(SDIO_FLAG_DTTMOUT)) {
		status = SD_DATA_TIMEOUT;
		sdio_flag_clear(SDIO_FLAG_DTTMOUT);
		return status;
	} else if (RESET != sdio_flag_get(SDIO_FLAG_RXORE)) {
		status = SD_RX_OVERRUN_ERROR;
		sdio_flag_clear(SDIO_FLAG_RXORE);
		return status;
	} else if (RESET != sdio_flag_get(SDIO_FLAG_STBITE)) {
		status = SD_START_BIT_ERROR;
		sdio_flag_clear(SDIO_FLAG_STBITE);
		return status;
	}
	while (RESET != sdio_flag_get(SDIO_FLAG_RXDTVAL)) {
		*psdstatus = sdio_data_read();
		++psdstatus;
	}

	/* clear the SDIO_INTC flags */
	sdio_flag_clear(SDIO_MASK_INTC_FLAGS);
	psdstatus -= 16;
	for (count = 0; count < 16; count++) {
		psdstatus[count] = ((psdstatus[count] & SD_MASK_0_7BITS) << 24) |((psdstatus[count] & SD_MASK_8_15BITS) << 8) | 
						   ((psdstatus[count] & SD_MASK_16_23BITS) >> 8) |((psdstatus[count] & SD_MASK_24_31BITS) >> 24);
	}

	return status;
}

/*!
	\brief      stop an ongoing data transfer
	\param[in]  none
	\param[out] none
	\retval     sdcard_error_t
*/
sdcard_error_t sd_transfer_stop(void)
{
	sdcard_error_t status = SD_OK;

	/* send CMD12(STOP_TRANSMISSION) to stop transmission */
	sdio_command_response_config(SD_CMD_STOP_TRANSMISSION, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	sdio_csm_enable();

	/* r1 error check */
	status = r1_error_check(SD_CMD_STOP_TRANSMISSION);

	return status;
}

/*!
	\brief      lock or unlock a card
	\param[in]  lockstate: the lock state
	  \arg        SD_LOCK: lock the SD card
	  \arg        SD_UNLOCK: unlock the SD card
	\param[out] none
	\retval     sdcard_error_t
*/
sdcard_error_t sd_lock_unlock(uint8_t lockstate)
{
	sdcard_error_t status = SD_OK;
	uint8_t cardstate = 0, tempbyte = 0;
	uint32_t pwd1 = 0, pwd2 = 0, response = 0;
	__IO uint32_t timeout = 0;
	uint16_t tempccc = 0;

	/* get the card command classes from CSD */
	tempbyte = (uint8_t)((sd_csd[1] & SD_MASK_24_31BITS) >> 24);
	tempccc = (uint16_t)((uint16_t)tempbyte << 4);
	tempbyte = (uint8_t)((sd_csd[1] & SD_MASK_16_23BITS) >> 16);
	tempccc |= (uint16_t)((uint16_t)(tempbyte & 0xF0) >> 4);

	if (0 == (tempccc & SD_CCC_LOCK_CARD)) {
		/* don't support the lock command */
		status = SD_FUNCTION_UNSUPPORTED;
		return status;
	}

	/* password pattern */
	pwd1 = (0x01020600|lockstate);
	pwd2 = 0x03040506;

	/* clear all DSM configuration */
	sdio_data_config(0, 0, SDIO_DATABLOCKSIZE_1BYTE);
	sdio_data_transfer_config(SDIO_TRANSMODE_BLOCK, SDIO_TRANSDIRECTION_TOCARD);
	sdio_dsm_disable();
	sdio_dma_disable();

	/* send CMD16(SET_BLOCKLEN) to set the block length */
	sdio_command_response_config(SD_CMD_SET_BLOCKLEN, (uint32_t)8, SDIO_RESPONSETYPE_SHORT);
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	sdio_csm_enable();

	/* r1 error check */
	status = r1_error_check(SD_CMD_SET_BLOCKLEN);
	if (SD_OK != status) {
		return status;
	}

	/* send CMD13(SEND_STATUS), addressed card sends its status register */
	sdio_command_response_config(SD_CMD_SEND_STATUS, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	sdio_csm_enable();

	/* r1 error check */
	status = r1_error_check(SD_CMD_SEND_STATUS);
	if (SD_OK != status) {
		return status;
	}

	response = sdio_response_get(SDIO_RESPONSE0);
	timeout = 100000;
	while ((0 == (response & SD_R1_READY_FOR_DATA)) && (timeout > 0)) {
		/* continue to send CMD13 to polling the state of card until buffer empty or timeout */
		--timeout;
		/* send CMD13(SEND_STATUS), addressed card sends its status registers */
		sdio_command_response_config(SD_CMD_SEND_STATUS, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
		sdio_wait_type_set(SDIO_WAITTYPE_NO);
		sdio_csm_enable();

		/* r1 error check */
		status = r1_error_check(SD_CMD_SEND_STATUS);
		if (SD_OK != status) {
			return status;
		}
		response = sdio_response_get(SDIO_RESPONSE0);
	}
	if (0 == timeout) {
		return SD_ERROR;
	}

	/* send CMD42(LOCK_UNLOCK) to set/reset the password or lock/unlock the card */
	sdio_command_response_config(SD_CMD_LOCK_UNLOCK, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	sdio_csm_enable();

	/* r1 error check */
	status = r1_error_check(SD_CMD_LOCK_UNLOCK);
	if (SD_OK != status) {
		return status;
	}

	response = sdio_response_get(SDIO_RESPONSE0);

	/* configure the SDIO data transmission */
	sdio_data_config(SD_DATATIMEOUT, (uint32_t)8, SDIO_DATABLOCKSIZE_8BYTES);
	sdio_data_transfer_config(SDIO_TRANSMODE_BLOCK, SDIO_TRANSDIRECTION_TOCARD);
	sdio_dsm_enable();

	/* write password pattern */
	sdio_data_write(pwd1);
	sdio_data_write(pwd2);

	/* error check */
	if (RESET != sdio_flag_get(SDIO_FLAG_DTCRCERR)) {
		status = SD_DATA_CRC_ERROR;
		sdio_flag_clear(SDIO_FLAG_DTCRCERR);
		return status;
	} else if (RESET != sdio_flag_get(SDIO_FLAG_DTTMOUT)) {
		status = SD_DATA_TIMEOUT;
		sdio_flag_clear(SDIO_FLAG_DTTMOUT);
		return status;
	} else if (RESET != sdio_flag_get(SDIO_FLAG_TXURE)) {
		status = SD_TX_UNDERRUN_ERROR;
		sdio_flag_clear(SDIO_FLAG_TXURE);
		return status;
	} else if (RESET != sdio_flag_get(SDIO_FLAG_STBITE)) {
		status = SD_START_BIT_ERROR;
		sdio_flag_clear(SDIO_FLAG_STBITE);
		return status;
	}

	/* clear the SDIO_INTC flags */
	sdio_flag_clear(SDIO_MASK_INTC_FLAGS);
	/* get the card state and wait the card is out of programming and receiving state */
	status = sd_card_state_get(&cardstate);
	while ((SD_OK == status) && ((SD_CARDSTATE_PROGRAMMING == cardstate) || (SD_CARDSTATE_RECEIVING == cardstate))) {
		status = sd_card_state_get(&cardstate);
	}

	return status;
}

/*!
	\brief      get the data transfer state
	\param[in]  none
	\param[out] none
	\retval     sdcard_error_t
*/
sd_transfer_state_enum sd_transfer_state_get(void)
{
	sd_transfer_state_enum transtate = SD_NO_TRANSFER;
	if (RESET != sdio_flag_get(SDIO_FLAG_TXRUN | SDIO_FLAG_RXRUN)) {
		transtate = SD_TRANSFER_IN_PROGRESS;
	}

	return transtate;
}

/*!
	\brief      get SD card capacity
	\param[in]  none
	\param[out] none
	\retval     capacity of the card(KB)
*/
uint32_t sd_card_capacity_get(void)
{
	uint8_t tempbyte = 0, devicesize_mult = 0, readblklen = 0;
	uint32_t capacity = 0, devicesize = 0;

	if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == cardtype) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == cardtype)) {
		/* calculate the c_size(device size) */
		tempbyte = (uint8_t)((sd_csd[1] & SD_MASK_8_15BITS) >> 8);
		devicesize |= (uint32_t)((uint32_t)(tempbyte & 0x03) << 10);
		tempbyte = (uint8_t)(sd_csd[1] & SD_MASK_0_7BITS);
		devicesize |= (uint32_t)((uint32_t)tempbyte << 2);
		tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_24_31BITS) >> 24);
		devicesize |= (uint32_t)((uint32_t)(tempbyte & 0xC0) >> 6);

		/* calculate the c_size_mult(device size multiplier) */
		tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_16_23BITS) >> 16);
		devicesize_mult = (tempbyte & 0x03) << 1;
		tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_8_15BITS) >> 8);
		devicesize_mult |= (tempbyte & 0x80) >> 7;

		/* calculate the read_bl_len */
		tempbyte = (uint8_t)((sd_csd[1] & SD_MASK_16_23BITS) >> 16);
		readblklen = tempbyte & 0x0F;

		/* capacity = BLOCKNR*BLOCK_LEN, BLOCKNR = (C_SIZE+1)*MULT, MULT = 2^(C_SIZE_MULT+2), BLOCK_LEN = 2^READ_BL_LEN */
		capacity = (devicesize + 1)*(1 << (devicesize_mult + 2));
		capacity *= (1 << readblklen);

		/* change the unit of capacity to KByte */
		capacity /= 1024;
	} else if (SDIO_HIGH_CAPACITY_SD_CARD == cardtype) {
		/* calculate the c_size */
		tempbyte = (uint8_t)(sd_csd[1] & SD_MASK_0_7BITS);
		devicesize = (uint32_t)((uint32_t)(tempbyte & 0x3F) << 16);
		tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_24_31BITS) >> 24);
		devicesize |= (uint32_t)((uint32_t)tempbyte << 8);
		tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_16_23BITS) >> 16);
		devicesize |= (uint32_t)tempbyte;

		/* capacity = (c_size + 1) * 512KByte */
		capacity = (devicesize + 1)*512;
	}

	return capacity;
}

/*!
	\brief      get the detailed information of the SD card based on received CID and CSD
	\param[in]  none
	\param[out] pcardinfo: a pointer that store the detailed card information
	\retval     sdcard_error_t
*/
sdcard_error_t sd_card_information_get(sd_card_info_struct *pcardinfo)
{
	sdcard_error_t status = SD_OK;
	uint8_t tempbyte = 0;

	if (pcardinfo == NULL) {
		status = SD_PARAMETER_INVALID;
		return status;
	}

	/* store the card type and RCA */
	pcardinfo->card_type = cardtype;
	pcardinfo->card_rca = sd_rca;

	/* CID byte 0 */
	tempbyte = (uint8_t)((sd_cid[0] & SD_MASK_24_31BITS) >> 24);
	pcardinfo->card_cid.mid = tempbyte;

	/* CID byte 1 */
	tempbyte = (uint8_t)((sd_cid[0] & SD_MASK_16_23BITS) >> 16);
	pcardinfo->card_cid.oid = (uint16_t)((uint16_t)tempbyte << 8);

	/* CID byte 2 */
	tempbyte = (uint8_t)((sd_cid[0] & SD_MASK_8_15BITS) >> 8);
	pcardinfo->card_cid.oid |= (uint16_t)tempbyte;

	/* CID byte 3 */
	tempbyte = (uint8_t)(sd_cid[0] & SD_MASK_0_7BITS);
	pcardinfo->card_cid.pnm0 = (uint32_t)((uint32_t)tempbyte << 24);

	/* CID byte 4 */
	tempbyte = (uint8_t)((sd_cid[1] & SD_MASK_24_31BITS) >> 24);
	pcardinfo->card_cid.pnm0 |= (uint32_t)((uint32_t)tempbyte << 16);

	/* CID byte 5 */
	tempbyte = (uint8_t)((sd_cid[1] & SD_MASK_16_23BITS) >> 16);
	pcardinfo->card_cid.pnm0 |= (uint32_t)((uint32_t)tempbyte << 8);

	/* CID byte 6 */
	tempbyte = (uint8_t)((sd_cid[1] & SD_MASK_8_15BITS) >> 8);
	pcardinfo->card_cid.pnm0 |= (uint32_t)(tempbyte);

	/* CID byte 7 */
	tempbyte = (uint8_t)(sd_cid[1] & SD_MASK_0_7BITS);
	pcardinfo->card_cid.pnm1 = tempbyte;

	/* CID byte 8 */
	tempbyte = (uint8_t)((sd_cid[2] & SD_MASK_24_31BITS) >> 24);
	pcardinfo->card_cid.prv = tempbyte;

	/* CID byte 9 */
	tempbyte = (uint8_t)((sd_cid[2] & SD_MASK_16_23BITS) >> 16);
	pcardinfo->card_cid.psn = (uint32_t)((uint32_t)tempbyte << 24);

	/* CID byte 10 */
	tempbyte = (uint8_t)((sd_cid[2] & SD_MASK_8_15BITS) >> 8);
	pcardinfo->card_cid.psn |= (uint32_t)((uint32_t)tempbyte << 16);

	/* CID byte 11 */
	tempbyte = (uint8_t)(sd_cid[2] & SD_MASK_0_7BITS);
	pcardinfo->card_cid.psn |= (uint32_t)tempbyte;

	/* CID byte 12 */
	tempbyte = (uint8_t)((sd_cid[3] & SD_MASK_24_31BITS) >> 24);
	pcardinfo->card_cid.psn |= (uint32_t)tempbyte;

	/* CID byte 13 */
	tempbyte = (uint8_t)((sd_cid[3] & SD_MASK_16_23BITS) >> 16);
	pcardinfo->card_cid.mdt = (uint16_t)((uint16_t)(tempbyte & 0x0F) << 8);

	/* CID byte 14 */
	tempbyte = (uint8_t)((sd_cid[3] & SD_MASK_8_15BITS) >> 8);
	pcardinfo->card_cid.mdt |= (uint16_t)tempbyte;

	/* CID byte 15 */
	tempbyte = (uint8_t)(sd_cid[3] & SD_MASK_0_7BITS);
	pcardinfo->card_cid.cid_crc = (tempbyte & 0xFE) >> 1;

	/* CSD byte 0 */
	tempbyte = (uint8_t)((sd_csd[0] & SD_MASK_24_31BITS) >> 24);
	pcardinfo->card_csd.csd_struct = (tempbyte & 0xC0) >> 6;

	/* CSD byte 1 */
	tempbyte = (uint8_t)((sd_csd[0] & SD_MASK_16_23BITS) >> 16);
	pcardinfo->card_csd.taac = tempbyte;

	/* CSD byte 2 */
	tempbyte = (uint8_t)((sd_csd[0] & SD_MASK_8_15BITS) >> 8);
	pcardinfo->card_csd.nsac = tempbyte;

	/* CSD byte 3 */
	tempbyte = (uint8_t)(sd_csd[0] & SD_MASK_0_7BITS);
	pcardinfo->card_csd.tran_speed = tempbyte;

	/* CSD byte 4 */
	tempbyte = (uint8_t)((sd_csd[1] & SD_MASK_24_31BITS) >> 24);
	pcardinfo->card_csd.ccc = (uint16_t)((uint16_t)tempbyte << 4);

	/* CSD byte 5 */
	tempbyte = (uint8_t)((sd_csd[1] & SD_MASK_16_23BITS) >> 16);
	pcardinfo->card_csd.ccc |= (uint16_t)((uint16_t)(tempbyte & 0xF0) >> 4);
	pcardinfo->card_csd.read_bl_len = tempbyte & 0x0F;

	/* CSD byte 6 */
	tempbyte = (uint8_t)((sd_csd[1] & SD_MASK_8_15BITS) >> 8);
	pcardinfo->card_csd.read_bl_partial = (tempbyte & 0x80) >> 7;
	pcardinfo->card_csd.write_blk_misalign = (tempbyte & 0x40) >> 6;
	pcardinfo->card_csd.read_blk_misalign = (tempbyte & 0x20) >> 5;
	pcardinfo->card_csd.dsp_imp = (tempbyte & 0x10) >> 4;

	if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == cardtype) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == cardtype)) {
		/* card is SDSC card, CSD version 1.0 */
		pcardinfo->card_csd.c_size = (uint32_t)((uint32_t)(tempbyte & 0x03) << 10);

		/* CSD byte 7 */
		tempbyte = (uint8_t)(sd_csd[1] & SD_MASK_0_7BITS);
		pcardinfo->card_csd.c_size |= (uint32_t)((uint32_t)tempbyte << 2);

		/* CSD byte 8 */
		tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_24_31BITS) >> 24);
		pcardinfo->card_csd.c_size |= (uint32_t)((uint32_t)(tempbyte & 0xC0) >> 6);
		pcardinfo->card_csd.vdd_r_curr_min = (tempbyte & 0x38) >> 3;
		pcardinfo->card_csd.vdd_r_curr_max = tempbyte & 0x07;

		/* CSD byte 9 */
		tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_16_23BITS) >> 16);
		pcardinfo->card_csd.vdd_w_curr_min = (tempbyte & 0xE0) >> 5;
		pcardinfo->card_csd.vdd_w_curr_max = (tempbyte & 0x1C) >> 2;
		pcardinfo->card_csd.c_size_mult = (tempbyte & 0x03) << 1;

		/* CSD byte 10 */
		tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_8_15BITS) >> 8);
		pcardinfo->card_csd.c_size_mult |= (tempbyte & 0x80) >> 7;

		/* calculate the card block size and capacity */
		pcardinfo->card_blocksize = 1 << (pcardinfo->card_csd.read_bl_len);
		pcardinfo->card_capacity = pcardinfo->card_csd.c_size + 1;
		pcardinfo->card_capacity *= (1 << (pcardinfo->card_csd.c_size_mult + 2));
		pcardinfo->card_capacity *= pcardinfo->card_blocksize;
	} else if (SDIO_HIGH_CAPACITY_SD_CARD == cardtype) {
		/* card is SDHC card, CSD version 2.0 */
		/* CSD byte 7 */
		tempbyte = (uint8_t)(sd_csd[1] & SD_MASK_0_7BITS);
		pcardinfo->card_csd.c_size = (uint32_t)((uint32_t)(tempbyte & 0x3F) << 16);

		/* CSD byte 8 */
		tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_24_31BITS) >> 24);
		pcardinfo->card_csd.c_size |= (uint32_t)((uint32_t)tempbyte << 8);

		/* CSD byte 9 */
		tempbyte = (uint8_t)((sd_csd[2] & SD_MASK_16_23BITS) >> 16);
		pcardinfo->card_csd.c_size |= (uint32_t)tempbyte;

		/* calculate the card block size and capacity */
		pcardinfo->card_blocksize = 512;
		pcardinfo->card_capacity = (pcardinfo->card_csd.c_size + 1) * 512 *1024;
	}

	pcardinfo->card_csd.erase_blk_en = (tempbyte & 0x40) >> 6;
	pcardinfo->card_csd.sector_size = (tempbyte & 0x3F) << 1;

	/* CSD byte 11 */
	tempbyte = (uint8_t)(sd_csd[2] & SD_MASK_0_7BITS);
	pcardinfo->card_csd.sector_size |= (tempbyte & 0x80) >> 7;
	pcardinfo->card_csd.wp_grp_size = (tempbyte & 0x7F);

	/* CSD byte 12 */
	tempbyte = (uint8_t)((sd_csd[3] & SD_MASK_24_31BITS) >> 24);
	pcardinfo->card_csd.wp_grp_enable = (tempbyte & 0x80) >> 7;
	pcardinfo->card_csd.r2w_factor = (tempbyte & 0x1C) >> 2;
	pcardinfo->card_csd.write_bl_len = (tempbyte & 0x03) << 2;

	/* CSD byte 13 */
	tempbyte = (uint8_t)((sd_csd[3] & SD_MASK_16_23BITS) >> 16);
	pcardinfo->card_csd.write_bl_len |= (tempbyte & 0xC0) >> 6;
	pcardinfo->card_csd.write_bl_partial = (tempbyte & 0x20) >> 5;

	/* CSD byte 14 */
	tempbyte = (uint8_t)((sd_csd[3] & SD_MASK_8_15BITS) >> 8);
	pcardinfo->card_csd.file_format_grp = (tempbyte & 0x80) >> 7;
	pcardinfo->card_csd.copy_flag = (tempbyte & 0x40) >> 6;
	pcardinfo->card_csd.perm_write_protect = (tempbyte & 0x20) >> 5;
	pcardinfo->card_csd.tmp_write_protect = (tempbyte & 0x10) >> 4;
	pcardinfo->card_csd.file_format = (tempbyte & 0x0C) >> 2;

	/* CSD byte 15 */
	tempbyte = (uint8_t)(sd_csd[3] & SD_MASK_0_7BITS);
	pcardinfo->card_csd.csd_crc = (tempbyte & 0xFE) >> 1;

	return status;
}

/*!
	\brief      check if the command sent error occurs
	\param[in]  none
	\param[out] none
	\retval     sdcard_error_t
*/
static sdcard_error_t cmdsent_error_check(void)
{
	sdcard_error_t status = SD_OK;
	__IO uint32_t timeout = 100000;
	/* check command sent flag */
	while ((RESET == sdio_flag_get(SDIO_FLAG_CMDSEND)) && (timeout > 0)) {
		--timeout;
	}

	/* command response is timeout */
	if (0 == timeout) {
		status = SD_CMD_RESP_TIMEOUT;
		return status;
	}
	/* if the command is sent, clear the SDIO_INTC flags */
	sdio_flag_clear(SDIO_MASK_INTC_FLAGS);

	return status;
}

/*!
	\brief      check if error type for R1 response
	\param[in]  resp: content of response
	\param[out] none
	\retval     sdcard_error_t
*/
static sdcard_error_t r1_error_type_check(uint32_t resp)
{
	sdcard_error_t status = SD_ERROR;

	/* check which error occurs */
	if (resp & SD_R1_OUT_OF_RANGE) {
		status = SD_OUT_OF_RANGE;
	} else if (resp & SD_R1_ADDRESS_ERROR) {
		status = SD_ADDRESS_ERROR;
	} else if (resp & SD_R1_BLOCK_LEN_ERROR) {
		status = SD_BLOCK_LEN_ERROR;
	} else if (resp & SD_R1_ERASE_SEQ_ERROR) {
		status = SD_ERASE_SEQ_ERROR;
	} else if (resp & SD_R1_ERASE_PARAM) {
		status = SD_ERASE_PARAM;
	} else if (resp & SD_R1_WP_VIOLATION) {
		status = SD_WP_VIOLATION;
	} else if (resp & SD_R1_LOCK_UNLOCK_FAILED) {
		status = SD_LOCK_UNLOCK_FAILED;
	} else if (resp & SD_R1_COM_CRC_ERROR) {
		status = SD_COM_CRC_ERROR;
	} else if (resp & SD_R1_ILLEGAL_COMMAND) {
		status = SD_ILLEGAL_COMMAND;
	} else if (resp & SD_R1_CARD_ECC_FAILED) {
		status = SD_CARD_ECC_FAILED;
	} else if (resp & SD_R1_CC_ERROR) {
		status = SD_CC_ERROR;
	} else if (resp & SD_R1_GENERAL_UNKNOWN_ERROR) {
		status = SD_GENERAL_UNKNOWN_ERROR;
	} else if (resp & SD_R1_CSD_OVERWRITE) {
		status = SD_CSD_OVERWRITE;
	} else if (resp & SD_R1_WP_ERASE_SKIP) {
		status = SD_WP_ERASE_SKIP;
	} else if (resp & SD_R1_CARD_ECC_DISABLED) {
		status = SD_CARD_ECC_DISABLED;
	} else if (resp & SD_R1_ERASE_RESET) {
		status = SD_ERASE_RESET;
	} else if (resp & SD_R1_AKE_SEQ_ERROR) {
		status = SD_AKE_SEQ_ERROR;
	}

	return status;
}

/*!
	\brief      check if error occurs for R1 response
	\param[in]  cmdindex: the index of command
	\param[out] none
	\retval     sdcard_error_t
*/
static sdcard_error_t r1_error_check(uint8_t cmdindex)
{
	sdcard_error_t status = SD_OK;
	uint32_t reg_status = 0, resp_r1 = 0;

	/* store the content of SDIO_STAT */
	reg_status = SDIO_STAT;
	while (!(reg_status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDTMOUT | SDIO_FLAG_CMDRECV))) {
		reg_status = SDIO_STAT;
	}

	/* check whether an error or timeout occurs or command response received */
	if (reg_status & SDIO_FLAG_CCRCERR) {
		status = SD_CMD_CRC_ERROR;
		sdio_flag_clear(SDIO_FLAG_CCRCERR);
		return status;
	} else if (reg_status & SDIO_FLAG_CMDTMOUT) {
		status = SD_CMD_RESP_TIMEOUT;
		sdio_flag_clear(SDIO_FLAG_CMDTMOUT);
		return status;
	}

	/* check whether the last response command index is the desired one */
	if (sdio_command_index_get() != cmdindex) {
		status = SD_ILLEGAL_COMMAND;
		return status;
	}

	/* clear all the SDIO_INTC flags */
	sdio_flag_clear(SDIO_MASK_INTC_FLAGS);
	/* get the SDIO response register 0 for checking */
	resp_r1 = sdio_response_get(SDIO_RESPONSE0);
	if (SD_ALLZERO == (resp_r1 & SD_R1_ERROR_BITS)) {
		/* no error occurs, return SD_OK */
		status = SD_OK;
		return status;
	}

	/* r1 error check */
	status = r1_error_type_check(resp_r1);

	return status;
}

/*!
	\brief      check if error occurs for R2 response
	\param[in]  none
	\param[out] none
	\retval     sdcard_error_t
*/
static sdcard_error_t r2_error_check(void)
{
	sdcard_error_t status = SD_OK;
	uint32_t reg_status = 0;

	/* store the content of SDIO_STAT */
	reg_status = SDIO_STAT;
	while (!(reg_status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDTMOUT | SDIO_FLAG_CMDRECV))) {
		reg_status = SDIO_STAT;
	}

	/* check whether an error or timeout occurs or command response received */
	if (reg_status & SDIO_FLAG_CCRCERR) {
		status = SD_CMD_CRC_ERROR;
		sdio_flag_clear(SDIO_FLAG_CCRCERR);
		return status;
	} else if (reg_status & SDIO_FLAG_CMDTMOUT) {
		status = SD_CMD_RESP_TIMEOUT;
		sdio_flag_clear(SDIO_FLAG_CMDTMOUT);
		return status;
	}

	/* clear all the SDIO_INTC flags */
	sdio_flag_clear(SDIO_MASK_INTC_FLAGS);

	return status;
}

/*!
	\brief      check if error occurs for R3 response
	\param[in]  none
	\param[out] none
	\retval     sdcard_error_t
*/
static sdcard_error_t r3_error_check(void)
{
	sdcard_error_t status = SD_OK;
	uint32_t reg_status = 0;

	/* store the content of SDIO_STAT */
	reg_status = SDIO_STAT;
	while (!(reg_status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDTMOUT | SDIO_FLAG_CMDRECV))) {
		reg_status = SDIO_STAT;
	}

	if (reg_status & SDIO_FLAG_CMDTMOUT) {
		status = SD_CMD_RESP_TIMEOUT;
		sdio_flag_clear(SDIO_FLAG_CMDTMOUT);
		return status;
	}

	/* clear all the SDIO_INTC flags */
	sdio_flag_clear(SDIO_MASK_INTC_FLAGS);

	return status;
}

/*!
	\brief      check if error occurs for R6 response
	\param[in]  cmdindex: the index of command
	\param[out] prca: a pointer that store the RCA of card
	\retval     sdcard_error_t
*/
static sdcard_error_t r6_error_check(uint8_t cmdindex, uint16_t *prca)
{
	sdcard_error_t status = SD_OK;
	uint32_t reg_status = 0, response = 0;

	/* store the content of SDIO_STAT */
	reg_status = SDIO_STAT;
	while (!(reg_status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDTMOUT | SDIO_FLAG_CMDRECV))) {
		reg_status = SDIO_STAT;
	}

	/* check whether an error or timeout occurs or command response received */
	if (reg_status & SDIO_FLAG_CCRCERR) {
		status = SD_CMD_CRC_ERROR;
		sdio_flag_clear(SDIO_FLAG_CCRCERR);
		return status;
	} else if (reg_status & SDIO_FLAG_CMDTMOUT) {
		status = SD_CMD_RESP_TIMEOUT;
		sdio_flag_clear(SDIO_FLAG_CMDTMOUT);
		return status;
	}

	/* check whether the last response command index is the desired one */
	if (sdio_command_index_get() != cmdindex) {
		status = SD_ILLEGAL_COMMAND;
		return status;
	}

	/* clear all the SDIO_INTC flags */
	sdio_flag_clear(SDIO_MASK_INTC_FLAGS);

	/* get the SDIO response register 0 for checking */
	response = sdio_response_get(SDIO_RESPONSE0);

	if (SD_ALLZERO == (response & (SD_R6_COM_CRC_ERROR | SD_R6_ILLEGAL_COMMAND | SD_R6_ERROR))) {
		*prca = (uint16_t)(response >> 16);
		return status;
	}

	/* if some error occurs, return the error type */
	if (response & SD_R6_COM_CRC_ERROR) {
		status = SD_COM_CRC_ERROR;
	} else if (response & SD_R6_ILLEGAL_COMMAND) {
		status = SD_ILLEGAL_COMMAND;
	} else if (response & SD_R6_ERROR) {
		status = SD_GENERAL_UNKNOWN_ERROR;
	}

	return status;
}

/*!
	\brief      check if error occurs for R7 response
	\param[in]  none
	\param[out] none
	\retval     sdcard_error_t
*/
static sdcard_error_t r7_error_check(void)
{
	sdcard_error_t status = SD_ERROR;
	uint32_t reg_status = 0;
	__IO uint32_t timeout = 100000;

	/* store the content of SDIO_STAT */
	reg_status = SDIO_STAT;
	while (!(reg_status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDTMOUT | SDIO_FLAG_CMDRECV)) && (timeout > 0)) {
		reg_status = SDIO_STAT;
		--timeout;
	}

	/* check the flags */
	if ((reg_status & SDIO_FLAG_CMDTMOUT) || (0 == timeout)) {
		status = SD_CMD_RESP_TIMEOUT;
		sdio_flag_clear(SDIO_FLAG_CMDTMOUT);
		return status;
	}

	if (reg_status & SDIO_FLAG_CMDRECV) {
		status = SD_OK;
		sdio_flag_clear(SDIO_FLAG_CMDRECV);
		return status;
	}

	return status;
}

/*!
	\brief      get the state which the card is in
	\param[in]  none
	\param[out] pcardstate: a pointer that store the card state
	  \arg        SD_CARDSTATE_IDLE: card is in idle state
	  \arg        SD_CARDSTATE_READY: card is in ready state
	  \arg        SD_CARDSTATE_IDENTIFICAT: card is in identificat state
	  \arg        SD_CARDSTATE_STANDBY: card is in standby state
	  \arg        SD_CARDSTATE_TRANSFER: card is in transfer state
	  \arg        SD_CARDSTATE_DATA: card is in data state
	  \arg        SD_CARDSTATE_RECEIVING: card is in receiving state
	  \arg        SD_CARDSTATE_PROGRAMMING: card is in programming state
	  \arg        SD_CARDSTATE_DISCONNECT: card is in disconnect state
	  \arg        SD_CARDSTATE_LOCKED: card is in locked state
	\retval     sdcard_error_t
*/
sdcard_error_t sd_card_state_get(uint8_t *pcardstate)
{
	sdcard_error_t status = SD_OK;
	__IO uint32_t reg_status = 0, response = 0;

	/* send CMD13(SEND_STATUS), addressed card sends its status register */
	sdio_command_response_config(SD_CMD_SEND_STATUS, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	sdio_csm_enable();

	/* store the content of SDIO_STAT */
	reg_status = SDIO_STAT;
	while (!(reg_status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDTMOUT | SDIO_FLAG_CMDRECV))) {
		reg_status = SDIO_STAT;
	}

	/* check whether an error or timeout occurs or command response received */
	if (reg_status & SDIO_FLAG_CCRCERR) {
		status = SD_CMD_CRC_ERROR;
		sdio_flag_clear(SDIO_FLAG_CCRCERR);
		return status;
	} else if (reg_status & SDIO_FLAG_CMDTMOUT) {
		status = SD_CMD_RESP_TIMEOUT;
		sdio_flag_clear(SDIO_FLAG_CMDTMOUT);
		return status;
	}

	/* command response received, store the response command index */
	reg_status = (uint32_t)sdio_command_index_get();
	if (reg_status != (uint32_t)SD_CMD_SEND_STATUS) {
		status = SD_ILLEGAL_COMMAND;
		return status;
	}

	/* clear all the SDIO_INTC flags */
	sdio_flag_clear(SDIO_MASK_INTC_FLAGS);

	/* get the SDIO response register 0 for checking */
	response = sdio_response_get(SDIO_RESPONSE0);
	*pcardstate = (uint8_t)((response >> 9) & 0x0000000F);

	if (SD_ALLZERO == (response & SD_R1_ERROR_BITS)) {
		/* no error occurs, return SD_OK */
		status = SD_OK;
		return status;
	}

	/* if some error occurs, return the error type */
	status = r1_error_type_check(response);

	return status;
}

/*!
	\brief      configure the bus width mode
	\param[in]  buswidth: the bus width
	  \arg        SD_BUS_WIDTH_1BIT: 1-bit bus width
	  \arg        SD_BUS_WIDTH_4BIT: 4-bit bus width
	\param[out] none
	\retval     sdcard_error_t
*/
static sdcard_error_t sd_bus_width_config(uint32_t buswidth)
{
	sdcard_error_t status = SD_OK;

	/* check whether the card is locked */
	if (sdio_response_get(SDIO_RESPONSE0) & SD_CARDSTATE_LOCKED) {
		status = SD_LOCK_UNLOCK_FAILED;
		return status;
	}

	/* get the SCR register */
	status = sd_scr_get(sd_rca, sd_scr);
	if (SD_OK != status) {
		return status;
	}

	if (SD_BUS_WIDTH_1BIT == buswidth) {
		if (SD_ALLZERO != (sd_scr[1] & buswidth)) {
			/* send CMD55(APP_CMD) to indicate next command is application specific command */
			sdio_command_response_config(SD_CMD_APP_CMD, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
			sdio_wait_type_set(SDIO_WAITTYPE_NO);
			sdio_csm_enable();
			/* check if some error occurs */
			status = r1_error_check(SD_CMD_APP_CMD);
			if (SD_OK != status) {
				return status;
			}

			/* send ACMD6(SET_BUS_WIDTH) to define the data bus width */
			sdio_command_response_config(SD_ACMD_SET_BUS_WIDTH, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
			sdio_wait_type_set(SDIO_WAITTYPE_NO);
			sdio_csm_enable();

			/* r1 error check */
			status = r1_error_check(SD_ACMD_SET_BUS_WIDTH);
			if (SD_OK != status) {
				return status;
			}
		} else {
			status = SD_OPERATION_IMPROPER;
		}

		return status;

	} else if (SD_BUS_WIDTH_4BIT == buswidth) {
		if (SD_ALLZERO != (sd_scr[1] & buswidth)) {
			/* send CMD55(APP_CMD) to indicate next command is application specific command */
			sdio_command_response_config(SD_CMD_APP_CMD, (uint32_t)sd_rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
			sdio_wait_type_set(SDIO_WAITTYPE_NO);
			sdio_csm_enable();

			/* r1 error check */
			status = r1_error_check(SD_CMD_APP_CMD);
			if (SD_OK != status) {
				return status;
			}

			/* send ACMD6(SET_BUS_WIDTH) to define the data bus width */
			sdio_command_response_config(SD_ACMD_SET_BUS_WIDTH, (uint32_t)0x2, SDIO_RESPONSETYPE_SHORT);
			sdio_wait_type_set(SDIO_WAITTYPE_NO);
			sdio_csm_enable();

			/* r1 error check */
			status = r1_error_check(SD_ACMD_SET_BUS_WIDTH);
			if (SD_OK != status) {
				return status;
			}
		} else {
			status = SD_OPERATION_IMPROPER;
		}

		return status;
	} else {
		status = SD_PARAMETER_INVALID;

		return status;
	}
}

/*!
	\brief      get the SCR of corresponding card
	\param[in]  rca: RCA of a card
	\param[out] pscr: a pointer that store the SCR content
	\retval     sdcard_error_t
*/
static sdcard_error_t sd_scr_get(uint16_t rca, uint32_t *pscr)
{
	sdcard_error_t status = SD_OK;
	uint32_t temp_scr[2] = {0, 0}, idx_scr = 0;

	/* send CMD16(SET_BLOCKLEN) to set block length */
	sdio_command_response_config(SD_CMD_SET_BLOCKLEN, (uint32_t)8, SDIO_RESPONSETYPE_SHORT);
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	sdio_csm_enable();

	/* r1 error check */
	status = r1_error_check(SD_CMD_SET_BLOCKLEN);
	if (SD_OK != status) {
		return status;
	}

	/* send CMD55(APP_CMD) to indicate next command is application specific command */
	sdio_command_response_config(SD_CMD_APP_CMD, (uint32_t)rca << SD_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	sdio_csm_enable();

	/* r1 error check */
	status = r1_error_check(SD_CMD_APP_CMD);
	if (SD_OK != status) {
		return status;
	}

	/* configure SDIO data */
	sdio_data_config(SD_DATATIMEOUT, (uint32_t)8, SDIO_DATABLOCKSIZE_8BYTES);
	sdio_data_transfer_config(SDIO_TRANSMODE_BLOCK, SDIO_TRANSDIRECTION_TOSDIO);
	sdio_dsm_enable();

	/* send ACMD51(SEND_SCR) to read the SD configuration register */
	sdio_command_response_config(SD_ACMD_SEND_SCR, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	sdio_csm_enable();

  /* error check */
	status = r1_error_check(SD_ACMD_SEND_SCR);
	if (SD_OK != status) {
		return status;
	}

	/* store the received SCR */
	while (!sdio_flag_get(SDIO_FLAG_DTCRCERR | SDIO_FLAG_DTTMOUT | SDIO_FLAG_RXORE | SDIO_FLAG_DTBLKEND | SDIO_FLAG_STBITE)) {
		if (RESET != sdio_flag_get(SDIO_FLAG_RXDTVAL)) {
			*(temp_scr + idx_scr) = sdio_data_read();
			++idx_scr;
		}
	}

  /* error check */
	if (RESET != sdio_flag_get(SDIO_FLAG_DTCRCERR)) {
		status = SD_DATA_CRC_ERROR;
		sdio_flag_clear(SDIO_FLAG_DTCRCERR);
		return status;
	} else if (RESET != sdio_flag_get(SDIO_FLAG_DTTMOUT)) {
		status = SD_DATA_TIMEOUT;
		sdio_flag_clear(SDIO_FLAG_DTTMOUT);
		return status;
	} else if (RESET != sdio_flag_get(SDIO_FLAG_RXORE)) {
		status = SD_RX_OVERRUN_ERROR;
		sdio_flag_clear(SDIO_FLAG_RXORE);
		return status;
	} else if (RESET != sdio_flag_get(SDIO_FLAG_STBITE)) {
		status = SD_START_BIT_ERROR;
		sdio_flag_clear(SDIO_FLAG_STBITE);
		return status;
	}

	/* clear all the SDIO_INTC flags */
	sdio_flag_clear(SDIO_MASK_INTC_FLAGS);

	/* readjust the temp SCR value */
	*(pscr) = ((temp_scr[1] & SD_MASK_0_7BITS) << 24) | ((temp_scr[1] & SD_MASK_8_15BITS) << 8) | 
				((temp_scr[1] & SD_MASK_16_23BITS) >> 8) | ((temp_scr[1] & SD_MASK_24_31BITS) >> 24);
	*(pscr + 1) = ((temp_scr[0] & SD_MASK_0_7BITS) << 24) | ((temp_scr[0] & SD_MASK_8_15BITS) << 8) | 
				((temp_scr[0] & SD_MASK_16_23BITS) >> 8) | ((temp_scr[0] & SD_MASK_24_31BITS) >> 24);

	return status;
}

/*!
	\brief      get the data block size
	\param[in]  bytesnumber: the number of bytes
	\param[out] none
	\retval     data block size
	  \arg        SDIO_DATABLOCKSIZE_1BYTE: block size = 1 byte
	  \arg        SDIO_DATABLOCKSIZE_2BYTES: block size = 2 bytes
	  \arg        SDIO_DATABLOCKSIZE_4BYTES: block size = 4 bytes
	  \arg        SDIO_DATABLOCKSIZE_8BYTES: block size = 8 bytes
	  \arg        SDIO_DATABLOCKSIZE_16BYTES: block size = 16 bytes
	  \arg        SDIO_DATABLOCKSIZE_32BYTES: block size = 32 bytes
	  \arg        SDIO_DATABLOCKSIZE_64BYTES: block size = 64 bytes
	  \arg        SDIO_DATABLOCKSIZE_128BYTES: block size = 128 bytes
	  \arg        SDIO_DATABLOCKSIZE_256BYTES: block size = 256 bytes
	  \arg        SDIO_DATABLOCKSIZE_512BYTES: block size = 512 bytes
	  \arg        SDIO_DATABLOCKSIZE_1024BYTES: block size = 1024 bytes
	  \arg        SDIO_DATABLOCKSIZE_2048BYTES: block size = 2048 bytes
	  \arg        SDIO_DATABLOCKSIZE_4096BYTES: block size = 4096 bytes
	  \arg        SDIO_DATABLOCKSIZE_8192BYTES: block size = 8192 bytes
	  \arg        SDIO_DATABLOCKSIZE_16384BYTES: block size = 16384 bytes
*/
static uint32_t sd_datablocksize_get(uint16_t bytesnumber)
{
	uint8_t exp_val = 0;

	/* calculate the exponent of 2 */
	while (1 != bytesnumber) {
		bytesnumber >>= 1;
		++exp_val;
	}

	return DATACTL_BLKSZ(exp_val);
}

/*!
	\brief      configure the GPIO of SDIO interface
	\param[in]  none
	\param[out] none
	\retval     none
*/
//static void gpio_config(void)
//{
	/* configure the SDIO_DAT0(PC8), SDIO_DAT1(PC9), SDIO_DAT2(PC10), SDIO_DAT3(PC11), SDIO_CLK(PC12) and SDIO_CMD(PD2) */
//	gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
//	gpio_init(GPIOD, GPIO_MODE_AF_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_2);
//}

/*!
	\brief      configure the RCU of SDIO and DMA
	\param[in]  none
	\param[out] none
	\retval     none
*/
//static void rcu_config(void)
//{
	//rcu_periph_clock_enable(RCU_SDIO);
	//rcu_periph_clock_enable(RCU_DMA1);
	//rcu_periph_clock_enable(RCU_GPIOC);
	//rcu_periph_clock_enable(RCU_GPIOD);
	//rcu_periph_clock_enable(RCU_AF);
//}

/*!
	\brief      configure the DMA1 channel 3 for transferring data
	\param[in]  srcbuf: a pointer point to a buffer which will be transferred
	\param[in]  bufsize: the size of buffer(not used in flow controller is peripheral)
	\param[out] none
	\retval     none
*/
static void dma_transfer_config(uint32_t *srcbuf, uint32_t bufsize)
{
	dma_parameter_struct dma_sdio;

	/* clear all the interrupt flags */
	dma_flag_clear(DMA1, DMA_CH3, DMA_FLAG_G);
	dma_flag_clear(DMA1, DMA_CH3, DMA_FLAG_FTF);
	dma_flag_clear(DMA1, DMA_CH3, DMA_FLAG_HTF);
	dma_flag_clear(DMA1, DMA_CH3, DMA_FLAG_ERR);
	dma_channel_disable(DMA1, DMA_CH3);
	dma_deinit(DMA1, DMA_CH3);

	/* configure the DMA1 channel 3 */
	dma_sdio.periph_addr = (uint32_t)SDIO_FIFO_ADDR;
	dma_sdio.memory_addr = (uint32_t)srcbuf;
	dma_sdio.direction = DMA_MEMORY_TO_PERIPHERAL;
	dma_sdio.number = bufsize / 4;
	dma_sdio.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
	dma_sdio.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
	dma_sdio.periph_width = DMA_PERIPHERAL_WIDTH_32BIT;
	dma_sdio.memory_width = DMA_MEMORY_WIDTH_32BIT; 
	dma_sdio.priority = DMA_PRIORITY_ULTRA_HIGH;
	dma_init(DMA1, DMA_CH3, &dma_sdio);
	dma_circulation_disable(DMA1, DMA_CH3);
	dma_interrupt_enable(DMA1, DMA_CH3, DMA_INT_FTF);
	dma_channel_enable(DMA1, DMA_CH3);
}

/*!
	\brief      configure the DMA1 channel 3 for receiving data
	\param[in]  dstbuf: a pointer point to a buffer which will receive data
	\param[in]  bufsize: the size of buffer(not used in flow controller is peripheral)
	\param[out] none
	\retval     none
*/
static void dma_receive_config(uint32_t *dstbuf, uint32_t bufsize)
{
	dma_parameter_struct dma_struct;

	/* clear all the interrupt flags */
	dma_flag_clear(DMA1, DMA_CH3, DMA_FLAG_G);
	dma_flag_clear(DMA1, DMA_CH3, DMA_FLAG_FTF);
	dma_flag_clear(DMA1, DMA_CH3, DMA_FLAG_HTF);
	dma_flag_clear(DMA1, DMA_CH3, DMA_FLAG_ERR);
	dma_channel_disable(DMA1, DMA_CH3);
	dma_deinit(DMA1, DMA_CH3);

	/* configure the DMA1 channel 3 */
	dma_struct.periph_addr = (uint32_t)SDIO_FIFO_ADDR;
	dma_struct.memory_addr = (uint32_t)dstbuf;
	dma_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
	dma_struct.number = bufsize / 4;
	dma_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
	dma_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
	dma_struct.periph_width = DMA_PERIPHERAL_WIDTH_32BIT;
	dma_struct.memory_width = DMA_MEMORY_WIDTH_32BIT; 
	dma_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
	dma_init(DMA1, DMA_CH3, &dma_struct);
	dma_circulation_disable(DMA1, DMA_CH3);
	dma_interrupt_enable(DMA1, DMA_CH3, DMA_INT_FTF);
	dma_channel_enable(DMA1, DMA_CH3);
}

//#endif /* ONBOARD_SDIO */
#endif /* ARDUINO_ARCH_GD32 */
