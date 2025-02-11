/*!
	\file    gd32f30x_usart.c
	\brief   USART driver

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

#include "gd32f30x_usart.h"

/* USART register bit offset */
#define GP_GUARD_TIME_OFFSET      ((uint32_t)8U)
#define CTL3_SC_RETRY_NUM_OFFSET  ((uint32_t)1U)
#define RT_BL_OFFSET              ((uint32_t)24U)

/*!
	\brief      reset USART/UART 
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_deinit(usart_peripheral_t *usart_pdata)
{
	switch(usart_pdata->baseAddress) {
	case USART0_BASE:
		rcu_periph_reset_enable(RCU_USART0RST);
		rcu_periph_reset_disable(RCU_USART0RST);
		break;
	case USART1_BASE:
		rcu_periph_reset_enable(RCU_USART1RST);
		rcu_periph_reset_disable(RCU_USART1RST);
		break;
	case USART2_BASE:
		rcu_periph_reset_enable(RCU_USART2RST);
		rcu_periph_reset_disable(RCU_USART2RST);
		break;
	case UART3_BASE:
		rcu_periph_reset_enable(RCU_UART3RST);
		rcu_periph_reset_disable(RCU_UART3RST);
		break;
	case UART4_BASE:
		rcu_periph_reset_enable(RCU_UART4RST);
		rcu_periph_reset_disable(RCU_UART4RST);
		break;
	default:
		break;
	}
}

/*!
	\brief      configure USART baud rate
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/ 
void usart_baudrate_set(usart_peripheral_t *usart_pdata)
{
	uint32_t uclk = 0U, intdiv = 0U, fradiv = 0U, udiv = 0U;

	switch(usart_pdata->baseAddress) {
		case USART0_BASE:
			uclk = rcu_clock_freq_get(CK_APB2);
			break;
		case USART1_BASE:
			uclk = rcu_clock_freq_get(CK_APB1);
			break;
		case USART2_BASE:
			uclk = rcu_clock_freq_get(CK_APB1);
			break;
		case UART3_BASE:
			uclk = rcu_clock_freq_get(CK_APB1);
			break;
		case UART4_BASE:
			uclk = rcu_clock_freq_get(CK_APB1);
			break;
		default:
			break;
	}

	/* configure the value of USART_BAUD (oversampling by 16) */
	udiv = (uclk + usart_pdata->baudrate / 2U) / usart_pdata->baudrate;
	intdiv = udiv & 0xfff0U;
	fradiv = udiv & 0xfU;

	USART_BAUD(usart_pdata->baseAddress) = ((USART_BAUD_FRADIV | USART_BAUD_INTDIV) & (intdiv | fradiv));
}

/*!
	\brief      configure USART parity
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	  \arg        USART_PM_NONE: no parity
	  \arg        USART_PM_ODD:  odd parity
	  \arg        USART_PM_EVEN: even parity 
	\retval     none
*/
void usart_parity_config(usart_peripheral_t *usart_pdata)
{
	/* clear USART_CTL0 PM, PCEN bits */
	USART_CTL0(usart_pdata->baseAddress) &= ~(USART_CTL0_PM | USART_CTL0_PCEN);
	/* configure USART parity mode */
	USART_CTL0(usart_pdata->baseAddress) |= usart_pdata->parityBits;
}

/*!
	\brief      configure USART word length
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
				only one parameter can be selected which is shown as below:
	  \arg        USART_WL_8BIT: 8 bits
	  \arg        USART_WL_9BIT: 9 bits
	\retval     none
*/
void usart_word_length_set(usart_peripheral_t *usart_pdata)
{
	/* clear USART_CTL0 WL bit */
	USART_CTL0(usart_pdata->baseAddress) &= ~USART_CTL0_WL;
	/* configure USART word length */
	USART_CTL0(usart_pdata->baseAddress) |= usart_pdata->wordLength;
}

/*!
	\brief      configure USART stop bit length
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
				only one parameter can be selected which is shown as below:
	  \arg        USART_STB_1BIT:   1 bit
	  \arg        USART_STB_0_5BIT: 0.5 bit, not available for UARTx(x=3,4)
	  \arg        USART_STB_2BIT:   2 bits
	  \arg        USART_STB_1_5BIT: 1.5 bits, not available for UARTx(x=3,4)
	\retval     none
*/
void usart_stop_bit_set(usart_peripheral_t *usart_pdata)
{
	/* clear USART_CTL1 STB bits */
	USART_CTL1(usart_pdata->baseAddress) &= ~USART_CTL1_STB; 
	/* configure USART stop bits */
	USART_CTL1(usart_pdata->baseAddress) |= usart_pdata->stopBits;
}

/*!
	\brief      enable USART
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_enable(usart_peripheral_t *usart_pdata)
{
	USART_CTL0(usart_pdata->baseAddress) |= USART_CTL0_UEN;
}

/*!
	\brief      disable USART
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_disable(usart_peripheral_t *usart_pdata)
{
	USART_CTL0(usart_pdata->baseAddress) &= ~(USART_CTL0_UEN);
}

/*!
	\brief      enable USART transmitter
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_transmit_enable(usart_peripheral_t *usart_pdata)
{
	USART_CTL0(usart_pdata->baseAddress) &= ~(USART_CTL0_TEN);
	USART_CTL0(usart_pdata->baseAddress) |= (USART_CTL0_TEN & USART_TRANSMIT_ENABLE);
}

/*!
	\brief      disable USART transmitter
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_transmit_disable(usart_peripheral_t *usart_pdata)
{
	USART_CTL0(usart_pdata->baseAddress) &= ~(USART_CTL0_TEN);
	USART_CTL0(usart_pdata->baseAddress) |= (USART_CTL0_TEN & USART_TRANSMIT_DISABLE);
}

/*!
	\brief      enable USART receiver
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_receive_enable(usart_peripheral_t *usart_pdata)
{
	USART_CTL0(usart_pdata->baseAddress) &= ~(USART_CTL0_REN);
	USART_CTL0(usart_pdata->baseAddress) |= (USART_CTL0_REN & USART_RECEIVE_ENABLE);
}

/*!
	\brief      disable USART receiver
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_receive_disable(usart_peripheral_t *usart_pdata)
{
	USART_CTL0(usart_pdata->baseAddress) &= ~(USART_CTL0_REN);
	USART_CTL0(usart_pdata->baseAddress) |= (USART_CTL0_REN & USART_RECEIVE_DISABLE);
}

/*!
	\brief      data is transmitted/received with the LSB/MSB first
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
				only one parameter can be selected which is shown as below:
	  \arg        USART_MSBF_LSB: LSB first
	  \arg        USART_MSBF_MSB: MSB first
	\retval     none
*/
void usart_data_first_config(usart_peripheral_t *usart_pdata)
{
	USART_CTL3(usart_pdata->baseAddress) &= ~(USART_CTL3_MSBF); 
	USART_CTL3(usart_pdata->baseAddress) |= usart_pdata->bitOrder;
}

/*!
	\brief      configure USART inversion
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	  \arg        USART_DINV_ENABLE: data bit level inversion
	  \arg        USART_DINV_DISABLE: data bit level not inversion
	  \arg        USART_TXPIN_ENABLE: TX pin level inversion
	  \arg        USART_TXPIN_DISABLE: TX pin level not inversion
	  \arg        USART_RXPIN_ENABLE: RX pin level inversion
	  \arg        USART_RXPIN_DISABLE: RX pin level not inversion
	\retval     none
*/
void usart_invert_config(usart_peripheral_t *usart_pdata)
{
	/* inverted or not the specified siginal */ 
	switch(usart_pdata->invertType) {
	case USART_DINV_ENABLE:
		/* data bit level inversion */
		USART_CTL3(usart_pdata->baseAddress) |= USART_CTL3_DINV;
		break;
	case USART_TXPIN_ENABLE:
		/* TX pin level inversion */
		USART_CTL3(usart_pdata->baseAddress) |= USART_CTL3_TINV;
		break;
	case USART_RXPIN_ENABLE:
		/* RX pin level inversion */
		USART_CTL3(usart_pdata->baseAddress) |= USART_CTL3_RINV;
		break;
	case USART_DINV_DISABLE:
		/* data bit level not inversion */
		USART_CTL3(usart_pdata->baseAddress) &= ~(USART_CTL3_DINV);
		break;
	case USART_TXPIN_DISABLE:
		/* TX pin level not inversion */
		USART_CTL3(usart_pdata->baseAddress) &= ~(USART_CTL3_TINV);
		break;
	case USART_RXPIN_DISABLE:
		/* RX pin level not inversion */
		USART_CTL3(usart_pdata->baseAddress) &= ~(USART_CTL3_RINV);
		break;
	default:
		break;
	}
}

/*!
	\brief      enable receiver timeout of USART
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_receiver_timeout_enable(usart_peripheral_t *usart_pdata)
{
	USART_CTL3(usart_pdata->baseAddress) |= USART_CTL3_RTEN;
}

/*!
	\brief      disable receiver timeout of USART
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_receiver_timeout_disable(usart_peripheral_t *usart_pdata)
{
	USART_CTL3(usart_pdata->baseAddress) &= ~(USART_CTL3_RTEN);
}

/*!
	\brief      set the receiver timeout threshold of USART
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\param[in]  timeout: 0-0xFFFFFF
	\retval     none
*/
void usart_receiver_timeout_threshold_config(usart_peripheral_t *usart_pdata, uint32_t timeout)
{
	USART_RT(usart_pdata->baseAddress) &= ~(USART_RT_RT);
	USART_RT(usart_pdata->baseAddress) |= timeout;
}

/*!
	\brief      USART transmit data function
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\param[in]  data: data of transmission 
	\retval     none
*/
void usart_data_transmit(usart_peripheral_t *usart_pdata, uint16_t data)
{
	USART_DATA(usart_pdata->baseAddress) = USART_DATA_DATA & (uint32_t)data;
}

/*!
	\brief      USART receive data function
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     data of received
*/
uint16_t usart_data_receive(usart_peripheral_t *usart_pdata)
{
	return (uint16_t)(GET_BITS(USART_DATA(usart_pdata->baseAddress), 0U, 8U));
}

/*!
	\brief      configure the address of the USART in wake up by address match mode
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\param[in]  addr: address of USART/UART
	\retval     none
*/
void usart_address_config(usart_peripheral_t *usart_pdata, uint8_t addr)
{
	USART_CTL1(usart_pdata->baseAddress) &= ~(USART_CTL1_ADDR);
	USART_CTL1(usart_pdata->baseAddress) |= (USART_CTL1_ADDR & (uint32_t)addr);
}

/*!
	\brief      receiver in mute mode
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_mute_mode_enable(usart_peripheral_t *usart_pdata)
{
	USART_CTL0(usart_pdata->baseAddress) |= USART_CTL0_RWU;
}

/*!
	\brief      receiver in active mode
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_mute_mode_disable(usart_peripheral_t *usart_pdata)
{
	USART_CTL0(usart_pdata->baseAddress) &= ~(USART_CTL0_RWU);
}

/*!
	\brief      configure wakeup method in mute mode
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	  \arg        USART_WM_IDLE: idle line
	  \arg        USART_WM_ADDR: address mask
	\retval     none
*/
void usart_mute_mode_wakeup_config(usart_peripheral_t *usart_pdata)
{
	USART_CTL0(usart_pdata->baseAddress) &= ~(USART_CTL0_WM);
	USART_CTL0(usart_pdata->baseAddress) |= usart_pdata->wakeupMethod;
}

/*!
	\brief      enable LIN mode
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_lin_mode_enable(usart_peripheral_t *usart_pdata)
{   
	USART_CTL1(usart_pdata->baseAddress) |= USART_CTL1_LMEN;
}

/*!
	\brief      disable LIN mode
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_lin_mode_disable(usart_peripheral_t *usart_pdata)
{   
	USART_CTL1(usart_pdata->baseAddress) &= ~(USART_CTL1_LMEN);
}

/*!
	\brief      configure lin break frame length
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	  \arg        USART_LBLEN_10B: 10 bits
	  \arg        USART_LBLEN_11B: 11 bits
	\retval     none
*/
void usart_lin_break_detection_length_config(usart_peripheral_t *usart_pdata)
{
	USART_CTL1(usart_pdata->baseAddress) &= ~(USART_CTL1_LBLEN);
	USART_CTL1(usart_pdata->baseAddress) |= (USART_CTL1_LBLEN & usart_pdata->lineBreakFrameLength);
}

/*!
	\brief      send break frame
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_send_break(usart_peripheral_t *usart_pdata)
{
	USART_CTL0(usart_pdata->baseAddress) |= USART_CTL0_SBKCMD;
}

/*!
	\brief      enable half duplex mode
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_halfduplex_enable(usart_peripheral_t *usart_pdata)
{   
	USART_CTL2(usart_pdata->baseAddress) |= USART_CTL2_HDEN;
}

/*!
	\brief      disable half duplex mode
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_halfduplex_disable(usart_peripheral_t *usart_pdata)
{  
	USART_CTL2(usart_pdata->baseAddress) &= ~(USART_CTL2_HDEN);
}

/*!
	\brief      enable CK pin in synchronous mode
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_synchronous_clock_enable(usart_peripheral_t *usart_pdata)
{
	USART_CTL1(usart_pdata->baseAddress) |= USART_CTL1_CKEN;
}

/*!
	\brief      disable CK pin in synchronous mode
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_synchronous_clock_disable(usart_peripheral_t *usart_pdata)
{
	USART_CTL1(usart_pdata->baseAddress) &= ~(USART_CTL1_CKEN);
}

/*!
	\brief      configure USART synchronous mode parameters
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	  \arg        USART_CLEN_NONE: there are 7 CK pulses for an 8 bit frame and 8 CK pulses for a 9 bit frame 
	  \arg        USART_CLEN_EN:   there are 8 CK pulses for an 8 bit frame and 9 CK pulses for a 9 bit frame
	  \arg        USART_CPH_1CK: first clock transition is the first data capture edge 
	  \arg        USART_CPH_2CK: second clock transition is the first data capture edge
	  \arg        USART_CPL_LOW:  steady low value on CK pin 
	  \arg        USART_CPL_HIGH: steady high value on CK pin
	\retval     none
*/
void usart_synchronous_clock_config(usart_peripheral_t *usart_pdata)
{
	USART_CTL1(usart_pdata->baseAddress) &= ~(USART_CTL1_CLEN | USART_CTL1_CPH | USART_CTL1_CPL);
	USART_CTL1(usart_pdata->baseAddress) |= (USART_CTL1_CLEN & usart_pdata->clockConfig.clockPulseLength) | (USART_CTL1_CPH & usart_pdata->clockConfig.clockPhase) | (USART_CTL1_CPL & usart_pdata->clockConfig.clockPolarity);
}

/*!
	\brief      configure guard time value in smartcard mode
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\param[in]  guat: guard time value, 0-0xFF
	\retval     none
*/
void usart_guard_time_config(usart_peripheral_t *usart_pdata, uint8_t guard_time)
{
	USART_GP(usart_pdata->baseAddress) &= ~(USART_GP_GUAT);
	USART_GP(usart_pdata->baseAddress) |= (USART_GP_GUAT & ((uint32_t)guard_time << GP_GUARD_TIME_OFFSET));
}

/*!
	\brief      enable smartcard mode
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_smartcard_mode_enable(usart_peripheral_t *usart_pdata)
{
	USART_CTL2(usart_pdata->baseAddress) |= USART_CTL2_SCEN;
}

/*!
	\brief      disable smartcard mode
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_smartcard_mode_disable(usart_peripheral_t *usart_pdata)
{
	USART_CTL2(usart_pdata->baseAddress) &= ~(USART_CTL2_SCEN);
}

/*!
	\brief      enable NACK in smartcard mode
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_smartcard_mode_nack_enable(usart_peripheral_t *usart_pdata)
{
	USART_CTL2(usart_pdata->baseAddress) |= USART_CTL2_NKEN;
}

/*!
	\brief      disable NACK in smartcard mode
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_smartcard_mode_nack_disable(usart_peripheral_t *usart_pdata)
{
	USART_CTL2(usart_pdata->baseAddress) &= ~(USART_CTL2_NKEN);
}

/*!
	\brief      configure smartcard auto-retry number
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\param[in]  sc_retry_num: smartcard auto-retry number
	\retval     none
*/
void usart_smartcard_autoretry_config(usart_peripheral_t *usart_pdata, uint8_t sc_retry_num)
{
	USART_CTL3(usart_pdata->baseAddress) &= ~(USART_CTL3_SCRTNUM);
	USART_CTL3(usart_pdata->baseAddress) |= (USART_CTL3_SCRTNUM & ((uint32_t)sc_retry_num << CTL3_SC_RETRY_NUM_OFFSET));
}

/*!
	\brief      configure block length in Smartcard T=1 reception
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\param[in]  sc_block_length: block length
	\retval     none
*/
void usart_block_length_config(usart_peripheral_t *usart_pdata, uint8_t sc_block_length)
{
	USART_RT(usart_pdata->baseAddress) &= ~(USART_RT_BL);
	USART_RT(usart_pdata->baseAddress) |= (USART_RT_BL & ((uint32_t)sc_block_length << RT_BL_OFFSET));
}

/*!
	\brief      enable IrDA mode
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_irda_mode_enable(usart_peripheral_t *usart_pdata)
{
	USART_CTL2(usart_pdata->baseAddress) |= USART_CTL2_IREN;
}

/*!
	\brief      disable IrDA mode
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_irda_mode_disable(usart_peripheral_t *usart_pdata)
{
	USART_CTL2(usart_pdata->baseAddress) &= ~(USART_CTL2_IREN);
}

/*!
	\brief      configure the peripheral clock prescaler in USART IrDA low-power mode
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\param[in]  prescaler: 0x00-0xFF
	\retval     none
*/
void usart_prescaler_config(usart_peripheral_t *usart_pdata, uint8_t prescaler)
{
	USART_GP(usart_pdata->baseAddress) &= ~(USART_GP_PSC);
	USART_GP(usart_pdata->baseAddress) |= (uint32_t)prescaler;
}

/*!
	\brief      configure IrDA low-power
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\param[in]  irlp: IrDA low-power or normal
				only one parameter can be selected which is shown as below:
	  \arg        USART_IRLP_LOW: low-power
	  \arg        USART_IRLP_NORMAL: normal
	\retval     none
*/
void usart_irda_lowpower_config(usart_peripheral_t *usart_pdata, uint32_t irda_low_power)
{
	USART_CTL2(usart_pdata->baseAddress) &= ~(USART_CTL2_IRLP);
	USART_CTL2(usart_pdata->baseAddress) |= (USART_CTL2_IRLP & irda_low_power);
}

/*!
	\brief      enable hardware flow control RTS
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_hardware_flow_rts_enable(usart_peripheral_t *usart_pdata)
{
	USART_CTL2(usart_pdata->baseAddress) &= ~(USART_CTL2_RTSEN);
	USART_CTL2(usart_pdata->baseAddress) |= (USART_CTL2_RTSEN & USART_RTS_ENABLE);
}

/*!
	\brief      disable hardware flow control RTS
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_hardware_flow_rts_disable(usart_peripheral_t *usart_pdata)
{
	USART_CTL2(usart_pdata->baseAddress) &= ~(USART_CTL2_RTSEN);
	USART_CTL2(usart_pdata->baseAddress) |= (USART_CTL2_RTSEN & USART_RTS_DISABLE);
}

/*!
	\brief      enable hardware flow control CTS
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_hardware_flow_cts_enable(usart_peripheral_t *usart_pdata)
{
	USART_CTL2(usart_pdata->baseAddress) &= ~(USART_CTL2_CTSEN);
	USART_CTL2(usart_pdata->baseAddress) |= (USART_CTL2_CTSEN & USART_CTS_ENABLE);
}

/*!
	\brief      disable hardware flow control CTS
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_hardware_flow_cts_disable(usart_peripheral_t *usart_pdata)
{
	USART_CTL2(usart_pdata->baseAddress) &= ~(USART_CTL2_CTSEN);
	USART_CTL2(usart_pdata->baseAddress) |= (USART_CTL2_CTSEN & USART_CTS_DISABLE);
}

/*!
	\brief      enable USART DMA reception
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_dma_receive_enabke(usart_peripheral_t *usart_pdata)
{
	USART_CTL2(usart_pdata->baseAddress) &= ~(USART_CTL2_DENR);
	USART_CTL2(usart_pdata->baseAddress) |= (USART_CTL2_DENR & USART_RECEIVE_DMA_ENABLE);
}

/*!
	\brief      disable USART DMA reception
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_dma_receive_disable(usart_peripheral_t *usart_pdata)
{
	USART_CTL2(usart_pdata->baseAddress) &= ~(USART_CTL2_DENR);
	USART_CTL2(usart_pdata->baseAddress) |= (USART_CTL2_DENR & USART_RECEIVE_DMA_DISABLE);
}

/*!
	\brief      enable USART DMA transmission
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_dma_transmit_enable(usart_peripheral_t *usart_pdata)
{
	USART_CTL2(usart_pdata->baseAddress) &= ~(USART_CTL2_DENT);
	USART_CTL2(usart_pdata->baseAddress) |= (USART_CTL2_DENT & USART_TRANSMIT_DMA_ENABLE);
}

/*!
	\brief      disable USART DMA transmission
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\retval     none
*/
void usart_dma_transmit_disable(usart_peripheral_t *usart_pdata)
{
	USART_CTL2(usart_pdata->baseAddress) &= ~(USART_CTL2_DENT);
	USART_CTL2(usart_pdata->baseAddress) |= (USART_CTL2_DENT & USART_TRANSMIT_DMA_DISABLE);
}

/*!
	\brief      get flag in STAT0/STAT1 register
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\param[in]  flag: USART flags, refer to usart_flag_enum
				only one parameter can be selected which is shown as below:
	  \arg        USART_FLAG_CTS: CTS change flag
	  \arg        USART_FLAG_LBD: LIN break detected flag 
	  \arg        USART_FLAG_TBE: transmit data buffer empty 
	  \arg        USART_FLAG_TC: transmission complete 
	  \arg        USART_FLAG_RBNE: read data buffer not empty 
	  \arg        USART_FLAG_IDLE: IDLE frame detected flag 
	  \arg        USART_FLAG_ORERR: overrun error 
	  \arg        USART_FLAG_NERR: noise error flag 
	  \arg        USART_FLAG_FERR: frame error flag 
	  \arg        USART_FLAG_PERR: parity error flag 
	  \arg        USART_FLAG_BSY: busy flag 
	  \arg        USART_FLAG_EB: end of block flag 
	  \arg        USART_FLAG_RT: receiver timeout flag 
	\retval     FlagStatus: SET or RESET
*/
FlagStatus usart_flag_get(usart_peripheral_t *usart_pdata, usart_flag_enum flag)
{
	if ((USART_REG_VAL(usart_pdata->baseAddress, flag) & BIT(USART_BIT_POS(flag))) != RESET) {
		return SET;
	} else {
		return RESET;
	}
}

/*!
	\brief      clear flag in STAT0/STAT1 register
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\param[in]  flag: USART flags, refer to usart_flag_enum
				only one parameter can be selected which is shown as below:
	  \arg        USART_FLAG_CTS: CTS change flag
	  \arg        USART_FLAG_LBD: LIN break detected flag
	  \arg        USART_FLAG_TC: transmission complete
	  \arg        USART_FLAG_RBNE: read data buffer not empty
	  \arg        USART_FLAG_EB: end of block flag
	  \arg        USART_FLAG_RT: receiver timeout flag
	\retval     none
*/
void usart_flag_clear(usart_peripheral_t *usart_pdata, usart_flag_enum flag)
{
	USART_REG_VAL(usart_pdata->baseAddress, flag) = ~BIT(USART_BIT_POS(flag));
}

/*!
	\brief      enable USART interrupt
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\param[in]  interrupt: USART interrupts, refer to usart_interrupt_enum
				only one parameter can be selected which is shown as below:
	  \arg        USART_INT_PERR: parity error interrupt
	  \arg        USART_INT_TBE: transmitter buffer empty interrupt
	  \arg        USART_INT_TC: transmission complete interrupt
	  \arg        USART_INT_RBNE: read data buffer not empty interrupt and overrun error interrupt
	  \arg        USART_INT_IDLE: IDLE line detected interrupt
	  \arg        USART_INT_LBD: LIN break detected interrupt
	  \arg        USART_INT_ERR: error interrupt
	  \arg        USART_INT_CTS: CTS interrupt
	  \arg        USART_INT_RT: interrupt enable bit of receive timeout event
	  \arg        USART_INT_EB: interrupt enable bit of end of block event
	\retval     none
*/
void usart_interrupt_enable(usart_peripheral_t *usart_pdata, usart_interrupt_enum interrupt)
{
	USART_REG_VAL(usart_pdata->baseAddress, interrupt) |= BIT(USART_BIT_POS(interrupt));
}

/*!
	\brief      disable USART interrupt
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\param[in]  interrupt: USART interrupts, refer to usart_interrupt_enum
				only one parameter can be selected which is shown as below:
	  \arg        USART_INT_PERR: parity error interrupt
	  \arg        USART_INT_TBE: transmitter buffer empty interrupt
	  \arg        USART_INT_TC: transmission complete interrupt
	  \arg        USART_INT_RBNE: read data buffer not empty interrupt and overrun error interrupt
	  \arg        USART_INT_IDLE: IDLE line detected interrupt
	  \arg        USART_INT_LBD: LIN break detected interrupt
	  \arg        USART_INT_ERR: error interrupt
	  \arg        USART_INT_CTS: CTS interrupt
	  \arg        USART_INT_RT: interrupt enable bit of receive timeout event
	  \arg        USART_INT_EB: interrupt enable bit of end of block event
	\retval     none
*/
void usart_interrupt_disable(usart_peripheral_t *usart_pdata, usart_interrupt_enum interrupt)
{
	USART_REG_VAL(usart_pdata->baseAddress, interrupt) &= ~BIT(USART_BIT_POS(interrupt));
}

/*!
	\brief      get USART interrupt and flag status
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\param[in]  int_flag: USART interrupt flags, refer to usart_interrupt_flag_enum
				only one parameter can be selected which is shown as below:
	  \arg        USART_INT_FLAG_PERR: parity error interrupt and flag
	  \arg        USART_INT_FLAG_TBE: transmitter buffer empty interrupt and flag
	  \arg        USART_INT_FLAG_TC: transmission complete interrupt and flag
	  \arg        USART_INT_FLAG_RBNE: read data buffer not empty interrupt and flag
	  \arg        USART_INT_FLAG_RBNE_ORERR: read data buffer not empty interrupt and overrun error flag
	  \arg        USART_INT_FLAG_IDLE: IDLE line detected interrupt and flag
	  \arg        USART_INT_FLAG_LBD: LIN break detected interrupt and flag
	  \arg        USART_INT_FLAG_CTS: CTS interrupt and flag
	  \arg        USART_INT_FLAG_ERR_ORERR: error interrupt and overrun error
	  \arg        USART_INT_FLAG_ERR_NERR: error interrupt and noise error flag
	  \arg        USART_INT_FLAG_ERR_FERR: error interrupt and frame error flag
	  \arg        USART_INT_FLAG_EB: interrupt enable bit of end of block event and flag
	  \arg        USART_INT_FLAG_RT: interrupt enable bit of receive timeout event and flag
	\retval     FlagStatus: SET or RESET
*/
FlagStatus usart_interrupt_flag_get(usart_peripheral_t *usart_pdata, usart_interrupt_flag_enum int_flag)
{
	uint32_t interruot_enable = 0U, flagstatus = 0U;
	/* get the interrupt enable bit status */
	interrupt_state = (USART_REG_VAL(usart_pdata->baseAddress, int_flag) & BIT(USART_BIT_POS(int_flag)));
	/* get the corresponding flag bit status */
	flagstatus = (USART_REG_VAL2(usart_pdata->baseAddress, int_flag) & BIT(USART_BIT_POS2(int_flag)));

	if ((flagstatus != 0U) && (interrupt_state != 0U)) {
		return SET;
	} else {
		return RESET; 
	}
}

/*!
	\brief      clear USART interrupt flag in STAT0/STAT1 register
	\param[in]  usart_pdata: pointer to usart_peripheral_t structure
	\param[in]  int_flag: USART interrupt flags, refer to usart_interrupt_flag_enum
				only one parameter can be selected which is shown as below:
	  \arg        USART_INT_FLAG_CTS: CTS change flag
	  \arg        USART_INT_FLAG_LBD: LIN break detected flag
	  \arg        USART_INT_FLAG_TC: transmission complete
	  \arg        USART_INT_FLAG_RBNE: read data buffer not empty
	  \arg        USART_INT_FLAG_EB: end of block flag
	  \arg        USART_INT_FLAG_RT: receiver timeout flag
	\retval     none
*/
void usart_interrupt_flag_clear(usart_peripheral_t *usart_pdata, usart_interrupt_flag_enum int_flag)
{
	USART_REG_VAL2(usart_pdata->baseAddress, int_flag) = ~BIT(USART_BIT_POS2(int_flag));
}
