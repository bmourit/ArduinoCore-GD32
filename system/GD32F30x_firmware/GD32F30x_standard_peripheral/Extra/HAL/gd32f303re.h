/**
 *  Copyright (c) 2024, BMourit ProjectHALGD
 *
 *  This HAL library is a distict, but derived from,
 *  the GigaDevice provided firmware files, so:
 *
 *  Copyright (c) 2020, GigaDevice Semiconductor Inc.
 *  
 *  Which in turn are clearly based on code from STMicroelectronics. We'll attempt
 *  to right the wrong of GigaDevice with:
 *
 *  Copyright (c) 2017, STMicroelectronics. All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without modification, 
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this 
 *      list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice, 
 *      this list of conditions and the following disclaimer in the documentation 
 *      and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the names of its contributors 
 *      may be used to endorse or promote products derived from this software without 
 *      specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 *  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 *  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
 *  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 *  OF SUCH DAMAGE.
 */

#ifndef GD32F303RE_H
#define GD32F303RE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef GD32F30X_HD
#define GD32F30X_HD
#endif

/* configuration of the Cortex-M4 processor and core peripherals */
#define __CM4_REV                 0x0001    // Core revision r0p1
//#define __MPU_PRESENT           1         // GD32F30x provide MPU
                                            //
                                            // NOTE:
                                            //       GigaDevice provided FW has this enabled
                                            //       but it is unclear is some are missing
                                            //       both MPU and FPU.
                                            //
#define __MPU_PRESENT             0         // GD32F30x provide MPU
#define __NVIC_PRIO_BITS          4         // GD32F30x uses 4 bits for the priority levels
#define __Vendor_SysTickConfig    0         // set to 1 if different sysTick config is used
//#define __FPU_PRESENT           1         // some GD32F303RET6 MCUs shipped without an FPU


typedef enum IRQn
{
    /* Cortex-M4 Processor Exceptions Numbers */
    NonMaskableInt_IRQn          = -14,    /*!< 2 Non Maskable Interrupt                                 */
    HardFault_IRQn               = -13,    /*!< 3 HardFault Interrupt                                    */
    MemoryManagement_IRQn        = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                  */
    BusFault_IRQn                = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                          */
    UsageFault_IRQn              = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                        */
    SVCall_IRQn                  = -5,     /*!< 11 Cortex-M4 SV call interrupt                           */
    DebugMonitor_IRQn            = -4,     /*!< 12 Cortex-M4 Debug mMonitor Interrupt                     */
    PendSV_IRQn                  = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                           */
    SysTick_IRQn                 = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                       */

    /* GD32F303RE Specific Interruput Numbers */
    WWDGT_IRQn                   = 0,      /*!< window watchDog timer interrupt                          */
    LVD_IRQn                     = 1,      /*!< LVD through EXTI line detect interrupt                   */
    TAMPER_IRQn                  = 2,      /*!< tamper through EXTI line detect                          */
    RTC_IRQn                     = 3,      /*!< RTC through EXTI line interrupt                          */
    FMC_IRQn                     = 4,      /*!< FMC interrupt                                            */
    RCU_CTC_IRQn                 = 5,      /*!< RCU and CTC interrupt                                    */
    EXTI0_IRQn                   = 6,      /*!< EXTI line 0 interrupt                                    */
    EXTI1_IRQn                   = 7,      /*!< EXTI line 1 interrupt                                    */
    EXTI2_IRQn                   = 8,      /*!< EXTI line 2 interrupt                                    */
    EXTI3_IRQn                   = 9,      /*!< EXTI line 3 interrupt                                    */
    EXTI4_IRQn                   = 10,     /*!< EXTI line 4 interrupt                                    */
    DMA0_Channel0_IRQn           = 11,     /*!< DMA0 channel0 interrupt                                  */
    DMA0_Channel1_IRQn           = 12,     /*!< DMA0 channel1 interrupt                                  */
    DMA0_Channel2_IRQn           = 13,     /*!< DMA0 channel2 interrupt                                  */
    DMA0_Channel3_IRQn           = 14,     /*!< DMA0 channel3 interrupt                                  */
    DMA0_Channel4_IRQn           = 15,     /*!< DMA0 channel4 interrupt                                  */
    DMA0_Channel5_IRQn           = 16,     /*!< DMA0 channel5 interrupt                                  */
    DMA0_Channel6_IRQn           = 17,     /*!< DMA0 channel6 interrupt                                  */
    ADC0_1_IRQn                  = 18,     /*!< ADC0 and ADC1 interrupt                                  */
    USBD_HP_CAN0_TX_IRQn         = 19,     /*!< CAN0 TX interrupts                                       */
    USBD_LP_CAN0_RX0_IRQn        = 20,     /*!< CAN0 RX0 interrupts                                      */
    CAN0_RX1_IRQn                = 21,     /*!< CAN0 RX1 interrupt                                       */
    CAN0_EWMC_IRQn               = 22,     /*!< CAN0 EWMC interrupt                                      */
    EXTI5_9_IRQn                 = 23,     /*!< EXTI[9:5] interrupts                                     */
    TIMER0_BRK_IRQn              = 24,     /*!< TIMER0 break interrupt                                   */
    TIMER0_UP_IRQn               = 25,     /*!< TIMER0 update interrupt                                  */
    TIMER0_TRG_CMT_IRQn          = 26,     /*!< TIMER0 trigger and commutation interrupt                 */
    TIMER0_Channel_IRQn          = 27,     /*!< TIMER0 channel capture compare interrupt                 */
    TIMER1_IRQn                  = 28,     /*!< TIMER1 interrupt                                         */
    TIMER2_IRQn                  = 29,     /*!< TIMER2 interrupt                                         */
    TIMER3_IRQn                  = 30,     /*!< TIMER3 interrupt                                         */
    I2C0_EV_IRQn                 = 31,     /*!< I2C0 event interrupt                                     */
    I2C0_ER_IRQn                 = 32,     /*!< I2C0 error interrupt                                     */
    I2C1_EV_IRQn                 = 33,     /*!< I2C1 event interrupt                                     */
    I2C1_ER_IRQn                 = 34,     /*!< I2C1 error interrupt                                     */
    SPI0_IRQn                    = 35,     /*!< SPI0 interrupt                                           */
    SPI1_IRQn                    = 36,     /*!< SPI1 interrupt                                           */
    USART0_IRQn                  = 37,     /*!< USART0 interrupt                                         */
    USART1_IRQn                  = 38,     /*!< USART1 interrupt                                         */
    USART2_IRQn                  = 39,     /*!< USART2 interrupt                                         */
    EXTI10_15_IRQn               = 40,     /*!< EXTI[15:10] interrupts                                   */
    RTC_Alarm_IRQn               = 41,     /*!< RTC alarm interrupt                                      */
    USBD_WKUP_IRQn               = 42,     /*!< USBD Wakeup interrupt                                    */
    TIMER7_BRK_IRQn              = 43,     /*!< TIMER7 break interrupt                                   */
    TIMER7_UP_IRQn               = 44,     /*!< TIMER7 update interrupt                                  */
    TIMER7_TRG_CMT_IRQn          = 45,     /*!< TIMER7 trigger and commutation interrupt                 */
    TIMER7_Channel_IRQn          = 46,     /*!< TIMER7 channel capture compare interrupt                 */
    ADC2_IRQn                    = 47,     /*!< ADC2 global interrupt                                    */
    EXMC_IRQn                    = 48,     /*!< EXMC global interrupt                                    */
    SDIO_IRQn                    = 49,     /*!< SDIO global interrupt                                    */
    TIMER4_IRQn                  = 50,     /*!< TIMER4 global interrupt                                  */
    SPI2_IRQn                    = 51,     /*!< SPI2 global interrupt                                    */
    UART3_IRQn                   = 52,     /*!< UART3 global interrupt                                   */
    UART4_IRQn                   = 53,     /*!< UART4 global interrupt                                   */
    TIMER5_IRQn                  = 54,     /*!< TIMER5 global interrupt                                  */
    TIMER6_IRQn                  = 55,     /*!< TIMER6 global interrupt                                  */
    DMA1_Channel0_IRQn           = 56,     /*!< DMA1 channel0 global interrupt                           */
    DMA1_Channel1_IRQn           = 57,     /*!< DMA1 channel1 global interrupt                           */
    DMA1_Channel2_IRQn           = 58,     /*!< DMA1 channel2 global interrupt                           */
    DMA1_Channel3_Channel4_IRQn  = 59,     /*!< DMA1 channel3 and channel4 global Interrupt              */
} IRQn_Type;

/* includes */
#include "core_cm4.h"
#include "system_gd32f30x.h"
#include <stdint.h>

typedef struct ADC_reg_struct {
    __IO uint32_t STAT;
    __IO uint32_t CTL0;
    __IO uint32_t CTL1;
    __IO uint32_t SAMPT0;
    __IO uint32_t SAMPT1;
    __IO uint32_t IOFF0;
    __IO uint32_t IOFF1;
    __IO uint32_t IOFF2;
    __IO uint32_t IOFF3;
    __IO uint32_t WDHT;
    __IO uint32_t WDLT;
    __IO uint32_t RSQ0;
    __IO uint32_t RSQ1;
    __IO uint32_t RSQ2;
    __IO uint32_t ISQ;
    __IO uint32_t IDATA0;
    __IO uint32_t IDATA1;
    __IO uint32_t IDATA2;
    __IO uint32_t IDATA3;
    __IO uint32_t RDATA;
    __IO uint32_t OVSAMPCTL;
} ADC_reg_t;

typedef struct BKP_reg_struct {
    __IO uint32_t RES0;
    __IO uint32_t DATA0;
    __IO uint32_t DATA1;
    __IO uint32_t DATA2;
    __IO uint32_t DATA3;
    __IO uint32_t DATA4;
    __IO uint32_t DATA5;
    __IO uint32_t DATA6;
    __IO uint32_t DATA7;
    __IO uint32_t DATA8;
    __IO uint32_t DATA9;
    __IO uint32_t DATA10;
    __IO uint32_t OCTL;
    __IO uint32_t TPCTL;
    __IO uint32_t TPCS;
    __IO uint32_t RES1[2];
    __IO uint32_t DATA11;
    __IO uint32_t DATA12;
    __IO uint32_t DATA13;
    __IO uint32_t DATA14;
    __IO uint32_t DATA15;
    __IO uint32_t DATA16;
    __IO uint32_t DATA17;
    __IO uint32_t DATA18;
    __IO uint32_t DATA19;
    __IO uint32_t DATA20;
    __IO uint32_t DATA21;
    __IO uint32_t DATA22;
    __IO uint32_t DATA23;
    __IO uint32_t DATA24;
    __IO uint32_t DATA25;
    __IO uint32_t DATA26;
    __IO uint32_t DATA27;
    __IO uint32_t DATA28;
    __IO uint32_t DATA29;
    __IO uint32_t DATA30;
    __IO uint32_t DATA31;
    __IO uint32_t DATA32;
    __IO uint32_t DATA33;
    __IO uint32_t DATA34;
    __IO uint32_t DATA35;
    __IO uint32_t DATA36;
    __IO uint32_t DATA37;
    __IO uint32_t DATA38;
    __IO uint32_t DATA39;
    __IO uint32_t DATA40;
    __IO uint32_t DATA41;
} BKP_reg_t;

typedef struct CAN_reg_struct {
    __IO uint32_t CTL;
    __IO uint32_t STAT;
    __IO uint32_t TSTAT;
    __IO uint32_t RFIFO0;
    __IO uint32_t RFIFO1;
    __IO uint32_t INTEN;
    __IO uint32_t ERR;
    __IO uint32_t BT;
    uint32_t RES0[88];
    __IO uint32_t TMI0;
    __IO uint32_t TMP0;
    __IO uint32_t TMPDATA00;
    __IO uint32_t TMPDATA10;
    __IO uint32_t TMI1;
    __IO uint32_t TMP1;
    __IO uint32_t TMPDATA01;
    __IO uint32_t TMPDATA11;
    __IO uint32_t TMI2;
    __IO uint32_t TMP2;
    __IO uint32_t TMDATA02;
    __IO uint32_t TMDATA12;
    __IO uint32_t RFIFIMI0;
    __IO uint32_t RFIFIMP0;
    __IO uint32_t RFIFIMDATA00;
    __IO uint32_t RFIFIMDATA10;
    __IO uint32_t RFIFIMI1;
    __IO uint32_t RFIFIMP2;
    __IO uint32_t RFIFIMDATA01;
    __IO uint32_t RFIFIMDATA11;
    uint32_t RES1[12];
    __IO uint32_t FCTL;
    __IO uint32_t FMCFG;
    uint32_t RES2;
    __IO uint32_t FSCFG;
    uint32_t RES3;
    __IO uint32_t FAFIFO;
    uint32_t RES4;
    __IO uint32_t FW;
    uint32_t RES5[8];
    __IO uint32_t F0DATA0;
    __IO uint32_t F0DATA1;
    __IO uint32_t F1DATA0;
    __IO uint32_t F1DATA1;
    __IO uint32_t F2DATA0;
    __IO uint32_t F2DATA1;
    __IO uint32_t F3DATA0;
    __IO uint32_t F3DATA1;
    __IO uint32_t F4DATA0;
    __IO uint32_t F4DATA1;
    __IO uint32_t F5DATA0;
    __IO uint32_t F5DATA1;
    __IO uint32_t F6DATA0;
    __IO uint32_t F6DATA1;
    __IO uint32_t F7DATA0;
    __IO uint32_t F7DATA1;
    __IO uint32_t F8DATA0;
    __IO uint32_t F8DATA1;
    __IO uint32_t F9DATA0;
    __IO uint32_t F9DATA1;
    __IO uint32_t F10DATA0;
    __IO uint32_t F10DATA1;
    __IO uint32_t F11DATA0;
    __IO uint32_t F11DATA1;
    __IO uint32_t F12DATA0;
    __IO uint32_t F12DATA1;
    __IO uint32_t F13DATA0;
    __IO uint32_t F13DATA1;
    __IO uint32_t F14DATA0;
    __IO uint32_t F14DATA1;
    __IO uint32_t F15DATA0;
    __IO uint32_t F15DATA1;
    __IO uint32_t F16DATA0;
    __IO uint32_t F16DATA1;
    __IO uint32_t F17DATA0;
    __IO uint32_t F17DATA1;
    __IO uint32_t F18DATA0;
    __IO uint32_t F18DATA1;
    __IO uint32_t F19DATA0;
    __IO uint32_t F19DATA1;
    __IO uint32_t F20DATA0;
    __IO uint32_t F20DATA1;
    __IO uint32_t F21DATA0;
    __IO uint32_t F21DATA1;
    __IO uint32_t F22DATA0;
    __IO uint32_t F22DATA1;
    __IO uint32_t F23DATA0;
    __IO uint32_t F23DATA1;
    __IO uint32_t F24DATA0;
    __IO uint32_t F24DATA1;
    __IO uint32_t F25DATA0;
    __IO uint32_t F25DATA1;
    __IO uint32_t F26DATA0;
    __IO uint32_t F26DATA1;
    __IO uint32_t F27DATA0;
    __IO uint32_t F27DATA1;
} CAN_reg_t;

typedef struct CRC_reg_struct {
    __IO uint32_t DATA;
    __IO uint8_t FDATA;
    uint8_t RES0[3];
    __IO uint32_t CTL;
} CRC_reg_t;

typedef struct CTC_reg_struct {
    IO uint32_t CTL0;
    IO uint32_t CTL1;
    IO uint32_t STAT;
    IO uint32_t INTC;
} CTC_reg_t;

typedef struct DAC_reg_struct {
    __IO uint32_t CTL;
    __IO uint32_t SWT;
    __IO uint32_t DAC0_R12DH;
    __IO uint32_t DAC0_L12DH;
    __IO uint32_t DAC0_R8DH;
    __IO uint32_t DAC1_R12DH;
    __IO uint32_t DAC1_L12DH;
    __IO uint32_t DAC1_R8DH;
    __IO uint32_t DACC_R12DH;
    __IO uint32_t DACC_L12DH;
    __IO uint32_t DACC_R8DH;
    __IO uint32_t DAC0_DO;
    __IO uint32_t DAC1_DO;
} DAC_reg_t;

typedef struct DBG_reg_struct {
    __IO uint32_t ID;
    __IO uint32_t CTL0;
} DBG_reg_t;

typedef struct {
    __IO uint32_t CHxCTL;
    __IO uint32_t CHxCNT;
    __IO uint32_t CHxPADDR;
    __IO uint32_t CHxMADDR;
} dma_channel_t;

typedef struct DMA_reg_struct {
    __IO uint32_t INTF;
    __IO uint32_t INTC;
    dma_channel_t CH_Params[7];
} DMA_reg_t;

typedef struct {
    __IO uint32_t SNCTLx;
    __IO uint32_t SNTCFGx;
} EXTM_SRAM_NOR1_t;

typedef struct {
    __IO uint32_t SNWTCFGx;
    uint32_t RES0;
} EXTM_SRAM_NOR2_t;

typedef struct {
    __IO uint32_t NPCTLx;
    __IO uint32_t NPINTENx;
    __IO uint32_t NPCTCFGx;
    __IO uint32_t NPATCFGx;
    uint32_t RES0;
    __IO uint32_t NECCx;
    uint32_t RES1[2];
} EXTM_NAND_PCCARD1_t;

typedef struct {
    __IO uint32_t NPCTL;
    __IO uint32_t NPINTEN;
    __IO uint32_t NPCTCFG;
    __IO uint32_t NPATCFG;
    __IO uint32_t PIOTCFG;
} EXTM_NAND_PCCARD2_t;

typedef struct EXTM_reg_struct {
    EXTM_SRAM_NOR1_t SNBank1[4];
    uint32_t RES0[16];
    EXTM_NAND_PCCARD1_t NPBank1[2]
    EXTM_NAND_PCCARD2_t NPBank2;
    uint32_t RES1[20];
    EXTM_SRAM_NOR2_t SNBank2[3];
} EXTM_reg_t;

typedef struct EXTI_reg_struct {
    __IO uint32_t INTEN;
    __IO uint32_t EVEN;
    __IO uint32_t RTEN;
    __IO uint32_t FTEN;
    __IO uint32_t SWIEV;
    __IO uint32_t PD;
} EXTI_reg_t;

typedef struct FMC_reg_struct {
    __IO uint32_t WS;
    __IO uint32_t KEY0;
    __IO uint32_t OBKEY;
    __IO uint32_t STAT0;
    __IO uint32_t CTL0;
    __IO uint32_t ADDR0;
    uint32_t RES0;
    __IO uint32_t OBSTAT;
    __IO uint32_t WP;
    uint32_t RES1[8];
    __IO uint32_t KEY1;
    uint32_t RES2;
    __IO uint32_t STAT1;
    __IO uint32_t CTL1;
    __IO uint32_t ADDR1;
    uint32_t RES3[41];
    __IO uint32_t WSEN;
    __IO uint32_t PID;
} FMC_reg_t;

typedef struct OB_reg_struct {
    __IO uint16_t SPC;
    __IO uint16_t USER;
    __IO uint16_t DATA1;
    __IO uint16_t DATA2;
    __IO uint16_t WP0;
    __IO uint16_t WP1;
    __IO uint16_t WP2;
    __IO uint16_t WP3;
} OB_reg_t;

typedef struct FWDGT_reg_struct {
    __IO uint32_t CTL;
    __IO uint32_t PSC;
    __IO uint32_t RLD;
    __IO uint32_t STAT;
} FWDGT_reg_t;

typedef struct GPIO_reg_struct {
    __IO uint32_t CTL0;
    __IO uint32_t CTL1;
    __IO uint32_t ISTAT;
    __IO uint32_t OCTL;
    __IO uint32_t BOP;
    __IO uint32_t BC;
    __IO uint32_t LOCK;
    __IO uint32_t SPD;
} GPIO_reg_t;

typedef struct AFIO_reg_struct {
    __IO uint32_t EC;
    __IO uint32_t PCF0;
    __IO uint32_t EXTISS0;
    __IO uint32_t EXTISS1;
    __IO uint32_t EXTISS2;
    __IO uint32_t EXTISS3;
    uint32_t RES0;
    __IO uint32_t PFC1;
    __IO uint32_t CPSCTL;
} AFIO_reg_t;

typedef struct I2C_reg_struct {
    __IO uint32_t CTL0;
    __IO uint32_t CTL1;
    __IO uint32_t SADDR0;
    __IO uint32_t SADDR1;
    __IO uint32_t DATA;
    __IO uint32_t STAT0;
    __IO uint32_t STAT1;
    __IO uint32_t CKCFG;
    __IO uint32_t RT;
    __IO uint32_t FMPCFG;
} I2C_reg_t;

typedef struct PMU_reg_struct {
    __IO uint32_t CTL;
    __IO uint32_t CS;    
} PMU_reg_t;

typedef struct RCU_reg_struct {
    __IO uint32_t CTL;
    __IO uint32_t CFG0;
    __IO uint32_t INTR;
    __IO uint32_t APB2RST;
    __IO uint32_t APB1RST;
    __IO uint32_t AHBEN;
    __IO uint32_t APB2EN;
    __IO uint32_t APB1EN;
    __IO uint32_t BDCTL;
    __IO uint32_t RSTSCK;
    uint32_t RES0;
    __IO uint32_t CFG1;
    uint32_t RES1;
    __IO uint32_t DSV;
    uint32_t RES2[34];
    __IO uint32_t ADDCTL;
    uint32_t RES3[2];
    __IO uint32_t ADDINTR;
    uint32_t RES4[4];
    __IO uint32_t ADDAPB1RST;
    __IO uint32_t ADDAPB1EN;
} RCU_reg_t;

typedef struct RTC_reg_struct {
    __IO uint32_t INTEN;
    __IO uint32_t CTL;
    __IO uint32_t PSCH;
    __IO uint32_t PSCL;
    __IO uint32_t DIVH;
    __IO uint32_t DIVL;
    __IO uint32_t CNTH;
    __IO uint32_t CNTL;
    __IO uint32_t ALRMH;
    __IO uint32_t ALRML;
} RTC_reg_t;

typedef struct SDIO_reg_struct {
    __IO uint32_t PWRCTL;
    __IO uint32_t CLKCTL;
    __IO uint32_t CMDAGMT;
    __IO uint32_t CMDCTL;
    __IO uint32_t RSPCMDIDX;
    __I uint32_t RESP0;
    __I uint32_t RESP1;
    __I uint32_t RESP2;
    __I uint32_t RESP3;
    __IO uint32_t DATATO;
    __IO uint32_t DATALEN;
    __IO uint32_t DATACTL;
    __I uint32_t DATACNT;
    __I uint32_t STAT;
    __IO uint32_t INTC;
    __IO uint32_t INTEN;
    uint32_t RES0[2];
    __I uint32_t FIFOCNT;
    uint32_t RES1[13];
    __IO uint32_t FIFO;
} SDIO_reg_t;

typedef struct SPI_reg_struct {
    __IO uint32_t CTL0;
    __IO uint32_t CTL1;
    __IO uint32_t STAT;
    __IO uint32_t DATA;
    __IO uint32_t CRCPOLY;
    __IO uint32_t RCRC;
    __IO uint32_t TCRC;
    __IO uint32_t I2SCTL;
    __IO uint32_t I2SPSC;
    uint32_t RES0[23];
    __IO uint32_t QCTL;
} SPI_reg_t;

typedef struct TIMER_reg_struct {
    __IO uint32_t CTL0;
    __IO uint32_t CTL1;
    __IO uint32_t SMCFG;
    __IO uint32_t DMAINTEN;
    __IO uint32_t INTF;
    __IO uint32_t SWEVG;
    __IO uint32_t CHCTL0;
    __IO uint32_t CHCTL1;
    __IO uint32_t CHCTL2;
    __IO uint32_t CNT;
    __IO uint32_t PSC;
    __IO uint32_t CAR;
    __IO uint32_t CREP;
    __IO uint32_t CH0CV;
    __IO uint32_t CH1CV;
    __IO uint32_t CH2CV;
    __IO uint32_t CH3CV;
    __IO uint32_t CCHP;
    __IO uint32_t DMACFG;
    __IO uint32_t DMATB;
    __IO uint32_t IRMP;
    uint32_t RES0[42];
    __IO uint32_t CFG;
} TIMER_reg_t;

typedef struct USART_reg_struct {
    __IO uint32_t STAT0;
    __IO uint32_t DATA;
    __IO uint32_t BAUD;
    __IO uint32_t CTL0;
    __IO uint32_t CTL1;
    __IO uint32_t CTL2;
    __IO uint32_t GP;
    uint32_t RES0[25];
    __IO uint32_t CTL3;
    __IO uint32_t RT;
    __IO uint32_t STAT1;
} USART_reg_t;

typedef struct WWDGT_reg_struct {
    __IO uint32_t CTL;
    __IO uint32_t CFG;
    __IO uint32_t STAT;
} WWDGT_reg_t;


#define FLASH_BASE              0x08000000UL
#define SRAM_BASE               0x20000000UL
#define PERIPHERAL_BASE         0x40000000UL

#define SRAM_BITBAND_BASE       0x22000000UL
#define PERIPH_BITBAND_BASE     0x42000000UL

/* External Memory Bus and Base Registers */
#define EXMC_BUS_BASE           0x60000000UL        /* EXMC_BUS is an AHB bus for EXMC */
#define EXMC_BASE               0xA0000000UL

/* Peripheral Busses Base Addresses */
#define AHB_BASE            (PERIPHERAL_BASE + 0x00018000UL)
#define APB1_BASE           PERIPHERAL_BASE
#define APB2_BASE           (PERIPHERAL_BASE + 0x00010000UL)

/* Perihperal Base Register Addresses */
#define ADC0_BASE           (APB2_BASE + 0x00002400UL)
#define ADC1_BASE           (APB2_BASE + 0x00002800UL)
#define ADC2_BASE           (APB2_BASE + 0x00003C00UL)
#define AFIO_BASE           (APB2_BASE + 0x00000000UL)
#define BKP_BASE            (APB1_BASE + 0x00006C00UL)
#define CAN0_BASE           (APB1_BASE + 0x00006400UL)
#define CAN1_BASE           (APB1_BASE + 0x00006800UL)
#define CRC_BASE            (AHB_BASE + 0x0000B000UL)
#define CTC_BASE            (APB1_BASE + 0x0000C800UL)
#define DAC_BASE            (APB1_BASE + 0x00007400UL)
#define DBG_BASE            0xE0042000UL
#define DMA0_BASE           (AHB_BASE + 0x00008000UL)
#define DMA1_BASE           (AHB_BASE + 0x00008400UL)
#define EXTI_BASE           (APB2_BASE + 0x00000400UL)
#define FMC_BASE            (AHB_BASE + 0x0000A000UL)
#define FWDGT_BASE          (APB1_BASE + 0x00003000UL)
#define GPIOA_BASE          (APB2_BASE + 0x00000800UL)
#define GPIOB_BASE          (APB2_BASE + 0x00000C00UL)
#define GPIOC_BASE          (APB2_BASE + 0x00001000UL)
#define GPIOD_BASE          (APB2_BASE + 0x00001400UL)
#define GPIOE_BASE          (APB2_BASE + 0x00001800UL)
#define GPIOF_BASE          (APB2_BASE + 0x00001C00UL)
#define GPIOG_BASE          (APB2_BASE + 0x00002000UL)
#define I2C0_BASE           (APB1_BASE + 0x00005400UL)
#define I2C1_BASE           (APB1_BASE + 0x00005800UL)
#define OB_BASE             0x1FFFF800UL
#define PMU_BASE            (APB1_BASE + 0x00007000UL)
#define RCU_BASE            (AHB_BASE + 0x00009000UL)
#define RTC_BASE            (APB1_BASE + 0x00002800UL)
#define SDIO_BASE           (AHB_BASE + 0x00000000UL)
#define SPI0_BASE           (APB2_BASE + 0x00003000UL)
#define SPI1_BASE           (APB1_BASE + 0x00003800UL)
#define SPI2_BASE           (APB1_BASE + 0x00003C00UL)
#define TIMER0_BASE         (APB1_BASE + 0x00012C00UL)
#define TIMER1_BASE         (APB1_BASE + 0x00000000UL)
#define TIMER2_BASE         (APB1_BASE + 0x00000400UL)
#define TIMER3_BASE         (APB1_BASE + 0x00000800UL)
#define TIMER4_BASE         (APB1_BASE + 0x00000C00UL)
#define TIMER5_BASE         (APB1_BASE + 0x00001000UL)
#define TIMER6_BASE         (APB1_BASE + 0x00001400UL)
#define TIMER7_BASE         (APB1_BASE + 0x00013400UL)
#define TIMER8_BASE         (APB1_BASE + 0x00014C00UL)
#define TIMER9_BASE         (APB1_BASE + 0x00015000UL)
#define TIMER10_BASE        (APB1_BASE + 0x00015400UL)
#define TIMER11_BASE        (APB1_BASE + 0x00001800UL)
#define TIMER12_BASE        (APB1_BASE + 0x00001C00UL)
#define TIMER13_BASE        (APB1_BASE + 0x00002000UL)
#define USART0_BASE         (APB2_BASE + 0x00003800UL)
#define USART1_BASE         (APB1_BASE + 0x00004400UL)
#define USART2_BASE         (APB1_BASE + 0x00004800UL)
#define UART3_BASE          (APB1_BASE + 0x00004C00UL)
#define UART4_BASE          (APB1_BASE + 0x00005000UL)
#define USBFS_BASE          (AHB_BASE + 0x0FFE8000UL)
#define WWDGT_BASE          (APB1_BASE + 0x00002C00UL)


#define ADC0        ((ADC_reg_t *)ADC0_BASE)
#define ADC1        ((ADC_reg_t *)ADC1_BASE)
#define ADC2        ((ADC_reg_t *)ADC2_BASE)
#define AFIO        ((AFIO_reg_t *)AFIO_BASE)
#define BKP         ((BKP_reg_t *)BKP_BASE)
#define CAN0        ((CAN_reg_t *)CAN0_BASE)
#define CAN1        ((CAN_reg_t *)CAN1_BASE)
#define CRC         ((CRC_reg_t *)CRC_BASE)
#define CTC         ((CTC_reg_t *)CTC_BASE)
#define DAC         ((DAC_reg_t *)DAC_BASE)
#define DBG         ((DBG_reg_t *)DBG_BASE)
#define DMA0        ((DMA_reg_t *)DMA0_BASE)
#define DMA1        ((DMA_reg_t *)DMA1_BASE)
#define EXTI        ((EXTI_reg_t *)EXTI_BASE)
#define FMC         ((FMC_reg_t *)RMC_BASE)
#define FWDGT       ((FWDGT_reg_t *)FWDGT_BASE)
#define GPIOA       ((GPIO_reg_t *)GPIOA_BASE)
#define GPIOB       ((GPIO_reg_t *)GPIOB_BASE)
#define GPIOC       ((GPIO_reg_t *)GPIOC_BASE)
#define GPIOD       ((GPIO_reg_t *)GPIOD_BASE)
#define GPIOE       ((GPIO_reg_t *)GPIOE_BASE)
#define GPIOF       ((GPIO_reg_t *)GPIOF_BASE)
#define GPIOG       ((GPIO_reg_t *)GPIOG_BASE)
#define I2C0        ((I2C_reg_t *)I2C0_BASE)
#define I2C1        ((I2C_reg_t *)I2C1_BASE)
#define OB          ((OB_reg_t *)OB_BASE)
#define PMU         ((PMU_reg_t *)PMU_BASE)
#define RCU         ((RCU_reg_t *)RCU_BASE)
#define RTC         ((RTC_reg_t *)RTC_BASE)
#define SDIO        ((SDIO_reg_t *)SDIO_BASE)
#define SPI0        ((SPI_reg_t *)SPI0_BASE)
#define SPI1        ((SPI_reg_t *)SPI1_BASE)
#define SPI2        ((SPI_reg_t *)SPI2_BASE)
#define TIMER0      ((TIMER_reg_t *)TIMER0_BASE)
#define TIMER1      ((TIMER_reg_t *)TIMER1_BASE)
#define TIMER2      ((TIMER_reg_t *)TIMER2_BASE)
#define TIMER3      ((TIMER_reg_t *)TIMER3_BASE)
#define TIMER4      ((TIMER_reg_t *)TIMER4_BASE)
#define TIMER5      ((TIMER_reg_t *)TIMER5_BASE)
#define TIMER6      ((TIMER_reg_t *)TIMER6_BASE)
#define TIMER7      ((TIMER_reg_t *)TIMER7_BASE)
#define TIMER8      ((TIMER_reg_t *)TIMER8_BASE)
#define TIMER9      ((TIMER_reg_t *)TIMER9_BASE)
#define TIMER10     ((TIMER_reg_t *)TIMER10_BASE)
#define TIMER11     ((TIMER_reg_t *)TIMER11_BASE)
#define TIMER12     ((TIMER_reg_t *)TIMER12_BASE)
#define TIMER13     ((TIMER_reg_t *)TIMER13_BASE)
#define USART0      ((USART_reg_t *)USART0_BASE)
#define USART1      ((USART_reg_t *)USART1_BASE)
#define USART2      ((USART_reg_t *)USART2_BASE)
#define UART3       ((USART_reg_t *)UART3_BASE)
#define UART4       ((USART_reg_t *)UART4_BASE)
// TODO: Add support for USBFS
//#define USBFS     ((USBFS_reg_t *)USBFS_BASE)
#define WWDGT       ((WWDGT_reg_t *)WWDGT_BASE)

#ifdef __cplusplus
}
#endif

