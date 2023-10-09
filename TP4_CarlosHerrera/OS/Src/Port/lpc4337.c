#ifdef LPC4337

#include "lpc4337.h"


osIRQVector irqVector[IRQ_NUMBER] = { 0 };

/* Handler interrupt */
void DAC_IRQHandler(void)           {   osIRQHandler(DAC_IRQn);         }
void M0APP_IRQHandler(void)         {   osIRQHandler(M0APP_IRQn);       }
void DMA_IRQHandler(void)           {   osIRQHandler(DMA_IRQn);         }
void FLASH_EEPROM_IRQHandler(void)  {   osIRQHandler(RESERVED1_IRQn);   }
void ETH_IRQHandler(void)           {   osIRQHandler(ETHERNET_IRQn);    }
void SDIO_IRQHandler(void)          {   osIRQHandler(SDIO_IRQn);        }
void LCD_IRQHandler(void)           {   osIRQHandler(LCD_IRQn);         }
void USB0_IRQHandler(void)          {   osIRQHandler(USB0_IRQn);        }
void USB1_IRQHandler(void)          {   osIRQHandler(USB1_IRQn);        }
void SCT_IRQHandler(void)           {   osIRQHandler(SCT_IRQn);         }
void RIT_IRQHandler(void)           {   osIRQHandler(RITIMER_IRQn);     }
void TIMER0_IRQHandler(void)        {   osIRQHandler(TIMER0_IRQn);      }
void TIMER1_IRQHandler(void)        {   osIRQHandler(TIMER1_IRQn);      }
void TIMER2_IRQHandler(void)        {   osIRQHandler(TIMER2_IRQn);      }
void TIMER3_IRQHandler(void)        {   osIRQHandler(TIMER3_IRQn);      }
void MCPWM_IRQHandler(void){osIRQHandler(       MCPWM_IRQn       );}
void ADC0_IRQHandler(void){osIRQHandler(        ADC0_IRQn        );}
void I2C0_IRQHandler(void){osIRQHandler(        I2C0_IRQn        );}
void SPI_IRQHandler(void){osIRQHandler(         I2C1_IRQn        );}
void I2C1_IRQHandler(void){osIRQHandler(        SPI_INT_IRQn     );}
void ADC1_IRQHandler(void){osIRQHandler(        ADC1_IRQn        );}
void SSP0_IRQHandler(void){osIRQHandler(        SSP0_IRQn        );}
void SSP1_IRQHandler(void){osIRQHandler(        SSP1_IRQn        );}
void UART0_IRQHandler(void){osIRQHandler(       USART0_IRQn      );}
void UART1_IRQHandler(void){osIRQHandler(       UART1_IRQn       );}
void UART2_IRQHandler(void){osIRQHandler(       USART2_IRQn      );}
void UART3_IRQHandler(void){osIRQHandler(       USART3_IRQn      );}
void I2S0_IRQHandler(void){osIRQHandler(        I2S0_IRQn        );}
void I2S1_IRQHandler(void){osIRQHandler(        I2S1_IRQn        );}
void SPIFI_IRQHandler(void){osIRQHandler(       RESERVED4_IRQn   );}
void SGPIO_IRQHandler(void){osIRQHandler(       SGPIO_INT_IRQn   );}
void GPIO0_IRQHandler(void){osIRQHandler(       PIN_INT0_IRQn    );}
void GPIO1_IRQHandler(void){osIRQHandler(       PIN_INT1_IRQn    );}
void GPIO2_IRQHandler(void){osIRQHandler(       PIN_INT2_IRQn    );}
void GPIO3_IRQHandler(void){osIRQHandler(       PIN_INT3_IRQn    );}
void GPIO4_IRQHandler(void){osIRQHandler(       PIN_INT4_IRQn    );}
void GPIO5_IRQHandler(void){osIRQHandler(       PIN_INT5_IRQn    );}
void GPIO6_IRQHandler(void){osIRQHandler(       PIN_INT6_IRQn    );}
void GPIO7_IRQHandler(void){osIRQHandler(       PIN_INT7_IRQn    );}
void GINT0_IRQHandler(void){osIRQHandler(       GINT0_IRQn       );}
void GINT1_IRQHandler(void){osIRQHandler(       GINT1_IRQn       );}
void EVRT_IRQHandler(void){osIRQHandler(        EVENTROUTER_IRQn );}
void CAN1_IRQHandler(void){osIRQHandler(        C_CAN1_IRQn      );}
void ADCHS_IRQHandler(void){osIRQHandler(       ADCHS_IRQn       );}
void ATIMER_IRQHandler(void){osIRQHandler(      ATIMER_IRQn      );}
void RTC_IRQHandler(void){osIRQHandler(         RTC_IRQn         );}
void WDT_IRQHandler(void){osIRQHandler(         WWDT_IRQn        );}
void M0SUB_IRQHandler(void){osIRQHandler(       M0SUB_IRQn       );}
void CAN0_IRQHandler(void){osIRQHandler(        C_CAN0_IRQn      );}
void QEI_IRQHandler(void){osIRQHandler(         QEI_IRQn         );}

void osIRQHandler(osIRQnType irqType)
{
}

#endif // LPC4337

