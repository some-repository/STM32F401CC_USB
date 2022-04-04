#define MCO

#include "stm32f401xc.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h" 

#define USB_OTG_DEV  ((USB_OTG_DeviceTypeDef *)((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))

#define RX_FIFO_SIZE     128
#define TX_FIFO_EP0_SIZE 128
#define TX_FIFO_EP1_SIZE 128
#define TX_FIFO_EP2_SIZE 128
#define TX_FIFO_EP3_SIZE 128

/*
 * Clock on GPIOC and set led pin
 */
void GPIO_config(void)
{
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
}

static void delay (void)
{
    int i = 0;
    for (i = 0; i < 1000000; i++);
    i = 0;
}

void MCO_config (void) //SYSCLK/2 on PA8
{
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed (GPIOA, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetAFPin_8_15 (GPIOA, LL_GPIO_PIN_8, LL_GPIO_AF_0);
    LL_RCC_ConfigMCO (LL_RCC_MCO1SOURCE_PLLCLK, LL_RCC_MCO1_DIV_2); //only MCO1 is available on this MCU
}

void RCC_config (void)
{
    LL_RCC_DeInit ();
    LL_FLASH_SetLatency (LL_FLASH_LATENCY_2);
    #if defined (MCO)
        MCO_config ();
    #endif //MCO

    /* Enable HSE and wait for activation*/
    LL_RCC_HSE_Enable ();
    while (LL_RCC_HSE_IsReady () != 1);

    LL_RCC_PLL_Disable ();
    LL_RCC_PLL_ConfigDomain_SYS (LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 336, LL_RCC_PLLP_DIV_4); //336 is PLLN
    LL_RCC_PLL_ConfigDomain_48M (LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 336, LL_RCC_PLLQ_DIV_7);
    LL_RCC_PLL_Enable ();
    while (LL_RCC_PLL_IsReady () != 1);

    LL_RCC_SetAHBPrescaler (LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler (LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler (LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource (LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource () != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);
    while (LL_RCC_GetUSBClockFreq (LL_RCC_USB_CLKSOURCE) == LL_RCC_PERIPH_FREQUENCY_NO);
}

void USB_config (void)
{
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_12, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed (GPIOA, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinSpeed (GPIOA, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetAFPin_8_15 (GPIOA, LL_GPIO_PIN_11, LL_GPIO_AF_10); //DM
    LL_GPIO_SetAFPin_8_15 (GPIOA, LL_GPIO_PIN_12, LL_GPIO_AF_10); //DP

    LL_AHB2_GRP1_EnableClock (LL_AHB2_GRP1_PERIPH_OTGFS);

    /* Core */
    while (!(USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));
    USB_OTG_FS->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
    while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_CSRST);
    while (!(USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL)); 
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS; // F4 MCU needs to set this bit and turn on the internal pull-up resistor
    USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD | (0x06 << USB_OTG_GUSBCFG_TRDT_Pos) | USB_OTG_GUSBCFG_PHYSEL | (17 << USB_OTG_GUSBCFG_TOCAL_Pos); // Set to Device mode
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_IEPINT |   // Enable USB IN TX endpoint interrupt
                           USB_OTG_GINTMSK_OEPINT |   // Enable USB OUT RX endpoint interrupt
                           USB_OTG_GINTMSK_RXFLVLM |  // USB reciving
                           USB_OTG_GINTMSK_MMISM |    // OTG interrupt
                           USB_OTG_GINTMSK_OTGINT;    // Mode mismatch interrupt
    USB_OTG_FS->GAHBCFG = (USB_OTG_GAHBCFG_GINT | USB_OTG_GAHBCFG_TXFELVL); // GINTMSK = 1 and USB_OTG_GAHBCFG_TXFELVL = 1 (interrupt on completely empty TX buffer)
    /* Device */
    USB_OTG_DEV->DCFG |= USB_OTG_DCFG_NZLSOHSK | USB_OTG_DCFG_DSPD_1 | USB_OTG_DCFG_DSPD_0; //Full speed, STALL for all OUT requests

    /* Buffers */
    USB_OTG_FS->GRXFSIZ = RX_FIFO_SIZE; // size is in 32-bit words !
    USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = (TX_FIFO_EP0_SIZE << 16) | RX_FIFO_SIZE;              // Set the position and size of the transmit buffer
    USB_OTG_FS->DIEPTXF[1] = (TX_FIFO_EP1_SIZE << 16) | (RX_FIFO_SIZE + TX_FIFO_EP0_SIZE); // Set the position and size of the transmit buffer     
    USB_OTG_FS->DIEPTXF[2] = (TX_FIFO_EP2_SIZE << 16) | (RX_FIFO_SIZE + TX_FIFO_EP0_SIZE + TX_FIFO_EP1_SIZE);
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_SOFM |     // Start of frame interrupt
                           USB_OTG_GINTMSK_USBRST |   // Reset interrupt
                           USB_OTG_GINTMSK_ENUMDNEM | // Enumeration done interrupt
                           USB_OTG_GINTMSK_USBSUSPM | // USB suspend interrupt
                           USB_OTG_GINTMSK_ESUSPM;    // Early USB suspend interrupt
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_PWRDWN; // enable USB PHY
    /* Interrupt */
    //NVIC_SetPriority(OTG_FS_IRQn, 1);
    NVIC_EnableIRQ(OTG_FS_IRQn); 
}

void OTG_FS_IRQHandler (void)
{   
    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST)   // Reset Interrupt
    {
        USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_USBRST; // Clear the flag by writing 1
        //LL_GPIO_SetOutputPin (GPIOC, LL_GPIO_PIN_13);
    }
    if(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_ENUMDNE)
    { 
        USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_ENUMDNE; // Clear the flag by writing 1
        LL_GPIO_SetOutputPin (GPIOC, LL_GPIO_PIN_13); 
    }
}

int main (void)
{
    RCC_config ();
    GPIO_config ();
    USB_config ();

    while (1) 
    {
        /*LL_GPIO_SetOutputPin (GPIOC, LL_GPIO_PIN_13);
        delay ();
        LL_GPIO_ResetOutputPin (GPIOC, LL_GPIO_PIN_13);
        delay ();*/
    }
}
