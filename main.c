//#define MCO

#include "stm32f401xc.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include <stddef.h> 
#include "usb.h"

void RCC_config (void);
void MCO_config (void);
void GPIO_config (void);
void UART_config (void);

void GPIO_config(void) // Clock on GPIOC and set LED pin (PC13)
{
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
}

void MCO_config (void) // SYSCLK/2 on PA8
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

void UART_config (void)
{
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock (LL_APB2_GRP1_PERIPH_USART1);
    // PA9 (TX1)
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed (GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetAFPin_8_15 (GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_7);

    LL_USART_SetDataWidth (USART1, LL_USART_DATAWIDTH_8B);
    LL_USART_SetParity (USART1, LL_USART_PARITY_NONE);
    LL_USART_SetOverSampling (USART1, LL_USART_OVERSAMPLING_16);
    LL_USART_SetStopBitsLength (USART1, LL_USART_STOPBITS_1);
    LL_USART_SetBaudRate (USART1, 84000000, LL_USART_OVERSAMPLING_16, UART_SPEED);
    LL_USART_SetTransferDirection (USART1, LL_USART_DIRECTION_TX);
    LL_USART_SetHWFlowCtrl (USART1, LL_USART_HWCONTROL_NONE);
    LL_USART_ConfigAsyncMode (USART1);
    LL_USART_Enable (USART1);
}

void OTG_FS_IRQHandler (void)
{   
    LL_GPIO_ResetOutputPin (GPIOC, LL_GPIO_PIN_13);
    print ("interrupt entry");
    
    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST)   // Reset Int
    {
        print (" > USBRST");        
        USB_RST_interrupt_handler ();
    }

    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_ENUMDNE)
    { 
        print (" > ENUMDNE");
        USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_ENUMDNE; // Clear the flag 
        USB_INEP(0)->DIEPCTL &= 0xFFFFFFFC; // reset bits 0 and 1 to set maximum packet size of MAX_PACKET_SIZE_EP0 = 64 bytes for EP0 TX&RX 
    }

    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL) // there is at least one packet pending to be read from the RxFIFO
    {
        print (" > RXFLVL");
        USB_OTG_FS->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM; // Mask the RXFLVL interrupt until reading the packet from the receive FIFO is done

        uint32_t grxstsp = USB_OTG_FS->GRXSTSP;                                              // Rx packet status register
        uint16_t bcnt = ((grxstsp & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_Pos);      // BCNT (length)
        uint8_t pktsts = ((grxstsp & USB_OTG_GRXSTSP_PKTSTS) >> USB_OTG_GRXSTSP_PKTSTS_Pos); // Packet status
        uint8_t dpid = ((grxstsp & USB_OTG_GRXSTSP_DPID) >> USB_OTG_GRXSTSP_DPID_Pos);       // Data PID
        uint8_t epnum = ((grxstsp & USB_OTG_GRXSTSP_EPNUM) >> USB_OTG_GRXSTSP_EPNUM_Pos);    // Indicates EP number to which the current received packet belongs

        if (bcnt != 0) // Reading an empty receive FIFO can result in undefined core behavior
        {
            if (pktsts == DATA)              // Data
            {                        
                print (" > DATA packet received"); 
                read_ep (epnum, &bufRx [countRx], bcnt); // Read data
                send_ep (epnum, &bufRx [countRx], bcnt); // Echo data back
                print (&bufRx [countRx]);
                epNumLastRx = epnum; // Save last endpoint num
                //countRx += bcnt;     // Save all len
            }
            else if ((pktsts == SETUP) && (bcnt == 0x8) && (dpid == 0) && (epnum == 0))              // Setup data packet received
            {    
                print (" > SETUP packet received");                    // Read setup packet
                read_ep (epnum, bufRx, bcnt);
            }
        }
        if (pktsts == SETUP_Done)
        {
            print (" > SETUP done packet received");
        }

        USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM; // Unmask the RXFLVL interrupt after reading the packet from the receive FIFO
    }

    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_OEPINT) // OUT -> RX endpoint interrupt
    {         
        print (" > OEPINT");
        uint32_t epNum; 
        uint32_t epInt;
        epNum = USB_OTG_DEV->DAINT;
        epNum &= USB_OTG_DEV->DAINTMSK;
        if (epNum & EP0_OUT_INT)                       // EP0 OUT RX Interrupt
        {      
            epInt = USB_OUTEP(0)->DOEPINT;
            epInt &= USB_OTG_DEV->DOEPMSK;
            if (epInt & USB_OTG_DOEPINT_STUP)
            {
                print (" > STUP");
                setup (bufRx);                            // Parse setup packet
            }
            USB_OUTEP(0)->DOEPINT = epInt;              // Clear flag
            USB_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA; // Enable endpoint, Clear NAK bit
        }   
        if (epNum & EP1_OUT_INT)                        // EP1 OUT RX Interrupt
        {
            epInt = USB_OUTEP(1)->DOEPINT;
            epInt &= USB_OTG_DEV->DOEPMSK;
            USB_OUTEP(1)->DOEPINT = epInt;               // Clear flag
            USB_OUTEP(1)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA; // Enable endpoint, Clear NAK bit
        }               
    }  

    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_IEPINT)   // IN -> TX endpoint interrupt
    {     
        print (" > IEPINT");
        uint32_t epNum;
        uint32_t epInt;      
        epNum = USB_OTG_DEV->DAINT;
        epNum &= USB_OTG_DEV->DAINTMSK;
        if (epNum & EP0_IN_INT)                          // EP0 IN TX Interrupt
        {
            epInt = USB_INEP(0)->DIEPINT;
            epInt &= USB_OTG_DEV->DIEPMSK;
            if (epInt & USB_OTG_DIEPINT_XFRC)
            {
                print (" > Transfer finished on EP0");
                //countTx = 0;
            }
            USB_INEP(0)->DIEPINT = epInt;                // Clear flag              
        }
        if (epNum & EP1_IN_INT)                          // EP1 IN TX Interrupt
        {
            epInt = USB_INEP(1)->DIEPINT;
            epInt &= USB_OTG_DEV->DIEPMSK;
            if (epInt & USB_OTG_DIEPINT_XFRC)
            {
                print (" > Transfer finished on EP1");
                countTx = 0;
            }
            USB_INEP(1)->DIEPINT = epInt;                // Clear flag             
        }                 
    }
    
    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_MMIS)
    {
        print (" > MMIS");
        USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_MMIS;
    }  
    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_SOF)
    {     
        print (" > SOF");
        USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_SOF;     // Clear the flag (rc_w1)
    }    
    
    print (" > interrupt exit\n");
    LL_GPIO_SetOutputPin (GPIOC, LL_GPIO_PIN_13);
}

int main (void)
{
    RCC_config ();
    GPIO_config ();
    UART_config ();
    USB_config ();
    print ("\033[H\033[2J\033[3J"); // Clear screen
    print ("print test\n");

    while (1) 
    {

    }
}
