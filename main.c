#define MCO

#include "stm32f401xc.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include <stddef.h> 

#define USB_OTG_DEV  ((USB_OTG_DeviceTypeDef *) ((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))
#define USB_INEP(i)  ((USB_OTG_INEndpointTypeDef *) (( uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (i) * USB_OTG_EP_REG_SIZE))
#define USB_OUTEP(i) ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (i) * USB_OTG_EP_REG_SIZE))
#define USB_FIFO(i)  *(volatile uint32_t *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + ((i) * USB_OTG_FIFO_SIZE))

#define RX_FIFO_SIZE     80 // size is in 32-bit words
#define TX_FIFO_EP0_SIZE 80 // sum of all FIFO sizes is not grater than 320 words
#define TX_FIFO_EP1_SIZE 80
#define TX_FIFO_EP2_SIZE 80

uint8_t bufRX [72] = {0};

void send_ep (const uint8_t ep, const uint8_t *buf, const uint8_t len);
void read_ep (const uint8_t ep, uint8_t *buf, const uint8_t len);
void USB_config (void);
void RCC_config (void);
void MCO_config (void);
void GPIO_config(void);
void USB_device_setup (uint8_t *buf);

/*
 * Clock on GPIOC and set led pin
 */
void GPIO_config(void)
{
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
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
    USB_OTG_FS->GINTSTS = 0; // clear OTG_FS_GINTSTS register at initialization before unmasking the interrupt bits
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_IEPINT |   // Enable USB IN TX endpoint interrupt
                           USB_OTG_GINTMSK_OEPINT |   // Enable USB OUT RX endpoint interrupt
                           USB_OTG_GINTMSK_RXFLVLM |  // USB recieving
                           USB_OTG_GINTMSK_MMISM |    // OTG interrupt
                           USB_OTG_GINTMSK_OTGINT;    // Mode mismatch interrupt
    USB_OTG_FS->GAHBCFG = (USB_OTG_GAHBCFG_GINT | USB_OTG_GAHBCFG_TXFELVL); // GINTMSK = 1 and USB_OTG_GAHBCFG_TXFELVL = 1 (interrupt on completely empty TX buffer)
    /* Device */
    USB_OTG_DEV->DCFG |= USB_OTG_DCFG_NZLSOHSK | USB_OTG_DCFG_DSPD_1 | USB_OTG_DCFG_DSPD_0; //Full speed, STALL for all OUT requests

    /* Buffers */
    
    
    //USB_OTG_FS->DIEPTXF[1] = (TX_FIFO_EP1_SIZE << 16) | (RX_FIFO_SIZE + TX_FIFO_EP0_SIZE); // Set the position and size of the transmit buffer     
    //USB_OTG_FS->DIEPTXF[2] = (TX_FIFO_EP2_SIZE << 16) | (RX_FIFO_SIZE + TX_FIFO_EP0_SIZE + TX_FIFO_EP1_SIZE);
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_SOFM |     // Start of frame interrupt
                           USB_OTG_GINTMSK_USBRST |   // Reset interrupt
                           USB_OTG_GINTMSK_ENUMDNEM | // Enumeration done interrupt
                           USB_OTG_GINTMSK_USBSUSPM | // USB suspend interrupt
                           USB_OTG_GINTMSK_ESUSPM;    // Early USB suspend interrupt
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_PWRDWN; // enable USB PHY
    /* Interrupt */
    //NVIC_SetPriority(OTG_FS_IRQn, 1);
    NVIC_EnableIRQ (OTG_FS_IRQn);
    USB_INEP(0)->DIEPCTL &= 0xFFFFFFFC; // (reset bits 0 and 1) 64 bytes for full speed 
}

void OTG_FS_IRQHandler (void)
{   
    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST)   // Reset Interrupt
    {
        USB_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK; // Set the NAK bit for all OUT endpoints
        USB_OUTEP(1)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
        USB_OUTEP(2)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
        USB_OUTEP(3)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;

        USB_OTG_DEV->DAINTMSK |= 0x10001U;              // EP0 OUT, EP0 IN Interupt
        USB_OTG_DEV->DOEPMSK |= USB_OTG_DOEPMSK_STUPM | // Enable setup-done interrupt
                                USB_OTG_DOEPMSK_XFRCM;  // Enable tx-done interrupt                                   
        USB_OTG_DEV->DIEPMSK |= USB_OTG_DIEPMSK_TOM   | // Timeout irq
                                USB_OTG_DIEPMSK_XFRCM;
        // buffers
        USB_OTG_FS->GRXFSIZ = RX_FIFO_SIZE; // size is in 32-bit words
        USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = (TX_FIFO_EP0_SIZE << 16) | RX_FIFO_SIZE; // Set the position and size of the EP0 transmit buffer
        USB_OUTEP(0)->DOEPTSIZ = (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos) |
                             USB_OTG_DOEPTSIZ_STUPCNT | (3 * 8); // Allow 3 setup packets of 8 bytes                                  
        USB_OTG_DEV->DCFG &= ~USB_OTG_DCFG_DAD;         // Clear address    
        USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_USBRST;   // Clear flag
        USB_OUTEP(0)->DOEPCTL = USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK; // Enable endpoint, Clear NAK bit

        USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_USBRST; // Clear the flag by writing 1
        //LL_GPIO_SetOutputPin (GPIOC, LL_GPIO_PIN_13);
    }

    if(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_ENUMDNE)
    { 
        USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_ENUMDNE; // Clear the flag by writing 1
        /*if ((USB_OTG_DEV->DSTS & (USB_OTG_DSTS_ENUMSPD_0 | USB_OTG_DSTS_ENUMSPD_1)) == 6) //check if full speed
            {
                LL_GPIO_SetOutputPin (GPIOC, LL_GPIO_PIN_13); 
            }*/
    }

    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL) // there is at least one packet pending to be read from the RxFIFO
    {
        USB_OTG_FS->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM; // Mask the RXFLVL interrupt until reading the packet from the receive FIFO

        uint32_t grxstsp = USB_OTG_FS->GRXSTSP;                                              // Rx packet status register
        uint16_t bcnt = ((grxstsp & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_Pos);      // BCNT (length)
        uint8_t pktsts = ((grxstsp & USB_OTG_GRXSTSP_PKTSTS) >> USB_OTG_GRXSTSP_PKTSTS_Pos); // Packet status
        uint8_t dpid = ((grxstsp & USB_OTG_GRXSTSP_DPID) >> USB_OTG_GRXSTSP_DPID_Pos);       // Data PID
        uint8_t epnum = ((grxstsp & USB_OTG_GRXSTSP_EPNUM) >> USB_OTG_GRXSTSP_EPNUM_Pos);    // Indicates EP number to which the current received packet belongs
        if (bcnt != 0) // Reading an empty receive FIFO can result in undefined core behavior
        {
            switch (epnum) 
            {
                case 0:
                {
                    if ((pktsts == 0x06) && (bcnt == 0x8) && (dpid == 0)) // setup packet received
                    {
                        read_ep (epnum, bufRX, bcnt); // Read setup packet
                    }
                    break;
                }
            
                /*if (pktsts == 0x06) // if setup packet
                {   
                    
                    const uint8_t bRequest = bufRX [1];
                    USB_OTG_DEV->DCFG |= (((uint32_t) bufRX [2]) << 4);
                    send_ep (0, 0, 0); // send zero length packet 
                    LL_GPIO_SetOutputPin (GPIOC, LL_GPIO_PIN_13);                
                }*/
                default: // wrong EP number
                    break;
            } 
        }

        USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM; // Unmask the RXFLVL interrupt after reading the packet from the receive FIFO  
    }

    if(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_OEPINT) // OUT -> RX endpoint interrupt
    {         
        uint32_t epNum; 
        uint32_t epInt;
        epNum = USB_OTG_DEV->DAINT;
        epNum &= USB_OTG_DEV->DAINTMSK;
    }
}

void read_ep (const uint8_t ep, uint8_t *buf, const uint8_t len)
{
    int16_t i; 
    uint32_t word;
    for(i = 0; i < ((len + 3) / 4); i++)
        {
            word = USB_FIFO(ep); //read 32 bit word from RX FIFO
            buf [4 * i] = (uint8_t) (word & 0xFF);
            buf [(4 * i) + 1] = (uint8_t) ((word & 0xFF00) >> 8);
            buf [(4 * i) + 2] = (uint8_t) ((word & 0xFF0000) >> 16);
            buf [(4 * i) + 3] = (uint8_t) ((word & 0xFF000000) >> 24);
        }
}

void send_ep (const uint8_t ep, const uint8_t *buf, const uint8_t len)
{
    if ((len != 0) && (buf == NULL))
    {
        return;
    }
    uint16_t i;
    USB_INEP(ep)->DIEPTSIZ = (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos) | len;       // Set outbound txlen
    USB_INEP(ep)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;   // Enable endpoint, clear NAK bit     
    for(i = 0; i < ((len + 3) / 4); i++) 
        {
            USB_FIFO(ep) = (((uint32_t) buf [i]) | (((uint32_t) bufRX [i+1]) << 8) | (((uint32_t) bufRX [i+2]) << 16) | (((uint32_t) bufRX [i+3]) << 24));                      // Copy data 
        } 
}

void USB_device_setup (uint8_t *buf)
{
    const uint8_t bmRequestType = buf[0];
    const uint8_t bRequest = buf[1];
    const uint16_t wValue = buf[2] | ((uint16_t)buf[3] << 8);
    const uint16_t wIndex = buf[4] | ((uint16_t)buf[5] << 8);  
    const uint16_t wLength = buf[6] | ((uint16_t)buf[7] << 8); 
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
