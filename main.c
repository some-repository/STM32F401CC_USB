//#define MCO

#include "stm32f401xc.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include <stddef.h> 
#include "usb.h"

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
    }

    if(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_ENUMDNE)
    { 
        USB_INEP(0)->DIEPCTL &= 0xFFFFFFFC; // reset bits 0 and 1 to set maximum packet size of MAX_PACKET_SIZE_EP0 = 64 bytes for EP0 TX&RX  
        USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_ENUMDNE; // Clear the flag by writing 1
    }

    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL) // there is at least one packet pending to be read from the RxFIFO
    {
        USB_OTG_FS->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM; // Mask the RXFLVL interrupt until reading the packet from the receive FIFO is done

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
                    if ((pktsts == SETUP) && (bcnt == 0x8) && (dpid == 0)) // setup packet received
                    {
                        read_ep (epnum, bufRX, bcnt); // Read setup packet
                    }
                    else if (pktsts == SETUP_Done)
                    {

                    }
                    break;
                }
                default: // wrong EP number
                    break;
            } 
        }

        USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM; // Unmask the RXFLVL interrupt after reading the packet from the receive FIFO  
    }

    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_OEPINT) // OUT -> RX endpoint interrupt
    {         
        uint32_t epNum; 
        uint32_t epInt;

        epNum = USB_OTG_DEV->DAINT;
        epNum &= USB_OTG_DEV->DAINTMSK;

        if (epNum & EP0_OUT_INT) // EP0 OUT RX Interrupt
        {      
            epInt = USB_OUTEP(0)->DOEPINT;
            epInt &= USB_OTG_DEV->DOEPMSK;

            if (epInt & USB_OTG_DOEPINT_STUP) // On this interrupt, the application can decode the received SETUP data packet.
            {
                USB_device_setup (bufRX); // Decode setup packet
            }

            USB_OUTEP(0)->DOEPINT = epInt; // Clear interrupt flags in DOEPINT register
            USB_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA; // Enable endpoint, Clear NAK bit
        }   

        if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_IEPINT) // OUT -> TX endpoint interrupt
        {
            uint32_t epNum; 
            uint32_t epInt;

            epNum = USB_OTG_DEV->DAINT;
            epNum &= USB_OTG_DEV->DAINTMSK;

            if (epNum & EP0_OUT_INT) // EP0 OUT RX Interrupt
            {      
                epInt = USB_INEP(0)->DIEPINT;
                epInt &= USB_OTG_DEV->DIEPMSK;

                USB_INEP(0)->DIEPINT = epInt; // Clear interrupt flags in DOEPINT register
            }   
        }

        if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_MMIS)
        {
            USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_MMIS; // Clear the flag (rc_w1)
        }  

        if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_SOF) 
        {     
            USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_SOF; // Clear the flag (rc_w1)
        }    
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
    if (len > MAX_PACKET_SIZE_EP0) // check if len <= MAX_PACKET_SIZE_EP0
    {
        return;
    }

    while ((((USB_INEP(ep)->DTXFSTS) & 0xFFFF) * 4) < (uint32_t) len); // wait until there is enough space in TX FIFO

    uint16_t i;
    USB_INEP(ep)->DIEPTSIZ = (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos) | len;       // one packet of len bytes
    USB_INEP(ep)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;   // Enable endpoint, clear NAK bit     
    for (i = 0; i < ((len + 3) / 4); i++) 
        {
            USB_FIFO(ep) = (((uint32_t) buf [i]) | (((uint32_t) buf [i+1]) << 8) | (((uint32_t) buf [i+2]) << 16) | (((uint32_t) buf [i+3]) << 24));                      // Copy data 
        } 
}

void stall_TX_ep (uint8_t ep)
{
    USB_INEP(ep)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
}

void USB_device_setup (uint8_t *buf)
{
    const uint8_t bmRequestType = buf [0];
    const uint8_t bRequest = buf [1];
    const uint16_t wValue = buf [2] | ((uint16_t) buf [3] << 8);
    const uint16_t wIndex = buf [4] | ((uint16_t) buf [5] << 8);  
    const uint16_t wLength = buf [6] | ((uint16_t) buf [7] << 8);

    switch (bmRequestType & REQUEST_RECIPIENT_MASK)
    {
        case RECIPIENT_DEVICE: // Recipient is device
        {
            switch (bmRequestType & REQUEST_TYPE_MASK)
            {
                case REQUEST_CLASS:
                case REQUEST_VENDOR:
                    break;
                case REQUEST_STANDARD:
                switch (bRequest)
                {
                    case GET_DESCRIPTOR:
                    {
                        get_descriptor (wValue, wLength);
                        break;
                    }                               
                    case SET_ADDRESS:
                    {
                        set_address (buf [2]);                                                     
                        break;
                    }
                    case SET_CONFIGURATION:
                    {
                        //setConfig ();
                        break;
                    }
                    case GET_CONFIGURATION:
                        break;
                    case GET_STATUS:
                        break;
                    case SET_FEATURE:
                        break;
                    case CLEAR_FEATURE:
                        break;
                    default:
                        break;
                }
            }
        }
        case RECIPIENT_ENDPOINT:
            break;
        case RECIPIENT_INTERFACE:
            break;
        default:
            break;
    }
}

void set_address (uint8_t address)
{
    USB_OTG_DEV->DCFG |= ((uint32_t) address << 4);
    send_ep (0, 0, 0); 
}

void get_descriptor (uint16_t wValue, uint16_t wLength)
{
    uint8_t *pbuf;
    uint8_t len = 0;
    switch (wValue >> 8)    
    {
        case DESC_DEVICE: // Request device descriptor
        {
            pbuf = (uint8_t*) desc_device; 
            len = 18;             
            break;  
        }
                         
        case DESC_CONFIG: // Request configuration descriptor
        {
            pbuf = (uint8_t*) desc_config;
            len = sizeof (desc_config);
            break;  
        }
                          
        case DESC_STRING: // Request string descriptor
        {
            switch (wValue & 0xFF) // Request string descriptor
            {
                case DESC_STR_LANGID:  // Lang
                {
                    pbuf = (uint8_t*) desc_lang;
                    len = sizeof (desc_lang);   
                    break;
                }
                                                                  
                case DESC_STR_MFC:     // Manufacturer
                {
                    /*pbuf = (uint8_t*) desc_vendor; 
                    len = sizeof (desc_vendor); */
                    break;
                }
                    
                case DESC_STR_PRODUCT: // Product
                {
                    /*pbuf = (uint8_t*) desc_product; 
                    len = sizeof (desc_product);*/
                    break;
                }
                    
                case DESC_STR_SERIAL:  // SerialNumber
                {
                    /*pbuf = (uint8_t*) desc_serial;
                    len = sizeof (desc_serial); */
                    break;
                }
                    
                case DESC_STR_CONFIG:  // Config
                {
                    pbuf = (uint8_t*) desc_config;
                    len = sizeof (desc_config);
                    break;
                }
                    
                case DESC_STR_INTERFACE:// Interface
                    break;
                default:
                    break;
            }
            break;   
        }

        default:
        {
            stall_TX_ep (0); // we don't know how to handle this request                      
            break;  
        }    
    }
    if (len)
    {    
        send_ep (0, pbuf, MIN(len, wLength));
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

    }
}
