#include "stm32f401xc.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h" // it's for UART debug print
#include <stddef.h> 
#include <stdlib.h> // for itoa
#include "usb.h"

uint8_t bufRX [MAX(MAX_PACKET_SIZE_EP0, MAX_PACKET_SIZE_EP1)] = {0};
uint8_t bufTX [MAX(MAX_PACKET_SIZE_EP0, MAX_PACKET_SIZE_EP1)] = {0};

const uint8_t desc_device [] =
{
    0x12,                       /* bLength */
    0x01,                       /* bDescriptorType */
    0x10,                       /* bcdUSB, 1.1 */
    0x01,
    0xFF,                       /* bDeviceClass */
    0x80,                       /* bDeviceSubClass */
    0x55,                       /* bDeviceProtocol */
    0x40,                       /* bMaxPacketSize */
    0x09,                       /* idVendor, LOBYTE(USBD_VID) */ // VID = 0x1209, see pid.codes
    0x12,                       /* idVendor, HIBYTE(USBD_VID) */
    0x01,                       /* idProduct, LOBYTE(USBD_PID) */ // PID = 0x0001, see pid.codes
    0x00,                       /* idProduct, HIBYTE(USBD_PID) */
    0x00,                       /* bcdDevice rel. 1.00 */
    0x01,
    0x00,                       /* Index of manufacturer string */
    0x00,                       /* Index of product string */
    0x00,                       /* Index of serial number string */
    1                           /* bNumConfigurations */
};

const uint8_t desc_config []=
{
    /* Configuration Descriptor */
    0x09,                                       /* bLength: Configuration Descriptor size */
    0x02,                                       /* bDescriptorType: Configuration */
    0x20,                                       /* wTotalLength:no of returned bytes */
    0x00,
    0x01,                                       /* bNumInterfaces: 1 interface */
    0x01,                                       /* bConfigurationValue: Configuration value */
    0x00,                                       /* iConfiguration: Index of string descriptor describing the configuration */
    0x80,                                       /* bmAttributes: Bus Powered according to user configuration */
    0x32,                                       /* MaxPower 100 mA */
    /*---------------------------------------------------------------------------*/
 
    /* Interface Descriptor */
    0x09,                                       /* bLength: Interface Descriptor size */
    0x04,                                       /* bDescriptorType: Interface */
    0x00,                                       /* bInterfaceNumber: Number of Interface */
    0x00,                                       /* bAlternateSetting: Alternate setting */
    0x02,                                       /* bNumEndpoints: 2 endpoints used */
    0xFF,                                       /* bInterfaceClass: Custom Interface Class */
    0x80,                                       /* bInterfaceSubClass: Custom */
    0x55,                                       /* bInterfaceProtocol: Custom */
    0x00,                                       /* iInterface: */
 
    /* Endpoint 1 IN Descriptor */
    0x07,                                       /* bLength: Endpoint Descriptor size */
    0x05,                                       /* bDescriptorType: Endpoint */
    0x81,                                       /* bEndpointAddress */
    0x02,                                       /* bmAttributes: Bulk */
    MAX_PACKET_SIZE_EP1,                                        /* wMaxPacketSize: */
    0x00,
    0x00,                                       /* bInterval: */
 
    /* Endpoint 1 OUT Descriptor */
    0x07,                                       /* bLength: Endpoint Descriptor size */
    0x05,                                       /* bDescriptorType: Endpoint */
    0x01,                                       /* bEndpointAddress */
    0x02,                                       /* bmAttributes: Bulk */
    MAX_PACKET_SIZE_EP1,                                        /* wMaxPacketSize: */
    0x00,
    0x00                                        /* bInterval: */
};

const uint8_t desc_lang[] =
{
    0x04, // descriptor length
    0x03, // descriptor type - string desc
    0x09, // N bytes of LangID
    0x04  // LangID = 0x0409: U.S. English
};

void USB_config (void)
{
    // GPIO
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_12, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed (GPIOA, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinSpeed (GPIOA, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetAFPin_8_15 (GPIOA, LL_GPIO_PIN_11, LL_GPIO_AF_10); //DM
    LL_GPIO_SetAFPin_8_15 (GPIOA, LL_GPIO_PIN_12, LL_GPIO_AF_10); //DP

    // USB clock enable
    LL_AHB2_GRP1_EnableClock (LL_AHB2_GRP1_PERIPH_OTGFS);

    // USB core
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
                           USB_OTG_GINTMSK_MMISM;    // Mode mismatch interrupt
                           /*USB_OTG_GINTMSK_OTGINT*/;  // OTG interrupt  
    USB_OTG_FS->GAHBCFG = USB_OTG_GAHBCFG_GINT; // GINTMSK = 1
    // Device
    USB_OTG_DEV->DCFG |= USB_OTG_DCFG_NZLSOHSK | USB_OTG_DCFG_DSPD_1 | USB_OTG_DCFG_DSPD_0; //Full speed, STALL for all OUT requests

    USB_OTG_FS->GINTMSK |= /*USB_OTG_GINTMSK_SOFM | */    // Start of frame interrupt
                           USB_OTG_GINTMSK_USBRST |   // Reset interrupt
                           USB_OTG_GINTMSK_ENUMDNEM; /* | // Enumeration done interrupt
                           USB_OTG_GINTMSK_USBSUSPM | // USB suspend interrupt
                           USB_OTG_GINTMSK_ESUSPM |    // Early USB suspend interrupt
                           USB_OTG_GINTMSK_GINAKEFFM | // Global non-periodic IN NAK effective interrupt
                           USB_OTG_GINTMSK_GONAKEFFM; */ // Global non-periodic OUT NAK effective interrupt
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_PWRDWN; // enable USB PHY
    // Interrupt
    NVIC_SetPriority(OTG_FS_IRQn, 1);
    NVIC_EnableIRQ (OTG_FS_IRQn);
}

void read_ep (const uint8_t ep, uint8_t *buf, const size_t len)
{
    uint16_t i; 
    uint32_t word;
    for (i = 0; i < ((len + 3) / 4); i++)
    {
        word = USB_FIFO(ep); //read 32 bit word from RX FIFO
        buf [4 * i] = (uint8_t) (word & 0xFF);
        buf [(4 * i) + 1] = (uint8_t) ((word & 0xFF00) >> 8);
        buf [(4 * i) + 2] = (uint8_t) ((word & 0xFF0000) >> 16);
        buf [(4 * i) + 3] = (uint8_t) ((word & 0xFF000000) >> 24);
    }
}

void send_ep (const uint8_t ep, const uint8_t *buf, const size_t len)
{
    if ((len != 0) && (buf == NULL))
    {
        return;
    }
    if (len > MAX_PACKET_SIZE_EP0) // check if len <= MAX_PACKET_SIZE_EP0
    {
        return;
    }

    while ((((USB_INEP(ep)->DTXFSTS) & 0xFFFF) * 4) < len); // wait until there is enough space in TX FIFO

    size_t i, j;
    uint32_t tmp = 0;
    size_t current_countTX = ((USB_INEP(ep)->DIEPTSIZ) >> USB_OTG_DIEPTSIZ_PKTCNT_Pos);
    USB_INEP(ep)->DIEPTSIZ = ((current_countTX + 1) << USB_OTG_DIEPTSIZ_PKTCNT_Pos) | len; // one packet of len bytes
    USB_INEP(ep)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK; // Enable endpoint, clear NAK bit     
    for (i = 0; i < ((len + 3) / 4); i++) 
    {
        tmp = 0;
        for (j = 0; j < 4; j++)
        {
            if (((i * 4) + j) < len)
            {
                tmp |= (((uint32_t) *(buf + (4 * i) + j)) << (8 * j));
            }
        }
        USB_FIFO(ep) = tmp; // Copy data 
    }
    print (">send_ep: PKTCNT'=");
    char buf_itoa [4] = {0};
    print (itoa (((USB_INEP(ep)->DIEPTSIZ) >> USB_OTG_DIEPTSIZ_PKTCNT_Pos), buf_itoa, 10));
    print (">");
}

void stall_TX_ep (uint8_t ep)
{
    USB_INEP(ep)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
    print ("stallTx\n");
}

void setup (uint8_t *buf)
{
    const uint8_t bmRequestType = buf [0];
    const uint8_t bRequest = buf [1];
    const uint16_t wValue = (uint16_t) (buf [2] | (((uint16_t) buf [3]) << 8));
    const uint16_t wIndex = (uint16_t) (buf [4] | (((uint16_t) buf [5]) << 8));  
    const uint16_t wLength = (uint16_t) (buf [6] | (((uint16_t) buf [7]) << 8));

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
                        print (" > GET DESCRIPTOR");
                        getDesc (wValue, wLength);
                        break;
                    }                               
                    case SET_ADDRESS:
                    {
                        print (" > SET ADDRESS");
                        
                        set_address (buf [2]);                                                     
                        break;
                    }
                    case SET_CONFIGURATION:
                    {
                        print (" > SET_CONFIGURATION");
                        setConfig();
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
    USB_OTG_DEV->DCFG |= (((uint32_t) address) << 4); // SEE ERRATA 2.8.4
    send_ep (0, 0, 0); 
}

void getDesc (uint32_t wValue, uint32_t wLength)
{
    const uint8_t *pbuf;
    uint8_t len = 0;
    switch (wValue >> 8)    
    {
        case DESC_DEVICE:                  // Request device descriptor
            pbuf = desc_device; 
            len = sizeof (desc_device);
            print (" > DESC_DEVICE, 18");
            break;  
        case DESC_CONFIG:                  // Request configuration descriptor
            pbuf = desc_config;
            len = sizeof(desc_config);
            print (" > DESC_CONFIG, 32");
            break;   
        case DESC_STRING:                  // Request string descriptor
            switch (wValue & 0xFF)         // Request string descriptor
            {
                case DESC_STR_LANGID:  // Lang
                    pbuf = desc_lang;
                    len = sizeof (desc_lang);
                    print (" > DESC_STR_LANGID, 4");   
                    break;                                              
                case DESC_STR_CONFIG:  // Config
                    pbuf = desc_config;
                    len = sizeof (desc_config);
                    print (" > DESC_STR_CONFIG, 32");
                    break;
                case DESC_STR_INTERFACE:// Interface
                    break;
                default:
                    break;
            } 
            break;                                                           
        case DESC_OTHER_CONFIG:
            stall_TX_ep (0); // we don't know how to handle this request                      
            break;
        default:
            break;
    }
    if (len)
    {    
        send_ep (0, pbuf, MIN(len,wLength));   
    }  
}
 
void setConfig (void)
{               
    /* Open EP1 IN */
    USB_INEP(1)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM |     // Set DATA0
                            (1 << USB_OTG_DIEPCTL_TXFNUM_Pos)|   // TxFIFO number
                            USB_OTG_DIEPCTL_EPTYP_1 |            // Endpoint type bulk
                            USB_OTG_DIEPCTL_USBAEP |             // Active endpoint
                            MAX_PACKET_SIZE_EP1;                 // Max packet size
    /* Open EP1 OUT */
    USB_OUTEP(1)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA |             // Endpoint enable
                             USB_OTG_DOEPCTL_SD0PID_SEVNFRM |    // Set DATA0
                             USB_OTG_DOEPCTL_CNAK |              // Clear NAK
                             USB_OTG_DOEPCTL_EPTYP_1 |           // Endpoint type bulk
                             USB_OTG_DOEPCTL_USBAEP |            // Active endpoint 
                             MAX_PACKET_SIZE_EP1;                // Max packet size
    
    USB_OTG_DEV->DAINTMSK = EP1_OUT_INT | EP0_OUT_INT | EP1_IN_INT | EP0_IN_INT; // Enable interrupts for EP0 and EP1
    send_ep (0, 0, 0);
}

void USB_RST_interrupt_handler (void)
{
    USB_OTG_DEV->DCTL &= ~USB_OTG_DCTL_RWUSIG;     // Wakeup signal disable    
    for (uint8_t i = 0U; i < 4; i++)                 // Clear any pending EP flags
    {
        USB_INEP(i)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
        USB_INEP(i)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
        USB_OUTEP(i)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
        USB_OUTEP(i)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;               
    }

    USB_OTG_DEV->DAINTMSK |= 0x10001U;              // EP0 OUT, EP0 IN Interupt
    USB_OTG_DEV->DOEPMSK |= USB_OTG_DOEPMSK_STUPM | // Enable setup-done interrupt
                            USB_OTG_DOEPMSK_EPDM  | // Enable EP-disabled irq
                            USB_OTG_DOEPMSK_XFRCM;  // Enable tx-done interrupt                                   
    USB_OTG_DEV->DIEPMSK |= USB_OTG_DIEPMSK_TOM   | // Timeout irq
                            USB_OTG_DIEPMSK_XFRCM |
                            USB_OTG_DIEPMSK_EPDM;
    // buffers
    USB_OTG_FS->GRXFSIZ = RX_FIFO_SIZE; // size is in 32-bit words
    USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = (TX_FIFO_EP0_SIZE << 16) | RX_FIFO_SIZE; // Set the position and size of the EP0 transmit buffer
    USB_OTG_FS->DIEPTXF[0] = (TX_FIFO_EP1_SIZE << 16) | (RX_FIFO_SIZE + TX_FIFO_EP0_SIZE); // Set the position and size of the transmit buffer     
    USB_OUTEP(0)->DOEPTSIZ = (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos) |
                             USB_OTG_DOEPTSIZ_STUPCNT | (3 * 8); // Allow 3 setup packets of 8 bytes                                  
    USB_OTG_DEV->DCFG &= ~USB_OTG_DCFG_DAD;         // Clear address    
    USB_OTG_FS->GINTSTS = USB_OTG_GINTSTS_USBRST;   // Clear the flag by writing 1
    USB_OUTEP(0)->DOEPCTL = USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK; // Enable endpoint, Clear NAK bit
}

void flushTx (void) // clear data stored in TX FIFO (data will be lost)
{
    print ("flushTx\n");
    USB_OTG_FS->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (1 << 10);
    while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH);
}
 
void flushRx (void) // clear data stored in RX FIFO (data will be lost)
{
    print ("flushRx\n");
    USB_OTG_FS->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
    while (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH);
}

void print (const char* ptr) // UART debug print
{
    uint16_t i = 0;
    uint8_t byte = 0;
    while ((byte = (uint8_t) ptr [i]))
    {
        while (!LL_USART_IsActiveFlag_TXE (USART1));
        LL_USART_TransmitData8 (USART1, byte);
        i++;
    }
}