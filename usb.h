/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef USB_H
#define USB_H
/* Define to prevent recursive inclusion -------------------------------------*/

#define USB_OTG_DEV  ((USB_OTG_DeviceTypeDef *) ((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))
#define USB_INEP(i)  ((USB_OTG_INEndpointTypeDef *) (( uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (i) * USB_OTG_EP_REG_SIZE))
#define USB_OUTEP(i) ((USB_OTG_OUTEndpointTypeDef *) ((uint32_t)USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (i) * USB_OTG_EP_REG_SIZE))
#define USB_FIFO(i)  *(volatile uint32_t *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + ((i) * USB_OTG_FIFO_SIZE))

#define MIN(a, b)  (((a) < (b)) ? (a) : (b))

#define SETUP_Done  0x04
#define SETUP       0x06
#define DATA        0x02

#define EP0_OUT_INT                0x00010000
#define EP1_OUT_INT                0x00020000
#define EP2_OUT_INT                0x00040000

#define REQUEST_RECIPIENT_MASK 0b11111
#define RECIPIENT_DEVICE       0
#define RECIPIENT_INTERFACE    1
#define RECIPIENT_ENDPOINT     2

#define REQUEST_TYPE_MASK 0b1100000
#define REQUEST_STANDARD  0
#define REQUEST_CLASS     0b0100000
#define REQUEST_VENDOR    0b1000000

#define  GET_STATUS                0x00U
#define  CLEAR_FEATURE             0x01U
#define  SET_FEATURE               0x03U
#define  SET_ADDRESS               0x05U
#define  GET_DESCRIPTOR            0x06U
#define  SET_DESCRIPTOR            0x07U
#define  GET_CONFIGURATION         0x08U
#define  SET_CONFIGURATION         0x09U
#define  GET_INTERFACE             0x0AU
#define  SET_INTERFACE             0x0BU
#define  SYNCH_FRAME               0x0CU

#define  DESC_DEVICE               0x01U
#define  DESC_CONFIG               0x02U
#define  DESC_STRING               0x03U
#define  DESC_INTERFACE            0x04U
#define  DESC_ENDPOINT             0x05U
#define  DESC_DEVICE_QUALIFIER     0x06U
#define  DESC_OTHER_CONFIG         0x07U
#define  DESC_IAD                  0x0BU
#define  DESC_BOS                  0x0FU

#define  DESC_STR_LANGID           0x00U
#define  DESC_STR_MFC              0x01U
#define  DESC_STR_PRODUCT          0x02U
#define  DESC_STR_SERIAL           0x03U
#define  DESC_STR_CONFIG           0x04U
#define  DESC_STR_INTERFACE        0x05U
#define  REQ_DEVICE                0x00U
#define  REQ_INTERFACE             0x01U
#define  REQ_ENDPOINT              0x02U

#define  REQ_STANDARD              0x00U
#define  REQ_CLASS                 0x20U
#define  REQ_VENDOR                0x40U
#define  REQ_MASK                  0x60U

#define EP0_OUT_INT                0x00010000
#define EP1_OUT_INT                0x00020000
#define EP2_OUT_INT                0x00040000
 
#define EP0_IN_INT                 0x0001
#define EP1_IN_INT                 0x0002
#define EP2_IN_INT                 0x0004

#define MAX_PACKET_SIZE_EP0 64U
#define RX_FIFO_SIZE     80 // size is in 32-bit words
#define TX_FIFO_EP0_SIZE 80 // sum of all FIFO sizes is not grater than 320 words
#define TX_FIFO_EP1_SIZE 80
#define TX_FIFO_EP2_SIZE 80

#define UART_SPEED 115200

void send_ep (const uint8_t ep, const uint8_t *buf, const size_t len);
//void send_ep_long (const uint8_t ep, const uint8_t *buf, const uint8_t len);
void read_ep (const uint8_t ep, uint8_t *buf, const size_t len);
void USB_config (void);
void USB_RST_interrupt_handler (void);
//void USB_device_setup (uint8_t *buf);
void set_address (uint8_t address);
void stall_TX_ep (uint8_t ep);
//void get_descriptor (uint16_t wValue, uint16_t wLength);
void print (const char* ptr);

extern uint8_t bufRx [];
extern uint8_t epNumLastRx;
extern volatile size_t countTx;
extern volatile size_t countRx;

//void sendEnd (const uint8_t ep);
void setup (uint8_t *buf);
void getDesc (uint32_t wValue, uint32_t wLength);
void setConfig (void);
void flushTx (void);
void flushRx (void);
//void sendData (const uint8_t ep, const uint8_t *buf, size_t len); // Sending data   
//uint16_t readData (const uint8_t ep, uint8_t *buf);                // Reciv data

extern const uint8_t desc_device [];
extern const uint8_t desc_config [];
extern const uint8_t desc_lang [];

#endif /* USB_H */