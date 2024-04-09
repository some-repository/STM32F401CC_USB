/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SPI_H
#define SPI_H
/* Define to prevent recursive inclusion -------------------------------------*/

void SPI_config (void);
void SPI_duplex_exchange (const uint8_t *TX_array, uint8_t *RX_array, const size_t length);

#endif /* SPI_H */