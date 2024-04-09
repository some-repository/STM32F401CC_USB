#include "stm32f401xc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"
#include <stddef.h> 
#include "spi.h"

void SPI_config (void)
{
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock (LL_APB2_GRP1_PERIPH_SPI1);

    // PA4 (CS)
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed (GPIOA, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    // PA5 (SCK)
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed (GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetAFPin_0_7 (GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_5);
    // PA6 (MISO)
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed (GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetAFPin_0_7 (GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF_5);
    // PA7 (MOSI)
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed (GPIOA, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetAFPin_0_7 (GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_5);

    LL_SPI_SetClockPolarity (SPI1, LL_SPI_POLARITY_LOW);
    LL_SPI_SetClockPhase (SPI1, LL_SPI_PHASE_1EDGE);
    LL_SPI_SetMode (SPI1, LL_SPI_MODE_MASTER);
    LL_SPI_SetBaudRatePrescaler (SPI1, LL_SPI_BAUDRATEPRESCALER_DIV256);
    LL_SPI_SetTransferBitOrder (SPI1, LL_SPI_MSB_FIRST);
    LL_SPI_SetNSSMode (SPI1, LL_SPI_NSS_SOFT);
    LL_SPI_SetTransferDirection (SPI1, LL_SPI_FULL_DUPLEX);
    LL_SPI_SetDataWidth (SPI1, LL_SPI_DATAWIDTH_8BIT);
    /*LL_SPI_EnableIT_TXE (SPI1);
    NVIC_SetPriority (SPI1_IRQn, 2);
    NVIC_EnableIRQ (SPI1_IRQn);*/
    LL_SPI_Enable (SPI1);
}

void SPI_duplex_exchange (const uint8_t *TX_array, uint8_t *RX_array, const size_t length)
{
    LL_GPIO_ResetOutputPin (GPIOA, LL_GPIO_PIN_4); // set CS to LOW state

    for (size_t i = 0; i < length; i++) 
    {
        LL_SPI_TransmitData8 (SPI1, TX_array [i]);
        while (LL_SPI_IsActiveFlag_BSY (SPI1) || (!LL_SPI_IsActiveFlag_RXNE (SPI1))); // wait until SPI is not busy and received byte is in data register
        RX_array [i] = LL_SPI_ReceiveData8 (SPI1);
    }

    LL_GPIO_SetOutputPin (GPIOA, LL_GPIO_PIN_4); // set CS to HIGH state
}