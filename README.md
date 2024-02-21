Download nRF SDK 17 and softdevice S140, unpack in this directory. The directory should be called sdk.

https://www.nordicsemi.com/Products/Development-software/nRF5-SDK/Download

Important: The SPI driver in sdk/modules/nrfx/drivers/src/nrfx_spim.c contains a delay on 200ms after setting SS pin. Look for nrf_delay_ms(200). This line should be removed or SPI access will be very slow. Using hardware SS pin activation would solve be a better solution.

Project files are provided for Segger Embedded Studio
