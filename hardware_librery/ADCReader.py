#!/usr/bin/python3

import signal
import sys
import spidev

class ADCReader:
    """
    ADCReader class represented by MPC3002 ADC with 2 channels input.
    """

    def __init__(self, spi_ch=0):
        """
        Initialize the ADCReader.

        :param spi_ch: SPI channel (default: 0)
        """
        # Enable SPI
        self.spi = spidev.SpiDev()
        self.spi.open(0, spi_ch)
        self.spi.max_speed_hz = 1200000

        signal.signal(signal.SIGINT, self.close)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self, signal=None, frame=None):
        """
        Close the SPI connection and exit the program.
        """
        self.spi.close()
        sys.exit(0)

    def get_adc(self, channel):
        """
        Read the ADC value from the specified channel.

        :param channel: ADC channel (0 or 1)
        :return: Voltage reading from the ADC
        """
        # Make sure ADC channel is 0 or 1
        if channel != 0:
            channel = 1

        # Construct SPI message
        msg = 0b11
        msg = ((msg << 1) + channel) << 5
        msg = [msg, 0b00000000]
        reply = self.spi.xfer2(msg)

        # Construct single integer out of the reply (2 bytes)
        adc = 0
        for n in reply:
            adc = (adc << 8) + n

        # Last bit (0) is not part of ADC value, shift to remove it
        adc = adc >> 1

        # Calculate voltage from ADC value
        # considering the soil moisture sensor is working at 5V
        voltage = (5 * adc) / 1024

        return voltage