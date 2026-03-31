"""
ADS7830 8-bit ADC Driver for MicroPython
Based on the Arduino library from ControlEverything.com
Ported for MONA ESP robot
"""

from machine import I2C
import time

# I2C Addresses
ADS7830_DEFAULT_ADDRESS = 0x48  # ADDR = GND
ADS7830_VDD_ADDRESS = 0x49      # ADDR = VDD
ADS7830_SDA_ADDRESS = 0x4A      # ADDR = SDA
ADS7830_SCL_ADDRESS = 0x4B      # ADDR = SCL

# Conversion delay in milliseconds
ADS7830_CONVERSIONDELAY = 5

# Command byte register bits
# Single-Ended/Differential Inputs
ADS7830_REG_COMMAND_SD_DIFF = 0x00    # Differential Inputs
ADS7830_REG_COMMAND_SD_SINGLE = 0x80  # Single-Ended Inputs

# Channel selection for single-ended mode (according to datasheet Table 2)
ADS7830_REG_COMMAND_CH_SINGLE_0 = 0x00
ADS7830_REG_COMMAND_CH_SINGLE_2 = 0x10
ADS7830_REG_COMMAND_CH_SINGLE_4 = 0x20
ADS7830_REG_COMMAND_CH_SINGLE_6 = 0x30
ADS7830_REG_COMMAND_CH_SINGLE_1 = 0x40
ADS7830_REG_COMMAND_CH_SINGLE_3 = 0x50
ADS7830_REG_COMMAND_CH_SINGLE_5 = 0x60
ADS7830_REG_COMMAND_CH_SINGLE_7 = 0x70

# Channel selection for differential mode
ADS7830_REG_COMMAND_CH_DIFF_0_1 = 0x00  # P = CH0, N = CH1
ADS7830_REG_COMMAND_CH_DIFF_2_3 = 0x10  # P = CH2, N = CH3
ADS7830_REG_COMMAND_CH_DIFF_4_5 = 0x20  # P = CH4, N = CH5
ADS7830_REG_COMMAND_CH_DIFF_6_7 = 0x30  # P = CH6, N = CH7
ADS7830_REG_COMMAND_CH_DIFF_1_0 = 0x40  # P = CH1, N = CH0
ADS7830_REG_COMMAND_CH_DIFF_3_2 = 0x50  # P = CH3, N = CH2
ADS7830_REG_COMMAND_CH_DIFF_5_4 = 0x60  # P = CH5, N = CH4
ADS7830_REG_COMMAND_CH_DIFF_7_6 = 0x70  # P = CH7, N = CH6

# Power-Down Selection
ADS7830_REG_COMMAND_PD_PDADCONV = 0x00    # Power Down Between Conversions
ADS7830_REG_COMMAND_PD_IROFF_ADON = 0x04  # Internal Ref OFF, ADC ON
ADS7830_REG_COMMAND_PD_IRON_ADOFF = 0x08  # Internal Ref ON, ADC OFF
ADS7830_REG_COMMAND_PD_IRON_ADON = 0x0C   # Internal Ref ON, ADC ON

# SD Mode enum-like constants
SDMODE_DIFF = ADS7830_REG_COMMAND_SD_DIFF
SDMODE_SINGLE = ADS7830_REG_COMMAND_SD_SINGLE

# PD Mode enum-like constants
PDADCONV = ADS7830_REG_COMMAND_PD_PDADCONV
PDIROFF_ADON = ADS7830_REG_COMMAND_PD_IROFF_ADON
PDIRON_ADOFF = ADS7830_REG_COMMAND_PD_IRON_ADOFF
PDIRON_ADON = ADS7830_REG_COMMAND_PD_IRON_ADON


class ADS7830:
    """ADS7830 8-bit ADC driver class"""
    
    # Single-ended channel mapping
    _SINGLE_CHANNEL_MAP = {
        0: ADS7830_REG_COMMAND_CH_SINGLE_0,
        1: ADS7830_REG_COMMAND_CH_SINGLE_1,
        2: ADS7830_REG_COMMAND_CH_SINGLE_2,
        3: ADS7830_REG_COMMAND_CH_SINGLE_3,
        4: ADS7830_REG_COMMAND_CH_SINGLE_4,
        5: ADS7830_REG_COMMAND_CH_SINGLE_5,
        6: ADS7830_REG_COMMAND_CH_SINGLE_6,
        7: ADS7830_REG_COMMAND_CH_SINGLE_7,
    }
    
    # Differential channel mapping (using channel pair as key)
    _DIFF_CHANNEL_MAP = {
        (0, 1): ADS7830_REG_COMMAND_CH_DIFF_0_1,
        (1, 0): ADS7830_REG_COMMAND_CH_DIFF_1_0,
        (2, 3): ADS7830_REG_COMMAND_CH_DIFF_2_3,
        (3, 2): ADS7830_REG_COMMAND_CH_DIFF_3_2,
        (4, 5): ADS7830_REG_COMMAND_CH_DIFF_4_5,
        (5, 4): ADS7830_REG_COMMAND_CH_DIFF_5_4,
        (6, 7): ADS7830_REG_COMMAND_CH_DIFF_6_7,
        (7, 6): ADS7830_REG_COMMAND_CH_DIFF_7_6,
    }
    
    def __init__(self, i2c: I2C, address: int = ADS7830_DEFAULT_ADDRESS):
        """
        Initialize ADS7830 ADC
        
        Args:
            i2c: I2C bus object
            address: I2C address of the device
        """
        self._i2c = i2c
        self._address = address
        self._conversion_delay = ADS7830_CONVERSIONDELAY
        self._sd_mode = SDMODE_SINGLE
        self._pd_mode = PDIROFF_ADON
    
    @property
    def sd_mode(self) -> int:
        """Get Single-Ended/Differential mode"""
        return self._sd_mode
    
    @sd_mode.setter
    def sd_mode(self, mode: int):
        """Set Single-Ended/Differential mode"""
        self._sd_mode = mode
    
    @property
    def pd_mode(self) -> int:
        """Get Power-Down mode"""
        return self._pd_mode
    
    @pd_mode.setter
    def pd_mode(self, mode: int):
        """Set Power-Down mode"""
        self._pd_mode = mode
    
    def _write_command(self, cmd: int):
        """Write command byte to ADC"""
        self._i2c.writeto(self._address, bytes([cmd]))
    
    def _read_result(self) -> int:
        """Read conversion result from ADC"""
        data = self._i2c.readfrom(self._address, 1)
        return data[0]
    
    def measure_single_ended(self, channel: int) -> int:
        """
        Read single-ended ADC value from specified channel
        
        Args:
            channel: Channel number (0-7)
            
        Returns:
            8-bit unsigned ADC value (0-255)
        """
        if channel < 0 or channel > 7:
            return 0
        
        # Build config byte
        config = self._sd_mode | self._pd_mode | self._SINGLE_CHANNEL_MAP[channel]
        
        # Write config and read result
        self._write_command(config)
        time.sleep_ms(self._conversion_delay)
        return self._read_result()
    
    def measure_differential(self, positive_ch: int, negative_ch: int) -> int:
        """
        Read differential ADC value
        
        Args:
            positive_ch: Positive input channel
            negative_ch: Negative input channel
            
        Returns:
            8-bit signed ADC value
        """
        channel_pair = (positive_ch, negative_ch)
        if channel_pair not in self._DIFF_CHANNEL_MAP:
            return 0
        
        # Build config byte
        config = self._sd_mode | self._pd_mode | self._DIFF_CHANNEL_MAP[channel_pair]
        
        # Write config and read result
        self._write_command(config)
        time.sleep_ms(self._conversion_delay)
        
        raw_adc = self._read_result()
        # Convert to signed value
        if raw_adc > 127:
            return raw_adc - 256
        return raw_adc
