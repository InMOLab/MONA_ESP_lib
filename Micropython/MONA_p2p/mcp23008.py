"""
MCP23008 I2C GPIO Expander Driver for MicroPython
Simplified implementation for MONA ESP robot
"""

from machine import I2C

# Default I2C address
MCP23008_DEFAULT_ADDRESS = 0x20

# Register addresses
MCP23008_IODIR = 0x00    # I/O Direction Register
MCP23008_IPOL = 0x01     # Input Polarity Register
MCP23008_GPINTEN = 0x02  # Interrupt-on-Change Control Register
MCP23008_DEFVAL = 0x03   # Default Value Register
MCP23008_INTCON = 0x04   # Interrupt Control Register
MCP23008_IOCON = 0x05    # Configuration Register
MCP23008_GPPU = 0x06     # Pull-Up Resistor Configuration Register
MCP23008_INTF = 0x07     # Interrupt Flag Register
MCP23008_INTCAP = 0x08   # Interrupt Capture Register
MCP23008_GPIO = 0x09     # GPIO Port Register
MCP23008_OLAT = 0x0A     # Output Latch Register

# Pin modes
INPUT = 1
OUTPUT = 0

# Pin states
HIGH = 1
LOW = 0


class MCP23008:
    """MCP23008 8-bit I/O Expander driver class"""
    
    def __init__(self, i2c: I2C, address: int = MCP23008_DEFAULT_ADDRESS):
        """
        Initialize MCP23008
        
        Args:
            i2c: I2C bus object
            address: I2C address of the device
        """
        self._i2c = i2c
        self._address = address
        self._iodir = 0xFF  # All inputs by default
        self._gpio = 0x00   # All low by default
        
        # Initialize device
        self._write_register(MCP23008_IODIR, self._iodir)
        self._write_register(MCP23008_GPIO, self._gpio)
    
    def _write_register(self, reg: int, value: int):
        """Write a byte to register"""
        self._i2c.writeto(self._address, bytes([reg, value]))
    
    def _read_register(self, reg: int) -> int:
        """Read a byte from register"""
        self._i2c.writeto(self._address, bytes([reg]))
        data = self._i2c.readfrom(self._address, 1)
        return data[0]
    
    def pin_mode(self, pin: int, mode: int):
        """
        Set pin mode (INPUT or OUTPUT)
        
        Args:
            pin: Pin number (0-7)
            mode: INPUT (1) or OUTPUT (0)
        """
        if pin < 0 or pin > 7:
            return
        
        if mode == INPUT:
            self._iodir |= (1 << pin)
        else:
            self._iodir &= ~(1 << pin)
        
        self._write_register(MCP23008_IODIR, self._iodir)
    
    def digital_write(self, pin: int, value: int):
        """
        Write to output pin
        
        Args:
            pin: Pin number (0-7)
            value: HIGH (1) or LOW (0)
        """
        if pin < 0 or pin > 7:
            return
        
        if value:
            self._gpio |= (1 << pin)
        else:
            self._gpio &= ~(1 << pin)
        
        self._write_register(MCP23008_GPIO, self._gpio)
    
    def digital_read(self, pin: int) -> int:
        """
        Read from input pin
        
        Args:
            pin: Pin number (0-7)
            
        Returns:
            HIGH (1) or LOW (0)
        """
        if pin < 0 or pin > 7:
            return 0
        
        gpio_val = self._read_register(MCP23008_GPIO)
        return (gpio_val >> pin) & 0x01
    
    def pull_up(self, pin: int, enable: bool):
        """
        Enable/disable internal pull-up resistor
        
        Args:
            pin: Pin number (0-7)
            enable: True to enable, False to disable
        """
        if pin < 0 or pin > 7:
            return
        
        gppu = self._read_register(MCP23008_GPPU)
        
        if enable:
            gppu |= (1 << pin)
        else:
            gppu &= ~(1 << pin)
        
        self._write_register(MCP23008_GPPU, gppu)
