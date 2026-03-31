"""
WS2812B NeoPixel LED Driver for MicroPython
Uses the built-in neopixel module
"""

from machine import Pin
from neopixel import NeoPixel


class NeoPixelLED:
    """Simple NeoPixel LED wrapper class"""
    
    def __init__(self, pin: int, num_leds: int = 1):
        """
        Initialize NeoPixel LED
        
        Args:
            pin: GPIO pin number
            num_leds: Number of LEDs in the strip
        """
        self._pin = Pin(pin, Pin.OUT)
        self._np = NeoPixel(self._pin, num_leds)
        self._num_leds = num_leds
        self.clear()
    
    def clear(self):
        """Turn off all LEDs"""
        for i in range(self._num_leds):
            self._np[i] = (0, 0, 0)
        self._np.write()
    
    def fill(self, r: int, g: int, b: int):
        """
        Fill all LEDs with the same color
        
        Args:
            r: Red component (0-255)
            g: Green component (0-255)
            b: Blue component (0-255)
        """
        for i in range(self._num_leds):
            self._np[i] = (r, g, b)
        self._np.write()
    
    def set_pixel(self, index: int, r: int, g: int, b: int):
        """
        Set a specific LED color
        
        Args:
            index: LED index
            r: Red component (0-255)
            g: Green component (0-255)
            b: Blue component (0-255)
        """
        if 0 <= index < self._num_leds:
            self._np[index] = (r, g, b)
            self._np.write()
    
    def show(self):
        """Update the LED strip (write changes)"""
        self._np.write()
