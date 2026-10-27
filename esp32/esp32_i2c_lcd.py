import time
from lcd_api import LcdApi


class I2cLcd(LcdApi):
    MASK_RS = 0x01
    MASK_RW = 0x02
    MASK_E = 0x04
    SHIFT_BACKLIGHT = 3
    SHIFT_DATA = 4

    def __init__(self, i2c, i2c_addr, num_lines, num_columns):
        super().__init__(num_lines, num_columns)   # FIX: sets num_lines / num_columns
        self.i2c = i2c
        self.i2c_addr = i2c_addr
        self.backlight = True

        time.sleep_ms(20)
        self.hal_write_init_nibble(0x03)
        time.sleep_ms(5)
        self.hal_write_init_nibble(0x03)
        time.sleep_ms(1)
        self.hal_write_init_nibble(0x03)
        self.hal_write_init_nibble(0x02)

        self.hal_write_command(self.LCD_FUNCTION | self.LCD_FUNCTION_2LINES)
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY)
        self.hal_write_command(self.LCD_ENTRY_MODE | self.LCD_ENTRY_INC)  # FIX: explicit entry mode
        self.clear()

    def hal_write_init_nibble(self, nibble):
        byte = (nibble << self.SHIFT_DATA) | (self.backlight << self.SHIFT_BACKLIGHT)
        self.i2c.writeto(self.i2c_addr, bytes([byte | self.MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytes([byte]))

    def hal_write_command(self, cmd):
        self.hal_write_byte(cmd, 0)
        if cmd <= 3:                 # FIX: clear / home need ~2 ms to complete
            time.sleep_ms(5)

    def hal_write_char(self, char):
        self.hal_write_byte(char, self.MASK_RS)

    def hal_write_byte(self, data, mode):
        high = (data & 0xF0) | (self.backlight << self.SHIFT_BACKLIGHT) | mode
        low = ((data << 4) & 0xF0) | (self.backlight << self.SHIFT_BACKLIGHT) | mode
        self.i2c.writeto(self.i2c_addr, bytes([high | self.MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytes([high]))
        self.i2c.writeto(self.i2c_addr, bytes([low | self.MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytes([low]))