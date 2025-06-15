import time
import sys
import RPi.GPIO as GPIO
import Ports

# import LCD_I2C.lcd_driver as lcd_driver # Dòng này để dùng trên phần cứng

# Cần thay đổi địa chỉ thư viện
sys.path.append("C:/Users/ADMIN/Desktop/Project/Rasp/VXL_VDK/19_5/Final/ManualDataRead/LCD_I2C")
import lcd_driver

lcd = lcd_driver.lcd()
lcd.lcd_clear()

GPIO.setmode(GPIO.BCM)

port = Ports.UART()

clock_pin = 12
miso_pin = 13
mosi_pin = 16
cs_pin = 19

class MCP3008():
    def __init__(self, cspin, clockpin, mosipin, misopin):
        self.cspin_ = cspin
        self.clockpin_ = clockpin
        self.mosipin_ = mosipin
        self.misopin_ = misopin

        GPIO.setup(self.cspin_, GPIO.OUT)
        GPIO.setup(self.clockpin_, GPIO.OUT)
        GPIO.setup(self.mosipin_, GPIO.OUT)
        GPIO.setup(self.misopin_, GPIO.IN)

    def readADC(self, channel_num):
        if (channel_num > 7) or (channel_num < 0):
            return -1

        GPIO.output(self.cspin_, GPIO.HIGH)
        GPIO.output(self.clockpin_, GPIO.LOW)
        GPIO.output(self.cspin_, GPIO.LOW)

        command_out = channel_num
        command_out |= 0x18
        command_out <<= 3

        for i in range(5):
            if command_out & 0x80:
                GPIO.output(self.mosipin_, GPIO.HIGH)
            else:
                GPIO.output(self.mosipin_, GPIO.LOW)
            command_out <<= 1
            # port.send(f"Sent command order '{i}': '{bin(command_out)}'/r/n")
            GPIO.output(self.clockpin_, GPIO.HIGH)
            GPIO.output(self.clockpin_, GPIO.LOW)

        adc_out = 0

        for i in range(12):
            GPIO.output(self.clockpin_, GPIO.HIGH)
            GPIO.output(self.clockpin_, GPIO.LOW)
            adc_out <<= 1
            if GPIO.input(self.misopin_):
                adc_out |= 0x1
            # port.send(f"Received data order '{i}': '{bin(adc_out)}'/r/n")

        GPIO.output(self.cspin_, GPIO.HIGH)
        # port.send(f"Received final data '{adc_out}'/r/n")
        adc_out <<= 1

        return adc_out
    
def main():
    input = MCP3008(
        cspin=cs_pin,
        clockpin=clock_pin,
        mosipin=mosi_pin,
        misopin=miso_pin
    )

    while True:
        lcd.lcd_clear()
        lcd.lcd_display_string("Reading...", 1, 0)
        output_data = input.readADC(channel_num=4)

        port.send(f"Received output_data '{bin(output_data)}'/r/n")

        adc_value = 5.0 * ( output_data / (2** 12 -1))

        lcd.lcd_clear()
        lcd.lcd_display_string("Sensor data:", 1, 0)
        lcd.lcd_display_string(f"{adc_value}", 2, 0)
        time.sleep(1.0)

if __name__ == '__main__':
    main()