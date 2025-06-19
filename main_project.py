import time
import sys
import RPi.GPIO as GPIO
import Ports
import threading

# import LCD_I2C.lcd_driver as lcd_driver # Dòng này để dùng trên phần cứng

# Cần thay đổi địa chỉ thư viện
sys.path.append("C:/Users/ADMIN/Desktop/Project/Rasp/VXL_VDK/19_5/Final/Group2_Final/LCD_I2C")
import lcd_driver

lcd = lcd_driver.lcd()
lcd.lcd_clear()

GPIO.setmode(GPIO.BCM)

SMOKE_THRESHOLD = 1.5  # Threshold for smoke detection in volts

port = Ports.UART()

clock_pin = 12
miso_pin = 13
mosi_pin = 16
cs_pin = 19

led_pin = 20
horn_pin = 21
emergency_pin = 26

# Global variables for controlling LED and horn
led_blink_active = False
horn_active = False
stop_threads = False
emergency_stop = False
input = None

class MCP3008():
    """Class to handle MCP3008 ADC communication"""
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
        """Read ADC value from specified channel"""
        if (channel_num > 7) or (channel_num < 0):
            return -1

        GPIO.output(self.cspin_, GPIO.HIGH)
        GPIO.output(self.clockpin_, GPIO.LOW)

        # Start the communication
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
            # port.send(f"Sent command order '{i}': '{bin(command_out)}'\r\n")
            GPIO.output(self.clockpin_, GPIO.HIGH)
            GPIO.output(self.clockpin_, GPIO.LOW)

        adc_out = 0

        for i in range(12):
            GPIO.output(self.clockpin_, GPIO.HIGH)
            GPIO.output(self.clockpin_, GPIO.LOW)
            adc_out <<= 1
            if GPIO.input(self.misopin_):
                adc_out |= 0x1
            # port.send(f"Received data order '{i}': '{bin(adc_out)}'\r\n")

        GPIO.output(self.cspin_, GPIO.HIGH)
        adc_out >>= 1 
        # port.send(f"Received final data '{adc_out}'\r\n")

        return adc_out

def setup_alert_pins():
    """Setup GPIO pins for LED and horn"""
    GPIO.setup(led_pin, GPIO.OUT)
    GPIO.setup(horn_pin, GPIO.OUT)
    GPIO.output(led_pin, GPIO.LOW)
    GPIO.output(horn_pin, GPIO.LOW)

def led_blink_thread():
    """Thread function for blinking LED"""
    global led_blink_active, stop_threads
    while True:
        while not stop_threads and not emergency_stop:
            if led_blink_active:
                GPIO.output(led_pin, GPIO.HIGH)
                time.sleep(0.5) 
                if not stop_threads:
                    GPIO.output(led_pin, GPIO.LOW)
                    time.sleep(0.5)
            else:
                GPIO.output(led_pin, GPIO.LOW)
                time.sleep(0.1)
        time.sleep(0.1)

def horn_control_thread():
    """Thread function for controlling horn"""
    global horn_active, stop_threads
    while True:
        while not stop_threads and not emergency_stop:
            if horn_active:
                GPIO.output(horn_pin, GPIO.HIGH)
            else:
                GPIO.output(horn_pin, GPIO.LOW)
            time.sleep(0.1)
        time.sleep(0.1)

def control_alerts(adc_value):
    """Control LED and horn based on ADC value"""
    global led_blink_active, horn_active
    
    if adc_value > SMOKE_THRESHOLD:
        led_blink_active = True
        horn_active = True
    else:
        led_blink_active = False
        horn_active = False

def setup_interrupt():
    """Setup GPIO26 for interrupt when it goes high"""
    global emergency_pin

    GPIO.setup(emergency_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    GPIO.add_event_detect(emergency_pin, GPIO.RISING, callback=interrupt_handler, bouncetime=100)

def interrupt_handler():
    """Handler function called when GPIO26 goes high"""
    port.send("Emergency button pressed!\r\n")

    global led_blink_active, horn_active, input, stop_threads, emergency_stop, lcd
    
    stop_threads = True
    GPIO.output(led_pin, GPIO.LOW)
    GPIO.output(horn_pin, GPIO.LOW)
    
    emergency_stop = True

    GPIO.output(input.cspin_, GPIO.HIGH) # Stop ADC communication
    lcd.lcd_clear()


def main():
    global stop_threads, input, emergency_stop
    
    # Setup alert pins
    setup_alert_pins()
    setup_interrupt()
    
    # Create and start threads for LED and horn control
    led_thread = threading.Thread(target=led_blink_thread, daemon=True)
    horn_thread = threading.Thread(target=horn_control_thread, daemon=True)
    
    led_thread.start()
    horn_thread.start()
    
    input = MCP3008(
        cspin=cs_pin,
        clockpin=clock_pin,
        mosipin=mosi_pin,
        misopin=miso_pin
    )

    try:
        while True:
            while not emergency_stop:
                lcd.lcd_clear()
                lcd.lcd_display_string("Reading...", 1, 0)
                output_data = input.readADC(channel_num=4)

                # port.send(f"Received output_data '{bin(output_data)}'\r\n")

                adc_value = 5.0 * ( output_data / (2** 10 -1))

                lcd.lcd_clear()
                lcd.lcd_display_string("Sensor data:", 1, 0)
                lcd.lcd_display_string(f"{adc_value:.2f}V", 2, 0)

                # Control alerts based on ADC value
                control_alerts(adc_value)

                time.sleep(1.0) 

            lcd.lcd_clear()
            time.sleep(10.0)
            emergency_stop = False
            stop_threads = False
            
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        # Cleanup
        stop_threads = True
        GPIO.output(led_pin, GPIO.LOW)
        GPIO.output(horn_pin, GPIO.LOW)
        lcd.lcd_clear()
        GPIO.cleanup()

if __name__ == '__main__':
    main()