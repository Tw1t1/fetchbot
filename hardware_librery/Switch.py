#!/usr/bin/python3

import RPi.GPIO as GPIO
import time

class BumperSwitch:
    def __init__(self, pin):
        self.pin = pin
        self.callbacks = {'pressed': None, 'released': None}
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        GPIO.add_event_detect(self.pin, GPIO.BOTH, callback=self._event_handler)
        
    def _event_handler(self, channel):
        if GPIO.input(channel) == GPIO.LOW:
            if self.callbacks['pressed']:
                self.callbacks['pressed']()
        else:
            if self.callbacks['released']:
                self.callbacks['released']()
                
    def on_pressed(self, callback):
        self.callbacks['pressed'] = callback
        
    def on_released(self, callback):
        self.callbacks['released'] = callback
        
    def get_state(self):
        return GPIO.input(self.pin)
    
    def cleanup(self):
        GPIO.cleanup(self.pin)

# Example usage:
def pressed_callback():
    print("Bumper pressed!")

def released_callback():
    print("Bumper released!")

if __name__ == "__main__":
    try:
        bumper = BumperSwitch(pin=4)
        bumper.on_pressed(pressed_callback)
        bumper.on_released(released_callback)
        
        print("Bumper Switch Sensor Test (Ctrl+C to exit)")
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting program")
    finally:
        bumper.cleanup()

