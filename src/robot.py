import time
from src.utils import *


class Robot():
    def __init__(self, CFG: dict):

        self.CFG = CFG

        if not CFG['sim']:
            from pipuck.pipuck import PiPuck
            self.pipuck = PiPuck(epuck_version=2)
            self.epuck = self.pipuck.epuck

            self.leds = [0, 1, 2]
            
        else:
            from controller import Robot as WebotsEPuck
            self.pipuck = WebotsEPuck()                     # used as real pipuck
            self.epuck = VirtualEPuck(pipuck=self.pipuck)   # used as real epuck

            if "ROB_" in self.pipuck.getName():
                self.CFG['my_id'] = self.pipuck.getName().split("_")[1] # leave it a string
                if self.CFG['dt'] is None:
                    self.CFG['dt'] = self.pipuck.getBasicTimeStep() / 1000.0
            
            self.leds = [
                self.pipuck.getDevice("led0"),
                self.pipuck.getDevice("led1"),
                self.pipuck.getDevice("led2"),
                self.pipuck.getDevice("led3"),
                self.pipuck.getDevice("led4"),
                self.pipuck.getDevice("led5"),
                self.pipuck.getDevice("led6"),
                self.pipuck.getDevice("led7"),
            ]

            self.prox_sensors = [
                self.pipuck.getDevice("ps0"),
                self.pipuck.getDevice("ps1"),
                self.pipuck.getDevice("ps2"),
                self.pipuck.getDevice("ps3"),
                self.pipuck.getDevice("ps4"),
                self.pipuck.getDevice("ps5"),
                self.pipuck.getDevice("ps6"),
                self.pipuck.getDevice("ps7"),
            ]

            for ps in self.prox_sensors:
                ps.enable(int(self.CFG['dt'] * 1000))

    def step(self, seconds: float = None):
        s = seconds or self.CFG['dt']
        if self.CFG['sim']:
            self.pipuck.step(int(s*1000))
        else:
            time.sleep(s)

    def get_config(self):
        return self.CFG # updated version of the CFG argument used for instantiation of this Robot        

    def set_led_color(self, led_index: int, color: str):
        if self.CFG['sim']:
            self.leds[led_index].set(1 if color != 'off' else 0)
        else:
            self.pipuck.set_led_colour(led_index, color)

    def set_leds_color(self, color: Union[str, int]):
        if self.CFG['sim']:
            # in sim only on/off (default: red) LED is possible
            if isinstance(color, str):
                color = 0 if color == "off" else 1
            for led in self.leds:
                led.set(int(color))
        else:
            if isinstance(color, str):
                self.pipuck.set_leds_color(color)
            elif isinstance(color, int):
                for led in self.leds:
                    self.pipuck.set_led_raw(led, color)
            else:
                print("Received invalid led color -> Command ignored")
    
    def blink(self, count: int = 3, color: str = 'blue'):
        if self.CFG["sim"] and self.CFG["led"] == "off":
            self.step() # Do not blink if leds are off
        else:
            for i in range(count):
                self.set_leds_color(color)
                self.step(0.2)
                self.set_leds_color("off")
                self.step(0.2)
                # Note: time.sleep instead of step wouldn't work for sim 
                # because setting leds takes effect after step

    def set_motor_speeds(self, left_speed: int, right_speed: int):
        self.epuck.set_motor_speeds(left_speed, right_speed)

    def stop(self):
        """Stop motors."""
        self.epuck.set_motor_speeds(0, 0)
    
    def drive_forward(self, speed: int):
        """Turn an angular amount at a given speed."""
        self.pipuck.epuck.set_motor_speeds(speed, speed)

    def read_proximity_sensors(self):
        """Read the values of the proximity sensors"""
        if self.CFG['sim']:
            return [ps.getValue() for ps in self.prox_sensors]
        else:
            readings = self.epuck.ir_reflected
            return readings
        

class VirtualEPuck():
    """
    Class to emulate epuck functionality for Webots' epuck, which in turn is used as pipuck.
    Use this only for simulation - as epuck attribute for the Robot class.
    """
    def __init__(self, pipuck = None):
        self.pipuck = pipuck
        self.left_motor = self.pipuck.getDevice('left wheel motor')
        self.right_motor = self.pipuck.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.max_speed = self.left_motor.getMaxVelocity()

    def set_motor_speeds(self, left_speed: int, right_speed: int):
            factor = self.max_speed/1000
            self.left_motor.setVelocity(left_speed*factor)
            self.right_motor.setVelocity(right_speed*factor)
