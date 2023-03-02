from ctre.led import CANdle
from ctre.led import LEDStripType
from ctre.led import (
    RainbowAnimation,
    TwinkleAnimation,
    ColorFlowAnimation,
    FireAnimation,
    SingleFadeAnimation,
    StrobeAnimation

)

BRIGHTNESS = 0.4
FORWARD = ColorFlowAnimation.Direction.Forward
BACKWARD = ColorFlowAnimation.Direction.Backward
NUM_LEDS = 8 + 47
RAINBOW = RainbowAnimation(1, 0.9, NUM_LEDS)
TWINKLE = TwinkleAnimation(0, 225, 0, 225, 0.5, NUM_LEDS)
COLORFLOWFORWARD = ColorFlowAnimation(125, 235, 0, 0, 0.5, NUM_LEDS, FORWARD)
COLORFLOWBACKWARD = ColorFlowAnimation(125, 235, 3, 0, 0.5, NUM_LEDS, BACKWARD)
FIRE = FireAnimation(1, 0.5, NUM_LEDS, 0.7, 0.3)


class FROGLED:
    def __init__(self, canID):
        self.candle = CANdle(canID)
        self.candle.configLEDType(LEDStripType.GRB)
        self.candle.configBrightnessScalar(BRIGHTNESS)
        self.Default()

    def ColorChangeYellow(self):
        self.candle.setLEDs(235, 229, 52)

    def ColorChangePurple(self):
        self.candle.setLEDs(107, 27, 125)

    def ColorChangeGreen(self):
        self.candle.setLEDs(0, 255, 0)

    def Default(self):
        self.Magenta()

    def Rainbow(self):
        self.candle.animate(RAINBOW)

    def Twinkle(self):
        self.candle.animate(TWINKLE)

    def Fire(self):
        self.candle.animate(FIRE)

    def Magenta(self):
        self.candle.setLEDs(200, 0, 70)

    def LightGreen(self):
        self.candle.setLEDs(51, 255, 51)

    def Mint(self):
        self.candle.setLEDs(153, 255, 255)

    def LightPink(self):
        self.candle.setLEDs(255, 153, 255)
