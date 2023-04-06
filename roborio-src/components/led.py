from ctre.led import CANdle
from ctre.led import LEDStripType
from ctre.led import (
    RainbowAnimation,
    TwinkleAnimation,
    ColorFlowAnimation,
    FireAnimation,
    SingleFadeAnimation,
    StrobeAnimation,
    BaseTwoSizeAnimation,
    LarsonAnimation
)

BRIGHTNESS = 0.4
FORWARD = ColorFlowAnimation.Direction.Forward
BACKWARD = ColorFlowAnimation.Direction.Backward
NUM_CANDLE_LEDS = 8
NUM_STRIP_LEDS = 63
NUM_TOTAL_LEDS = NUM_CANDLE_LEDS + NUM_STRIP_LEDS  # the 8 LEDs of the CANdle + the strip
RAINBOW = RainbowAnimation(1, 0.9, NUM_TOTAL_LEDS)
TWINKLE = TwinkleAnimation(0, 225, 0, 225, 0.5, NUM_TOTAL_LEDS)
COLORFLOWFORWARD = ColorFlowAnimation(125, 235, 0, 0, 0.5, NUM_TOTAL_LEDS, FORWARD)
COLORFLOWBACKWARD = ColorFlowAnimation(125, 235, 3, 0, 0.5, NUM_TOTAL_LEDS, BACKWARD)
REDALLIANCE = StrobeAnimation(255, 0, 0, 0, 0.05, NUM_STRIP_LEDS, NUM_CANDLE_LEDS)
BLUEALLIANCE = StrobeAnimation(0, 0, 255, 0, 0.05, NUM_STRIP_LEDS, NUM_CANDLE_LEDS)
FIRE = FireAnimation(1, 0.5, NUM_TOTAL_LEDS + 15, 0.7, 0.3, False, NUM_CANDLE_LEDS)


class FROGLED:
    def __init__(self, canID):
        self.candle = CANdle(canID)
        self.candle.configLEDType(LEDStripType.GRB)
        self.candle.configBrightnessScalar(BRIGHTNESS)
        # self.default()

    def larsonAnimation(self, r, g, b, speed):
        self.candle.animate(
            LarsonAnimation(
                r, g, b, 0, speed, NUM_STRIP_LEDS, LarsonAnimation.BounceMode.Front, 7, NUM_CANDLE_LEDS
            )   
        )
    def yellow(self):
        self.candle.setLEDs(250, 129, 7)

    def redAlliance(self):
        self.candle.animate(REDALLIANCE)

    def blueAlliance(self):
        self.candle.animate(BLUEALLIANCE)

    def yellowPocketSlow(self):
        self.larsonAnimation(
                250, 129, 7, 0.5
        )

    def yellowPocketFast(self):
        self.larsonAnimation(
                250, 129, 7, 1.0
        )

    def purplePocketSlow(self):
        self.larsonAnimation(
                167, 16, 201, 0.5
        )

    def purplePocketFast(self):
        self.larsonAnimation(
                167, 16, 201, 1.0
        )

    def purple(self):
        self.candle.setLEDs(167, 16, 201)

    def green(self):
        self.candle.setLEDs(0, 255, 0)

    def rainbow(self):
        self.candle.animate(RAINBOW)

    def twinkle(self):
        self.candle.animate(TWINKLE)

    def fire(self):
        self.candle.animate(FIRE)

    def magenta(self):
        self.candle.setLEDs(200, 0, 70)

    def lightGreen(self):
        self.candle.setLEDs(51, 255, 51)

    def mint(self):
        self.candle.setLEDs(153, 255, 255)

    def lightPink(self):
        self.candle.setLEDs(255, 153, 255)

if __name__ == '__main__':
    pass