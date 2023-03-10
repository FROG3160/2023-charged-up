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
FIRE = FireAnimation(1, 0.5, NUM_TOTAL_LEDS + 15, 0.7, 0.3, False, NUM_CANDLE_LEDS)


class FROGLED:
    def __init__(self, canID):
        self.candle = CANdle(canID)
        self.candle.configLEDType(LEDStripType.GRB)
        self.candle.configBrightnessScalar(BRIGHTNESS)
        self.Default()

    def larsonAnimation(self, r, g, b, speed):
        self.candle.animate(
            LarsonAnimation(
                r, g, b, 0, speed, NUM_STRIP_LEDS, LarsonAnimation.BounceMode.Front, 7, NUM_CANDLE_LEDS
            )   
        )
    def Yellow(self):
        #self.candle.setLEDs(235, 229, 52)
        pass

    def yellowPocketSlow(self):
        self.larsonAnimation(
                235, 229, 52, 0.25
        )
        pass

    def yellowPocketFast(self):
        self.larsonAnimation(
                235, 229, 52, 0.75
        )
        pass

    def purplePocketSlow(self):
        # self.candle.animate(
        #     LarsonAnimation(
        #         107, 27, 125, 0, 0.25, NUM_STRIP_LEDS, LarsonAnimation.BounceMode.Front, 7, NUM_CANDLE_LEDS
        #     )   
        # )
        pass

    def purplePocketFast(self):
        # self.candle.animate(
        #     LarsonAnimation(
        #         107, 27, 125, 0, 0.75, NUM_STRIP_LEDS, LarsonAnimation.BounceMode.Front, 7, NUM_CANDLE_LEDS
        #     )   
        # )
        pass

    def Purple(self):
        #self.candle.setLEDs(107, 27, 125)
        pass

    def Green(self):
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

if __name__ == '__main__':
    pass