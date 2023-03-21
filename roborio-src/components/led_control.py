from magicbot.state_machine import StateMachine
from magicbot import state, timed_state, feedback, tunable, default_state
from components.led import FROGLED

class LedControl(StateMachine):

    leds: FROGLED

    def __init__(self):
        self.last_state = None

    #Default State
    @state
    def default(self):
        self.leds.fire()

    @state(first=True)
    def nothing(self):
        pass

    # Seeking Cone - Larson Yellow Slow
    @state
    def seeking_cone(self):
        self.leds.yellowPocketSlow()        

    # Seeking Cube - Larson Purple Slow
    @state
    def seeking_cube(self):
        self.leds.purplePocketSlow()

    # Found Cone - Larson Animation Yellow Fast
    @state
    def found_cone(self):
        self.leds.yellowPocketFast()

    #Found Cube - Larson Animation Purple Fast
    @state
    def found_cube(self):
        self.leds.purplePocketFast()

    # Holding Cone - Solid Yellow
    @state
    def holding_cone(self):
        self.leds.yellow()

    # Holding Cube - Solid Purple
    @state
    def holding_cube(self):
        self.leds.purple()
