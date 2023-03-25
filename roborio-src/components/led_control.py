from magicbot.state_machine import StateMachine
from magicbot import state, timed_state, feedback, tunable, default_state
from components.led import FROGLED

class LedControl(StateMachine):

    leds: FROGLED
    VERBOSE_LOGGING = True

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
    def seeking_cone(self, initial_call):
        if initial_call:
            self.leds.yellowPocketSlow()        

    # Seeking Cube - Larson Purple Slow
    @state
    def seeking_cube(self, initial_call):
        if initial_call:
            self.leds.purplePocketSlow()

    # Found Cone - Larson Animation Yellow Fast
    @state
    def found_cone(self,initial_call):
        if initial_call:
            self.leds.yellowPocketFast()

    #Found Cube - Larson Animation Purple Fast
    @state
    def found_cube(self, initial_call):
        if initial_call:
            self.leds.purplePocketFast()

    # Holding Cone - Solid Yellow
    @state
    def holding_cone(self, initial_call):
        if initial_call:
            self.leds.yellow()

    # Holding Cube - Solid Purple
    @state
    def holding_cube(self, initial_call):
        if initial_call:
            self.leds.purple()
