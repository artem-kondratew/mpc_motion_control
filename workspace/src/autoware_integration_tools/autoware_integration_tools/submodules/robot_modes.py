from transitions import Machine


class RobotModes:
    
    states = [
        'manual',
        'autonomous',
        'crab',
        'emergency_stop',
    ]

    colors = {
        'manual':         (255, 0, 0, 1.0),      # red
        'autonomous':     (8, 192, 100, 1.0),    # green
        'crab':           (245, 183, 142, 1.0),  # orange
        'emergency_stop': (255, 255, 255, 1.0),  # white
    }

    transitions = [
        {'trigger': 'set_manual',         'source': 'autonomous',     'dest': 'manual'},
        {'trigger': 'set_manual',         'source': 'emergency_stop', 'dest': 'manual'},

        {'trigger': 'set_autonomous',     'source': 'manual',         'dest': 'autonomous'},
        {'trigger': 'set_autonomous',     'source': 'emergency_stop', 'dest': 'autonomous'},

        {'trigger': 'enable_crab',        'source': 'manual',         'dest': 'crab'},
        {'trigger': 'disable_crab',       'source': 'crab',           'dest': 'manual'},

        {'trigger': 'set_emergency_stop', 'source': 'manual',         'dest': 'emergency_stop'},
        {'trigger': 'set_emergency_stop', 'source': 'autonomous',     'dest': 'emergency_stop'},
        {'trigger': 'set_emergency_stop', 'source': 'crab',           'dest': 'emergency_stop'},
    ]

    def __init__(self):
        self.machine = Machine(
            model=self,
            states=RobotModes.states,
            transitions=RobotModes.transitions,
            initial='autonomous',
        )

    @property
    def color(self):
        return self.colors[self.state]
