from transitions import Machine

# State machine for the interaction node - keeps track of what state the whole system is in.
# This is for IRA collaborative painting.

class InterationStateMachine():
    
    states = ['startup_ready', 'startup_pic', 'your_turn', 'your_turn_pic', 'comment', 'my_turn', 'my_turn_pic' 'ask_done', 'completed']
    
    transitions = [
        { 'trigger': 'to_startup_ready', 'source': 'completed', 'dest': 'startup_ready'},

        { 'trigger': 'to_startup_pic', 'source': 'startup_ready', 'dest': 'startup_pic'},

        { 'trigger': 'to_your_turn', 'source': 'startup_pic', 'dest': 'your_turn'},

        { 'trigger': 'to_your_turn_pic', 'source': 'your_turn', 'dest': 'your_turn_pic' },

        { 'trigger': 'to_comment', 'source': 'your_turn_pic', 'dest': 'comment' },

        { 'trigger': 'to_my_turn', 'source': 'comment', 'dest': 'my_turn' },

        { 'trigger': 'to_my_turn_pic', 'source': 'my_turn', 'dest': 'my_turn_pic' },

        { 'trigger': 'to_your_turn', 'source': 'my_turn_pic', 'dest': 'your_turn' },
        { 'trigger': 'to_ask_done', 'source': 'my_turn_pic', 'dest': 'ask_done' },
        
        { 'trigger': 'to_your_turn', 'source': 'ask_done', 'dest': 'your_turn' },
        { 'trigger': 'to_completed', 'source': 'ask_done', 'dest': 'completed' }
    ]

    def __init__(self):

        # Initialize the state machine

        self.machine = Machine(
            model=self, 
            states=InterationStateMachine.states, 
            transitions=InterationStateMachine.transitions, 
            initial='scanning'
        )



# TODO Add in a user input check state for if the canvas/paper and paint are ready. 