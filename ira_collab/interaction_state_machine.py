from transitions import Machine

# State machine for the interaction node - keeps track of what state the whole system is in.
# This is for IRA collaborative painting.

class InterationStateMachine():
    
    states = ['startup', 'your_turn', 'looking', 'comment', 'my_turn', 'ask_done', 'completed']
    
    transitions = [
        { 'trigger': 'to_startup', 'source': 'completed', 'dest': 'startup'},

        { 'trigger': 'to_your_turn', 'source': 'startup', 'dest': 'your_turn'},

        { 'trigger': 'to_looking', 'source': 'your_turn', 'dest': 'looking' },

        { 'trigger': 'to_comment', 'source': 'looking', 'dest': 'comment' },

        { 'trigger': 'to_my_turn', 'source': 'comment', 'dest': 'my_turn' },

        { 'trigger': 'to_ask_done', 'source': 'my_turn', 'dest': 'ask_done' },

        { 'trigger': 'to_your_turn', 'source': 'my_turn', 'dest': 'your_turn' },
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