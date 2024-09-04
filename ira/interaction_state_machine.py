from transitions import Machine

# State machine for the interaction node - keeps track of what state the whole system is in.
# This is for IRA painting people's faces.

class InterationStateMachine():
    
    states = ['scanning', 'found_unknown', 'found_known', 'found_noone','say_painted_recently', 
              'too_far', 'painting', 'completed']
    
    transitions = [
        { 'trigger': 'to_scanning', 'source': 'scanning', 'dest': 'scanning'}, #done

        { 'trigger': 'to_found_unknown', 'source': 'scanning', 'dest': 'found_unknown'}, #done
        { 'trigger': 'to_found_known', 'source': 'scanning', 'dest': 'found_known'}, #done
        { 'trigger': 'to_found_noone', 'source': 'scanning', 'dest': 'found_noone'}, #done

        { 'trigger': 'to_scanning', 'source': 'found_noone', 'dest': 'scanning'}, #done

        { 'trigger': 'to_say_painted_recently', 'source': 'found_known', 'dest': 'say_painted_recently' }, #Done
        { 'trigger': 'to_painting', 'source': 'found_known', 'dest': 'painting' }, #done
        { 'trigger': 'to_too_far', 'source': 'found_known', 'dest': 'too_far' }, #done
        { 'trigger': 'to_scanning', 'source': 'found_known', 'dest': 'scanning' }, #done

        { 'trigger': 'to_scanning', 'source': 'say_painted_recently', 'dest': 'scanning' }, #done

        { 'trigger': 'to_scanning', 'source': 'too_far', 'dest': 'scanning' }, #done

        { 'trigger': 'to_painting', 'source': 'found_unknown', 'dest': 'painting' }, #done
        { 'trigger': 'to_too_far', 'source': 'found_unknown', 'dest': 'too_far' }, #done
        { 'trigger': 'to_scanning', 'source': 'found_unknown', 'dest': 'scanning' }, #done

        { 'trigger': 'to_completed', 'source': 'painting', 'dest': 'completed' }, #done

        { 'trigger': 'to_scanning', 'source': 'completed', 'dest': 'scanning' }, #done
    ]

    def __init__(self):

        # Initialize the state machine

        self.machine = Machine(
            model=self, 
            states=InterationStateMachine.states, 
            transitions=InterationStateMachine.transitions, 
            initial='scanning'
        )
