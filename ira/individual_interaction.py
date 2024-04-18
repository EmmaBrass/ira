
import datetime
# Once one face has been found & locked onto, this is for that interation 'session'
# with the individual, until their painting is complete or they have disappeared.

# The individial interaction outcome should be added to the dict assigned to their face encoding.

class IndivInteraction():
    def __init__(self, face) -> None:
        self.face = face

    def run(self):
        """
        Run the interaction program, and return the result of the interaction.
        """
        # Check the face dict of past interaction outcomes
        if self.face.known == True:
            last_interaction = self.face.past_interactions[-1]
            # Check how long it has been since latest interaction...
            # if a long time, say hey, I know you, I've seen you x times and mostly recently on y date, then paint
            # lots of options here.  Based of if last interation was successfully painting or they ran of.
            # Including arm and eye movements!
        else:
            # If face is unkown, will not have any past interactions.
            # Just go on to complimenting and then painting!
            pass
        # Try to centre the face if not centred
        # then paint the face

        # Use self.face.add_interaction(date, outcome) to update the past interaction list.
        # Possible outcomes: 
        # successfully painted, 
        # left before they could be painted,
        # never got close enough to be painted
        # successfully painted but then didn't stick around to see the outcome! ?
        self.face.add_interaction(datetime.now(), "outcome")