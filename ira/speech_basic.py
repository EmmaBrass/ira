# A class for interaction sounds between the user and Penelope

import time
import gtts
from playsound import playsound
import keyboard
import logging

class Interaction:

    def __init__(self):
        self.logger = logging.getLogger("main_logger")
        self.name = None

    def start(self):
        self.logger.info("Speaking interaction message for start of the program.")
        tts = gtts.gTTS(f"{self.name}, nice to meet you.")
        tts.save("mp3/instructions.mp3")
        playsound("mp3/instructions.mp3")

    def recognised_face(self, name=None):
        self.logger.info("Speaking greeting message for a recognised face.")
        if name != None:
            tts = gtts.gTTS(f"{name}, we have met before, nice to see you again!")
        else:
            tts = gtts.gTTS(f"Oh hello, I know you, nice to see you again!")
        tts.save("mp3/instructions.mp3")
        playsound("mp3/instructions.mp3")

    def new_face(self):
        self.logger.info("Speaking greeting message for a new face.")
        tts = gtts.gTTS(f"Oh, I don't believe we have met before, \
            the pleasure is all mine.")
        tts.save("mp3/instructions.mp3")
        playsound("mp3/instructions.mp3")




# below messages are old ones from before:        

    def another_mark(self):
        self.logger.info("Speaking interaction message for having another go.")
        tts = gtts.gTTS(f"I'm not done yet! Give me a second.")
        tts.save("mp3/instructions.mp3")
        playsound("mp3/instructions.mp3")

    def looks_finished(self):
        self.logger.info("Speaking interaction message for canvas looking finished.")
        tts = gtts.gTTS(f"It looks like a masterpiece already.  Want to keep going?")
        tts.save("mp3/instructions.mp3")
        playsound("mp3/instructions.mp3")

    def hands_away(self):
        self.logger.info("Speaking interaction message for keeping hands away from the canvas.")
        tts = gtts.gTTS(f"Please keep your hands away from the canvas for now.")
        tts.save("mp3/instructions.mp3")
        playsound("mp3/instructions.mp3")

    def your_turn_no_react(self):
        self.logger.info("Speaking interaction message for human's turn.")
        tts = gtts.gTTS(f"{self.name}, make any marks you do NOT want me to react to.")
        tts.save("mp3/instructions.mp3")
        playsound("mp3/instructions.mp3")
        # start a timer... end turn when 60 seconds has passed, or when user presses any key.
        turn_over = False
        while turn_over == False:
            time.sleep(1)
            if keyboard.read_key():
                print("User pressed a key - turn is ending.")
                turn_over = True
        tts = gtts.gTTS(f"Taking a picture - Please move your hands away from the canvas.")
        tts.save("mp3/instructions.mp3")
        playsound("mp3/instructions.mp3")

    def your_turn_short(self):
        self.logger.info("Speaking interaction message for human's turn.")
        tts = gtts.gTTS(f"{self.name}, your turn.  I will react to these marks.")
        tts.save("mp3/instructions.mp3")
        playsound("mp3/instructions.mp3")
        # start a timer... end turn when 60 seconds has passed, or when user presses any key.
        turn_over = False
        while turn_over == False:
            time.sleep(1)
            if keyboard.read_key():
                print("User pressed a key - turn is ending.")
                turn_over = True
        tts = gtts.gTTS(f"My turn!  Please move your hands away from the canvas.")
        tts.save("mp3/instructions.mp3")
        playsound("mp3/instructions.mp3")

    def your_turn_long(self):
        self.logger.info("Speaking interaction message for human's turn.")
        tts = gtts.gTTS(f"{self.name}, now it's your turn to make a mark.  Please use just one color, and make just one mark - do not lift the brush off the canvas. \
           Your turn will end after 60 seconds.  Or, if you want to end it earlier, press any key on the keyboard to let me know you are finished.")
        tts.save("mp3/instructions.mp3")
        playsound("mp3/instructions.mp3")
        # start a timer... end turn when 60 seconds has passed, or when user presses any key.
        time_to_end = 60
        while time_to_end >0:
            time_to_end -= 1
            time.sleep(1)
            if keyboard.read_key():
                print("User pressed a key - turn is ending.")
                break
        tts = gtts.gTTS(f"Your turn is now over, please move your hands away from the canvas.")
        tts.save("mp3/instructions.mp3")
        playsound("mp3/instructions.mp3")

    def color_choice(self):
        self.logger.info("Speaking interaction message for choosing a color")
        tts = gtts.gTTS(f"Color choice time.")
        tts.save("mp3/instructions.mp3")
        playsound("mp3/instructions.mp3")
        numbers_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        correct_input = False
        while correct_input == False:
            col_choice = input("Press zero and then enter to let me choose the color.  Type a number and then press enter to choose the color pot yourself.")
            digit = int(col_choice[-1])
            if digit == 0:
                self.logger.info("I will pick the color")
                correct_input = True
            elif digit in numbers_list:
                self.logger.info("You have chosen color pot %s", col_choice)
                correct_input = True
            else:
                self.logger.warning("Input is not valid!")
        return digit

    def go_again_ask(self):
        self.logger.info("Speaking interaction message for doing this mark response again.")
        tts = gtts.gTTS(f"Would you like me to respond to this mark again?")
        tts.save("mp3/instructions.mp3")
        playsound("mp3/instructions.mp3")
        correct_input = False
        while correct_input == False:
            result = input("Would you like me to try respond to this mark again? Press y and then enter for yes, or n for no.")
            char = result[-1]
            if char == "y":
                self.logger.info("I will try to respond again.")
                correct_input = True
            elif char == "n":
                self.logger.info("We will move on.")
                correct_input = True
            else:
                self.logger.warning("Input is not valid!")
        return char

    def layer_again_ask(self):
        self.logger.info("Speaking interaction message for doing this layer over again.")
        tts = gtts.gTTS(f"Would you like me to do another layer?")
        tts.save("mp3/instructions.mp3")
        playsound("mp3/instructions.mp3")
        correct_input = False
        while correct_input == False:
            result = input("Would you like me to do another layer? Press y and then enter for yes, or n for no.")
            char = result[-1]
            if char == "y":
                self.logger.info("I will do another layer.")
                correct_input = True
            elif char == "n":
                self.logger.info("We will move on.")
                correct_input = True
            else:
                self.logger.warning("Input is not valid!")
        return char
