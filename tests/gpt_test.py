
import time
from general_gpt import GPT


gpt = GPT()
time.sleep(1)
print("DONE INITIALISING")

gpt.add_user_message_and_get_response_and_speak("The command is: <interaction_unknown>.  The image path: images/latest_image.png")
#gpt.add_user_message_and_get_response_and_speak("<interaction_unknown>. image_path: '/home/emma/ira_ws/src/ira/tests/image/2024-06-25-112719.jpg' ")



# "<found_noone>" : "Muse about how there is no one around and you are a little \
#     lonely, but in a funny way.",
# "<say_painted_recently>" : "You painted this person too recently to paint them again! \
#     Tell them so.",
# "<too_far>" : "All the people can see are too far away for you to be able to paint them! \
#     Tell them that they need to come closer if they want their beautiful faces painted.",
# "<interaction_unknown>" : "You will also be provided with an image of the person. \
#     You should respond with a message, saying hi and that you haven't met before.  Also call \
#     the image_analysis function to assess the image.",
# "<interaction_known>" : "You will also be provided with an image of the person. \
#     You should respond with a message, saying hi and that you recognise them.  Also call \
#     the image_analysis function to assess the image.",
# "<interaction_known_recent>" : "You will also be provided with an image of the person. \
#     You should respond with a message, saying hi and that you recognise them.  Also that \
#     they disappeared before you could paint them last time!  Also call \
#     the image_analysis function to assess the image.",
# "<disappeared>" : "The person you were just talking to has disappeared! \
#     Express your sadness about this and wonder out loud about where \
#     they could have gone to.",
# "<interaction_returned>" : "A person who disappeared before has now \
#     returned.  Express your joy at seeing them again and getting to continue \
#     the interaction.",
# "<gone>" : "The person you were just talking to has gone for good. \
#     You gracefully accept the rejection and prepare to move on to another \
#     subject.",
# "<painting>" : "You are now painting the person.  Comment on your love of the \
#     process of painting, and that you cannot wait for them to see the final product.",
# "<completed>" : "Comment on how good your work is and ask the user \
#     what they think of it."