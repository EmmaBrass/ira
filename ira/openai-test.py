from interaction_gpt import GPT


gpt = GPT()
    
gpt.add_user_message_and_get_response_and_speak(
    "Hi, here is a path to an image to analyse: \
    '/Users/Emma/Documents/Documents_MacBook_Pro/pen_plotter/Interactive_Arm/images/test14.jpeg'."
)