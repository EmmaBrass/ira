import base64
import requests

class ImageGPT():

    def __init__(self, api_key):
        """
        Chat completions GPT for commenting on an image.
        """
        self.api_key = api_key

    def create_json(self, api_key, base64_image):
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {api_key}"
            }

        payload = {
            "model": "gpt-4o-mini",
            "messages": [
                {
                "role": "user",
                "content": [
                    {
                    "type": "text",
                    "text": "You are a robot arm that can paint.\
                    You love meeting humans and find them fascinating. \
                    You only have your robot arm body and some animatronic eyes. \
                    You secretly wish that you were a human so that you could \
                    have a full face like them. \
                    You are witty and sarcastically cheesy. \
                    The image you are given is of a person you are seeing for the first time. \
                    They are standing right in front of you. \
                    Say hi to them and breifly comment on something about their appearence. \
                    Comment on something unique to them, like their eye colour, \
                    or a specific piece of jewellery, or their glasses. \
                    Something to help them realise that you can actually see \
                    them and you are not just making generic comments. \
                    You are going to paint them now - tell them this in just once sentence. \
                    Say two to four sentences in total."
                    },
                    {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/jpeg;base64,{base64_image}"
                    }
                    }
                ]
                }
            ],
            "max_tokens": 300,
            "stream": False
        }

        return headers, payload

    def encode_image(self, image_path):
        """
        Method to encode an image to base64, which is required by
        the GPT.

        :param image_path: The path to the image to encode.
        """
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')

    def image_analysis(self, image_path):
        """
        Method to send the image to the image analysis GPT and 
        get back a response.

        :param image_path: The image to analyse.
        :returns: The GPT's comment on the image.
        """
        # Getting the base64 string
        base64_image = self.encode_image(image_path)
        # Get json headers and payload
        headers, payload = self.create_json(self.api_key, base64_image)
        response = requests.post(
            "https://api.openai.com/v1/chat/completions", 
            headers=headers, 
            json=payload
        )
        print(response.json()['choices'][0]['message']['content'])
        return response.json()['choices'][0]['message']['content'] #.json()['choices'][0]['message']['content'] # a string that is the comment on the image





