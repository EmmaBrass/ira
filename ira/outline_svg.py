
# Take an image and get the outline for drawing as a .svg file.
# use xDOG to get outline
# and then use Potrace to convert to .svg

import cv2
import numpy as np
import matplotlib.pyplot as plt
import face_recognition
from scipy.interpolate import splprep, splev
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision


class SVG():

    def __init__(self) -> None:
        pass

    def draw_smooth_curve(self, image, points):
        """
        Draw a smooth curve on the image based on given points using Catmull-Rom splines.
        """
        # Separate points into two lists
        x, y = zip(*points)
        x = np.array(x)
        y = np.array(y)

        # Fit spline to points
        tck, u = splprep([x, y], s=0)
        
        # Evaluate spline over a fine grid (increase 1000 for smoother curve)
        new_points = splev(np.linspace(0, 1, 1000), tck)
        
        # Draw the interpolated curve on the image
        for i in range(len(new_points[0]) - 1):
            cv2.line(
                image, 
                (int(new_points[0][i]), int(new_points[1][i])), 
                (int(new_points[0][i+1]), int(new_points[1][i+1])), 
                color=(255,255,255), 
                thickness=1
            )

    def resize_and_pad(self, image:np.ndarray, height:int = 256, width:int = 256):
        """
        Input preprocessing function, takes input image in np.ndarray format, 
        resizes it to fit specified height and width with preserving aspect ratio 
        and adds padding on top or right side to complete target height x width rectangle.
        
        :param image: input image in np.ndarray format
        :param height (int, *optional*, 256): target height
        :param width (int, *optional*, 256): target width

        :returns padded_img (np.ndarray): processed image
        :returns padding_info (Tuple[int, int]): information about padding size, 
        for postprocessing
        """
        h, w = image.shape[:2]
        if h < w:
            img = cv2.resize(image, (width, np.floor(h / (w / width)).astype(int)))
        else:
            img = cv2.resize(image, (np.floor(w / (h / height)).astype(int), height))
        
        r_h, r_w = img.shape[:2]
        right_padding = width - r_w
        top_padding = height - r_h
        padded_img = cv2.copyMakeBorder(img, top_padding, 0, 0, right_padding, cv2.BORDER_CONSTANT)
        return padded_img, (top_padding, right_padding)
    
    def find_outline(self, image):
        """
        Given a cropped image of a person's head, return an outline
        drawing-type bitmap image. 
        """
        # Height and width that will be used by the model
        DESIRED_HEIGHT = 256
        DESIRED_WIDTH = 256
        # Perform image resizing with padding
        resized_image, _ = self.resize_and_pad(image, DESIRED_HEIGHT, DESIRED_WIDTH)
        cv2.imwrite("images/resized_image.png", resized_image) 
        output_image = self.find_selfie_segments(resized_image)
        
        large_resized_original, _ = self.resize_and_pad(image, 1028, 1028)
        large_resized_output, _ = self.resize_and_pad(output_image, 1028, 1028)
        output_image = self.find_facial_features(large_resized_original, large_resized_output)

        cv2.imwrite("images/FINAL_image.png", output_image) 

    def find_selfie_segments(self, image):
        """
        Segmentation of clothes, hair, face skin, body skin, from MediaPipe.
        """

        BaseOptions = mp.tasks.BaseOptions
        ImageSegmenter = mp.tasks.vision.ImageSegmenter
        ImageSegmenterOptions = mp.tasks.vision.ImageSegmenterOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        # Colors
        BG_COLOR = (0, 0, 0) # gray
        HAIR_COLOR = (255, 255, 255) # white
        BODY_SKIN_COLOR=(255,0,0)
        FACE_SKIN_COLOR=(0,100,0)
        CLOTHES_COLOR=(0,0,255)
        OTHERS_COLOR=(255,0,100)

        # Create the options that will be used for ImageSegmenter
        model_path = "/home/emma/models/selfie_multiclass_256x256.tflite"

        base_options = BaseOptions(model_asset_path=model_path)
        options = ImageSegmenterOptions(
            base_options=base_options,
            running_mode=VisionRunningMode.IMAGE,
            output_confidence_masks=False,
            output_category_mask=True
        )

        # Create the image segmenter
        with ImageSegmenter.create_from_options(options) as segmenter:

            # Create the MediaPipe image file that will be segmented
            #image = mp.Image.create_from_file(resized_image)
            image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)

            # Retrieve the masks for the segmented image
            segmentation_result = segmenter.segment(image)
            print(segmentation_result)
            category_mask = segmentation_result.category_mask

            print(category_mask)
            print(type(category_mask))

            # Generate solid color images for showing the output segmentation mask.
            image_data = image.numpy_view()
            bg_image = np.zeros(image_data.shape, dtype=np.uint8)
            bg_image[:] = BG_COLOR
            hair_image = np.zeros(image_data.shape, dtype=np.uint8)
            hair_image[:] = HAIR_COLOR
            body_image = np.zeros(image_data.shape, dtype=np.uint8)
            body_image[:] = BODY_SKIN_COLOR
            face_image = np.zeros(image_data.shape, dtype=np.uint8)
            face_image[:] = FACE_SKIN_COLOR
            clothes_image = np.zeros(image_data.shape, dtype=np.uint8)
            clothes_image[:] = CLOTHES_COLOR
            others_image = np.zeros(image_data.shape, dtype=np.uint8)
            others_image[:] = OTHERS_COLOR

            output_image = np.zeros(image_data.shape, dtype=np.uint8)

            images = [bg_image, hair_image, body_image, face_image, clothes_image, others_image]

            # Iterate over each category and apply the mask to the output image
            for category in range(6):
                # Create a binary mask for the current category
                condition = (category_mask.numpy_view() == category)
                # Apply the mask to the output image
                output_image = np.where(condition[..., None], images[category], output_image)

            cv2.imwrite("images/output_image.png", output_image) 

        return output_image 

    def find_facial_features(self, image, output_image):
        """
        Using facial landmark detection from MediaPipe for outline of 
        key facial features: eyebrows, eyes, nose, mouth.
        """

        mp_drawing = mp.solutions.drawing_utils
        mp_drawing_styles = mp.solutions.drawing_styles
        mp_face_mesh = mp.solutions.face_mesh

        # For static image:
        drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)
        jaw_idx = [234, 93, 132, 58, 172, 136, 150, 149, 176, 148, 152, 
        377, 400, 378, 379, 365, 397, 288, 361, 323, 454]
        left_eye_idx = [133, 173, 157, 158, 159, 160, 161, 246, 33, 7, 
        163, 144, 145, 153, 154, 155]
        right_eye_idx = [362, 398, 384, 385, 386, 387, 388, 466, 263, 
        249, 390, 373, 374, 380, 381, 382]
        left_eyebrow_idx = [107, 66, 105, 63, 70, 156, 46, 53, 52, 65, 55]
        right_eyebrow_idx = [336, 296, 334, 293, 300, 383, 276, 283, 282, 295, 285]
        nose_idx = [209, 49, 64, 240, 75, 79, 237, 141, 94, 370, 
        457, 309, 305, 460, 294, 279, 429]
        top_lip_idx = [13, 312, 311, 310, 415, 306, 291, 409, 270, 269, 267, 
        0, 37, 39, 40, 185, 61, 78, 191, 80, 81, 82]
        bottom_lip_idx = [14, 317, 402, 318, 324, 306, 291, 375, 321, 405, 314, 17, 84, 181, 91, 146, 61, 78, 95, 88, 178, 87]
        left_iris_idx = [470, 469, 472,471]
        right_iris_idx = [475, 474, 477, 476]
        nose_bridge_left_idx = [245, 188,174, 236]
        nose_bridge_right_idx = [465, 412, 399, 456]
        image_hight, image_width, _ = image.shape
        with mp_face_mesh.FaceMesh(
                static_image_mode=True,
                max_num_faces=1,
                refine_landmarks=True,
                min_detection_confidence=0.5
            ) as face_mesh:
            results = face_mesh.process(image)
            # Print and draw face mesh landmarks on the image.
            if not results.multi_face_landmarks:
                print("Error!")
            for face_landmarks in results.multi_face_landmarks: # https://stackoverflow.com/questions/67141844/python-how-to-get-face-mesh-landmarks-coordinates-in-mediapipe
                jaw = []
                left_eye = []
                right_eye = []
                left_iris = []
                right_iris = []
                left_eyebrow = []
                right_eyebrow = []
                nose = []
                top_lip = []
                bottom_lip = []
                nose_bridge_left = []
                nose_bridge_right = []
                # for i in jaw_idx:
                #     x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                #     y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                #     jaw.append([x_coodinate,y_coodinate])
                #     cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in left_eye_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    left_eye.append([x_coodinate,y_coodinate])
                    cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in right_eye_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    right_eye.append([x_coodinate,y_coodinate])
                for i in left_iris_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    left_iris.append([x_coodinate,y_coodinate])
                    cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in right_iris_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    right_iris.append([x_coodinate,y_coodinate])
                    cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in left_eyebrow_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    left_eyebrow.append([x_coodinate,y_coodinate])
                    cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in right_eyebrow_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    right_eyebrow.append([x_coodinate,y_coodinate])
                    cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in nose_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    nose.append([x_coodinate,y_coodinate])
                    cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in top_lip_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    top_lip.append([x_coodinate,y_coodinate])
                    cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in bottom_lip_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    bottom_lip.append([x_coodinate,y_coodinate])
                    cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in nose_bridge_left_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    nose_bridge_left.append([x_coodinate,y_coodinate])
                    cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in nose_bridge_right_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    nose_bridge_right.append([x_coodinate,y_coodinate])
                    #cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                # self.draw_smooth_curve(output_image, jaw)
                self.draw_smooth_curve(output_image, left_eye)
                self.draw_smooth_curve(output_image, right_eye)
                self.draw_smooth_curve(output_image, left_iris)
                self.draw_smooth_curve(output_image, right_iris)
                self.draw_smooth_curve(output_image, left_eyebrow)
                self.draw_smooth_curve(output_image, right_eyebrow)
                self.draw_smooth_curve(output_image, nose)
                self.draw_smooth_curve(output_image, top_lip)
                self.draw_smooth_curve(output_image, bottom_lip)
                self.draw_smooth_curve(output_image, nose_bridge_left)
                #self.draw_smooth_curve(output_image, nose_bridge_right)
                
            # Display Image
            cv2.imwrite("images/annotated.png", output_image) 

        return output_image
                

    def find_outline_dog(self, image):
        """
        OLD - likely won't use.
        """
        # Parameters
        gamma = 0.98
        phi = 50
        epsilon = -0.1
        k = 1.6
        sigma = 3

        # Read and preprocess the image
        input_im = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        input_im = input_im.astype(np.float32) / 255.0

        # Apply Gaussian filters
        g_filtered_im1 = cv2.GaussianBlur(input_im, (0, 0), sigma)
        g_filtered_im2 = cv2.GaussianBlur(input_im, (0, 0), sigma * k)

        # Difference of Gaussians
        differenced_im2 = g_filtered_im1 - (gamma * g_filtered_im2)

        # Extended difference of gaussians
        for i in range(differenced_im2.shape[0]):
            for j in range(differenced_im2.shape[1]):
                if differenced_im2[i, j] < epsilon:
                    differenced_im2[i, j] = 1
                else:
                    differenced_im2[i, j] = 1 + np.tanh(phi * (differenced_im2[i, j]))

        # Display XDoG Filtered Image
        plt.figure(), plt.imshow(differenced_im2, cmap='gray')
        plt.title('XDoG Filtered Image')

        xdog_filtered_image = differenced_im2

        # Take mean of XDoG Filtered image to use in thresholding operation
        mean_value = np.mean(xdog_filtered_image)

        # Thresholding
        xdog_filtered_image[xdog_filtered_image <= mean_value] = 0.0
        xdog_filtered_image[xdog_filtered_image > mean_value] = 1.0

        # Display Thresholded XDoG Filtered Image
        plt.figure(), plt.imshow(xdog_filtered_image, cmap='gray')
        plt.title('Thresholded XDoG Filtered Image')

        # Save XDoG Filtered Image and the thresholded one
        cv2.imwrite('XDoGFilter.jpg', np.uint8(differenced_im2 * 255))
        cv2.imwrite('XDoGFilterThresholded.jpg', np.uint8(xdog_filtered_image * 255))
