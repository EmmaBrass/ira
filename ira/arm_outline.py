
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

# TODO what do I want to get from here?
# Need to look into arm commands more
# A series of straight points along the line that 
# the robot can move between in linear mode,
# to make what appears to be a straight line?

class Outline():

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
        return padded_img, [top_padding, right_padding]
    
    def find_contours_coordinates(self, image):
        """
        Finds canny edges image with white lines on black background.
        Finds contours using opencv function.
        Returns a list of coordinate arrays, along with image dimensions.
        These can then be turned into paths for robot motion.
        """


        canny_image = self.no_background_canny(image)
        resized_image, padding = self.resize_and_pad(image, 1028, 1028)
        self.find_facial_features(resized_image, canny_image)
        cv2.imwrite('images/with_features.png', canny_image)

        # Threshold the image to ensure it is binary
        _, binary_image = cv2.threshold(canny_image, 127, 255, cv2.THRESH_BINARY)

        # Remove the padding
        # Determine the new dimensions
        height, width = binary_image.shape[:2]  # Get the original dimensions
        new_height = height - padding[0]
        new_width = width - padding[1]

        # Ensure the new dimensions are valid
        if new_height <= 0 or new_width <= 0:
            raise ValueError("The amount to remove is too large, resulting in a non-positive dimension.")

        # Crop the image using array slicing
        cropped_image = binary_image[padding[0]:height, 0:new_width]

        # Find contours of the white lines
        contours, _ = cv2.findContours(cropped_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find the lengths and keep only the longer contours
        contour_lengths = []
        long_contours = []
        for contour in contours:
            length = cv2.arcLength(contour, True)  # True indicates the contour is closed
            contour_lengths.append(length)
            if length > 10:
                long_contours.append(contour)
        print("contour_lengths", contour_lengths)

        # Extract the coordinates of the contours
        path_points = []
        for contour in long_contours:
            contour_points = []
            for point in contour:
                contour_points.append(tuple(point[0]))  # Each point is a tuple of (x, y)
            path_points.append(contour_points)

        # Optionally draw the path on the original image to visualize
        output_image = cv2.cvtColor(cropped_image, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(output_image, contours, -1, (255,0,0), 1)
        cv2.drawContours(output_image, long_contours, -1, (0,255,0), 1)

        # Display the result
        cv2.imshow('Path Visualization', output_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Save the path visualization (optional)
        cv2.imwrite('images/path_visualization.png', output_image)

        # Print the path points
        print("Path points:", path_points)
        print("image dimensions: ", cropped_image.shape[1], cropped_image.shape[0] )

        return path_points, cropped_image.shape[1], cropped_image.shape[0] # image points, image x dimension, image y dimension
    
    def no_background_canny(self, image):
        """
        Removes background using segmentation model.
        Fills in details using Canny edge dection.
        Adds in jaw line if no beard?
        """
        # Height and width that will be used by the model
        DESIRED_HEIGHT = 256
        DESIRED_WIDTH = 256
        # Perform image resizing with padding
        resized_image, _ = self.resize_and_pad(image, DESIRED_HEIGHT, DESIRED_WIDTH)
        cv2.imwrite("images/resized_image.png", resized_image) 
        small_no_background_mask = self.find_no_background_mask(resized_image)
        small_face_mask = self.find_face_mask(resized_image)
        small_not_face_mask = self.find_not_face_mask(resized_image)
        large_resized_face_mask, _ = self.resize_and_pad(small_face_mask, 1028, 1028)
        large_resized_no_background_mask, _ = self.resize_and_pad(small_no_background_mask, 1028, 1028)
        large_resized_not_face_mask, _ = self.resize_and_pad(small_not_face_mask, 1028, 1028)
        large_resized_original, _ = self.resize_and_pad(image, 1028, 1028)

        cv2.imwrite("images/large_resized_face_mask.png", large_resized_face_mask)
        cv2.imwrite("images/large_resized_no_background_mask.png", large_resized_no_background_mask)  
        cv2.imwrite("images/large_resized_original.png", large_resized_original) 

        # Turn no background mask to greyscale
        if len(large_resized_no_background_mask.shape) == 3:
            gray_no_background_mask = cv2.cvtColor(large_resized_no_background_mask, cv2.COLOR_BGR2GRAY)
            print("here")
        else:
            gray_no_background_mask = large_resized_no_background_mask

        # Ensure the mask is a binary mask with values 0 and 255
        _, binary_mask = cv2.threshold(gray_no_background_mask, 127, 255, cv2.THRESH_BINARY)
        cv2.imwrite("images/binary_mask.png", binary_mask) 

        # Apply Gaussian blur to smooth the edges
        blurred_mask = cv2.GaussianBlur(binary_mask, (15, 15), 0)
        cv2.imwrite("images/blurred_mask.png", blurred_mask) 

        # Re-apply thresholding to keep the mask binary
        _, smoothed_mask = cv2.threshold(blurred_mask, 127, 255, cv2.THRESH_BINARY)

        # Apply morphological closing to further refine the edges
        kernel = np.ones((5, 5), np.uint8)
        closed_mask = cv2.morphologyEx(smoothed_mask, cv2.MORPH_CLOSE, kernel)
        cv2.imwrite("images/closed_mask.png", closed_mask) 

        # Turn original image to greyscale
        if len(large_resized_original.shape) == 3:
            gray = cv2.cvtColor(large_resized_original, cv2.COLOR_BGR2GRAY)
            print("here")
        else:
            gray = large_resized_original

        # Apply the mask to the image
        no_background = cv2.bitwise_and(gray, closed_mask)
        cv2.imwrite("images/no_background.png", no_background) 

        # Turn face mask to greyscale
        if len(large_resized_face_mask.shape) == 3:
            gray_face_mask = cv2.cvtColor(large_resized_face_mask, cv2.COLOR_BGR2GRAY)
            print("here")
        else:
            gray_face_mask = large_resized_face_mask

        # Ensure face mask is binary
        face_mask = cv2.threshold(gray_face_mask, 127, 255, cv2.THRESH_BINARY)[1]

        print("face_mask shape:", face_mask.shape)
        print("gray shape:", gray.shape)

        # Extract the pixel values where the mask is white
        face_masked_pixels = gray[face_mask == 255]

        # Compute the median of the masked pixel intensities
        v = np.median(face_masked_pixels)
        print("v", v)

        # TODO apply a blur of (9,9) only to the not_face parts of the image!
        # Turn not_face mask to greyscale
        if len(large_resized_not_face_mask.shape) == 3:
            gray_not_face_mask = cv2.cvtColor(large_resized_not_face_mask, cv2.COLOR_BGR2GRAY)
            print("here")
        else:
            gray_not_face_mask = large_resized_not_face_mask

        # Ensure the mask is a binary mask with values 0 and 255
        _, binary_not_face_mask = cv2.threshold(gray_not_face_mask, 127, 255, cv2.THRESH_BINARY)
        cv2.imwrite("images/not_face_mask.png", binary_not_face_mask) 

        inverse_binary_not_face_mask = cv2.bitwise_not(binary_not_face_mask)

        # Apply a blur to the whole (no background) image
        blurred = cv2.GaussianBlur(no_background, (5,5), 0)

        # Apply extreme Gaussian blur to the entire image
        very_blurred = cv2.GaussianBlur(no_background, (9, 9), 0)

        # Combine the blurred image and the original image using the masks
        blurred_part = cv2.bitwise_and(very_blurred, binary_not_face_mask)
        original_part = cv2.bitwise_and(blurred, inverse_binary_not_face_mask)
        result = cv2.add(blurred_part, original_part)
        cv2.imwrite("images/blurred_not_face.png", blurred_part) 
        cv2.imwrite("images/is_face.png", original_part) 
        cv2.imwrite("images/combined_blurred.png", result) 

        # Setting canny edge detection parameter values based on the face mask pixels
        sigma = 0.45
        t_lower = 0 #int(max(0, (1.0-sigma)*v))
        t_upper = 50 #int(min(255, (sigma*v))) #TODO needs adjusting?

        cv2.imwrite("images/blurred.png", blurred) 

        result_blurred = cv2.Canny(result, t_lower, t_upper)
        result_no_blur = cv2.Canny(no_background, t_lower, t_upper)

        canny_face_mask = cv2.Canny(face_mask, t_lower, t_upper)
        cv2.imwrite("images/canny_face_mask.png", canny_face_mask) 


        cv2.imwrite("images/original.png", image) 
        cv2.imwrite("images/canny_image_no_blur.png", result_no_blur) 
        cv2.imwrite("images/canny_image_blurred.png", result_blurred) 

        return result_blurred 
    

    def find_not_face_mask(self, image):
        """
        Segmentation of image to return a mask of not face (and not background).
        """

        BaseOptions = mp.tasks.BaseOptions
        ImageSegmenter = mp.tasks.vision.ImageSegmenter
        ImageSegmenterOptions = mp.tasks.vision.ImageSegmenterOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        # Colors
        BG_COLOR = (0, 0, 0) # black
        HAIR_COLOR = (255, 255, 255) # white
        BODY_SKIN_COLOR = (255, 255, 255)
        FACE_SKIN_COLOR = (0, 0, 0)
        CLOTHES_COLOR = (255, 255, 255)
        OTHERS_COLOR = (0, 0, 0)

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

            cv2.imwrite("images/hair_mask.png", output_image) 

        return output_image 
        

    def find_no_background_mask(self,image):
        """
        Segmentation of image to return a mask of everything 
        but that background.
        """

        BaseOptions = mp.tasks.BaseOptions
        ImageSegmenter = mp.tasks.vision.ImageSegmenter
        ImageSegmenterOptions = mp.tasks.vision.ImageSegmenterOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        # Colors
        BG_COLOR = (0, 0, 0) # black
        HAIR_COLOR = (255, 255, 255) # white
        BODY_SKIN_COLOR = (255,255,255)
        FACE_SKIN_COLOR = (255,255,255)
        CLOTHES_COLOR = (255,255,255)
        OTHERS_COLOR = (255,255,255)

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

            cv2.imwrite("images/no_background_mask.png", output_image) 

        return output_image 

    def find_face_mask(self, image):
        """
        Segmentation of the image and then returns a mask consisting of the 
        face skin only, in white, on a black background.
        """

        BaseOptions = mp.tasks.BaseOptions
        ImageSegmenter = mp.tasks.vision.ImageSegmenter
        ImageSegmenterOptions = mp.tasks.vision.ImageSegmenterOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        # Colors
        BG_COLOR = (0, 0, 0) # black
        HAIR_COLOR = (0, 0, 0) # black
        BODY_SKIN_COLOR = (0, 0, 0) # black
        FACE_SKIN_COLOR = (255, 255, 255) # white

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

            output_image = np.zeros(image_data.shape, dtype=np.uint8)

            images = [bg_image, hair_image, body_image, face_image]

            # Iterate over each category and apply the mask to the output image
            for category in range(4):
                # Create a binary mask for the current category
                condition = (category_mask.numpy_view() == category)
                # Apply the mask to the output image
                output_image = np.where(condition[..., None], images[category], output_image)

            cv2.imwrite("images/face_mask.png", output_image) 

        return output_image 


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
        BODY_SKIN_COLOR=(200,200,200)
        FACE_SKIN_COLOR=(150,150,150)
        CLOTHES_COLOR=(100,100,100)
        OTHERS_COLOR=(50,50,50)

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
    
    def find_segmentation_canny(self, image):
        """
        Returns canny edges of a segmentated image showing edges
        between hair, face, body, clothes, accessories.
        """

        # Height and width that will be used by the model
        DESIRED_HEIGHT = 256
        DESIRED_WIDTH = 256
        # Perform image resizing with padding
        resized_image, _ = self.resize_and_pad(image, DESIRED_HEIGHT, DESIRED_WIDTH)
        cv2.imwrite("images/resized_image.png", resized_image) 
        output_image = self.find_selfie_segments(resized_image)
        
        large_resized_original, _ = self.resize_and_pad(image, 1028, 1028)
        large_resized_segments, _ = self.resize_and_pad(output_image, 1028, 1028)

        cv2.imwrite("images/large_resized_segments.png", large_resized_segments) 

        # Turn original image to greyscale
        if len(large_resized_segments.shape) == 3:
            gray = cv2.cvtColor(large_resized_segments, cv2.COLOR_BGR2GRAY)
        else:
            gray = large_resized_segments

        cv2.imwrite("images/gray_segments.png", gray)

        t_lower = 0 
        t_upper = 50

        result_no_blur = cv2.Canny(gray, t_lower, t_upper)

        cv2.imwrite("images/original_segments.png", large_resized_original) 
        cv2.imwrite("images/canny_segments_no_blur.png", result_no_blur) 

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
        jaw_idx = [172, 136, 150, 149, 176, 148, 152, 
        377, 400, 378, 379, 365, 397]
        # Full list of jaw points: [234, 93, 132, 58, 172, 136, 150, 149, 176, 148, 152, 
        # 377, 400, 378, 379, 365, 397, 288, 361, 323, 454]
        left_eye_idx = [133, 173, 157, 158, 159, 160, 161, 246, 33, 7, 
        163, 144, 145, 153, 154, 155]
        right_eye_idx = [362, 398, 384, 385, 386, 387, 388, 466, 263, 
        249, 390, 373, 374, 380, 381, 382]
        left_eyebrow_idx = [107, 66, 105, 63, 70, 156, 46, 53, 52, 65, 55]
        right_eyebrow_idx = [336, 296, 334, 293, 300, 383, 276, 283, 282, 295, 285]
        nose_idx = [64, 240, 75, 79, 237, 141, 94, 370, 
        457, 309, 305, 460, 294]
        # full list of nose points: [209, 49, 64, 240, 75, 79, 237, 141, 94, 370, 
        #457, 309, 305, 460, 294, 279, 429]
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
                for i in jaw_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    jaw.append([x_coodinate,y_coodinate])
                    #cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in left_eye_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    left_eye.append([x_coodinate,y_coodinate])
                    #cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in right_eye_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    right_eye.append([x_coodinate,y_coodinate])
                for i in left_iris_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    left_iris.append([x_coodinate,y_coodinate])
                    #cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in right_iris_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    right_iris.append([x_coodinate,y_coodinate])
                    #cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in left_eyebrow_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    left_eyebrow.append([x_coodinate,y_coodinate])
                    #cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in right_eyebrow_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    right_eyebrow.append([x_coodinate,y_coodinate])
                    #cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in nose_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    nose.append([x_coodinate,y_coodinate])
                    #cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in top_lip_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    top_lip.append([x_coodinate,y_coodinate])
                    #cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in bottom_lip_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    bottom_lip.append([x_coodinate,y_coodinate])
                    #cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in nose_bridge_left_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    nose_bridge_left.append([x_coodinate,y_coodinate])
                    #cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                for i in nose_bridge_right_idx:
                    x_coodinate = int(face_landmarks.landmark[i].x * image_width)
                    y_coodinate = int(face_landmarks.landmark[i].y * image_hight)
                    nose_bridge_right.append([x_coodinate,y_coodinate])
                    #cv2.circle(output_image, (x_coodinate, y_coodinate), 2, (0, 0, 255), -1)
                self.draw_smooth_curve(output_image, jaw)
                # self.draw_smooth_curve(output_image, left_eye)
                # self.draw_smooth_curve(output_image, right_eye)
                # self.draw_smooth_curve(output_image, left_iris)
                # self.draw_smooth_curve(output_image, right_iris)
                # self.draw_smooth_curve(output_image, left_eyebrow)
                # self.draw_smooth_curve(output_image, right_eyebrow)
                self.draw_smooth_curve(output_image, nose)
                # self.draw_smooth_curve(output_image, top_lip)
                # self.draw_smooth_curve(output_image, bottom_lip)
                # self.draw_smooth_curve(output_image, nose_bridge_left)
                # self.draw_smooth_curve(output_image, nose_bridge_right)
                
            # Display Image
            cv2.imwrite("images/annotated.png", output_image) 

        return output_image
                