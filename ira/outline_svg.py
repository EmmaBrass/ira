
# Take an image and get the outline for drawing as a .svg file.
# use xDOG to get outline
# and then use Potrace to convert to .svg

import cv2
import numpy as np
import matplotlib.pyplot as plt
import face_recognition

class SVG():

    def __init__(self) -> None:
        pass

    def find_outline(self, image):

        output = cv2.edgePreservingFilter(image, flags=1, sigma_s=60, sigma_r=0.3)

        edges = cv2.Canny(output,75,150)

        # TODO use facial landmarks from facail_recognition package.
        face_landmarks_list = face_recognition.face_landmarks(image)

        left_eyebrow = np.array([list(ele) for ele in face_landmarks_list[0]['left_eyebrow']])

        print(left_eyebrow)

        cv2.polylines(edges, [left_eyebrow], False, (255,255,255), 1)

        # Display Thresholded XDoG Filtered Image
        cv2.imshow("output", edges)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


    def find_outline_dog(self, image):
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

