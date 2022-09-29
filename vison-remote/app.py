import imagezmq
import cv2 as cv

image_hub = imagezmq.ImageHub()

while True:
    image_name, image = image_hub.recv_image()
    cv.imshow(image_name, image)
    cv.waitKey(1)