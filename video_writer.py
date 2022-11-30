import cv2
import os

image_folder = 'images'
video_name = 'video.avi'

images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
images = [("drone_step"+str(i)+".png") for i in range(10,2990,10)]
# images = [0]*2990
# list = range(10, 2990, 10)
# for i in list:
#     images[int(i/10-1)] = os.path.join(image_folder,"drone_step"+str(i)+".png")
#     #images = images.append(os.path.join(image_folder,"drone_step"+str(i)+".png"))
# print(list)
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape

video = cv2.VideoWriter(video_name, 0, 10, (width,height))

for image in images:
    video.write(cv2.imread(os.path.join(image_folder, image)))

cv2.destroyAllWindows()
video.release()