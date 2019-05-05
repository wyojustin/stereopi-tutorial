# Copyright (C) 2019 Eugene Pomazov, <stereopi.com>, virt2real team
#
# This file is part of StereoPi tutorial scripts.
#
# StereoPi tutorial is free software: you can redistribute it 
# and/or modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
#
# StereoPi tutorial is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with StereoPi tutorial.  
# If not, see <http://www.gnu.org/licenses/>.
#
# Most of this code is updated version of 3dberry.org project by virt2real
# 
# Thanks to Adrian and http://pyimagesearch.com, as there are lot of
# code in this tutorial was taken from his lessons.
# 

import os
import time
from datetime import datetime
import picamera
from picamera import PiCamera
import cv2
import numpy as np
from stereovision.calibration import StereoCalibrator
from stereovision.exceptions import ChessboardNotFoundError

from config import camera_hflip, camera_vflip, photo_width, photo_height
from config import img_width, img_height, image_size
from config import rows, columns, square_size
from config import total_photos, countdown

# Photo session settings
font=cv2.FONT_HERSHEY_SIMPLEX # Cowntdown timer font
 
# Camera settimgs
cam_width = 1280              # Cam sensor width settings
cam_height = 480              # Cam sensor height settings

# Final image capture settings
scale_ratio = 0.5

# Camera resolution height must be dividable by 16, and width by 32
cam_width = int((cam_width+31)/32)*32
cam_height = int((cam_height+15)/16)*16
print ("Used camera resolution: "+str(cam_width)+" x "+str(cam_height))

# Buffer for captured image settings
img_width = int (cam_width * scale_ratio)
img_height = int (cam_height * scale_ratio)
capture = np.zeros((img_height, img_width, 4), dtype=np.uint8)
print ("Scaled image resolution: "+str(img_width)+" x "+str(img_height))

# Initialize the camera
camera = PiCamera(stereo_mode='side-by-side', stereo_decimate=False)
camera.resolution=(cam_width, cam_height)
camera.framerate = 5
camera.hflip = camera_hflip
camera.vflip = camera_vflip

calibrator = StereoCalibrator(rows, columns,
                              square_size, image_size)

# Lets start taking photos! 
counter = 0
t2 = datetime.now()
print ("Starting photo sequence")

all_left_corners = np.array([[0, 0]])
all_right_corners = np.array([[0, 0]])

for frame in camera.capture_continuous(capture, format="bgra", \
                  use_video_port=True, resize=(img_width,img_height)):
    t1 = datetime.now()
    cntdwn_timer = countdown - int ((t1-t2).total_seconds())
    # If cowntdown is zero - let's record next image
    if cntdwn_timer <= -1:
      ## check frame for two boards
      imgLeft = frame[0:img_height,0:img_width//2] #Y+H and X+W
      imgRight = frame[0:img_height,img_width//2:]
      try:
        left_corners = np.squeeze(calibrator._get_corners(imgLeft))
        right_corners = np.squeeze(calibrator._get_corners(imgRight))
        all_left_corners = np.vstack([all_left_corners,
                                      left_corners.astype(int)])
        all_right_corners = np.vstack([all_right_corners,
                                       right_corners.astype(int)])
      except ChessboardNotFoundError as error:
        print (error)
        print ("Pair No try again")
        t2 = datetime.now()
        continue

      counter += 1
      filename = './scenes/scene_'+str(img_width)+'x'+str(img_height)+'_'+\
                  str(counter) + '.png'
      cv2.imwrite(filename, frame)
      print (' ['+str(counter)+' of '+str(total_photos)+'] '+filename)
      t2 = datetime.now()
      time.sleep(1)
      cntdwn_timer = 0      # To avoid "-1" timer display 
      next
    # Draw cowntdown counter, seconds
    cv2.putText(frame, str(cntdwn_timer), (50,50), font, 2.0, (0,0,255),4, cv2.LINE_AA)
    for i in range(1, len(all_left_corners)):
        cv2.circle(frame, (all_left_corners[i, 0], all_left_corners[i, 1]),
                   2, (255, 0, 0), -1)
        cv2.circle(
            frame,
            (320 + all_right_corners[i, 0], all_right_corners[i, 1]),
             2, (0, 0, 255), -1)

    
    cv2.imshow("pair", frame[::-1,::-1])
    key = cv2.waitKey(1) & 0xFF
    
    # Press 'Q' key to quit, or wait till all photos are taken
    if (key == ord("q")) | (counter == total_photos):
      break
  
if counter == total_photos:
    print ("Photo sequence finished")


    # Global variables reset
    photo_counter = 0


    # Main pair cut cycle
    if (os.path.isdir("./pairs")==False):
        os.makedirs("./pairs")
    while photo_counter != total_photos:
        photo_counter +=1
        filename = './scenes/scene_'+str(photo_width)+'x'+str(photo_height)+\
                   '_'+str(photo_counter) + '.png'
        if os.path.isfile(filename) == False:
            print ("No file named "+filename)
            continue
        pair_img = cv2.imread(filename,-1)

        imgLeft = pair_img [0:img_height,0:img_width//2] #Y+H and X+W
        imgRight = pair_img [0:img_height,img_width//2:img_width]
        leftName = './pairs/left_'+str(photo_counter).zfill(2)+'.png'
        rightName = './pairs/right_'+str(photo_counter).zfill(2)+'.png'
        cv2.imwrite(leftName, imgLeft)
        cv2.imwrite(rightName, imgRight)
        print ('Pair No '+str(photo_counter)+' saved.')
    cv2.imshow("ImagePair", pair_img)
    print ('End cycle (type any key in preview window to end)')
    cv2.waitKey(0)
    
