import cv2
import numpy as np
from datetime import datetime
import array
import fcntl
import os
import argparse
from utils import ArducamUtils
import time
import sys
import RPi.GPIO as GPIO


#video_result1 = None

def resize(frame, dst_width):
    width = frame.shape[1]
    height = frame.shape[0]
    scale = dst_width * 1.0 / width
    return cv2.resize(frame, (int(scale * width), int(scale * height)))

def display(cap, arducam_utils,args,fps = False):
    exposure = 680
    counter = 0
    start_time = datetime.now()
    frame_count = 0
    start = time.time()
    
    #for video
    video_file_name = "videos/video_{}_{}.avi".format(datetime.now(),args.device)
    fcc = cv2.VideoWriter_fourcc('X','V','I','D')
    video_result1 = cv2.VideoWriter(video_file_name,fcc,20.0,(5120,800)) 
    
    date_now = datetime.now().strftime("%Y_%m_%d_%H_%M_%S_%f")
    img_folder_name = "image/{}_{}".format(date_now,args.device)
    if not os.path.exists(img_folder_name):
        os.makedirs(img_folder_name)
    
    while True:
        ret, frame = cap.read()
        counter += 1
        frame_count += 1

        if arducam_utils.convert2rgb == 0:
            w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            frame = frame.reshape(int(h), int(w))

        frame = arducam_utils.convert(frame)
        
        #frame = resize(frame, 1280.0)
        show_frame = frame = resize(frame, 1280.0)
        frame = resize(frame, 5120.0)
        # display
        cv2.imshow("Arducam", show_frame)
        ret = cv2.waitKey(1)
        # press 'q' to exit.
        if ret == ord('q'):
            break
        
         
        if ret == ord("="):
            exposure = exposure + 100
            if exposure > 65535 : 
               exposure = 65535 
            exp_str = "v4l2-ctl --set-ctrl=exposure={}".format(exposure)
            os.system(exp_str)
            print ("exposure reset to : " + str(exposure))
            #cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, exposure)
        if ret == ord("-"):
            exposure = exposure - 100
            if exposure < 0 : 
               exposure = 0 
            exp_str = "v4l2-ctl --set-ctrl=exposure={}".format(exposure)
            os.system(exp_str)
            print ("exposure reset to : " + str(exposure))        
        
        date_now2 = datetime.now().strftime("%Y_%m_%d_%H_%M_%S_%f")
        img_name = "{}/{}_{}.png".format(img_folder_name,date_now2,args.device)
        try:
            cv2.imwrite(img_name,frame)
        except:
            print("???? Unable to save, ????")
            break

        #frame = cv2.cvtColor(frame,cv2.COLOR_GRAY2BGR)
        #video_result1.write(frame)
        if args.device == 0:
            GPIO.output(37, GPIO.HIGH)
        if args.device == 1:
            GPIO.output(38, GPIO.HIGH)

        if fps and time.time() - start >= 1:
            if sys.version[0] == '2':
                print("fps: {}".format(frame_count))    
            else:
                print("fps: {}".format(frame_count),end='\r')
            start = time.time()
            frame_count = 0 

    end_time = datetime.now()
    elapsed_time = end_time - start_time
    avgtime = elapsed_time.total_seconds() / counter
    print ("Average time between frames: " + str(avgtime))
    print ("Average FPS: " + str(1/avgtime))

    
    GPIO.output(37, GPIO.LOW)
    GPIO.output(38, GPIO.LOW)
    cap.release()
    video_result1.release()

def print_test(v):
    while True:
        print("this is thread{}".format(v))

def fourcc(a, b, c, d):
    return ord(a) | (ord(b) << 8) | (ord(c) << 16) | (ord(d) << 24)

def pixelformat(string):
    if len(string) != 3 and len(string) != 4:
        msg = "{} is not a pixel format".format(string)
        raise argparse.ArgumentTypeError(msg)
    if len(string) == 3:
        return fourcc(string[0], string[1], string[2], ' ')
    else:
        return fourcc(string[0], string[1], string[2], string[3])

def show_info(arducam_utils):
    _, firmware_version = arducam_utils.read_dev(ArducamUtils.FIRMWARE_VERSION_REG)
    _, sensor_id = arducam_utils.read_dev(ArducamUtils.FIRMWARE_SENSOR_ID_REG)
    _, serial_number = arducam_utils.read_dev(ArducamUtils.SERIAL_NUMBER_REG)
    print("Firmware Version: {}".format(firmware_version))
    print("Sensor ID: 0x{:04X}".format(sensor_id))
    print("Serial Number: 0x{:08X}".format(serial_number))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Arducam Jetson Nano MIPI Camera Displayer.')

    parser.add_argument('-d', '--device', default=0, type=int, nargs='?',
                        help='/dev/videoX default is 0')
    parser.add_argument('-f', '--pixelformat', type=pixelformat,
                        help="set pixelformat")
    parser.add_argument('--width', type=lambda x: int(x,0),
                        help="set width of image")
    parser.add_argument('--height', type=lambda x: int(x,0),
                        help="set height of image")
    parser.add_argument('--fps', action='store_true', help="display fps")

    args = parser.parse_args()

    #time.sleep(50)

    #setup gpio pin
    # Set up the GPIO channel
    GPIO.setmode(GPIO.BOARD) 
    GPIO.setup(37, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(38, GPIO.OUT, initial=GPIO.LOW)

    # open camera
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
   

    # set pixel format
    if args.pixelformat != None:
        if not cap.set(cv2.CAP_PROP_FOURCC, args.pixelformat):
            print("Failed to set pixel format.")

    arducam_utils = ArducamUtils(args.device)
    #arducam_utils2 = ArducamUtils(1)

    show_info(arducam_utils)
    # turn off RGB conversion
    if arducam_utils.convert2rgb == 0:
        cap.set(cv2.CAP_PROP_CONVERT_RGB, arducam_utils.convert2rgb)
    # set width
    #if args.width != None:
    #    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    # set height
   # if args.height != None:
    #    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    
    #w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    #h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
  
    #fcc = cv2.VideoWriter_fourcc(*'XVID')
    #video_file_name = "video_{}_{}.avi".format(datetime.now(),args.device)
    #fcc = cv2.VideoWriter_fourcc('X','V','I','D')
    #video_result1 = cv2.VideoWriter(video_file_name,fcc,20.0,(1280,400))   

   

        

    # begin display
    display(cap, arducam_utils,args,args.fps)
    #args.device = 1
    #display(cap2, arducam_utils,args,args.fps)
    
    # release camera
    cap.release()
    #video_result1.release()
