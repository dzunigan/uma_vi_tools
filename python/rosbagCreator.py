#!/usr/bin/env python

# Copyright (C) 2019  Alberto Jaenal <alberto.jaenal at uma.es>

import time, sys, os
import argparse
import cv2
import numpy as np
import csv

import rosbag
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge, CvBridgeError



def getCamFoldersFromDir(dir_, imu_filename):
    '''Generates a list of all folders that start with cam e.g. cam0'''
    cam_folders = list()
    if os.path.exists(dir_):
        for fils in os.listdir(dir_):           
            if "cam" in fils and os.path.isdir(os.path.join(dir_, fils)):
                cam_folders.append(fils)
    else:
        raise NotFoundErr(dir_ + ' does not exist')
    
    assert(os.path.exists(os.path.join(dir_, imu_filename)), 'File ' + os.path.join(dir_, imu_filename) + ' not founded')
    
    print('\nDirectory :' + dir_)
    for camf in cam_folders:
        print('\t' + camf + '/')
    print('\t' + imu_filename+ '\n')
    
    return sorted(cam_folders)

def getImageFilesFromDir(dir_):
    '''Generates a list of tuples timestamp-files from the directory'''
    image_files = list()
    timestamps = list()
    exp_times = list()
    exposure = False
    
    if os.path.exists(os.path.join(dir_, 'data.csv')):
        with open(os.path.join(dir_, 'data.csv'), 'r') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            headers = next(reader, None)
            index_timestamp = [headers.index(x) for x in headers if 'timestamp' in x][0]
            if len([s for s in headers if 'exposure' in s]) > 0:
                print('\tAdding the exposure time in msg.header.frame_id...')
                exposure = True
                index_exposure = [headers.index(s) for s in headers if 'exposure' in s][0]
            
            if exposure:
                for row in reader:
                    timestamps.append(row[index_timestamp])
                    exp_times.append(row[index_exposure])
            else:
                for row in reader:
                    timestamps.append(row[index_timestamp])
                
    
    if os.path.exists(os.path.join(dir_, 'data')):
        image_files = sorted([os.path.join(dir_, 'data', x) for x in os.listdir(os.path.join(dir_, 'data')) if x.split('.')[-1] in ['png', 'jpg']],
                                key=lambda x: int(x.split('/')[-1].split('.')[0]))
                                
        assert(len(image_files) == len(timestamps), 
        'Not same timestamps than image files : %d, %d' % (len(image_files), len(timestamps)))
    else:
        raise NotFoundErr(os.path.join(dir_, 'data') + ' does not exist')
    
    print('\tFounded %d images!' % len(timestamps))
        
    #sort by timestamp
    if exposure:
        sort_list = sorted(zip(timestamps, image_files, exp_times))        
    else:
        sort_list = sorted(zip(timestamps, image_files))
    return sort_list


def loadImageToRosMsg(tup, bridge):
    if len(tup) != 2:
        timestamp_nsecs, filename, exp_time = tup
    else:
        timestamp_nsecs, filename = tup
    timestamp = rospy.Time( secs=int(str(timestamp_nsecs)[0:-9]), nsecs=int(str(timestamp_nsecs)[-9:]) )
    
    im = cv2.imread(filename, cv2.IMREAD_ANYDEPTH)
    encoding = "bgr8"
    if len(im.shape) == 2 or im.shape[-1] == 1:
        encoding = "mono8"
    rosimage = bridge.cv2_to_imgmsg(im, encoding=encoding)
    rosimage.header.stamp = timestamp
    if len(tup) != 2:
        rosimage.header.frame_id = str(exp_time)
    #rosimage.height = image_np.shape[0]
    #rosimage.width = image_np.shape[1]
    #rosimage.step = rosimage.width  #only with mono8! (step = width * byteperpixel * numChannels)
    #rosimage.encoding = "mono8"
    #rosimage.data = image_np.tostring()
    
    return rosimage, timestamp

def createImuMessge(timestamp_int, omega, alpha):
    timestamp_nsecs = str(timestamp_int)
    timestamp = rospy.Time( int(timestamp_nsecs[0:-9]), int(timestamp_nsecs[-9:]) )
    
    rosimu = Imu()
    rosimu.header.stamp = timestamp
    rosimu.angular_velocity.x = float(omega[0])
    rosimu.angular_velocity.y = float(omega[1])
    rosimu.angular_velocity.z = float(omega[2])
    rosimu.linear_acceleration.x = float(alpha[0])
    rosimu.linear_acceleration.y = float(alpha[1])
    rosimu.linear_acceleration.z = float(alpha[2])
    
    return rosimu, timestamp

if __name__ == '__main__':
    #Structure
    # dataset/cam0/data/*.png
    # dataset/cam0/data.csv    
    # dataset/camN/data/*.png    
    # dataset/camN/data.csv
    # dataset/imu_filename (imu.csv)
    
    #Output
    # dataset.bag

    #setup the argument list
    parser = argparse.ArgumentParser(description='Create a ROS bag using the images and imu data.')
    parser.add_argument('file_folder',  metavar='file_folder', nargs='?', help='Data folder')
    parser.add_argument('-if', '--imu_filename', default='imu0/data.csv', help='IMU filename (def.: imu0/data.csv)')
    #print help if no argument is specified
    if len(sys.argv)<2:
        parser.print_help()
        sys.exit(0)

    #parse the args
    parsed = parser.parse_args()
    #create the bag
    bridge = CvBridge()
    
    ## TODO: Generalize for several IMUs
    print('\nThis script assumes that there is only one imu!!!')
    try:
        bag = rosbag.Bag(parsed.file_folder + '.bag', 'w')
        
        #write images
        camfolders = getCamFoldersFromDir(parsed.file_folder, parsed.imu_filename)
        print('CAUTION!! ASSUMING THAT THE data.csv FILES ARE SORTED W.R.T. TIME')
        for camfolder in camfolders:
            print('Publishing topics from: ' + "/{0}/image_raw".format(camfolder))
            camdir = parsed.file_folder + "/{0}".format(camfolder)
            image_files = getImageFilesFromDir(camdir)
            for image_filename in image_files:
                image_msg, timestamp = loadImageToRosMsg(image_filename, bridge)
                
                bag.write("/{0}/image_raw".format(camfolder), image_msg, t=timestamp)
                
        #write imu data
        topic = '/' + parsed.imu_filename.replace('.csv', '')
        print('\nPublishing topics from: ' + topic)
        with open(os.path.join(parsed.file_folder, parsed.imu_filename), 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            print('\tFounded %d imu messages!' % (sum(1 for row in reader)-1))
            
        with open(os.path.join(parsed.file_folder, parsed.imu_filename), 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            headers = next(reader, None)
            for row in reader:
                imumsg, timestamp = createImuMessge(row[0], row[1:4], row[4:7])
                bag.write(topic, imumsg, t=timestamp)
        print('\nDONE\n')
    finally:
        bag.close()
    

