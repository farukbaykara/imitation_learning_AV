#! /usr/bin/env python3

import random

import cv2
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
from sklearn.utils import shuffle
import matplotlib.image as mpimg
from imgaug import augmenters as iaa
import cv2
import random
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from autoware_msgs.msg import VehicleCmd
import numpy as np


import base64
from io import BytesIO
import PIL
import cv2
import tensorflow as tf




from tensorflow.python.keras import Sequential
from tensorflow.python.keras.layers import Convolution2D, Flatten, Dense
from tensorflow.python.keras.optimizers import adam_v2




def getName(filePath):
    return os.path.basename(os.path.normpath(filePath))

def importDataInfo(path):
    coloums = ['Image','Steering']
    data = pd.read_csv(os.path.join(path, 'driving_log.csv'), names = coloums)
    #print(data.head())
    #print(data['Center'][0])
    #print(getName(data['Center'][0]))
    data['Image'] = data['Image'].apply(getName)
    data_steer = data['Steering'].to_numpy()
    data_steer = np.delete(data_steer, 0) #steering header deleted from first row.
    data_steer = data_steer.astype('float64')

    print('Total Images Imported:', data.shape[0])
    print('ananananananannananana',data_steer)
    print('--------------------',data)

    return data, data_steer

def balanceData(data, data_steer, display = True):
    nBins = 31
    samplesPerBin = 110
    data_steer_temp = data_steer
    # print(type(data['Steering']))
    hist, bins = np.histogram(data_steer,nBins)
    #print(bins)
    if display:
        center = (bins[:-1] + bins[1:]) * 0.5
        #print(center)
        plt.bar(center, hist, width = 0.06)
        plt.plot((-1, 1), (samplesPerBin, samplesPerBin))
        plt.show()

    removeIndexList = []

    for j in range(nBins):
        binDataList = []
        for i in range(len(data_steer_temp)):
            if data_steer_temp[i] >= bins[j] and data_steer_temp[i] <= bins[j + 1]:
                binDataList.append(i)
        binDataList = shuffle(binDataList)
        binDataList = binDataList[samplesPerBin:]
        removeIndexList.extend(binDataList)
    print('Removed Images: ', len(removeIndexList))
    data.drop(data.index[removeIndexList], inplace = True)
    
    #deleting first row, 'Steering' header cannot convert to float
    data.drop(data.index[0], inplace = True) 
    
    
    print('Remaining Images: ', len(data))
    print('-----------------------')
    print(data)
    if display:
        hist, _ = np.histogram(data_steer_temp,nBins)
        plt.bar(center, hist, width = 0.06)
        plt.plot((-1, 1), (samplesPerBin, samplesPerBin))
        plt.show()

    # data_steer = data['Steering'].to_numpy()
    # data_steer = data_steer.astype('float64')
    # print(type(data_steer))
    return data

def loadData(path, data):
    imagesPath = []
    steering = []
    for i in range(len(data)):
        indexedData = data.iloc[i]
        #print(indexedData)
        imagesPath.append(os.path.join(path, 'IMG', indexedData[0]))
        steering.append(float(indexedData[1]))
    imagesPath = np.asarray(imagesPath)
    steering = np.asarray(steering)
    return imagesPath, steering

# def augmentImage(imgPath, steering):
#     img = mpimg.imread(imgPath)
#     ## PAN
#     if np.random.rand() < 0.5:
#         pan = iaa.Affine(translate_percent={'x':(-0.1, 0.1), 'y':(-0.1, 0.1)})
#         img = pan.augment_image(img)
#     ## ZOOM
#     if np.random.rand() < 0.5:
#         zoom = iaa.Affine(scale=(1, 1.2))
#         img = zoom.augment_image(img)
#     ## BRIGHTNESS
#     if np.random.rand() < 0.5:
#         brightness = iaa.Multiply(0.4, 1.2)
#         img = brightness.augment_image(img)
#     ## FLIP
#     if np.random.rand() < 0.5:
#         img = cv2.flip(img, 1)
#         steering = -steering
#
#     return img, steering

def augmentImage(imgPath,steering):
    img =  mpimg.imread(imgPath)
    if np.random.rand() < 0.5:
        pan = iaa.Affine(translate_percent={"x": (-0.1, 0.1), "y": (-0.1, 0.1)})
        img = pan.augment_image(img)
    if np.random.rand() < 0.5:
        zoom = iaa.Affine(scale=(1, 1.2))
        img = zoom.augment_image(img)
    if np.random.rand() < 0.5:
        brightness = iaa.Multiply((0.2, 1.2))
        img = brightness.augment_image(img)
    if np.random.rand() < 0.5:
        img = cv2.flip(img, 1)
        steering = -steering
    return img, steering

def preProcessing(img):
    # cv2.imshow('aaa',img)
    # cv2.waitKey(0)
    img = img[640:840, :, :]
    # cv2.imshow('aaa',img)
    # cv2.waitKey(0)
    img =cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
    img = cv2.GaussianBlur(img, (3,3), 0)

    img =cv2.resize(img, (200, 66))
    img = img / 255
    return img

# imRe = preProcessing(mpimg.imread('test.jpg'))
# plt.imshow(imRe)
# plt.show()

def batchGen(imagesPath, steeringList, batchSize, trainFlag):
    while True:
        imgBatch = []
        steeringBatch = []

        for i in range(batchSize):
            index = random.randint(0, len(imagesPath) - 1)
            if trainFlag:
                img , steering = augmentImage(imagesPath[index], steeringList[index])
            else:
                img = mpimg.imread(imagesPath[index])
                steering = steeringList[index]
            img = preProcessing(img)
            imgBatch.append(img)
            steeringBatch.append(steering)
        yield (np.asarray(imgBatch), np.asarray(steeringBatch))

def createModel():
    model = Sequential()

    model.add(Convolution2D(24, (5, 5), (2,2), input_shape = (66, 200, 3), activation='elu'))
    model.add(Convolution2D(36, (5, 5), (2,2), activation='elu'))
    model.add(Convolution2D(48, (5, 5), (2,2), activation='elu'))
    model.add(Convolution2D(64, (3, 3), activation='elu'))
    model.add(Convolution2D(64, (3, 3), activation='elu'))

    model.add(Flatten())
    model.add(Dense(100, activation='elu'))
    model.add(Dense(50, activation='elu'))
    model.add(Dense(10, activation='elu'))
    model.add(Dense(1))

    model.compile(adam_v2.Adam(learning_rate = 0.0001), loss = 'mse')
    return model
