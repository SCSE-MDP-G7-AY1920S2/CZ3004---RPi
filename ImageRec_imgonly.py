
import cv2
import numpy as np
from random import *
import cnn
from PIL import Image
from extract_imgonly import extractBoundingBox

import torch
import torchvision.transforms as transforms


class IMAGEREC():
    def __init__(self):
        self.net = cnn.CNN()
        self.net.loadModel('Accuracy99.pkl')

        # self.classes  = ['1', '2', '3', '4', '5', 'A', 'B', 'C',
        #                    'D', 'Down', 'E', 'Left', 'Right', 'Stop', 'Up']

        self.classes = [6, 7, 8, 9, 10, 11, 12, 13, 14, 2, 15, 4, 3, 5, 1]
        self.transform = transforms.ToTensor()

    def predict(self, img):
        rects = extractBoundingBox(img)
        print("rects", rects)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        bb = []
        img_copy = img.copy()
        for i, rect in enumerate(rects):
            img = img_copy[rect[1]:rect[3], rect[0]:rect[2]]
            img = cv2.resize(img, (32, 32))
            img = Image.fromarray(img)
            img = self.transform(img)
            img.requires_grad = False
            bb.append(img)

        filtered = []
        print("len(bb) shouldn't be 0!", len(bb))
        if(len(bb)) != 0:
            boxs = torch.stack(bb)
            predictions = self.net(boxs)
            values, predictions = torch.max(predictions, 1)
            score, index = values.max(0)
            # if score < 0.5:
            #     return (None, 0)

            return (rects[index], self.classes[predictions[index]])
            '''
            for i, prediction in enumerate(predictions):
                if prediction.item() == 1:
                    filtered.append(i)
                '''
            '''
            if(len(filtered)) != 0:
                #filteredImg = torch.stack(boxs[filtered])
                predictions = self.net(boxs[filtered])
                #print(predictions.shape)
                values, prediction = torch.max(predictions, 1)
                score, index = values.max(0)
                if score < 0.85:
                    return (None, None)
                # ratio = float(rect[filtered[2]]) / float(rect[filtered[3]])
                return (rects[filtered[index]], self.classes[prediction[index]])
            else:
                return (None, None)
                '''
        else:
            return (None, 0)
