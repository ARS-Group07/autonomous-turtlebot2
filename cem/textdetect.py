#!/usr/bin/env python
import numpy as np
import argparse
import time
import cv2
from imutils.object_detection import non_max_suppression
import pytesseract 

def contrast(image):
    alpha = 3.0 # Simple contrast control
    beta = 70  # Simple brightness control
    out = cv2.addWeighted( image, alpha, image, 0, beta)
    return out







class TextDetector:
    def __init__(self):
        self.flag = False
    def detect(self, img):
        model = "/home/cem/EAST-Detector-for-text-detection-using-OpenCV/frozen_east_text_detection.pb"
        confidence = 0.3
        width = 320
        height = 320 
        
        image = img.copy()
        image = contrast(image)

        orig = image.copy()
        (H, W) = image.shape[:2]

        rW = W / float(width)
        rH = H / float(height)

        # resize the image and grab the new image dimensions
        image = cv2.resize(image, (width, height))
        (H, W) = image.shape[:2]
        layerNames = [
            "feature_fusion/Conv_7/Sigmoid",
            "feature_fusion/concat_3"]

        net = cv2.dnn.readNet(model)

        blob = cv2.dnn.blobFromImage(image, 1.0, (W, H),
                                    (123.68, 116.78, 103.94), swapRB=True, crop=False)
        start = time.time()
        net.setInput(blob)
        (scores, geometry) = net.forward(layerNames)
        end = time.time()

        print("[INFO] text detection took {:.6f} seconds".format(end - start))

        (numRows, numCols) = scores.shape[2:4]

        rects = []
        confidences = []

        # loop over the number of rows
        for y in range(0, numRows):
            # extract the scores (probabilities), followed by the geometrical
            # data used to derive potential bounding box coordinates that
            # surround text
            scoresData = scores[0, 0, y]
            xData0 = geometry[0, 0, y]
            xData1 = geometry[0, 1, y]
            xData2 = geometry[0, 2, y]
            xData3 = geometry[0, 3, y]
            anglesData = geometry[0, 4, y]

            # loop over the number of columns
            for x in range(0, numCols):
                # if our score does not have sufficient probability, ignore it
                if scoresData[x] < confidence:
                    continue

                # compute the offset factor as our resulting feature maps will
                # be 4x smaller than the input image
                (offsetX, offsetY) = (x * 4.0, y * 4.0)

                # extract the rotation angle for the prediction and then
                # compute the sin and cosine
                angle = anglesData[x]
                cos = np.cos(angle)
                sin = np.sin(angle)

                # use the geometry volume to derive the width and height of
                # the bounding box
                h = xData0[x] + xData2[x]
                w = xData1[x] + xData3[x]

                # compute both the starting and ending (x, y)-coordinates for
                # the text prediction bounding box
                endX = int(offsetX + (cos * xData1[x]) + (sin * xData2[x]))
                endY = int(offsetY - (sin * xData1[x]) + (cos * xData2[x]))
                startX = int(endX - w)
                startY = int(endY - h)

                # add the bounding box coordinates and probability score to
                # our respective lists
                rects.append((startX, startY, endX, endY))
                confidences.append(scoresData[x])

        # apply non-maxima suppression to suppress weak, overlapping bounding
        # boxes
        boxes = non_max_suppression(np.array(rects), probs=confidences)
        center = (0,0)
        # loop over the bounding boxes
        for (startX, startY, endX, endY) in boxes:
            
            #scale back to original
            startX = int(startX * rW)
            startY = int(startY * rH)
            endX = int(endX * rW)
            endY = int(endY * rH)
            
            # draw the bounding box on the image
            cv2.rectangle(orig, (startX, startY), (endX, endY), (0, 255, 0), 2)
            center = (int((startX + endX) /2),int((startY+ endY) /2) )

            #crop detected area and run tesseract recognition
            crop_img = orig[startY:endY, startX:endX]
            custom_oem_psm_config = r'--psm 10'
            print("Reading result")
            text = pytesseract.image_to_string(crop_img, config=custom_oem_psm_config)
            print(text)
            for i in text:
                if i == '5':
                    self.flag = True

            

        # show the output image
        cv2.imshow("Text Detection", orig)
        cv2.waitKey(3)
        if(self.flag):
            return (True, center)
        else:
            return (False, (0,0))




