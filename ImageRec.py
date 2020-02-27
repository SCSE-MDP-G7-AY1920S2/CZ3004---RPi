# import the necessary packages
import numpy as np
import time
import cv2
import os


class IMAGEREC():
    def __init__(self):
        labelsPath = os.path.sep.join(["yolo-coco", "obj.names"])
        self.LABELS = open(labelsPath).read().strip().split("\n")
        np.random.seed(42)
        self.COLORS = np.random.randint(
            0, 255, size=(len(self.LABELS), 3), dtype="uint8")
        weightsPath = os.path.sep.join(
            ["yolo-coco", "yolov3-tiny_obj_last.weights"])
        configPath = os.path.sep.join(["yolo-coco", "yolov3-tiny_obj.cfg"])
        print("[INFO] loading YOLO from disk...")
        self.net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)

    def predict(self, image):
        # image = cv2.imread(img)
        (H, W) = image.shape[:2]

        # determine only the *output* layer names that we need from YOLO
        ln = self.net.getLayerNames()
        ln = [ln[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        # construct a blob from the input image and then perform a forward
        # pass of the YOLO object detector, giving us our bounding boxes and
        # associated probabilities
        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416),
                                     swapRB=True, crop=False)
        self.net.setInput(blob)
        start = time.time()
        layerOutputs = self.net.forward(ln)
        end = time.time()

        # show timing information on YOLO
        print("[INFO] YOLO took {:.6f} seconds".format(end - start))

        # initialize our lists of detected bounding boxes, confidences, and
        # class IDs, respectively
        boxes = []
        confidences = []
        classIDs = []

        # loop over each of the layer outputs
        for output in layerOutputs:
            # loop over each of the detections
            for detection in output:
                # extract the class ID and confidence (i.e., probability) of
                # the current object detection
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]
                # filter out weak predictions by ensuring the detected
                # probability is greater than the minimum probability
                if confidence > 0.5:
                    # scale the bounding box coordinates back relative to the
                    # size of the image, keeping in mind that YOLO actually
                    # returns the center (x, y)-coordinates of the bounding
                    # box followed by the boxes' width and height
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")
                    # use the center (x, y)-coordinates to derive the top and
                    # and left corner of the bounding box
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))
                    # update our list of bounding box coordinates, confidences,
                    # and class IDs
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)

        # apply non-maxima suppression to suppress weak, overlapping bounding
        # boxes
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, 0.5,
                                0.3)

        # ensure at least one detection exists
        area, chosen_class, index = 0, -1, -1
        area_threshold = 4000
        if len(idxs) > 0:
            # loop over the indexes we are keeping
            for i in idxs.flatten():
                # extract the bounding box coordinates
                # (x, y) = (boxes[i][0], boxes[i][1])
                # (w, h) = (boxes[i][2], boxes[i][3])
                print("class", self.LABELS[classIDs[i]], "xc", boxes[i][0], "yc", boxes[i][1],
                      "w", boxes[i][2], "h", boxes[i][3], "area", boxes[i][2] * boxes[i][3])
                if boxes[i][2] * boxes[i][3] > area and boxes[i][2] * boxes[i][3] > area_threshold:
                    area = boxes[i][2] * boxes[i][3]
                    x, y, w, h = boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3]
                    index = i
                    chosen_class = self.LABELS[classIDs[index]]
            print("Chosen class", chosen_class, "xc", x,
                  "yc", y, "w", w, "h", h, "area", area)
            xstart, xend = int(x - (w / 2)), int(x + (w / 2))

            # Fallback if it takes too long - for img position (left, middle, right)
            # if xstart >= 0 and xend < 130:
            #     print("box", chosen_class, "is on the left")
            # elif xstart >= 130 and xend < 260:
            #     print("box", chosen_class, "is in the middle")
            # elif xstart > 260 and xend < 390:
            #     print("box", chosen_class, "is on the right")

            # draw a bounding box rectangle and label on the image
            color = [int(c) for c in self.COLORS[classIDs[index]]]
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
            text = "{}: {:.4f}".format(chosen_class, confidences[index])
            cv2.putText(image, text, (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            bounding_box = [(x, y, w, h)]

            # show the output image
            # cv2.imshow("Image", image)
            # cv2.waitKey(0)
            cv2.imwrite("correct_images/" + chosen_class + ".jpg", image)
            # return chosen_class, x, y, w, h, area
            return bounding_box, chosen_class
