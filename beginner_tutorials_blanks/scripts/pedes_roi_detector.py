#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

def non_maximum_suppression(bboxes, threshold=0.3):
    if len(bboxes) == 0:
        return []

    bboxes = sorted(bboxes, key=lambda detections: detections[3], reverse=True)
    new_bboxes = [bboxes[0]]
    bboxes.pop(0)

    for _, bbox in enumerate(bboxes):
        for new_bbox in new_bboxes:
            x1_tl = bbox[0]
            x2_tl = new_bbox[0]
            x1_br = bbox[0] + bbox[2]
            x2_br = new_bbox[0] + new_bbox[2]
            y1_tl = bbox[1]
            y2_tl = new_bbox[1]
            y1_br = bbox[1] + bbox[3]
            y2_br = new_bbox[1] + new_bbox[3]

            x_overlap = max(0, min(x1_br, x2_br) - max(x1_tl, x2_tl))
            y_overlap = max(0, min(y1_br, y2_br) - max(y1_tl, y2_tl))
            overlap_area = x_overlap * y_overlap

            area_1 = bbox[2] * new_bbox[3]
            area_2 = new_bbox[2] * new_bbox[3]
            total_area = area_1 + area_2 - overlap_area
            overlap_area = overlap_area / float(total_area)

            if overlap_area < threshold:
                new_bboxes.append(bbox)

    return new_bboxes

class PEDESDetector:
    def __init__(self):
        rospy.init_node('pedes_detector', anonymous=True)
        self.rate = rospy.Rate(20)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.pedes_detector = cv2.HOGDescriptor()
        self.pedes_detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self.crop_pts = np.array(
            [[
                [0, 236],
                [0, 360],
                [640, 360],
                [640, 236],
                [460, 220],
                [180, 220]
            ]]
        )
        rospy.loginfo("PEDESDetector initialized")

    def callback(self, msg):
        self.rate.sleep()

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img_bgr is None:
                rospy.logerr("Failed to decode image")
                return
            
            rospy.loginfo("Image received and decoded")

            roi_img = self.mask_roi(img_bgr)
            img_gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        (rects_temp, _) = self.pedes_detector.detectMultiScale(img_gray, winStride=(4, 4), padding=(8, 8), scale=1.05)

        if len(rects_temp) != 0:
            rects = non_maximum_suppression(rects_temp)
            rospy.loginfo(f"Detected {len(rects)} pedestrians")
            for (x, y, w, h) in rects:
                cv2.rectangle(roi_img, (x, y), (x + w, y + h), (0, 255, 255), 2)
        else:
            rospy.loginfo("No pedestrians detected")

        cv2.imshow("Image window", roi_img)
        cv2.waitKey(1)

    def mask_roi(self, img):
        h = img.shape[0]
        w = img.shape[1]

        if len(img.shape) == 3:
            # image shape : [h, w, 3]
            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)
            mask_value = (255, 255, 255)
        else:
            # binarized image or grayscale image : [h, w]
            mask = np.zeros((h, w), dtype=np.uint8)
            mask_value = (255)

        cv2.fillPoly(mask, self.crop_pts, mask_value)
        roi_img = cv2.bitwise_and(mask, img)
        return roi_img

if __name__ == '__main__':
    pedes_detector = PEDESDetector()
    rospy.spin()

