import cv2
import numpy as np
import math


class AreaOfInterestFinder:
    def __init__(self, grid, scale):
        self.grid = grid
        self.scale = scale
        cv2.namedWindow('Unexplored areas with centroids', 1)

        self.closest_dist, self.closest_area, self.closest_cx, self.closest_cy = 0, -1, 200, 200

    def get_grid_contours(self, robot_x, robot_y):
        image = self.grid.grid
        (h, w) = self.grid.grid.shape
        image = cv2.resize(image, (w * self.scale, h * self.scale))  # get larger version for display etc
        image = cv2.flip(image, 0)  # flip mask vertically cuz cv2 :)
        display_image = np.zeros([image.shape[0], image.shape[1], 3], dtype=np.uint8)
        display_image[..., 2] += 240
        unexplored_mask = cv2.inRange(image, 0.499, 0.501)

        # find all discrete unexplored areas
        contours, hierarchy = cv2.findContours(unexplored_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        self.closest_dist = 1e7
        for i, c in enumerate(contours):
            # don't include sub-contours ie. the holes in the white areas
            if hierarchy[0][i][3] != -1:
                continue

            m = cv2.moments(c)
            area = m['m00']

            # don't include very small areas as areas of interest
            if area < 200.:
                continue
            cx = int(m['m10'] / m['m00'])
            cy = int(m['m01'] / m['m00'])

            dist = math.sqrt((cx - robot_x) ** 2 + (cy - robot_y) ** 2)
            if dist < self.closest_dist:  # -> for furthest
                self.closest_area = area
                self.closest_dist = dist
                self.closest_cx = cx
                self.closest_cy = cy

            # draw a circle on each discrete object; size of circle corresponds to area of contour
            # bigger contours are more saturated
            sat = np.clip(int(6 * area ** 0.33), 50, 140)
            color = (88, sat, 180)
            cv2.fillPoly(display_image, [c], color)
            cv2.circle(display_image, (cx, cy), int((area ** 0.33) / 2), (18, 100, 220), -1)

        # the chosen area's circle will be brighter
        if len(contours) > 0:
            cv2.circle(display_image, (self.closest_cx, self.closest_cy),
                       int((self.closest_area ** 0.33) / 2), (18, 220, 220), -1)

        display_image = cv2.cvtColor(display_image, cv2.COLOR_HSV2BGR)
        cv2.imshow('Unexplored areas with centroids', display_image)
        cv2.waitKey(1)
