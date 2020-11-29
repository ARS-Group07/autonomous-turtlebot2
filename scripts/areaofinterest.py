import cv2


class AreaOfInterestFinder:
    def __init__(self, grid, scale):
        self.grid = grid
        self.scale = scale
        cv2.namedWindow('unexplored_contours', 1)

        self.largest_area, self.largest_cx, self.largest_cy = -1, 200, 200

    def get_grid_contours(self):
        image = self.grid.grid
        (h, w) = self.grid.grid.shape
        image = cv2.resize(image, (w * self.scale, h * self.scale))  # get larger version for display etc
        image = cv2.flip(image, 0)  # flip mask vertically cuz cv2 :)
        unexplored_mask = cv2.inRange(image, 0.499, 0.501)

        # find all discrete unexplored areas
        _, contours, hierarchy = cv2.findContours(unexplored_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        self.largest_area = 0
        for i, c in enumerate(contours):
            # don't include sub-contours ie. the holes in the white areas
            if hierarchy[0][i][3] != -1:
                continue

            m = cv2.moments(c)
            area = m['m00']

            # don't include very small areas as areas of interest
            if area < 25.:
                continue

            cx = int(m['m10'] / m['m00'])
            cy = int(m['m01'] / m['m00'])

            if (self.largest_area < area):
                self.largest_area = area
                self.largest_cx = cx
                self.largest_cy = cy
                print("New best contour: " + str(self.largest_cx) + ", " + str(self.largest_cy))

            # draw a circle on each discrete object; size of circle corresponds to area of contour
            cv2.circle(unexplored_mask, (cx, cy), int((area ** 0.33) / 2), 127, -1)


        cv2.imshow('unexplored_contours', unexplored_mask)
        cv2.waitKey(1)
