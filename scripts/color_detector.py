import cv2
import numpy as np

class ColorDetector:
    def __init__(self):
        self.color_ranges = {
            'yellow': ([20, 100, 100], [35, 255, 255]),
            'red1':   ([0, 120, 70], [10, 255, 255]),
            'red2':   ([170, 120, 70], [180, 255, 255]),  # red has two HSV zones
            'blue':   ([100, 150, 0], [140, 255, 255]),
            'white':  ([0, 0, 200], [180, 55, 255]),
            'green':  ([45, 100, 50], [85, 255, 255])
        }

    def get_mask(self, bgr_img, colors):
        """Returns combined binary mask for one or more colors."""
        if isinstance(colors, str):
            colors = [colors]

        hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        final_mask = None

        for color in colors:
            if color == 'red':
                # Red has two ranges
                lower1, upper1 = self.color_ranges['red1']
                lower2, upper2 = self.color_ranges['red2']
                mask1 = cv2.inRange(hsv, np.array(lower1), np.array(upper1))
                mask2 = cv2.inRange(hsv, np.array(lower2), np.array(upper2))
                mask = cv2.bitwise_or(mask1, mask2)
            elif color in self.color_ranges:
                lower, upper = self.color_ranges[color]
                mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            else:
                raise ValueError(f"Unsupported color: {color}")

            if final_mask is None:
                final_mask = mask
            else:
                final_mask = cv2.bitwise_or(final_mask, mask)

        return final_mask
