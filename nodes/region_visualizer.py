import cv2
import numpy as np
import yaml

class RegionVisualizer(object):

    def __init__(self,tracking_region_list):
        self.tracking_region_list = tracking_region_list
        self.is_first_image = True
        self.window_name = 'tracking_regions'
        self.window_num_pad = 1,11
        self.window_region_color = (193, 188, 27)
        self.window_object_color = (31, 165, 49)
        self.window_object_radius = 5

        self.find_centers = False 
        self.centers_count = 0

    def find_centers_from_image(self, image):
        """
        Temporary: move to some setup node etc.
        """
        threshold = 30
        min_area = 100
        image_gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        rval, threshold_image = cv2.threshold(image_gray, threshold, np.iinfo(image.dtype).max, cv2.THRESH_BINARY)
        dummy, contour_list, dummy = cv2.findContours(threshold_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        centroid_list = []
        filtered_contour_list = []
        for contour in contour_list:
            moments = cv2.moments(contour) 
            if moments['m00'] > 0: 
                centroidX = moments['m10']/moments['m00'] 
                centroidY = moments['m01']/moments['m00']
            else:
                continue
            area = cv2.contourArea(contour)
            if area > min_area:
                filtered_contour_list.append(contour)
                centroid_list.append([int(centroidX),int(centroidY)])
                
        contours_image = cv2.cvtColor(image_gray,cv2.COLOR_GRAY2BGR)
        cv2.drawContours(contours_image,filtered_contour_list,-1,(0,0,255),2)

        for cx,cy in centroid_list:
            cv2.circle(contours_image,(cx,cy),3,(0,255,0))

        self.centers_count += 1
        if self.centers_count == 50:
            print('writing centers')
            centroid_list.sort()   # This doesn't really work
            with open('centers.yaml','w') as f:
                yaml.dump(centroid_list, f)

        return contours_image 



    def update(self, image, tracked_obj_list):
        if image is None:
            return
        if self.is_first_image: 
            cv2.namedWindow(self.window_name,cv2.WINDOW_NORMAL)
            cv2.moveWindow(self.window_name, 100, 100)
            cv2.resizeWindow(self.window_name, 800,600)
            self.is_first_image = False 
            if self.find_centers:
                cv2.namedWindow('centers_image',cv2.WINDOW_NORMAL)
                cv2.moveWindow('centers_image', 120, 120)
                cv2.resizeWindow('centers_image', 800,600)

        if self.find_centers:
            # Temporary: move to another node which is run during setup
            centers_image = self.find_centers_from_image(image)
            cv2.imshow('centers_image', centers_image)
        else:
            for tracking_region in self.tracking_region_list: 
                # Draw boundary box for tracking region
                x0, y0 = tracking_region.lower_left
                x1, y1 = tracking_region.upper_right
                cv2.rectangle(image, (x0,y0), (x1,y1), self.window_region_color, 1)
                # Draw circle at center of tracking region
                tx = x0 + self.window_num_pad[0]
                ty = y0 + self.window_num_pad[1]
                cv2.putText(image,'{}'.format(tracking_region.number), (tx,ty), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.window_region_color)
                # Draw object found in region if any
                if tracking_region.obj is not None:
                    x = int(tracking_region.obj.position.x)
                    y = int(tracking_region.obj.position.y)
                    cv2.circle(image, (x,y), self.window_object_radius, self.window_object_color)
            cv2.imshow(self.window_name,image)

        cv2.waitKey(1)


