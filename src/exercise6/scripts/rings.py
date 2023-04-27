#!/usr/bin/python3

import sys
import rospy
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
import math
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose, PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class The_Ring:
    def __init__(self):
        rospy.init_node('image_converter', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Marker array object used for visualizations
        self.marker_array = MarkerArray()
        self.marker_num = 1

        # Subscribe to the image and/or depth topic
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        # self.depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.depth_callback)

        # Publiser for the visualization markers
        self.markers_pub = rospy.Publisher('rings', MarkerArray, queue_size=1000)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        self.candidates = []
        self.rings_n = []


    def get_pose(self,e,dist):
        # Calculate the position of the detected ellipse

        k_f = 525 # kinect focal length in pixels

        elipse_x = self.dims[1] / 2 - e[0][0]
        elipse_y = self.dims[0] / 2 - e[0][1]

        angle_to_target = np.arctan2(elipse_x,k_f)

        # Get the angles in the base_link relative coordinate system
        x,y = dist*np.cos(angle_to_target), dist*np.sin(angle_to_target)

        ### Define a stamped message for transformation - directly in "base_frame"
        #point_s = PointStamped()
        #point_s.point.x = x
        #point_s.point.y = y
        #point_s.point.z = 0.3
        #point_s.header.frame_id = "base_link"
        #point_s.header.stamp = rospy.Time(0)

        # Define a stamped message for transformation - in the "camera rgb frame"
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = rospy.Time(0)

        # Get the point in the "map" coordinate system
        point_world = self.tf_buf.transform(point_s, "map")

        # Create a Pose object with the same position
        pose = Pose()
        pose.position.x = point_world.point.x
        pose.position.y = point_world.point.y
        pose.position.z = point_world.point.z

        new_ring = True
        for i, c in enumerate(self.candidates):
            if abs(pose.position.x - c.pose.position.x) < 0.1 and abs(pose.position.y - c.pose.position.y) < 0.1:
                c.pose.position.x = (c.pose.position.x * self.rings_n[i] + pose.position.x) / (self.rings_n[i] + 1)
                c.pose.position.y = (c.pose.position.y * self.rings_n[i] + pose.position.y) / (self.rings_n[i] + 1)
                self.rings_n[i] += 1
                new_ring = False
                break
            if self.rings_n[i] == 5:
                marker = Marker()
                marker.header = c.header
                marker.pose = c.pose
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.frame_locked = False
                marker.lifetime = rospy.Duration(0)
                marker.id = self.marker_num
                marker.scale = Vector3(0.1, 0.1, 0.1)
                marker.color = ColorRGBA(0, 1, 0, 1)
                self.marker_array.markers.append(marker)
                self.marker_num += 1

        if new_ring:
            poseStamped = PoseStamped()
            poseStamped.pose = pose
            poseStamped.header.stamp = point_world.header.stamp
            poseStamped.header.frame_id = point_world.header.frame_id
            self.candidates.append(poseStamped)
            self.rings_n.append(1)

        self.markers_pub.publish(self.marker_array)


    def image_callback(self,data):
        # print('I got a new image!')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Set the dimensions of the image
        self.dims = cv_image.shape

        # Tranform image to gayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Do histogram equlization
        img = cv2.equalizeHist(gray)

        # Binarize the image, there are different ways to do it
        #ret, thresh = cv2.threshold(img, 50, 255, 0)
        #ret, thresh = cv2.threshold(img, 70, 255, cv2.THRESH_BINARY)
        thresh = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, 25)

        # Extract contours
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Example how to draw the contours, only for visualization purposes
        cv2.drawContours(img, contours, -1, (255, 0, 0), 3)
        cv2.imshow("Contour window",img)
        cv2.waitKey(1)

        # Fit elipses to all extracted contours
        elps = []
        for cnt in contours:
            #     print cnt
            #     print cnt.shape
            if cnt.shape[0] >= 20:
                ellipse = cv2.fitEllipse(cnt)
                elps.append(ellipse)


        # Find two elipses with same centers
        candidates = []
        for n in range(len(elps)):
            for m in range(n + 1, len(elps)):
                e1 = elps[n]
                e2 = elps[m]
                dist = np.sqrt(((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2))
                # print(dist)
                if dist < 5:
                    candidates.append((e1,e2))

        # print("Processing is done! found", len(candidates), "candidates for rings")

        try:
            depth_img = rospy.wait_for_message('/camera/depth/image_raw', Image)
        except Exception as e:
            print(e)

        # Extract the depth from the depth image
        for c in candidates:

            # the centers of the ellipses
            e1 = c[0]
            e2 = c[1]

            # drawing the ellipses on the image
            cv2.ellipse(cv_image, e1, (0, 255, 0), 2)
            cv2.ellipse(cv_image, e2, (0, 255, 0), 2)

            depth_image = self.bridge.imgmsg_to_cv2(depth_img, "32FC1")

            size = (e1[1][0]+e1[1][1])/2
            center = (e1[0][1], e1[0][0])

            x1 = int(center[0] - size / 2)
            x2 = int(center[0] + size / 2)
            x_min = x1 if x1>0 else 0
            x_max = x2 if x2<cv_image.shape[0] else cv_image.shape[0]

            y1 = int(center[1] - size / 2)
            y2 = int(center[1] + size / 2)
            y_min = y1 if y1 > 0 else 0
            y_max = y2 if y2 < cv_image.shape[1] else cv_image.shape[1]
            
            

            ring_depth = np.nanmean(np.where(cv_image[x_min:x_max, y_min:y_max,1] == 255, depth_image[x_min:x_max, y_min:y_max], float('nan')))
            center_depth = np.nanmean(depth_image[int(center[0] - size //4):int(center[0] + size //4), int(center[1] - size //4): int(center[1] + size//4)])

            # ring_depth = 0
            # center_depth = 0
            # ring_count = 0
            # center_count = 0
            # for x in range(x_min, x_max):
            #     greens_in_line = 0
            #     green = False
            #     line_sum_ring = 0
            #     line_count_ring = 0
            #     line_sum_center = 0
            #     line_count_center = 0
            #     for y in range(y_min, y_max):
            #         if (cv_image[x, y, 1] == 255):
            #             if (not green):
            #                 greens_in_line += 1
            #         else:
            #             green = False
            #         if greens_in_line % 2 != 0 and not math.isnan(depth_image[x, y]):
            #             line_sum_ring += depth_image[x, y]
            #             line_count_ring += 1
            #         if greens_in_line == 2 and not math.isnan(depth_image[x, y]):
            #             line_sum_center += depth_image[x, y]
            #             line_count_center += 1
            #             cv_image[x, y, :] = 255
            #         if greens_in_line == 4:
            #             break
            #     if greens_in_line == 2 or greens_in_line == 1:
            #         ring_depth += line_sum_ring
            #         ring_count += line_count_ring
            #     if greens_in_line == 4:
            #         ring_depth += line_sum_ring
            #         ring_count += line_count_ring
            #         center_depth += line_sum_center
            #         center_count += line_count_center
            # if (ring_count > 0):
            #     ring_depth = ring_depth / ring_count
            # if (center_count > 0):
            #     center_depth = center_depth / center_count
            
            if math.isnan(center_depth) or abs(ring_depth - center_depth) > 0.5:
                self.get_pose(e1, ring_depth)


            

        if len(candidates)>0:
            cv2.imshow("Image window",cv_image[x_min:x_max, y_min:y_max])
            self.depth_callback(depth_img)
            cv2.waitKey(1)

    def depth_callback(self,data):

        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)

        # Do the necessairy conversion so we can visuzalize it in OpenCV
        image_1 = depth_image / 65536.0 * 255
        image_1 =image_1/np.max(image_1)*255

        image_viz = np.array(image_1, dtype= np.uint8)

        cv2.imshow("Depth window", image_viz)
        cv2.waitKey(1)


def main():

    ring_finder = The_Ring()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
