#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy
import cv2
import tf
from geometry_msgs.msg import PointStamped 
from time import sleep
from doro_msgs.msg import ClusterArray

class TestProjAndCut:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image", Image, self.imageCB)
        self.camera_info_sub = rospy.Subscriber("camera_info", CameraInfo, self.cameraInfoCB)
        self.clusters_sub = rospy.Subscriber("clusters", ClusterArray, self.clustersCB)
        
        self.image_header = None
        self.clusters = None     
        self.image = None
        self.P = None
        self.listener = tf.TransformListener()
        
        self.clusters_available = False
        self.got_camera_info = False
        
    def clustersCB(self, data):
        self.clusters = data.clusters
        self.clusters_available = True;
        self.clusters_sub.unregister()
    
    def imageCB(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data)
        self.image_header = data.header
        
    def cameraInfoCB(self, data):
        print "Data: ", data.P
        self.P = numpy.reshape(data.P, (3,-1))
        self.got_camera_info = True
        print '!!!Got camera info!!!'
        print 'Projection Matrix is: ', self.P
        self.camera_info_sub.unregister()
        
    def projectPointFromOpenniToCamera(self, ptVectorInBaseFrame):
              
        trans, rot = self.listener.lookupTransform(self.image_header.frame_id, "/base_link",  rospy.Time())
        print trans, rot
        
        transMatrix = tf.transformations.translation_matrix(trans)
        rotMatrix = tf.transformations.quaternion_matrix(rot)
        transform = numpy.dot(transMatrix, rotMatrix);
        
        print 'Before: ', ptVectorInBaseFrame
        ptVectorInBaseFrame = numpy.reshape(ptVectorInBaseFrame, (4, 1))
                
        ptVectorTransformed = numpy.dot(transform, ptVectorInBaseFrame);
        
        print 'After: ', ptVectorTransformed
        pixels = numpy.dot(self.P, ptVectorTransformed)
        print 'Got X: ', pixels[0]/pixels[2], ' and Y: ', pixels[1]/pixels[2]
        return pixels
    
    def projectPointFromOpenniToCamera2(self, ptVectorInBaseFrame):
              
        trans, rot = self.listener.lookupTransform("/xtion_camera_depth_optical_frame", "/base_link",  rospy.Time())
        print trans, rot
        
        transMatrix = tf.transformations.translation_matrix(trans)
        rotMatrix = tf.transformations.quaternion_matrix(rot)
        transform = numpy.dot(transMatrix, rotMatrix);
        
        print 'Before: ', ptVectorInBaseFrame
        ptVectorInBaseFrame = numpy.reshape(ptVectorInBaseFrame, (4, 1))
        ptVectorTransformedToXtion = numpy.dot(transform, ptVectorInBaseFrame);
        
        #ptVectorTransformedToXtionRGB = numpy.dot(tf.transformations.translation_matrix((-0.05, 0.0, 0.0)), ptVectorTransformedToXtion)
               
        transMatrix2 = tf.transformations.translation_matrix ((-0.02441, 0.0675, 0.0506));
               
        ptVectorInLeftCameraOpticalFrame = numpy.dot(transMatrix2, ptVectorTransformedToXtion)
        
        print 'After: ', ptVectorTransformedToXtion
        print 'After2: ', ptVectorInLeftCameraOpticalFrame
        pixels = numpy.dot(self.P, ptVectorInLeftCameraOpticalFrame)
        print 'Got X: ', pixels[0]/pixels[2], ' and Y: ', pixels[1]/pixels[2]
        return pixels
    
    def oldMethodTransform(self):
        
        if "Left" in self.image_header.frame_id:
            s_x = 69.912
            e_x = 370.3688
            s_y = 106.8405
            e_y = 326.582
            cam_size_x = 1024.00
            cam_size_y = 768.00
        else:
            s_x = 233.7252
            e_x = 502.9227
            s_y = 184.198
            e_y = 384.231
            cam_size_x = 640.00
            cam_size_y = 480.00
            
        for cluster in self.clusters:
                   
            new_x_start = ((cluster.window[0] - s_x)*cam_size_x) / (e_x - s_x) 
            new_y_start = ((cluster.window[1] - s_y)*cam_size_y) / (e_y - s_y)
        
            new_x_end = ((cluster.window[2] - s_x)*cam_size_x) / (e_x - s_x)
            new_y_end = ((cluster.window[3] - s_y)*cam_size_y) / (e_y - s_y)
        
            cv2.rectangle(self.image, (int(new_x_start), int(new_y_start)), (int(new_x_end), int(new_y_end)) , 0xff0000)
        cv2.imshow('Transferred point', self.image)
        cv2.waitKey(0)
    
    def waitForCameraInfo(self):
        while(not self.got_camera_info):
            print 'Waiting for camera info...'
            sleep(0.1)
        print '!!!Got camera info!!!'
        print 'Projection Matrix is: ', self.P
    
    def drawCircleAndDisplay(self, x, y, z):
        pixels = self.projectPointFromOpenniToCamera(numpy.array([x, y, z, 1]))
        self.image = self.image
        cv2.circle(self.image, (pixels[0]/pixels[2], pixels[1]/pixels[2]), 20, 0x00ff00)
        cv2.imshow('Transferred point', self.image)
        cv2.waitKey(0)
        
    def drawCirclesAndDisplay(self, points):
        for i in range(len(points)):
            pixels = self.projectPointFromOpenniToCamera(numpy.array([points[i][0], points[i][1], points[i][2], 1.0]))
            cv2.circle(self.image, (pixels[0]/pixels[2], pixels[1]/pixels[2]), 20, 0xff0000)
            
        
        self.drawRectangleAndDisplay(self.image, points)
        #cv2.imshow('Transferred point', self.image)
        #cv2.waitKey(0)
    
    def drawRectangleAndDisplay(self, image_copy, four_points):
        for i in range(len(four_points)):
            #pixels1 = self.projectPointFromOpenniToCamera(numpy.array( [four_points[i][0] + (four_points[i][5]*1.5/2) , four_points[i][1] + (four_points[i][3]*1.3/2), four_points[i][2] + (four_points[i][4]/2), 1.00]))
            #pixels2 = self.projectPointFromOpenniToCamera(numpy.array( [four_points[i][0] - (four_points[i][5]*1.5/2) , four_points[i][1] - (four_points[i][3]*0.9/2), four_points[i][2] - (four_points[i][4]/2), 1.00]))
            pixels1 = self.projectPointFromOpenniToCamera(numpy.array( [four_points[i][0] + 0.025, four_points[i][1] + 0.03, four_points[i][2], 1.00 ]) )
            pixels2 = self.projectPointFromOpenniToCamera(numpy.array( [four_points[i][3], four_points[i][4], four_points[i][5] - 0.025, 1.00 ]) )
            cv2.circle(self.image, (pixels1[0]/pixels1[2], pixels1[1]/pixels1[2]), 20, 0x00ff00)
            cv2.rectangle(image_copy, (pixels1[0]/pixels1[2], pixels1[1]/pixels1[2]), (pixels2[0]/pixels2[2], pixels2[1]/pixels2[2]) , 0xff0000)
        cv2.imshow('Transferred point', image_copy)
        cv2.waitKey(0)
        
    def pointsCB(self, point):
        self.drawCircleAndDisplay(point.data[0], point.data[1], point.data[2])
        
    def showImage(self):
        if self.image != None:
            cv2.imshow('Transferred point', self.image)
            cv2.waitKey(0)
        

if __name__ == '__main__':
    rospy.init_node ('testProjct')
    tpac = TestProjAndCut() 
    tpac.waitForCameraInfo()
    while not tpac.clusters_available:
        print 'Wait for clusters...'
        sleep(0.5)
        if rospy.is_shutdown():
            exit()
    
    all_sets = numpy.empty((0,6))
    
    for cluster in tpac.clusters:
        # array_to_append = numpy.array([[cluster.centroid.point.x, cluster.centroid.point.y, cluster.centroid.point.z, cluster.cluster_size[0], cluster.cluster_size[2], cluster.cluster_size[1]]])
        array_to_append = numpy.array([[cluster.a.x, cluster.a.y, cluster.a.z, cluster.b.x, cluster.b.y, cluster.b.z]])
        all_sets = numpy.append(all_sets, array_to_append, axis=0)
          
    print all_sets
 
 
    sleep(2)
    #tpac.drawCirclesAndDisplay(all_sets)
    tpac.drawRectangleAndDisplay(tpac.image, all_sets)
    #tpac.oldMethodTransform();
    
        
