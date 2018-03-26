#!/usr/bin/env python
import rospy
import cv2

from ar_track_alvar_msgs.msg import AlvarMarkers
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# from scipy.interpolate import interp1d
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Int8


artag = AlvarMarkers()
sendpnt = Point()
newmarker = Marker()


class ar2pnt:

    def __init__(self):
        self.pnt = []
        self.calb = 0
        self.bridge = CvBridge()
        self.status = 0
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.imagecb)
        self.ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.arCb)
        self.marker = rospy.Subscriber('/visualization_marker', Marker, self.markerCb)
        self.pub = rospy.Publisher("ar_point",Point,queue_size=10)
        self.pubmarker = rospy.Publisher('/visualization_marker_update',Marker,queue_size=10)
        self.statsub = rospy.Subscriber('robot_state',Int8,self.stateCb)


    def stateCb(self,data):
        self.status = data.data
        rospy.loginfo(self.status)

    def arCb(self,data):
        artag = data

        tagnumber = len(artag.markers)
        if tagnumber > 0:
            x = artag.markers[0].pose.pose.position.x
            y = artag.markers[0].pose.pose.position.y
            z = artag.markers[0].pose.pose.position.z
            self.pnt = [x,y,z]
            rospy.loginfo([x,y,z])

    def markerCb(self,data):
        newmarker = data
        newmarker.pose.position.z = self.pnt[2] - self.calb
        newmarker.type = newmarker.SPHERE
        newmarker.scale.x = .1
        newmarker.scale.y = .1
        newmarker.scale.z = .1
        self.pubmarker.publish(newmarker)

    def imagecb(self,data):
        img_original = self.bridge.imgmsg_to_cv2(data, "bgr8")
        img_original = cv2.flip(img_original,1)
        # row,col,_ = img_original.shape
        if self.calb == 0:
            cv2.putText(img_original,'Calibrate 0 for Z.',(50,50),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2,cv2.LINE_AA)
            cv2.putText(img_original,"Hit 'c' when done.",(50,100),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2,cv2.LINE_AA)
            cv2.putText(img_original,"Feel free to recalibrate-",(50,150),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2,cv2.LINE_AA)
            cv2.putText(img_original,"during the session as well.",(50,200),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2,cv2.LINE_AA)
        elif self.status == 1:
            cv2.putText(img_original,'Planning...',(50,50),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2,cv2.LINE_AA)
        elif self.status == 3:
            cv2.putText(img_original,'Displaying Plan & Executing',(50,50),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2,cv2.LINE_AA)
            cv2.putText(img_original,"Wait 10 sec...",(50,100),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2,cv2.LINE_AA)
        elif self.status == 4:
            cv2.putText(img_original,'No Plan Found',(50,50),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2,cv2.LINE_AA)
            cv2.putText(img_original,"Try another point",(50,100),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2,cv2.LINE_AA)
        else:
            cv2.putText(img_original,'Move Tag around to-',(50,50),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2,cv2.LINE_AA)
            cv2.putText(img_original,'select a position.',(50,100),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2,cv2.LINE_AA)
            cv2.putText(img_original,"Hit 's' when done",(50,150),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2,cv2.LINE_AA)
            cv2.putText(img_original,"Recalibrate by hitting 'c'",(50,200),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2,cv2.LINE_AA)

        if self.pnt:
            pnt = ['x: '+str(round(self.pnt[0]*100,2))+'cm','y: '+str(round(self.pnt[1]*100,2))+'cm','z: '+str(round((-self.calb+self.pnt[2])*100,2))+'cm']
            str1 = ' \n'.join(pnt)
            for i, line in enumerate(str1.split('\n')):
                cv2.putText(img_original,line,(450,50+i*50),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2,cv2.LINE_AA)



        k=cv2.waitKey(3)
        if k == ord('c'):
            self.calb = self.pnt[2]
        elif k == ord('s'):
            sendpnt.x,sendpnt.y,sendpnt.z=self.pnt
            sendpnt.z = sendpnt.z - self.calb
            self.pub.publish(sendpnt)
            rospy.loginfo('Sent Point')
        cv2.namedWindow("Converted Image",cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Converted Image",320,240)
        cv2.imshow("Converted Image",img_original)
        cv2.waitKey(3)



if __name__ == "__main__":
    rospy.init_node('getArPos')
    ic = ar2pnt()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
