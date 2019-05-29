#!/usr/bin/env python2

# Copyright (C) 2018 Christian Berger
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

# sysv_ipc is needed to access the shared memory where the camera image is present.
import sysv_ipc
# numpy and cv2 are needed to access, modify, or display the pixels
import numpy
import cv2
# OD4Session is needed to send and receive messages
import OD4Session
# Import the OpenDLV Standard Message Set.
import opendlv_standard_message_set_v0_9_6_pb2

################################################################################
# This dictionary contains all distance values to be filled by function onDistance(...).
distances = { "front": 0.0, "left": 0.0, "right": 0.0, "rear": 0.0 };

################################################################################
# This callback is triggered whenever there is a new distance reading coming in.
def onDistance(msg, senderStamp, timeStamps):
    print "Received distance; senderStamp=" + str(senderStamp)
    print "sent: " + str(timeStamps[0]) + ", received: " + str(timeStamps[1]) + ", sample time stamps: " + str(timeStamps[2])
    print msg
    if senderStamp == 0:
        distances["front"] = msg.distance
    if senderStamp == 1:
        distances["left"] = msg.distance
    if senderStamp == 2:
        distances["rear"] = msg.distance
    if senderStamp == 3:
        distances["right"] = msg.distance

#method that helps to find the center of objects

def findCenters(cones):
    _, contours, _ = cv2.findContours(cones.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
    centres = []
    for i in range(len(contours)):
      moments = cv2.moments(contours[i])
      if moments['m00'] != 0:
       centres.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
    return centres


def keepDistance(distance, speed):
	optimalDistance=0.3
	originalSpeed=0.09
	detectionDistance=0.5
	if distance <= detectionDistance:
		if distance <= optimalDistance-0.03 and speed >=0.04:
			speed=speed-0.01
                        #speed=0
		#elif distance >= optimalDistance+0.03 and speed <=originalSpeed:
		#	speed=speed+0.005
			
	else:
		speed=originalSpeed

	return speed

 
fgbg=cv2.createBackgroundSubtractorMOG2(history=10,varThreshold=2,detectShadows=False)
minX=240
maxX=320
minY=140
maxY=470
#280
#360
#170
#470
difference=maxX-minX
differenceY=maxY-minY
prevAngle=0
seeingOrange=0
# Create a session to send and receive messages from a running OD4Session;
# Replay mode: CID = 253
# Live mode: CID = 112
# TODO: Change to CID 112 when this program is used on Kiwi.
session = OD4Session.OD4Session(cid=112)
# Register a handler for a message; the following example is listening
# for messageID 1039 which represents opendlv.proxy.DistanceReading.
# Cf. here: https://github.com/chalmers-revere/opendlv.standard-message-set/blob/master/opendlv.odvd#L113-L115
messageIDDistanceReading = 1039
session.registerMessageCallback(messageIDDistanceReading, onDistance, opendlv_standard_message_set_v0_9_6_pb2.opendlv_proxy_DistanceReading)
# Connect to the network session.
session.connect()

################################################################################
# The following lines connect to the camera frame that resides in shared memory.
# This name must match with the name used in the h264-decoder-viewer.yml file.
name = "/tmp/img.argb"
# Obtain the keys for the shared memory and semaphores.
keySharedMemory = sysv_ipc.ftok(name, 1, True)
keySemMutex = sysv_ipc.ftok(name, 2, True)
keySemCondition = sysv_ipc.ftok(name, 3, True)
# Instantiate the SharedMemory and Semaphore objects.
shm = sysv_ipc.SharedMemory(keySharedMemory)
mutex = sysv_ipc.Semaphore(keySemCondition)
cond = sysv_ipc.Semaphore(keySemCondition)
lower_blue= numpy.array([100,100,30])
upper_blue= numpy.array([150,255,255])

lower_yellow= numpy.array([23,41,133])
upper_yellow= numpy.array([40,150,255])

lower_black=numpy.array([0,0,0])
upper_black=numpy.array([69,69,69])

lower_orange=numpy.array([3,100,20])
upper_orange=numpy.array([6,255,255])

carSpeed=0.09

noYellow=0
noBlue=0
middlePoint=difference*0.5
################################################################################
# Main loop to process the next image frame coming in.
while True:
    # Wait for next notification.
    cond.Z()
    print "Received new frame."

    # Lock access to shared memory
    mutex.acquire()
    # Attach to shared memory.
    shm.attach()
    # Read shared memory into own buffer.
    buf = shm.read()
    # Detach to shared memory.
    shm.detach()
    # Unlock access to shared memory.
    mutex.release()

    # Turn buf into img array (640 * 480 * 4 bytes (ARGB)) to be used with OpenCV.
    img = numpy.frombuffer(buf, numpy.uint8).reshape(480, 640, 4)
    #change coord
    imgCopy=img
    imgOrangeConeLeft=imgCopy[minX:maxX,minY:minY+differenceY/2]
    imgOrangeConeRight=imgCopy[minX:maxX, minY+differenceY/2:maxY]

    img = img[minX:maxX, minY:maxY]

    hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsvLeft=cv2.cvtColor(imgOrangeConeLeft, cv2.COLOR_BGR2HSV)
    hsvRight=cv2.cvtColor(imgOrangeConeRight, cv2.COLOR_BGR2HSV)
    # cv2.rectangle(hsv,(200,640),(340,470),(0,0,255),2)
    imgCannyCopy=img

    ############################################################################
    # Use color filter to find blue and yellow cones
    identyfiedConesBlue = cv2.inRange(hsv, lower_blue, upper_blue)
    identyfiedConesYellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    identyfiedConesOrangeLeft =cv2.inRange(hsvLeft, lower_orange, upper_orange)
    identyfiedConesOrangeRight =cv2.inRange(hsvRight, lower_orange, upper_orange)
    #remove noise using erode and dilate for yellow and blue filter
 
    kernel=numpy.ones((5,5),numpy.uint8)
    blueCones=cv2.dilate(identyfiedConesBlue,kernel,iterations=1)


    kernel=numpy.ones((5,5),numpy.uint8)
    yellowCones=cv2.dilate(identyfiedConesYellow,kernel,iterations=1)
    orangeConesLeft=cv2.dilate(identyfiedConesOrangeLeft,kernel, iterations =1)
    orangeConesRight=cv2.dilate(identyfiedConesOrangeRight,kernel, iterations =1)

     # find black
    identyfiedcar = cv2.inRange(hsv,lower_black,upper_black)
    kernel=numpy.ones((5,5),numpy.uint8)
    car=cv2.dilate(identyfiedcar,kernel,iterations=1)
    #orangeCones=cv2.dilate(identyfiedConesOrange,kernel, iterations =1)

    #use Canny edge detection
    #edgeImage=cv2.Canny(imgCannyCopy,200,400)

    #use morphology
    edgeImage=cv2.Canny(imgCannyCopy,400,450)
    kernel=numpy.ones((15,15),numpy.uint8)
    edgeImage=cv2.morphologyEx(edgeImage,cv2.MORPH_CLOSE,kernel)


    #use home constructed mothod to find center of each object identified
    yellowCenters=findCenters(yellowCones)
    blueCenters=findCenters(blueCones)
    orangeCentersLeft=findCenters(orangeConesLeft)
    orangeCentersRight=findCenters(orangeConesRight)
    blackCenters=findCenters(identyfiedcar)

    #sort to see which object is closes

    sortedBlueCenter=sorted(blueCenters, key=lambda x: x[1], reverse= True)
    sortedYellowCenter=sorted(yellowCenters,key=lambda x: x[1], reverse=True)
    sortedOrangeCenterLeft=sorted(orangeCentersLeft,key=lambda x: x[1], reverse = True)
    sortedOrangeCenterRight=sorted(orangeCentersRight,key=lambda x: x[1], reverse = True)
    sortedBlackCenter=sorted(blackCenters,key=lambda x: x[1], reverse = True)

    if len(blueCenters) >1:
     closestBlue=numpy.array(sortedBlueCenter[1])
    elif len(blueCenters) > 0:
     closestBlue=numpy.array(sortedBlueCenter[0])
    else: 
     closestBlue=numpy.array([60,differenceY])
     noBlue=1

    if len(yellowCenters) >1:
     closestYellow=numpy.array(sortedYellowCenter[1])
    elif len(yellowCenters) >0:
     closestYellow=numpy.array(sortedYellowCenter[0])
    else:
     closestYellow=numpy.array([50,differenceY])
     noYellow=1

    if len(blackCenters) >1:
     closestBlack=sortedBlackCenter[1]
    if len(blackCenters) >0:
     closestBlack=sortedBlackCenter[0]
    else:
     closestBlack=(20,differenceY)
     noBlack=1


    if len(orangeCentersLeft) >0:
     orangeConeLeft=orangeCentersLeft[0]
     if orangeConeLeft[1]<closestYellow[1]:
       closestYellow[0]=orangeConeLeft[0]+minX
       #closestYellow[0]=1*orangeConeLeft[0]
       closestYellow[1]=orangeConeLeft[1]
       seeingOrange=1
    
    if len(orangeCentersRight) >0:
      orangeConeRight=orangeCentersRight[0]
      if orangeConeRight[1] < closestBlue[1]:
       closestBlue[0]=orangeConeRight[0]+difference/2+minX
       #closestBlue[0]=1*(orangeConeRight[0])
       closestBlue[1]=orangeConeRight[1]
       seeingOrange=1

    res= cv2.add(yellowCones,blueCones)


    # The following example is adding a red rectangle and displaying the result.
    #plot closest object to see that the method works
    # cv2.circle(res, (closestBlue[0],closestBlue[1]), 18, (255,150,255), -1)
    # cv2.circle(res, (closestYellow[0],closestYellow[1]), 18, (255,150,255), -1)

    #position_x=(closestBlue[0]-closestYellow[0])*0.5
    #position_y=(closestBlue[1]-closestYellow[1])*0.5

    position_x=(closestYellow[0]-closestBlue[0])*0.5
    position_y=(closestYellow[1]-closestBlue[1])*0.5


    if numpy.absolute(position_y) > 0.05:
     diagonal=numpy.sqrt(position_x**2+position_y**2)
     angle=3.1415*0.5-numpy.arctan2(position_x,position_y)
     #angle=numpy.arctan2(position_x,position_y)
    else:
     angle=0.0




    #if only one cone if found 
    """if noYellow +noBlue ==1:
      if noYellow=1:
        numerator=middlePoint*closestBlue[0]
        denominator=numpy.sqrt(closestBlue[0]**2+middlePoint**2)*closestBlue[1]
        angle=numpy.arccos(numerator/denominator)*0.5
      else:
        numerator=middlePoint*closestYellow[0]
        denominator=numpy.sqrt(closestYellow[0]**2+middlePoint**2)*closestYellow[1]
        angle=numpy.arccos(numerator/denominator)*0.5"""




    # lineThickness = 2
    # cv2.line(res, (maxX/2,maxY),(position_x,position_y),(255,150,255),lineThickness)
    # font= cv2.FONT_HERSHEY_SIMPLEX

   #find and mark coordiante to object
    edgeImageCroped=edgeImage[closestYellow[0]+30:closestBlue[0]-30, 1:maxY]
    closeCars=findCenters(edgeImageCroped)
    sortedCloseCars=sorted(closeCars,key=lambda x: x[1], reverse=True)
    """ if len(sortedCloseCars) > 0:
     closestCar=sortedCloseCars[0]
     cv2.circle(edgeImage, (closestYellow[0]+30+closestCar[0],closestCar[1]), 18, (255,150,255), -1)"""


    
    
    

    ############################################################################
    # Example: Accessing the distance readings.
    print "Front = " + str(distances["front"])
    print "Left = " + str(distances["left"])
    print "Right = " + str(distances["right"])
    print "Rear = " + str(distances["rear"])

    ############################################################################

    ############################################################################
    # Steering and acceleration/decelration.
    #
    # Uncomment the following lines to steer; range: +38deg (left) .. -38deg (right).
    # Value groundSteeringRequest.groundSteering must be given in radians (DEG/180. * PI).
    
    maximumDegree=0.2

    if angle > 0 and angle> maximumDegree:
     angle=maximumDegree

    if angle <0 and angle < -maximumDegree:
      angle=-maximumDegree

    if distances["front"] < 0.05:
      angle=0.00

    """ cv2.putText(res,'Angle='+str(angle),(200,20),font,1,(255,255,255),2,cv2.LINE_AA)
    cv2.imshow("image", res);
    cv2.waitKey(2);"""
    
    groundSteeringRequest = opendlv_standard_message_set_v0_9_6_pb2.opendlv_proxy_GroundSteeringRequest()
    #groundSteeringRequest.groundSteering=0
    #error=(numpy.absolute(angle)/maximumDegree)
    
    if noBlue + noYellow >1:
	error=0.7
    else:
	error=0.5


    if seeingOrange==1:
	error=0.01
	seeingOrange=0
    

    groundSteeringRequest.groundSteering=error*(0.7*angle+0.3*prevAngle)
    prevAngle=angle
    session.send(1090, groundSteeringRequest.SerializeToString());
    # Uncomment the following lines to accelerate/decelerate; range: +0.25 (forward) .. -1.0 (backwards).
    pedalPositionRequest = opendlv_standard_message_set_v0_9_6_pb2.opendlv_proxy_PedalPositionRequest()

    #when it sees a car
    if closestBlack[1] < 300:
     pedalPositionRequest.position=0
   # elif closestBlack[1] > 390:
    # pedalPositionRequest.position = 0.09      
    else:
     pedalPositionRequest.position=0.09

 
    """
    if distances["front"] < 0.03:
      pedalPositionRequest.position=0
    elif distances["front"] < 0.8:
      #if len(sortedCloseCars) > 0:
        carSpeed=keepDistance(distances["front"],carSpeed)
        pedalPositionRequest.position=carSpeed
    else:
        pedalPositionRequest.position = carSpeed
    
    """
    if distances["front"] < 0.05:
      pedalPositionRequest.position=0
    """else:
        pedalPositionRequest.position = 0.09"""
    
    session.send(1086, pedalPositionRequest.SerializeToString());

