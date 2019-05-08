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
	optimalDistance=0.4
	originalSpeed=0.11
	detectionDistance=0.5
	if distance <= detectionDistance:
		if distance <= optimalDistance-0.03 and speed >=0.2:
			speed=speed-0.01
		elif distance >= optimalDistance+0.03 and speed <= 0.15: 
			speed=originalSpeed+0.01
	else:
		speed=originalSpeed

	return speed

prevAngle=0
fgbg=cv2.createBackgroundSubtractorMOG2(history=10,varThreshold=2,detectShadows=False)
minX=220
maxX=300
minY=170
maxY=640

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
carSpeed=0.11
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
    img = img[minX:maxX, minY:maxY]
    hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    imgCannyCopy=img

    ############################################################################
    # Use color filter to find blue and yellow cones
    identyfiedConesBlue = cv2.inRange(hsv, lower_blue, upper_blue)
    identyfiedConesYellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    #remove noise using erode and dilate for yellow and blue filter
    kernel=numpy.ones((5,5),numpy.uint8)
    blueCones=cv2.dilate(identyfiedConesBlue,kernel,iterations=2)
    #kernel=numpy.ones((5,5),numpy.uint8)
    #blueCones=cv2.dilate(blueCones,kernel,iterations=1 )
    #blueCones=cv2.Canny(identyfiedConesBlue,400,450)
    #kernel=numpy.ones((15,15),numpy.uint8)
    #blueCones=cv2.morphologyEx(blueCones,cv2.MORPH_CLOSE,kernel)
    

    kernel=numpy.ones((5,5),numpy.uint8)
    #yellowCones=cv2.erode(identyfiedConesYellow,kernel,iterations=1)
    yellowCones=cv2.dilate(identyfiedConesYellow,kernel,iterations=1)
    #kernel=numpy.ones((3,3),numpy.uint8)
    #yellowCones=cv2.erode(yellowCones,kernel,iterations=1)
    #kernel=numpy.ones((5,5),numpy.uint8)
    #yellowCones=cv2.dilate(yellowCones,kernel,iterations=1)

    #use Canny edge detection
    #edgeImage=cv2.Canny(imgCannyCopy,200,400)

    #use morphology
    edgeImage=cv2.Canny(imgCannyCopy,400,450)
    kernel=numpy.ones((15,15),numpy.uint8)
    edgeImage=cv2.morphologyEx(edgeImage,cv2.MORPH_CLOSE,kernel)

    #use foreground extraction 
    """
    edgeImage=cv2.cvtColor(imgCannyCopy,cv2.COLOR_BGR2GRAY)
    edgeImage=cv2.bilateralFilter(edgeImage,4,250,250)
    edgeImage=fgbg.apply(edgeImage)"""

    #use home constructed mothod to find center of each object identified
    yellowCenters=findCenters(yellowCones)
    blueCenters=findCenters(blueCones)


    #sort to see which object is closes

    sortedBlueCenter=sorted(blueCenters, key=lambda x: x[1], reverse= True)
    sortedYellowCenter=sorted(yellowCenters,key=lambda x: x[1], reverse=True)

    if len(blueCenters) >1:
     closestBlue=sortedBlueCenter[1]
    elif len(blueCenters) > 0:
     closestBlue=sortedBlueCenter[0]
    else: 
     closestBlue=(100,430)

    if len(yellowCenters) >1:
     closestYellow=sortedYellowCenter[1]
    elif len(yellowCenters) >0:
     closestYellow=sortedYellowCenter[0]
    else:
     closestYellow=(20,430)

    
	
    res= cv2.add(yellowCones,blueCones)


    # The following example is adding a red rectangle and displaying the result.
    #plot closest object to see that the method works
    #cv2.circle(res, (closestBlue[0],closestBlue[1]), 18, (255,150,255), -1)
    #cv2.circle(res, (closestYellow[0],closestYellow[1]), 18, (255,150,255), -1)

    position_x=(closestYellow[0]+closestBlue[0])/2
    position_y=(closestYellow[1]+closestBlue[1])/2
    diagonal=numpy.sqrt(position_x**2+position_y**2)
    angle=numpy.arcsin(position_x/diagonal)

    lineThickness = 2
    cv2.line(res, (maxX/2,maxY),(position_x,position_y),(255,150,255),lineThickness)
    font= cv2.FONT_HERSHEY_SIMPLEX

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
      angle=0

    """ cv2.putText(res,'Angle='+str(angle),(200,20),font,1,(255,255,255),2,cv2.LINE_AA)
    cv2.imshow("image", res);
    cv2.waitKey(2);"""
    
    groundSteeringRequest = opendlv_standard_message_set_v0_9_6_pb2.opendlv_proxy_GroundSteeringRequest()
    groundSteeringRequest.groundSteering =0.7*2*(0.2*angle+0.8*prevAngle)
    prevAngle=angle
    session.send(1090, groundSteeringRequest.SerializeToString());
    # Uncomment the following lines to accelerate/decelerate; range: +0.25 (forward) .. -1.0 (backwards).
    pedalPositionRequest = opendlv_standard_message_set_v0_9_6_pb2.opendlv_proxy_PedalPositionRequest()
    
    """
    if distances["front"] < 0.03:
      pedalPositionRequest.position=0
    elif distances["front"] >=0.8:
      if len(sortedCloseCars) > 0:
        carSpeed=keepDistance(distances["front"],carSpeed)
        pedalPositionRequest.position=carSpeed
      else:
        pedalPositionRequest.position = 0.11

    """
    if distances["front"] < 0.05:
      pedalPositionRequest.position=0
    else:
        pedalPositionRequest.position = 0.11
    
    session.send(1086, pedalPositionRequest.SerializeToString());

