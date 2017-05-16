#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Workshop 2 : Follow the arrow
During the flight, the hexacopter must change in full autonomy, its
flightpath by following the direction indicated by a red arrow
drawn on the ground, then land on a dedicated landing area (black square)

written by Matthieu Magnon
2016-2017
Dassault UAV Challenge
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import time
import cv2
import numpy as np
import math
from PIL import Image

# Connect CAN camera

cap = cv2.VideoCapture(0)
print 'Connected to FPV camera' 
#cam.set(cv2.CAP_FFMPEG,True)
#cam.set(cv2.CAP_PROP_FPS,30)

#connection to pixhawk
connection_string = '127.0.0.1:14550'
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)

# PARAMETERS
alt=10 #altitude d'evolution initiale
dist_to_np = 20 #distance en meter pour aller au next poin 
initial_ground_speed = 3
reduced_ground_speed = 1

# Define functions

def arm_and_takeoff(aTargetAltitude):

  print "Basic pre-arm checks"
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print " Waiting for vehicle to initialise..."
    time.sleep(1)
        
  print "Arming motors"
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print " Waiting for arming..."
    time.sleep(1)

  print "Taking off!"
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print " Altitude: ", vehicle.location.global_relative_frame.alt 
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print "Reached target altitude"
      break
    time.sleep(1)

def compute_arrowsize(alt):
	alpha = 1
	beta =1
	nbpixel = alpha/alt + beta ## a modifier suivant les tests
	print 'altitude is :', alt
	print 'nb pixel in arrow is :' , nbpixel
	
	return nbpixel
    
def itisred(img,nbpixeltofind):
	#INPUT : image img, integer nbpixeltofind
	#OUTPUT : BOOLEAN
	
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	
	#dfinition de la gamme de couleur   selectionner
	lower = np.array([0,100,100]) # type de couleur hsv
	upper = np.array([10,255,255])
	
	mask = cv2.inRange(hsv, lower, upper)

	cv2.imwrite('0red_detection.jpg',mask)
	im3 = Image.open('0red_detection.jpg')
	
	count = 0
	for pixel in im3.getdata():
		if pixel == (1): 
			count += 1
	
	print 'nb red pixels : ' , count
	if count > nbpixeltofind:
		return 'TRUE'
	else:
		return 'FALSE'
             
def find_arrowdirection(img):
	
	#cette fonction retourne la direction de la fleche
	# si le retour = -1 alors cest qu'aucune direction n'a ete trouvee
	
	hsv = cv2.cvtColor(img ,cv2.COLOR_BGR2HSV)
	
	## FINDING RED
	lower = np.array([0,100,100]) #hsv
	upper = np.array([10,255,255])
	mask = cv2.inRange(hsv, lower, upper)
	
	### BLUR
	kernel = np.ones((6,6),np.float32)/25
	dst = cv2.filter2D(mask,-1,kernel)
	cv2.imwrite('0blur.jpg',dst)
	
	####ERODE
	se1 = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
	se2 = cv2.getStructuringElement(cv2.MORPH_RECT, (2,2))
	erosion = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, se1)
	erosion = cv2.morphologyEx(erosion, cv2.MORPH_OPEN, se2)
	
	se1 = cv2.getStructuringElement(cv2.MORPH_CROSS, (5,5)) #5
	se2 = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3)) #3
	erosion2 = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, se1)
	erosion2 = cv2.morphologyEx(erosion2, cv2.MORPH_OPEN, se2)
	se3 = cv2.getStructuringElement(cv2.MORPH_CROSS, (4,4)) #5
	se4 = cv2.getStructuringElement(cv2.MORPH_CROSS, (2,2)) #3
	erosion2 = cv2.morphologyEx(erosion2, cv2.MORPH_CLOSE, se3)
	cv2.imwrite('0erosion2ter.jpg',erosion2)
	erosion2 = cv2.morphologyEx(erosion2, cv2.MORPH_OPEN, se4)
	
	kernel1 = np.ones((2,2),np.uint8)
	erosion3 = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel1)
	cv2.imwrite('0erosion1.jpg',erosion)
	cv2.imwrite('0erosion2.jpg',erosion2)
	cv2.imwrite('0erosion3.jpg',erosion3)
	
	erosion = erosion2
	
	####EDGE
	edges = cv2.Canny(erosion,130,150,apertureSize = 3) #130 150 3
	cv2.imwrite('0canny.jpg',edges)
	
	#### HOUGH
	minLineLength = 20 # nb pixel formant la ligne : A MODIFIER
	#cv2.HoughLinesP(edges,p accuracy th accuracy in rad, 100, threshold minLineLength,maxLineGap)
	#lines = cv2.HoughLinesP(edges,rho = 1,theta = 1*np.pi/180,threshold = 10,minLineLength = 10,maxLineGap = 20)
	lines = cv2.HoughLinesP(edges,rho = 1,theta = 1*np.pi/180,threshold = 10,minLineLength = 10,maxLineGap = 20)
	
	i=1
	rho = []
	th = []
	
	if (len(lines) == 0):
		print 'no lines detected '
		theta = 1000
		return theta
		
	while i<= len(lines)-1:
	    for x1,y1,x2,y2 in lines[i]:
	        th_compute = round(np.arctan((0.0+x2-x1)/(y2-y1+0.01))*180/3.14,2)
	        rho_compute = round(LA.norm([x2-x1,y2-y1]),2)
	        if rho_compute > 30.0: # remove rho under 2.0
	            if th_compute == 0.0:
	                th_compute = 90.0
	            th.append(th_compute)
	            rho.append(rho_compute)
	            cv2.line(img,(x1,y1),(x2,y2),(255,255,0),2)
	    i = i+1
	
	# SO
	if (len(rho) == 0):
		print 'no lines detected '
		theta = 1000
		return theta
			
	print 'lines detected : ' , len(rho)
	
	# return index of rho from max to min
	idx_rho = np.argsort(rho)
	idx_rho = idx_rho[::-1] #invert array
	
	print 'main direction is (in deg) : ' , [ th[idx_rho[0]] , th[idx_rho[1]] , th[idx_rho[2]] ]
	print 'max line length : ' , [ rho[idx_rho[0]] , rho[idx_rho[1]] , rho[idx_rho[2]] ]
	
	# FIND BARYCENTRE
	npImg = np.asarray(hsv)
	coordList = np.argwhere( npImg == [0, 255, 255] ); ##### X et Y sont inverses
	
	sum_x = 0
	sum_y = 0
	x_array = []
	y_array = []
	for y,x,i in coordList:
	    cv2.circle(img, (x, y), 1, (255, 255, 255), -1)
	    sum_x = sum_x + x
	    sum_y = sum_y + y
	    x_array.append(x)
	    y_array.append(y)
	    
	cog_x = sum_x/len(coordList)
	cog_y = sum_y/len(coordList)
	
	cv2.circle(img, (cog_x, cog_y), 5, (100, 255, 255), -1)
	cv2.circle(img, (int(np.median(x_array)), int(np.median(y_array))), 5, (100, 100, 100), -1)
	
	print 'COG : ' , [cog_x, cog_y]
	print 'MED ; ' , [np.median(x_array),np.median(y_array)]
	print 'COG to find : ' , [466, 226]
	mask = cv2.inRange(hsv, lower, upper)
		
	contours = cv2.findContours(erosion, 1, 2)
	contours = contours[1]
	cnt = contours[0]
	
	print 'cnt : ', cnt
	
	area = cv2.contourArea(cnt)
	print 'arrow area : ', area
	
	perimeter = cv2.arcLength(cnt,True)
	print 'arrow perimeter : ', perimeter
	x_d,y_d,w_d,h_d = cv2.boundingRect(cnt)
	cv2.rectangle(img, (x_d,y_d), (x_d+w_d,y_d+h_d), [100,255,100],3)
	
	# compute angle de la diagonale :
	th_diag = round(np.arctan((0.0+w_d)/(h_d+0.001))*180/3.14,2)
	print 'diagonale angle is (in deg) :', th_diag
	
	# centre des diagonales
	cod_x = int(x_d + w_d/2)
	cod_y = int(y_d + h_d/2)
	cv2.circle(img, (cod_x, cod_y), 5, [100,255,100], 3)
	
	# vecteur u de la droite passant par le COD et perpendiculaire a l axe de la fleche
	u_x = int(cod_x + 100*np.sin((th[idx_rho[0]]+90)*np.pi/180))
	u_y = int(cod_y + 100*np.cos((th[idx_rho[0]]+90)*np.pi/180))
	
	# produit scalaire entre u et cog//cod
	scalar = np.vdot([u_x-cod_x,u_y-cod_y],[cog_x-cod_x,cog_y-cod_y])
	
	if scalar > 0:
	    print 'fleche vers la droite '
	    
	else:
	    print 'fleche vers la gauche '
	    
	
	
	return theta
    
def compute_GPScoord(theta,dist_to_np):
    
    """"
    from current point cp, compute next way point nwp with orientation theta and distance
    """
    time.sleep(3) # pour etre sur de ne pas bouger
    yaw = vehicle.attitude.yaw  #get current yaw
    
    cap = yaw + theta
    dNorth = np.cos(3.14/180 * cap)*dist_to_np
    dEast = np.sin(3.14/180 * cap)*dist_to_np
    
    ##### a definir
    
    # define next point np
    nwp = get_location_metres(vehicle.location.global_frame, dNorth, dEast)
    print 'nwp : ', nwp
    return nwp
    
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation;

def itisblack(img,nbpixeltofind):
	#INPUT : image img, integer nbpixeltofind
	#OUTPUT : BOOLEAN
	
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	
	#dfinition de la gamme de couleur   selectionner
	lower = np.array([0,100,100]) # type de couleur hsv
	upper = np.array([10,255,255]) 
	
	mask = cv2.inRange(hsv, lower, upper)

	cv2.imwrite('0red_detection.jpg',mask)
	im3 = Image.open('0red_detection.jpg')
	
	count = 0
	for pixel in im3.getdata():
		if pixel == (1): 
			count += 1
	
	print 'nb black pixels : ' , count
	if count > nbpixeltofind:
		return 'TRUE'
	else:
		return 'FALSE'
		
	#DETECTION CARRE
		

def itislandpad(img,nbpixeltofind):

	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	
	#RECHERCHE DE NOIR
	#dfinition de la gamme de couleur   selectionner
	lower = np.array([0,100,100]) # type de couleur hsv
	upper = np.array([10,255,255])
	
	mask = cv2.inRange(hsv, lower, upper)

	cv2.imwrite('0red_detection.jpg',mask)
	im3 = Image.open('0red_detection.jpg')
	
	count = 0
	for pixel in im3.getdata():
		if pixel == (1): 
			count += 1
	
	print 'nb red pixels : ' , count
	if count > nbpixeltofind:
		# DETECTION DE CARRE
		print('Enough black pixels found')
		print('Detecting square')
		
		
		
	else:
		print('Not enough black pixels found')
		return 'FALSE'


### SCRIPT

# initialisation
isred = 'FALSE'
isarrow = 'FALSE'
isblack = 'FALSE'
islandpad = 'FALSE'
landed = 'FALSE'
nbpixel=compute_arrowsize(alt) #evalue la taille de la fleche en pixel a cette altitude

#DEBUG


while(landed == 'FALSE'):
	
	print("Starting sequence")
	
	# Info Global
	#vehicle.GPSInfo(eph, epv, fix_type, satellites_visible)
	##print 'Nb Satellites : ', vehicle.GPSInfo.satellites_visible
				
	# decollage du point A jusqu'a alt metres
	arm_and_takeoff(alt)
		
	# go to point B
	pointB = LocationGlobalRelative(-35.361354, 149.165218, 20) # MODIFIER
	vehicle.simple_goto(pointB, groundspeed=initial_ground_speed)
		
	try:
		
		#acquisition image	
		ret, img = cap.read()
		
		#DEBUG
		nbpixel = 3500
		# presence fleche
		while isred == 'FALSE':
			ret, img = cap.read()
			isred = itisred(img,nbpixel*0.15) # des qu'il ya 15% de la fleche ds l'image
		
		# si presence de rouge supérieure au seuil nbpixel*0.15
		print('Red found')
		
		#reduire la vitesse
		vehicle.groundspeed = reduced_ground_speed
		print 'reducing speed to' , reduced_ground_speed
		time.sleep(2) #delay en seconde pour ralentir 
		
		# recherche de la fleche jusqu'au nouveau seuil nbpixel*0.90
		while isarrow == 'FALSE':
			ret, img = cap.read() #acquisition nouvelle image
			isarrow = itisred(img,nbpixel*0.90)
		
		# si presence de fleche   
		print('Arrow found')
		print('Stop at this point')
		vehicle.groundspeed = 0
		time.sleep(3) # delai de stabilisation
		
		# recherche de la direction de la fleche
		theta = find_arrowdirection(img)
		
		#calcul la prochaine destination
		next_coord = compute_GPScoord(theta,dist_to_np)
		point2 = LocationGlobalRelative(next_coord.lat,next_coord.lon,next_coord.alt)
		vehicle.simple_goto(point2, initial_ground_speed) #avec vitesse nomimale
		
		#recherche du carre noir du landpad au seuil nbpixel*0.15
		while isblack == 'FALSE':
			ret, img = cap.read() #acquisition nouvelle image
			isblack = itisblack(img,nbpixel*0.15)
		
		# si presence de noir superieure au seuil nbpixel*0.15
		print('Black found')
		
		#reduire la vitesse
		vehicle.groundspeed = reduced_ground_speed
		print 'reducing speed to', reduced_ground_speed
		
		#recherche du noir du landpad au seuil nbpixel*0.90
		while islandpad== 'FALSE': 
			ret, img = cap.read() #acquisition nouvelle image
			islandpad = itislandpad(img,nbpixel*0.90)
		
		# si presence du landpad   
		print('Landpad found')
		print('Stop at this point')
		vehicle.groundspeed = 0
		time.sleep(3) # delai de stabilisation
		print('Landing')
		vehicle.mode = VehicleMode("LAND")
		print('Drone Landed')
		print('Mission Sucessfull !')
		
		landed = 'TRUE'

	except:
		##dans ce cas, faire atterir le drone 
		print('Aborting mission...') 
		landed = 'TRUE' 
		vehicle.groundspeed = 0
		vehicle.mode = VehicleMode("LAND")
		landed = 'TRUE' 


