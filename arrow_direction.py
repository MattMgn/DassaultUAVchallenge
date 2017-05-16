#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Direction detection of a red arrow on the ground
 

written by Matthieu Magnon
2016-2017
Dassault UAV Challenge
"""

import cv2
import numpy as np
from numpy import linalg as LA
import operator
import time

t = time.time()

## FINDING RED
img = cv2.imread('grass_arrow_2.jpg') #2 simple, 3 hihg noise, low 4 noise, 5 direction2
img_original = img
hsv = cv2.cvtColor(img ,cv2.COLOR_BGR2HSV)

lower = np.array([0,100,100]) #hsv
upper = np.array([10,255,255])

mask = cv2.inRange(hsv, lower, upper)
cv2.imwrite('100red_detection.jpg',mask)

### BLUR
kernel = np.ones((6,6),np.float32)/25
dst = cv2.filter2D(mask,-1,kernel)
cv2.imwrite('101blur.jpg',dst)

####ERODE
se1 = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
se2 = cv2.getStructuringElement(cv2.MORPH_RECT, (2,2))
erosion = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, se1)
erosion = cv2.morphologyEx(erosion, cv2.MORPH_OPEN, se2)

se1 = cv2.getStructuringElement(cv2.MORPH_CROSS, (5,5)) #5
se2 = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3)) #3
erosion2 = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, se1)
#cv2.imwrite('02erosion2bis.jpg',erosion2)
erosion2 = cv2.morphologyEx(erosion2, cv2.MORPH_OPEN, se2)
se3 = cv2.getStructuringElement(cv2.MORPH_CROSS, (4,4)) #5
se4 = cv2.getStructuringElement(cv2.MORPH_CROSS, (2,2)) #3
erosion2 = cv2.morphologyEx(erosion2, cv2.MORPH_CLOSE, se3)
#cv2.imwrite('03erosion2ter.jpg',erosion2)
erosion2 = cv2.morphologyEx(erosion2, cv2.MORPH_OPEN, se4)

kernel1 = np.ones((2,2),np.uint8)
erosion3 = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel1)
cv2.imwrite('102erosion1.jpg',erosion)
cv2.imwrite('103erosion2.jpg',erosion2)
cv2.imwrite('104erosion3.jpg',erosion3)

erosion = erosion2

# EDGE
edges = cv2.Canny(erosion,130,150,apertureSize = 3) #130 150 3
cv2.imwrite('105canny.jpg',edges)

# HOUGH
minLineLength = 20
#cv2.HoughLinesP(edges,p accuracy th accuracy in rad, 100, threshold minLineLength,maxLineGap)
lines = cv2.HoughLinesP(edges,rho = 1,theta = 1*np.pi/180,threshold = 10,minLineLength = 10,maxLineGap = 20)
#lines = cv2.HoughLinesP(edges,rho = 1,theta = 1*np.pi/180,threshold = 10,minLineLength = 10,maxLineGap = 20)
#lines = cv2.HoughLinesP(edges, rho = 1,theta = 1*np.pi/180,threshold = 80,minLineLength = 10,maxLineGap = 20)

print 'Nb of lines detected : ' , len(lines) 

i=0
rho = []
th = []
while i<= len(lines)-1:
    for x1,y1,x2,y2 in lines[i]:
		th_compute = round(np.arctan((0.0+x2-x1)/(y2-y1+0.01))*180/3.14,2)
		rho_compute = round(LA.norm([x2-x1,y2-y1]),2)
		th.append(th_compute)
		rho.append(rho_compute)
		cv2.line(img,(x1,y1),(x2,y2),(255,255,0),2)
        
        #if rho_compute > 2.0: # remove rho under 2.0
            #if th_compute == 0.0:
                #th_compute = 90.0
            #th.append(th_compute)
            #rho.append(rho_compute)
            #cv2.line(img,(x1,y1),(x2,y2),(255,255,0),2)
    i = i+1
    
cv2.imwrite('106houghlines.jpg',img)

# return index of rho from max to min
idx_rho = np.argsort(rho)
idx_rho = idx_rho[::-1] #invert array

print 'main direction is (in deg) : ' , [ th[idx_rho[0]] , th[idx_rho[1]] , th[idx_rho[2]] ]
print 'max line length (in pixel) : ' , [ rho[idx_rho[0]] , rho[idx_rho[1]] , rho[idx_rho[2]] ]

# FIND Center of Gravity COG of red pixel
npImg = np.asarray(hsv)
coordList = np.argwhere( npImg == [0, 255, 255] ); ##### X et Y sont inverses

sum_x = 0
sum_y = 0
x_array = []
y_array = []
for y,x,i in coordList:
    cv2.circle(img_original, (x, y), 1, (255, 255, 255), -1)
    sum_x = sum_x + x
    sum_y = sum_y + y
    x_array.append(x)
    y_array.append(y)
    
cv2.imwrite('107highlightedredpixel.jpg',img_original)    

cog_x = sum_x/len(coordList)
cog_y = sum_y/len(coordList)

cv2.circle(img, (cog_x, cog_y), 5, (100, 255, 255), -1)
cv2.circle(img, (int(np.median(x_array)), int(np.median(y_array))), 5, (100, 100, 100), -1)

print 'COG : ' , [cog_x, cog_y]
print 'MED ; ' , [np.median(x_array),np.median(y_array)]
print 'COG to find : ' , [466, 226]

#OUT 1
cv2.imwrite('108houghlineswithCOGandMED.jpg',img)

#contours = cv2.findContours(erosion, 1, 2)
contours,hierarchy = cv2.findContours(erosion, 1, 2)
cnt = contours[0]

#print 'cnt : ', cnt

area = cv2.contourArea(cnt)
print 'arrow area : ', area

perimeter = cv2.arcLength(cnt,True)
print 'arrow perimeter : ', perimeter
x_d,y_d,w_d,h_d = cv2.boundingRect(cnt)
cv2.rectangle(img, (x_d,y_d), (x_d+w_d,y_d+h_d), [100,255,100],3)
cv2.imwrite('109boundingrectangle.jpg',img_original)

# compute diagonal angle :
th_diag = round(np.arctan((0.0+w_d)/(h_d+0.001))*180/3.14,2)
print 'diagonale angle is (in deg) :', th_diag

# Center of Diagonal COD : cross point between diagonal
cod_x = int(x_d + w_d/2)
cod_y = int(y_d + h_d/2)
cv2.circle(img, (cod_x, cod_y), 5, [100,255,100], 3)

# compute direction vector u of the line perpendicular to arrow axis and crossing CoD
# vecteur directeur u de la droite passant par le COD et perpendiculaire a l axe de la fleche
u_x = int(cod_x + 100*np.sin((th[idx_rho[0]]+90)*np.pi/180))
u_y = int(cod_y + 100*np.cos((th[idx_rho[0]]+90)*np.pi/180))
cv2.line(img_original, (cod_x,cod_y), (u_x,u_y), [100,255,100],3)
cv2.imwrite('110uvector.jpg',img_original)

# scalar product between vector u and 
# produit scalaire entre u et cog//cod
scalar = np.vdot([u_x-cod_x,u_y-cod_y],[cog_x-cod_x,cog_y-cod_y])

font = cv2.FONT_HERSHEY_SIMPLEX
if scalar > 0:
    print 'Arrow towards the right '
    cv2.putText(img,'direction : right',(10,20),font,1,(255,255,255))
else:
    print 'Arrow towards the left '
    cv2.putText(img,'direction : left',(10,20),font,1,(255,255,255))

#OUT 2

cv2.imwrite('109arrowdirection.jpg',img)



# END
elapsed = time.time() - t
print 'speed (ms) : ' , round(1000.00*elapsed,3)
