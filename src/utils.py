import numpy as np
import cv2

def reframing(x) :
	outy = []
	for elx in x :
		y = np.zeros_like(elx)
		#Z axis : forward <--> x
		y[2,0] = elx[0,0]
		#X axis : right <--> minus y
		y[0,0] = -elx[1,0]
		#Y axis : below <--> minus z 
		y[1,0] = -elx[2,0]
		#y[1,0] = -0.2
		outy.append(y)
	return outy
	
def makeboundingbox(x,size=0.2) :
	y1 = np.copy(x)
	y2 = np.copy(x)
	'''
	y1[0,0] -= 3*size
	y2[0,0] += 3*size
	y1[1,0] -= 3*size
	y2[1,0] += size
	y1[2,0] += 2*size
	y2[2,0] += 2*size
	'''
	y1[0,0] -= 2*size
	y2[0,0] += 2*size
	y1[1,0] -= 2*size
	y2[1,0] += 2*size
	y1[2,0] += 1*size
	y2[2,0] += 1*size
	return y1,y2

def draw(image,camera,x) :
	for j in range(len(x)) :
		for i in range(1) :
			xp = x[j].copy()
			xp[2,0] += i*0.1 
			pose3d1, pose3d2 = makeboundingbox(xp)
			pose2d = camera.project(xp)
			color = (255,255,255)
			cv2.circle(image, center=( pose2d[0], pose2d[1] ), radius=2, color=color, thickness=1)
			p1 = camera.project(pose3d1)
			p2 = camera.project(pose3d2)
			color = (255,255,255)
			cv2.rectangle(image, (p1[0], p1[1]), (p2[0], p2[1]), color=color, thickness=2)
			
			if i==0 :
				text = 'x={} / y={}'.format( int(pose2d[0]), int(pose2d[1]) )
		cv2.putText(image,text=text, org=(10,(j+1)*50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255,255,255), thickness=2)