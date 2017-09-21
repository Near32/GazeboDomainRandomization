from GazeboDomainRandom import * 
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
	y1[0,0] -= 3*size
	y2[0,0] += 3*size
	y1[1,0] -= 3*size
	y2[1,0] += size
	y1[2,0] += 2*size
	y2[2,0] += 2*size
	return y1,y2
	
	
def test_newconfig() :
	np.random.seed()
	#list of colors :
	colors = ['White', 'Blue', 'Orange', 'Pink', 'Green', 'Purple', 'Yellow', 'Red', 'Black', 'Grey']
	#list of shape type ;
	boxshape = 'box'
	rectangleshape = 'rectangle'
	sphereshape = 'sphere'
	coneshape = 'cone'
	cylindershape = 'cylinder'
	shape_types = [cylindershape, boxshape,sphereshape, rectangleshape]
	
	pose_min = np.concatenate( [2.0*np.ones((1,3)), np.zeros((1,3))], axis=1)	
	pose_min[0][0] = -1.0
	pose_min[0][1] = -10.0
	pose_min[0][2] = 0.0
	pose_max = np.concatenate( [2.5*np.ones((1,3)), np.zeros((1,3))], axis=1)
	pose_max[0][0] = 5.0
	pose_max[0][1] = -pose_min[0][1]
	pose_max[0][2] = 0.0

	max_occ = 5
	
	listin = list()
	
	for el in shape_types :
		shpr = ShapeParamsRanges(shape_type=el, shape_pose_min=pose_min, shape_pose_max=pose_max, shape_color_list=colors)
		shF = ShapeFactory(params=shpr)
		occ = OccParamsRanges(occ_min=1,occ_max=max_occ)
		listin.append( (shF, occ) )
	
	#Add the model :
	pose_min[0][0] = 0.5
	pose_min[0][1] = -5.0
	pose_min[0][2] = 0.0
	pose_max[0][0] = 5.0
	pose_max[0][1] = -pose_min[0][1]
	pose_max[0][2] = 0.0
	shpr = ShapeParamsRanges(shape_type='model', shape_pose_min=pose_min, shape_pose_max=pose_max, shape_color_list=colors)
	shF = ShapeFactory(params=shpr)
	occ = OccParamsRanges(occ_min=1,occ_max=3)
	listin.append( (shF, occ) )

	cfact = ConfigurationFactory(info=listin)
	cfact.init()
	
	#create new config :
	newconfig = cfact.generate()
	#spawn the new config :
	cfact.spawn(newconfig)
	
	
	#handle the camera :
	fovy = 160
	altitude = 0.5
	camera = Camera(fovy=fovy,altitude=altitude,port=cfact.port, env=cfact.env)
	camera.spawn()
	
	#retrieve the position of the model :
	shape_list = cfact.getCurrentConfiguration().getShapeList()
	model_params = [ shape for shape in shape_list if shape.getType() == 'model' ]
	pose3d = [ np.reshape( model.getPose()[0,0:3], (3,1) ) for model in model_params ]
	'''
	REFRAMING :
	IMPORTANT : the point must be specified in the camera frame :
	'''
	pose3d = reframing(pose3d)
	'''
	END OF REFRAMING
	'''
	
	time.sleep(2)
	
	alt_step = 0.2
	alt_min = 1.0
	alt_max = 2.0
	
	
	fovy_min = 45
	fovy_step = 30
	fovy_max = 185
	
	continuer = True
	while continuer :
		#print('MODEL POSE : {}'.format(pose3d ) )
		pose2d = []
		for el in pose3d :
			pose2del = camera.project(el)
			pose2d.append(pose2del)
		#print('MODEL 2D POSE : {}'.format(pose2d ) )
		
		image = camera.getImage()
		
		if image is not None :
			draw(image,camera,pose3d)
		else :
			image = np.zeros( (480,640) )
				
		cv2.imshow('bounding box fovy={} alt={}'.format(fovy,altitude),image)
		key = cv2.waitKey(1) & 0xFF
		
		if key == ord('a') :
			altitude += alt_step
			if altitude >= alt_max :
				altitude = alt_min		
			camera.setAltitude(altitude)		
			#time.sleep(5.0)
			cv2.destroyAllWindows()
		elif key == ord('f') :
			fovy += fovy_step
			if fovy >= fovy_max :
				fovy = fovy_min
			camera.setFovy(fovy)
			#time.sleep(5.0)
			cv2.destroyAllWindows()
		elif key == ord('q') :
			continuer = False
		elif key == ord('n') :
			#create new config :
			newconfig = cfact.generate()
			index = 0
			for el in newconfig.shape_list :
				print(" new NEW {} : {}".format(index,el.getType()) )
				index += 1
			#spawn the new config :
			cfact.spawn(newconfig)
			#retrieve the position of the model :
			shape_list = cfact.getCurrentConfiguration().getShapeList()
			model_params = [ shape for shape in shape_list if shape.getType() == 'model' ]
			pose3d = [ np.reshape( model.getPose()[0,0:3], (3,1) ) for model in model_params ]
			#REFRAMING :
			pose3d = reframing(pose3d)
			


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
		
test_newconfig()

