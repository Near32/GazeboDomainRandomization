from GazeboDomainRandom import * 
import matplotlib.pyplot as plt
import cv2

def reframing(x) :
	y = np.zeros_like(x)
	#Z axis : forward <--> x
	y[2,0] = x[0,0]
	#X axis : right <--> minus y
	y[0,0] = -x[1,0]
	#Y axis : below <--> minus z 
	y[1,0] = -x[2,0]
	#y[1,0] = -0.2
	return y
	
def makebox(x) :
	y1 = np.copy(x)
	y2 = np.copy(x)
	y1[0,0] -= 0.75
	y2[0,0] += 0.75
	y1[1,0] -= 0.75
	y2[1,0] += 0.25
	return y1,y2
	
	
def test_camera() :
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
	pose_min[0][0] = 1.0
	pose_min[0][1] = -2.0
	pose_min[0][2] = 0.0
	pose_max = np.concatenate( [2.5*np.ones((1,3)), np.zeros((1,3))], axis=1)
	pose_max[0][0] = 5.0
	pose_max[0][1] = 2.0
	pose_max[0][2] = 0.0
	
	listin = list()
	'''
	for el in shape_types :
		shpr = ShapeParamsRanges(shape_type=el, shape_pose_min=pose_min, shape_pose_max=pose_max, shape_color_list=colors)
		shF = ShapeFactory(params=shpr)
		occ = OccParamsRanges(occ_min=1,occ_max=3)
		listin.append( (shF, occ) )
	'''
	
	#Add the model :
	shpr = ShapeParamsRanges(shape_type='model', shape_pose_min=pose_min, shape_pose_max=pose_max, shape_color_list=colors)
	shF = ShapeFactory(params=shpr)
	occ = OccParamsRanges(occ_min=1,occ_max=2)
	listin.append( (shF, occ) )

	cfact = ConfigurationFactory(info=listin)
	cfact.init()
	
	#create new config :
	newconfig = cfact.generate()
	
	#spawn the new config :
	cfact.spawn(newconfig)
	
	
	#handle the camera :
	camera = Camera(fovy=120,altitude=0.2,port=cfact.port, env=cfact.env)
	camera.spawn()
	
	#retrieve the position of the model :
	shape_list = cfact.getCurrentConfiguration().getShapeList()
	model_params = [ shape for shape in shape_list if shape.getType() == 'model' ][0]
	pose3d = model_params.getPose()[0,0:3]
	pose3d = np.reshape(pose3d, (3,1) )
	
	'''
	REFRAMING :
	IMPORTANT : the point must be specified in the camera frame :
	'''
	pose3d = reframing(pose3d)
	pose3d1, pose3d2 = makebox(pose3d)
	'''
	END OF REFRAMING
	'''
	
	time.sleep(2)
	
	while True :
		time.sleep(1)
		print('MODEL POSE : {}'.format(pose3d ) )
		pose2d = camera.project(pose3d)
		print('MODEL 2D POSE : {}'.format(pose2d ) )
		
		image = camera.getImage()
		if image is not None :
			pose2d = camera.project(pose3d)
			color = (0,255,0)
			cv2.circle(image, center=( pose2d[0], pose2d[1] ), radius=5, color=color, thickness=2, lineType=8, shift=0)
			p1 = camera.project(pose3d1)
			p2 = camera.project(pose3d2)
			color = (0,0,255)
			print(p1)
			print(p2)
			cv2.rectangle(image, (p1[0], p1[1]), (p2[0], p2[1]), color=color, thickness=1)
			plt.imshow(image)
			plt.show()
	
			
	
test_camera()

