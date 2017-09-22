from XMLGenerator import XMLGenerator, XMLObjectTags
from GazeboDomainRandom import *
from utils import reframing, makeboundingbox, draw
import cv2
import os 

def create_dataset(path='./dataset/') :
	np.random.seed()
	#XML generator :
	genXML = XMLGenerator(path_out=path)
	#index to name the pictures :
	date = '2017_09_22__'
	indexIMG = 0
	#create the images folder :
	pathimg = 'images/'
	if not os.path.exists(path+pathimg) :
		os.makedirs(path+pathimg)

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
	fovy = 120
	altitude = 0.45
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
	original_image = None
	
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
			original_image = image.copy()
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
		elif key == ord('p') :
			listobj = []
			for i in range(len(pose3d)) :
				xp = pose3d[i].copy()
				pose3d1, pose3d2 = makeboundingbox(xp)
				p1 = camera.project(pose3d1)
				p2 = camera.project(pose3d2)
				pose = [ int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1]) ]
				print(pose)
				listobj.append( XMLObjectTags(name='model',pose=pose) )
			filename = date+str(indexIMG)
			genXML.generate(object_tags=listobj, filename=filename)
			indexIMG+=1
			cv2.imwrite( path+pathimg+filename+'.png',original_image)



create_dataset(path='./dataset_test/')