from GazeboDomainRandom import * 

def create_env() :
	#list of colors :
	colors = ['White', 'Blue', 'Orange', 'Pink', 'Green', 'Purple', 'Yellow', 'Red', 'Black', 'Grey']
	#list of shape type ;
	boxshape = 'box'
	rectangleshape = 'rectangle'
	sphereshape = 'sphere'
	coneshape = 'cone'
	cylindershape = 'cylinder'
	shape_types = [cylindershape, boxshape,sphereshape, rectangleshape]
	
	pose_min = np.concatenate( [-5.0*np.ones((1,3)), np.zeros((1,3))], axis=1)	
	pose_min[0][2] = 0.0
	pose_max = np.concatenate( [5.0*np.ones((1,3)), np.zeros((1,3))], axis=1)
	pose_max[0][2] = 0.0
	
	listin = list()
	for el in shape_types :
		shpr = ShapeParamsRanges(shape_type=el, shape_pose_min=pose_min, shape_pose_max=pose_max, shape_color_list=colors)
		shF = ShapeFactory(params=shpr)
		occ = OccParamsRanges(occ_min=1,occ_max=3)
		listin.append( (shF, occ) )
	
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
	camera = Camera(fovy=120,altitude=1.0,port=cfact.port, env=cfact.env)
	camera.spawn()
	
	while True :
		time.sleep(20)
		
			
	
create_env()

