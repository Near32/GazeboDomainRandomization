import rospy
import subprocess
import time
import os
import numpy as np
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates
from geometry+msgs.msg import Pose, Point, Quaternion
from cv_bridge import CvBridge
import copy


class ShapeParams :
	def __init__(self, shape_type=None, shape_scale=1.0, shape_pose=None, shape_color=None):
		self.shape_type = shape_type
		self.shape_scale = shape_scale
		self.shape_pose = shape_pose
		self.shape_color = shape_color

		self.shape_name = None

	
	def getPose(self) :
		return self.shape_pose
		
	def getType(self) :
		return self.shape_type

	def setName(self,name) :
		self.shape_name = name

	def getName(self):
		return self.shape_name
					
class ShapeParamsRanges :
	def __init__(self, shape_type, shape_scale_max=1.0, shape_pose_min=None, shape_pose_max=None, shape_color_list=['White']) :
		self.shape_type = shape_type
		self.shape_scale_max= shape_scale_max
		self.shape_pose_min = shape_pose_min
		self.shape_pose_max = shape_pose_max
		self.shape_color_list = shape_color_list

			
class ShapeFactory :
	'''
	Factory of shapes...
	In : ShapeParamsranges
	Out : ShapeParams
	'''
	def __init__(self, params) :
		assert(isinstance(params,ShapeParamsRanges))
		
		self.params = params
		
	def generate(self) :
		'''
		Generate a new shape with respect to the ShapeParamsRanges information provided.
		Return ShapeParams.
		'''
		newshape_info = ShapeParams(shape_type=self.params.shape_type)
		
		#choose scale :
		scale = np.random.random_sample()*(self.params.shape_scale_max-0.25)+0.25
		newshape_info.shape_scale = scale
		
		#choose pose :
		newshape_info.shape_pose = np.concatenate( [ self.randomPose(), self.randomOrientation() ], axis=0).reshape((1,6))

		#choose color :
		nbrcolors = len( self.params.shape_color_list )
		newshape_info.shape_color = self.params.shape_color_list[ np.random.randint( nbrcolors) ]
		
		return newshape_info
	
	def randomPose(self) :
		interval = self.params.shape_pose_max-self.params.shape_pose_min
		return interval[0,:3]*np.random.random_sample((1,3))+self.params.shape_pose_min[:3]
	
	def randomOrientation(self) :
		interval = self.params.shape_pose_max-self.params.shape_pose_min
		return interval[0,3:]*np.random.random_sample((1,3))+self.params.shape_pose_min[3:]

	def changePose(self,shape_param) :
		shape_param.shape_pose[0,:3] = self.randomPose()
		return shape_param

	def changeOrientation(self,shape_param) :
		shape_param.shape_pose[0,3:] = self.randomOrientation()
		return shape_param
			

class Configuration :
	def __init__(self, shape_list=[],factory_list=[]) :
		self.shape_list = copy.deepcopy(shape_list)
		self.factory_list = copy.deepcopy(factory_list)
		
	def append(self, newshape_info,factory=None) :
		assert(isinstance(newshape_info,ShapeParams))
		
		self.shape_list.append( newshape_info)
		self.factory_list.append(factory)
	
	def getShapeList(self) :
		return self.shape_list	

	def changePose(self) :
		for shape,factory in zip(self.shape_list,self.factory_list) :
			shape = factory.chanagePose(shape)

	def changeOrientatoin(self) :
		for shape,factory in zip(self.shape_list,self.factory_list) :
			shape = factory.chanageOrientation(shape)


class OccParamsRanges :
	def __init__(self, occ_min=0, occ_max=2 ) :
		self.occ_min = occ_min
		self.occ_max = occ_max
		
				
class ConfigurationFactory :
	'''
	Factory of Configuration...
	In : list of tuples : ( ShapeFactory , OccParamsRanges )
	Out : Configuration
	'''
	def __init__(self, info=[], port=11311 ) :
		self.spawned = False
		self.info = info
		self.port = port
		
		self.env = os.environ
		self.env["ROS_MASTER_URI"] = 'http://localhost:'+str(self.port)
		self.env["GAZEBO_MASTER_URI"]='http://localhost:'+str(self.port+40)
	
		self.currentConfiguration = None
	
	def init_roscore(self):
		self.launcher_roscore = subprocess.Popen(['roscore -p '+str(self.port)+' '],shell=True,env=self.env)
		time.sleep(1)
	
	def init_gazebo(self) :
		command = ('roslaunch -p '+str(self.port)+' GazeboDomainRandom empty_world.launch')
		self.launcher_gazebo = subprocess.Popen( command, shell=True, env=self.env)
		rospy.loginfo("GAZEBO Domain Random : ENVIRONMENT "+str(self.port)+" : launching...")
		
	def init_node(self) :
		rospy.init_node('GazeboDomainRandom_node', anonymous=False)#, xmlrpc_port=self.port)#,tcpros_port=self.port)
		rospy.on_shutdown(self.close)
	
	def init_subpub(self) :
		self.pub_modelstate = rospy.Publisher( '/gazebo/set_model_state', ModelState, queue_size=10)

	def close(self) :
		#TODO :
		# end services...
		self.launcher_gazebo.kill()
		self.launcher_roscore.kill()
		
		command = 'pkill roslaunch'
		subprocess.Popen( command.split())
		
		rospy.loginfo("GAZEBO Domain Random : ENVIRONMENT "+str(self.port)+" : CLOSED.")
		
						
	def init(self) :
		'''
		Generate the Gazebo environment (launch roscore, gzserver and so on...)
		'''
		self.init_roscore()
		self.init_gazebo()
		self.init_node()	
		self.init_subpub()
		
		
		
	def generate(self) :
		newconfig = Configuration(shape_list=[])
		
		for (shapefactory,occ) in self.info :
			nbrocc = np.random.randint( low=occ.occ_min, high=occ.occ_max)
			
			for i in range(nbrocc) :
				newconfig.append( shapefactory.generate(), shapefactory )
			
		return newconfig
	
	def _erase(self) :
		counters = dict()
		for el in self.currentConfiguration.shape_list :
			stype = el.getType()
			
			if stype in counters :
				counters[stype] += 1
			else :
				counters[stype] = 0

			#rospy.loginfo('ERASE : {}'.format(stype) )
			command = "rosservice call gazebo/delete_model \"{model_name: "+stype+str(counters[stype])+"}\""
			rospy.loginfo('COMMAND : ERASE : {}'.format(command) )
			subprocess.Popen(command, shell=True, env=self.env)		
			
	def spawn(self, config) :
		assert(isinstance(config,Configuration))
		
		if self.spawned :
			self._erase()

		self.currentConfiguration = copy.deepcopy(config)

		shape_it = dict()
		
		for elem in config.shape_list :
			eltype = elem.shape_type
			elscale = elem.shape_scale
			elpose = elem.shape_pose.tolist()[0]
			
			elposeX = elpose[0]
			elposeY = elpose[1]
			elposeZ = elpose[2]
			elposeRoll = elpose[3]
			elposePitch = elpose[4]
			elposeYaw = elpose[5]
			elcolor = elem.shape_color
			
			if eltype in shape_it :
				shape_it[eltype] += 1
			else :
				shape_it[eltype] = 0
				
			elname = eltype+str(shape_it[eltype])
			elem.setName(elname)
			
			command = 'roslaunch -p '+str(self.port)+' GazeboDomainRandom {}.spawn.launch name:={} color:={} scale:={} X:={} Y:={} Z:={}'.format( eltype, elname, elcolor, elscale, elposeX, elposeY, elposeZ)
			rospy.loginfo('CONFIGURATION SPAWN : COMMAND : {}'.format(command) )
			subprocess.Popen( command, shell=True, env=self.env)
			time.sleep(1.0)
	
		self.spawned = True
		time.sleep(2.0)	
	
	def changeOrientation(self) :
		self.currentConfiguration.changeOrientation()

	def changePose(self) :
		self.currentConfiguration.changePose()

		
	def changeSpawned(self, config=None) :
		if config is None :
			config = self.currentConfiguration

		for elem in config.shape_list :
			modelstate = ModelState()

			elpose = elem.shape_pose.tolist()[0]
			
			pose = Point()
			quat = Quaternion()

			pose.x = elpose[0]
			pose.y = elpose[1]
			pose.z = elpose[2]
			modelstate.pose.position = pose

			quaternion = tf.transformations.quaternion_from_euler(
				elpose[3],
				elpose[4],
				elpose[5]
				)
			modelstate.pose.orientation = quaternion

			elname = elem.getName()
			modelstate.model_name = elname

			self.pub.modelstate.publish(modelstate)

			rospy.loginfo('CONFIGURATION SPAWNED : making changes... :\n {}'.format(modelstate) )
			time.sleep(0.1)


	def getCurrentConfiguration(self):
		return self.currentConfiguration	
		


class Camera :
	def __init__(self, fovy=75, altitude=1.0, port=11311, env=None) :
		self.fovy = fovy
		self.altitude = altitude
		self.spawned = False
		self.port = port
		self.env = env
		if self.env is None :
			self.env = os.environ
		
		self.K = np.zeros((3,3))
		self.K[0][0] = self.fovy
		self.K[1][1] = self.fovy
		self.K[2][2] = 1.0
		self.K = np.matrix(self.K)
		self.P = np.matrix( np.concatenate( [self.K, np.zeros((3,1)) ], axis=1) )
		
		self.camerainfo = None
		self.listener_camera_info = rospy.Subscriber( '/CAMERA/camera_info', CameraInfo, self.callbackCAMERAINFO )
		self.image = None
		self.bridge = CvBridge()
		self.listener_image = rospy.Subscriber( '/CAMERA/image_raw', Image, self.callbackIMAGE )
		
		
	def ros2np(self,img) :
		return self.bridge.imgmsg_to_cv2(img, "bgr8")

	
	def callbackIMAGE(self, image) :
		self.image = self.ros2np(image)
			
	def callbackCAMERAINFO(self, camerainfo) :
		self.camerainfo = camerainfo
		if self.camerainfo is not None :
			self.K = np.matrix( np.reshape( self.camerainfo.K, (3,3)) )
			self.P = np.matrix( np.reshape( self.camerainfo.P, (3,4)) )
	
	def getImage(self) :
		if self.image is not None :
			return self.image.copy()
		else :
			return None
				
	def setFovy(self, fovy) :
		self.fovy = fovy
		self.spawn()
		
	def setAltitude(self, altitude) :
		self.altitude = altitude
		self.spawn()
		
	def spawn(self,port=None,env=None) :
		if port is not None :
			self.port = port
		if env is not None :
			self.env = env
			
		
		if self.spawned :
			self._erase()
			
		command = 'roslaunch -p '+str(self.port)+' GazeboDomainRandom camera.spawn.launch fovy:={} Z:={}'.format( self.fovy, self.altitude)
		subprocess.Popen( command, shell=True, env=self.env)
		time.sleep(1.0)
		
		self.spawned = True
		rospy.loginfo('CAMERA SPAWNED : fovy={} // Z={}'.format(self.fovy, self.altitude) )
		
		
	def _erase(self) :
		command = "rosservice call gazebo/delete_model \"{model_name: CAMERA}\""
		subprocess.Popen(command, shell=True, env=self.env)		
		time.sleep(1.0)
	
	def project(self, x) :
		#homogenization :
		if x.shape[0] == 3 :
			x = np.concatenate( [x, np.ones((1,1)) ], axis=0 )
		x = np.matrix(x)
		
		#P = self.K * np.matrix( np.concatenate( [ np.ones((3,3)), np.reshape( [ -self.altitude, 0.0, 0.0 ], newshape=(3,1) )], axis=1) )
		Rt = np.matrix( [ [1, 0, 0, 0], [0, 1, 0, self.altitude+0.1], [0, 0, 1, 0] ] )
		P = self.K * Rt
		
		'''
		Rt = np.matrix( [ [1, 0, 0, 0], [0, 1, 0, self.altitude+0.1], [0, 0, 1, 0], [0, 0, 0, 1.0] ] )
		P = self.P * Rt
		'''
		projx = P * x
		#dehomogenization ;
		for i in range(3) :
			projx[i] /= projx[2]+1e-6
		
		return projx
			
