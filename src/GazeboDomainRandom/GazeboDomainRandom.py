import rospy
import subprocess
import time
import os
import numpy as np



class ShapeParams :
	def __init__(self, shape_type=None, shape_scale=1.0, shape_pose=None, shape_color=None):
		self.shape_type = shape_type
		self.shape_scale = shape_scale
		self.shape_pose = shape_pose
		self.shape_color = shape_color
		
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
		interval = self.params.shape_pose_max-self.params.shape_pose_min
		newshape_info.shape_pose = interval*np.random.random_sample((1,6))+self.params.shape_pose_min
		
		#choose color :
		nbrcolors = len( self.params.shape_color_list )
		newshape_info.shape_color = self.params.shape_color_list[ np.random.randint( nbrcolors) ]
		
		return newshape_info
		
class Configuration :
	def __init__(self, shape_list=[]) :
		self.shape_list = shape_list
		
	def append(self, newshape_info) :
		assert(isinstance(newshape_info,ShapeParams))
		
		self.shape_list.append( newshape_info)
		

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
		self.info = info
		self.port = port
		
		self.env = os.environ
		self.env["ROS_MASTER_URI"] = 'http://localhost:'+str(self.port)
		self.env["GAZEBO_MASTER_URI"]='http://localhost:'+str(self.port+40)
	
	
	def init_roscore(self):
		self.launcher_roscore = subprocess.Popen(['roscore -p '+str(self.port)+' '],shell=True,env=self.env)
		time.sleep(2)
	
	def init_gazebo(self) :
		command = ('roslaunch -p '+str(self.port)+' GazeboDomainRandom empty_world.launch')
		self.launcher_gazebo = subprocess.Popen( command, shell=True, env=self.env)
		rospy.loginfo("GAZEBO Domain Random : ENVIRONMENT "+str(self.port)+" : launching...")
		
	def init_node(self) :
		rospy.init_node('GazeboDomainRandom_node', anonymous=False)#, xmlrpc_port=self.port)#,tcpros_port=self.port)
		rospy.on_shutdown(self.close)
	
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
		
		
		
	def generate(self) :
		newconfig = Configuration()
		
		for (shapefactory,occ) in self.info :
			nbrocc = np.random.randint( low=occ.occ_min, high=occ.occ_max)
			
			for i in range(nbrocc) :
				newconfig.append( shapefactory.generate() )
			
		return newconfig
	
	
		
	def spawn(self, config) :
		assert(isinstance(config,Configuration))
		
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
			
			rospy.loginfo(elcolor)
			
			if eltype in shape_it :
				shape_it[eltype] += 1
			else :
				shape_it[eltype] = 0
				
			elname = eltype+str(shape_it[eltype])
			
			command = 'roslaunch -p '+str(self.port)+' GazeboDomainRandom {}.spawn.launch name:={} color:={} scale:={} X:={} Y:={} Z:={}'.format( eltype, elname, elcolor, elscale, elposeX, elposeY, elposeZ)
			subprocess.Popen( command, shell=True, env=self.env)
			time.sleep(1.0)
			
		
			
		
