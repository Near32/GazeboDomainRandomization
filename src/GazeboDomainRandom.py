import rospy
import subprocess
import time
import numpy as np



class ShapeParams :
	def __init__(self, shape_type=None, shape_scale=1.0, shape_pose=None, shape_color=None):
		self.shape_type = shape_type
		self.shape_scale = shape_scale
		self.shape_pose = shape_pose
		self.shape_color = shape_color
		
class ShapeParamsRanges :
	def__init__(self, shape_type, shape_scale_max=1.0, shape_pose_min=None, shape_pose_max=None, shape_color_list=['White']) :
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
	def __init__(self, paramsranges) :
		assert(paramsranges, isinstance(paramsranges,ShapeParamsRanges))
		
		self.paramsranges = paramsranges
		
	def generate(self) :
		'''
		Generate a new shape with respect to the ShapeParamsRanges information provided.
		Return ShapeParams.
		'''
		newshape_info = ShapeParams()
		#TODO : generate params for the new shape
		
		return shape_info 
		
class Configuration :
	def __init__(self, shape_list=[]) :
		self.shape_list = shape_list
		
	def append(self, newshape_info) :
		assert(newshape_info,isinstance(newshape_info,ShapeParams))
		
		self.shape_list.append( newshape_info)
		

class OccParamsRanges :
	def __init__(self, occ_min=0, occ_max=2 ) :
		self.occ_min = occ_min
		self.occ_max = occ_max
		
				
class ConfigurationFactory :
	'''
	Factory of Configuration...
	In : list of tuples : ( ShapeFactory , OccurencesParamsRanges )
	Out : Configuration
	'''
	def __init__(self, info=[] ) :
		self.info = info
		
	def generate(self) :
		newconfig = Configuration()
		#TODO : generate parms for the new config
		
		return newconfig
		
	def spawn(self, config) :
		assert(config,isinstance(config,Configuration))
		
		for elem in config.shape_list :
			eltype = elem.shape_type
			elscale = elem.shape_scale
			elpose = elem.shape_pose
			elposeX = elpose[0]
			elposeY = elpose[1]
			elposeZ = elpose[2]
			elposeRoll = elpose[3]
			elposePitch = elpose[4]
			elposeYaw = elpose[5]
			elcolor = elem.shape_color
			
			#TODO : subprocess roslaunch spawner ...
			
		
			
		
