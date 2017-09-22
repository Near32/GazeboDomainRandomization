import xml.etree.ElementTree as ET
import copy
import os

class XMLObjectTags :
	def __init__(self,name='name',pose=[0,0,0,0]):
		self.root = ET.Element('object')
		self.subs = dict()
		self.subs['name'] = ET.SubElement(self.root,'name')
		self.subs['name'].text = name
		self.subs['pose'] = ET.SubElement(self.root,'pose')
		self.subs['pose'].text = 'Front'
		self.subs['truncated'] = ET.SubElement(self.root,'truncated')
		self.subs['truncated'].text = str(0)
		self.subs['difficult'] = ET.SubElement(self.root,'difficult')
		self.subs['difficult'].text = str(0)
		self.subs['bndbox'] = dict()
		self.subs['bndbox']['root'] = ET.SubElement(self.root,'bndbox')
		self.subs['bndbox']['xmin'] = ET.SubElement(self.subs['bndbox']['root'],'xmin')
		self.subs['bndbox']['xmin'].text = str(pose[0])
		self.subs['bndbox']['xmax'] = ET.SubElement(self.subs['bndbox']['root'],'xmax')
		self.subs['bndbox']['xmax'].text = str(pose[1])
		self.subs['bndbox']['ymin'] = ET.SubElement(self.subs['bndbox']['root'],'ymin')
		self.subs['bndbox']['ymin'].text = str(pose[2])
		self.subs['bndbox']['ymax'] = ET.SubElement(self.subs['bndbox']['root'],'ymax')
		self.subs['bndbox']['ymax'].text = str(pose[3])
		
	def getRoot(self) :
		return self.root


class XMLGenerator :
	def __init__(self,path_out='./') :
		self.et_base = ET.parse('base_annotation.xml')
		self.path_out = path_out+'annotations/'
		if not os.path.exists(self.path_out):
			os.makedirs(self.path_out)

	def generate(self,object_tags,filename='filename') :
		et = copy.deepcopy(self.et_base)
		root = et.getroot()
		root.find('filename').text = filename

		for obj in object_tags :
			root.append(obj.getRoot())

		et.write('{}{}.xml'.format(self.path_out,filename) )



