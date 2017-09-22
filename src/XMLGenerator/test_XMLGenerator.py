from XMLGenerator import XMLObjectTags
from XMLGenerator import XMLGenerator

def test_xmlobj() :
	a = XMLObjectTags()

def test_xmlgen() :
	listobj = []
	for i in range(2) :
		listobj.append( XMLObjectTags() )

	genXML = XMLGenerator()

	genXML.generate(object_tags=listobj, filename='testgen')

#test_xmlobj()
test_xmlgen()