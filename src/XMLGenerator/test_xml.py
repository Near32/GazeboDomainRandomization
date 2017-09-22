import xml.etree.ElementTree

# Open original file
et = xml.etree.ElementTree.parse('file.xml')

# Append new tag: <a x='1' y='abc'>body text</a>
new_tag = xml.etree.ElementTree.SubElement(et.getroot(), 'a')
new_tag.text = 'body text'
new_tag.attrib['x'] = str(1) # must be str; cannot be an int
new_tag.attrib['y'] = 'abc'

# Write back to file
et.write('file_new.xml')