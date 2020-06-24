import xml.etree.ElementTree as ET




map_loc = '/home/shb/catkin_ws/src/anki_description/world/sample2.world'

tree = ET.parse(map_loc)
root = tree.getroot()

print(root[0][9])
for child in root[0][9]:
    print(child.tag, child.attrib)