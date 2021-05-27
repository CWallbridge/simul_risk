def downscaler(string,factor=0.01):
    #use blender built in downscaling with ctrl+a and then apply scale!
    #gazebo importer does not support scale. I tried a lot. 
    li = string.split()
    ili = []
    for i in range(len(li)):
        element = float(li[i])
        if i == 3 or i == 7 or i == 11:
            element=element*factor
        ili.append(str(element))
    return ' '.join(ili)
def downscaler_all(string,factor=0.01):
    li = string.split()
    ili = []
    for i in range(len(li)):
        element = float(li[i])
        if (i+1)%4 == 0 and (i+1)%16 != 0:
            element = element*factor
        ili.append(str(element))
    return ' '.join(ili)

from xml.dom import minidom
import xml.etree.ElementTree as ET
fix_animation = True
downscale = False


default_pose = '<root><translate sid="location">0 0 0</translate><rotate sid="rotationZ">0 0 1 0</rotate><rotate sid="rotationY">0 1 0 0</rotate><rotate sid="rotationX">1 0 0 0</rotate><scale sid="scale">1 1 1</scale></root>'#in future change me to file.
default_pose = minidom.parseString(default_pose).getElementsByTagName("root")[0]
default_pose1 = '<root><translate sid="location">0 0 0</translate><rotate sid="rotationZ">0 0 1 0</rotate><rotate sid="rotationY">0 1 0 0</rotate><rotate sid="rotationX">1 0 0 0</rotate><scale sid="scale">0.01 0.01 0.01</scale></root>'#in future change me to file.
default_pose1 = minidom.parseString(default_pose1).getElementsByTagName("root")[0]

# parse an xml file by name
mydoc = minidom.parse('temp.dae')

item = mydoc.getElementsByTagName('COLLADA')[0]
#Fix library_animation (remove the topmost container)
if fix_animation:
    library_animation = item.getElementsByTagName('library_animations')[0]
    animation_node = library_animation.getElementsByTagName('animation')[0]
    animation_name = animation_node.attributes['name'].value
    for node in animation_node.getElementsByTagName('animation'):
        library_animation.appendChild(node)
    library_animation.removeChild(animation_node)

#fix library_visual_scene (re-arrange nodes & add decomposed translation/rotation)
visual_scene = item.getElementsByTagName('visual_scene')[0]
visual_scene_root = visual_scene.getElementsByTagName('node')[0]
#replace the matrix representation with decomposed
visual_scene_root.removeChild(visual_scene_root.getElementsByTagName('matrix')[0])
for node in default_pose.childNodes:
    visual_scene_root.appendChild(node.cloneNode(True))
    
#remove the human node from main tree & add it as a standalone.
#More generally get a child that's type Node and is 1 level deep.
human_nodes = visual_scene_root.getElementsByTagName('node')#getElementById('human')
for human_node in human_nodes:
    if human_node.parentNode == visual_scene_root and human_node.attributes["type"].value == "NODE":#human_node.attributes['id'].value=="human":
        print("found child succesfully")
        break
visual_scene_root.removeChild(human_node)

#should probably make this into a seperate variable at some point
human_node.removeChild(human_node.getElementsByTagName('matrix')[0])

for node in default_pose.childNodes:        
    human_node.appendChild(node.cloneNode(True))

visual_scene.appendChild(human_node)

#fix colouring. This means converting from triangle representation to polylist.
#Don't know why blender does not export to polylist or whether this can be changed in blender \_(-_-)/-
triangles = item.getElementsByTagName("triangles")
for triangle in triangles:
    #get count attribute
    nr_triangles = int(triangle.attributes["count"].value)
    output = ""
    for i in range(nr_triangles):
        output += "3 "
    #change type to polylist
    print("IRAN")
    #triangle.attributes["vcount"].value=3
    triangle.tagName='polylist'
    vcount = mydoc.createElement("vcount")
    vcount_text = mydoc.createTextNode(output)
    vcount.appendChild(vcount_text)

    triangle.appendChild(vcount)
    
    #print(triangle.attributes["vcount"].value)



#Fix naming
xml = mydoc.toprettyxml()
print("Renaming string!")
print("ANIMATION NAME: " +animation_name)
for channel in item.getElementsByTagName("channel"):
    channel_target = channel.attributes['target'].value
    channel_target = channel_target.split("/",)[0]
    print(channel_target)
    print("Renaming to: ")
    print(channel_target[len(animation_name)+1:])
    xml = xml.replace(channel_target,channel_target[len(animation_name)+1:])#please work

#write to file
file = open("corrected.dae","w")
file.write(xml)
file.close()
#one specific item attribute
#print(items[0].attributes['name'].value)
#removeChild() & appendChild()

