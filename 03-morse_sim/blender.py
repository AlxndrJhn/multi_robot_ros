import bpy
import mathutils  # Imports Blender vector math utilities
import math  # Imports the standard Python math library
import sys,os
#sys.path.append(os.path.dirname(__file__))
sys.path.append('/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/01-ros_ws/src/agent/python_code/mapper_pycharm/')
from util.svg_to_map import svg_to_map

# get parameters
argv = sys.argv
argv = argv[argv.index("--") + 1:]  # get all args after "--"
#print(argv)  # --> ['example', 'args', '123']
if len(argv) == 0:
    image_name = "/home/alex/Dropbox/UFMG/04-2-2016/JornalPaper/02-code/mapper_pycharm/maps/01-simple_real.svg"
else:
    image_name = argv[0]
print(image_name)

res = 0.1
E, Obs, start, goal = svg_to_map('',fullpath=image_name)
w, h = E.bounds[2], E.bounds[3]

# clear scene
for item in bpy.data.objects:
    bpy.data.objects[item.name].select = True
bpy.ops.object.delete()

# Light
scene = bpy.context.scene
lamp_data = bpy.data.lamps.new(name="New Lamp", type='SUN')
# Create new object with our lamp datablock
lamp_object = bpy.data.objects.new(name="New Lamp", object_data=lamp_data)
# Link lamp object to the scene so it'll appear in this scene
scene.objects.link(lamp_object)
# Place lamp to a specified location
lamp_object.location = (w*res/2., h*res/2., 30.0)
# And finally select it make active
lamp_object.select = True
scene.objects.active = lamp_object

# create plane
#verts = [(0, 0, 0), (0, h*res, 0), (w*res, h*res, 0), (w*res, 0, 0)]
#faces = [(0, 1, 2, 3)]
# Define mesh and object variables
#mymesh = bpy.data.meshes.new("Plane")
#myobject = bpy.data.objects.new("Plane", mymesh)
# Set location and scene of object
#myobject.location = (0.0, 0.0, 0.0)
#bpy.context.scene.objects.link(myobject)
# Create mesh
#mymesh.from_pydata(verts, [], faces)
#mymesh.update(calc_edges=True)

bpy.ops.mesh.primitive_cube_add(location=(w*res/2., h*res/2., -0.005))
bpy.context.active_object.dimensions = (w*res,h*res,.01)

# Create obstacles
red = bpy.data.materials.new('RED')
red.diffuse_color = (1,0,0)
red.diffuse_shader = 'LAMBERT'
red.diffuse_intensity = 1.0
red.specular_color = (1,1,1)
red.specular_shader = 'COOKTORR'
red.specular_intensity = 0.5
red.alpha = 1
red.ambient = 1
for O in Obs:
    bpy.ops.mesh.primitive_cube_add(location=(O.centroid.x*res, O.centroid.y*res, res*10.))
    bpy.context.active_object.dimensions = ((O.bounds[2] - O.bounds[0])*res,(O.bounds[3] - O.bounds[1])*res,res*20.)
    bpy.context.active_object.data.materials.append(red)

bpy.data.scenes["Scene"].render.engine = 'BLENDER_GAME'
bpy.ops.wm.save_mainfile(filepath="/usr/share/morse/data/environments/exp/exp_env.blend")
exit(0)





