'''
    Python script to be run with blender to add normals to obj files
    from this directory, run
        blender --python add_obj_normals.py
'''

import bpy
import glob

fnames = glob.glob('*.obj') + glob.glob('agility/*.obj')

objs = [ob for ob in bpy.context.scene.objects if ob.type in ('CAMERA', 'MESH', 'LIGHT')]
bpy.ops.object.delete({"selected_objects": objs})

for fname in fnames:
    bpy.ops.import_scene.obj(filepath=fname)
    bpy.ops.export_scene.obj(filepath=fname, use_normals=True, use_materials=False)

    objs = [ob for ob in bpy.context.scene.objects if ob.type in ('MESH')]
    bpy.ops.object.delete({"selected_objects": objs})