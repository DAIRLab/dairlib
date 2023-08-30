'''
    Python script to be run with blender to convert meshes to gltf
    from this directory, run
        blender --python convert_obj_to_gltf.py
'''

import bpy
import glob

fnames = glob.glob('agility/*.obj')

objs = [ob for ob in bpy.context.scene.objects if ob.type in ('CAMERA', 'MESH')]
bpy.ops.object.delete({"selected_objects": objs})

for fname in fnames:
    savename = fname.replace('obj', 'glb')
    bpy.ops.import_scene.obj(filepath=fname)
    bpy.ops.export_scene.gltf(filepath=savename,
                              export_format='GLTF_EMBEDDED')

    objs = [ob for ob in bpy.context.scene.objects if ob.type in ('MESH')]
    bpy.ops.object.delete({"selected_objects": objs})