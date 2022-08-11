import os
import sys
from pydrake.common.yaml import yaml_load

def update_urdf(template_path, file_path, param):
  with open(template_path, 'r') as file :
    filedata = file.read()
  
  filedata = update_filedata(filedata, param)
  with open(file_path, 'w') as file:
    file.write(filedata)
  print("Finished updating {}".format(file_path))

def update_filedata(filedata, param):
  filedata = filedata.replace('BALL_RADIUS', str(param['ball_radius']))
  filedata = filedata.replace('FINGER_RADIUS', str(param['finger_radius']))
  
  EE_offset_str = '{} {} {}'.format(param['EE_offset'][0],
                                    param['EE_offset'][1],
                                    param['EE_offset'][2])
  filedata = filedata.replace('EE_OFFSET', EE_offset_str)

  # must replace MODEL_TABE_ORIGIN before TABLE_OFFSET since TABLE_ORIGIN
  # is a substring of MODEL_TABLE_ORIGIN
  filedata = filedata.replace('MODEL_TABLE_ORIGIN', str(param['model_table_offset']-0.05))
  filedata = filedata.replace('TABLE_ORIGIN', str(param['table_offset']-0.05))
  filedata = filedata.replace('<box size=\"0.3 0.3 TABLE_OFFSET\"/>', 
                              '<box size=\"0.3 0.3 {}\"/>'.format(abs(param['table_offset'])))
  
  return filedata

def main():
  urdf_path = "examples/franka_trajectory_following/robot_properties_fingers/urdf"
  parameters_file = "examples/franka_trajectory_following/parameters.yaml"
  param = yaml_load(filename=parameters_file)

  update_urdf("{}/franka_box_template.urdf".format(urdf_path), \
              "{}/franka_box.urdf".format(urdf_path), param)
  update_urdf("{}/sphere_template.urdf".format(urdf_path), \
              "{}/sphere.urdf".format(urdf_path), param)
  update_urdf("{}/trifinger_minimal_collision_2_template.urdf".format(urdf_path), \
              "{}/trifinger_minimal_collision_2.urdf".format(urdf_path), param)

if __name__== "__main__":
  main()
  

