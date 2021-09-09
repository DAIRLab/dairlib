import os     

directory = "../dairlib_data/goldilocks_models/planning/robot_1/"
data_path = directory + "data/"

if not os.path.exists(data_path):
    raise ValueError("ERROR: the data folder doesn't exist (%s)" % data_path)

new_data_path = ""
for i in range(1000):
    temp_path = directory + "data_" + str(i) + "/"
    if os.path.exists(temp_path):
        continue
    else:
        new_data_path = temp_path
        break

if len(new_data_path) == 0:
    raise ValueError("ERROR: Didn't get a new folder name. Maybe try to search over bigger index range.")

print("We will rename \n  %s \nto \n  %s \n" % (data_path, new_data_path))

cmd = "mv " + data_path + " " + new_data_path
print("Running command: " + cmd)

os.system(cmd)

print("done\n")
