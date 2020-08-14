echo If you ssh, run this sciprt in the background to prevent the program from being terminated due to disconnection

echo Please enter robot index
read robot
echo Please enter reduced order model index
read model

echo robot_option = $robot, rom_option = $model

echo We will delete the folder dairlib_data/goldilocks_models/find_models/robot_$robot/
while true; do
    read -p "Proceed? (y/n)" yn
    case $yn in
        [Yy]* ) make install; break;;
        [Nn]* ) exit;;
        * ) echo "Please answer yes or no.";;
    esac
done

rm -rf ../dairlib_data/goldilocks_models/find_models/robot_$robot/
mkdir -p ../dairlib_data/goldilocks_models/find_models/robot_$robot/nominal_no_constraint_traj/

./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=13 --N_sample_gi=3 --fix_node_number=true --iter_start=0 --max_outer_iter=0 --snopt_scaling=false | tee -a ../dairlib_data/goldilocks_models/find_models/robot_1/terminal_log

./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=13 --N_sample_gi=3 --fix_node_number=true --iter_start=0 --max_outer_iter=0 --snopt_scaling=true --start_current_iter_as_rerun=true | tee -a ../dairlib_data/goldilocks_models/find_models/robot_1/terminal_log

cp ../dairlib_data/goldilocks_models/find_models/robot_1/0_* ../dairlib_data/goldilocks_models/find_models/robot_1/nominal_no_constraint_traj/

./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=13 --N_sample_gi=3 --fix_node_number=true --iter_start=0 --max_outer_iter=1 --snopt_scaling=false --start_current_iter_as_rerun=true | tee -a ../dairlib_data/goldilocks_models/find_models/robot_1/terminal_log

./bazel-bin/examples/goldilocks_models/find_goldilocks_models --rom_option=$model --robot_option=$robot --N_sample_sl=13 --N_sample_gi=3 --fix_node_number=true --iter_start=1 --max_outer_iter=40 --snopt_scaling=true --start_current_iter_as_rerun=true | tee -a ../dairlib_data/goldilocks_models/find_models/robot_1/terminal_log

