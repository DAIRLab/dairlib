#for i in 00 01 02 03 04 06 07 08 09 10 16 17 18 19 20 21 22 23
#do
#	echo
#	echo $i
#	bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance /home/yuming/Desktop/temp/20210708_hardware_rom_controller_log/07_08_21/lcmlog-$i ROM_WALKING True
#done

################################################################################

eval_dir=../dairlib_data/goldilocks_models/hardware_cost_eval/
rm -rf $eval_dir

bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance /home/yuming/Desktop/temp/0507/lcmlog-02 ROM_WALKING true $eval_dir 1
bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance /home/yuming/Desktop/temp/0507/lcmlog-03 ROM_WALKING true $eval_dir 20
bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance /home/yuming/Desktop/temp/0507/lcmlog-04 ROM_WALKING true $eval_dir 40
bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance /home/yuming/Desktop/temp/0507/lcmlog-05 ROM_WALKING true $eval_dir 60
