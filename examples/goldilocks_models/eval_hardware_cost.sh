#for i in 00 01 02 03 04 06 07 08 09 10 16 17 18 19 20 21 22 23
#do
#	echo
#	echo $i
#	bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance /home/yuming/Desktop/temp/20210708_hardware_rom_controller_log/07_08_21/lcmlog-$i ROM_WALKING True
#done

################################################################################

log_dir=/home/yuming/Desktop/data_on_desktop/20220509_hardware_rom/05_09_22/
output_dir=../dairlib_data/goldilocks_models/hardware_cost_eval/
rm -rf $output_dir

bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance "$log_dir"lcmlog-00 ROM_WALKING true $output_dir 1
bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance "$log_dir"lcmlog-01 ROM_WALKING true $output_dir 1
bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance "$log_dir"lcmlog-02 ROM_WALKING true $output_dir 1
bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance "$log_dir"lcmlog-03 ROM_WALKING true $output_dir 100
