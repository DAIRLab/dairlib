#for i in 00 01 02 03 04 06 07 08 09 10 16 17 18 19 20 21 22 23
#do
#	echo
#	echo $i
#	bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance /home/yuming/Desktop/temp/20210708_hardware_rom_controller_log/07_08_21/lcmlog-$i ROM_WALKING True
#done

################################################################################

log_dir0=/home/yuming/Desktop/data_on_desktop/20220509_hardware_rom_and_cost_eval/05_09_22/
log_dir1=/home/yuming/Desktop/data_on_desktop/20220512_hardware_rom/yuming_rom_walking/
log_dir2=/home/yuming/Desktop/data_on_desktop/20220517_hardware_rom/yuming_rom_walking/
log_dir3=/home/yuming/Desktop/data_on_desktop/20220518_hardware_rom/yuming_rom_walking/
log_dir4=/home/yuming/Desktop/data_on_desktop/20220519_hardware_rom/yuming_rom_walking/

output_dir=../dairlib_data/goldilocks_models/hardware_cost_eval/
rm -rf $output_dir

bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance --file_path="$log_dir0"lcmlog-05 --controller_channel=ROM_WALKING --hardware --eval_dir=$output_dir --rom_iter_idx=1
bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance --file_path="$log_dir0"lcmlog-06 --controller_channel=ROM_WALKING --hardware --eval_dir=$output_dir --rom_iter_idx=1
bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance --file_path="$log_dir0"lcmlog-13 --controller_channel=ROM_WALKING --hardware --eval_dir=$output_dir --rom_iter_idx=100
bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance --file_path="$log_dir0"lcmlog-15 --controller_channel=ROM_WALKING --hardware --eval_dir=$output_dir --rom_iter_idx=60

bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance --file_path="$log_dir1"lcmlog-01 --controller_channel=ROM_WALKING --hardware --eval_dir=$output_dir --rom_iter_idx=1

bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance --file_path="$log_dir2"lcmlog-03 --controller_channel=ROM_WALKING --hardware --eval_dir=$output_dir --rom_iter_idx=1
bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance --file_path="$log_dir2"lcmlog-04 --controller_channel=ROM_WALKING --hardware --eval_dir=$output_dir --rom_iter_idx=1
bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance --file_path="$log_dir2"lcmlog-05 --controller_channel=ROM_WALKING --hardware --eval_dir=$output_dir --rom_iter_idx=1
bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance --file_path="$log_dir2"lcmlog-08 --controller_channel=ROM_WALKING --hardware --eval_dir=$output_dir --rom_iter_idx=100

bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance --file_path="$log_dir3"lcmlog-01 --controller_channel=ROM_WALKING --hardware --eval_dir=$output_dir --rom_iter_idx=60
bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance --file_path="$log_dir3"lcmlog-02 --controller_channel=ROM_WALKING --hardware --eval_dir=$output_dir --rom_iter_idx=60
bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance --file_path="$log_dir3"lcmlog-03 --controller_channel=ROM_WALKING --hardware --eval_dir=$output_dir --rom_iter_idx=60

bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance --file_path="$log_dir4"lcmlog-01 --controller_channel=ROM_WALKING --hardware --eval_dir=$output_dir --rom_iter_idx=1
bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance --file_path="$log_dir4"lcmlog-02 --controller_channel=ROM_WALKING --hardware --eval_dir=$output_dir --rom_iter_idx=100
bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance --file_path="$log_dir4"lcmlog-03 --controller_channel=ROM_WALKING --hardware --eval_dir=$output_dir --rom_iter_idx=100

# Note we you plot the cost, slice at pelvis height = 0.935
