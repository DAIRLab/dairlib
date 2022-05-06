# for i in 00 01 02 03 04 06 07 08 09 10
for i in 00 01 02 03 04 06 07 08 09 10 16 17 18 19 20 21 22 23
do
	echo
	echo $i
	bazel-bin/examples/goldilocks_models/eval_single_closedloop_performance /home/yuming/Desktop/temp/20210708_hardware_rom_controller_log/07_08_21/lcmlog-$i ROM_WALKING True
done
