for i in 00 01 02 03 04 06 07 08 09 10
do
	echo
	echo $i
	bazel-bin/examples/goldilocks_models/eval_single_sim_performance /home/yuming/Desktop/20210708_hardware_rom_controller_log/07_08_21/lcmlog-$i ROM_WALKING True
done
