
if [ "$HOSTNAME" = "dair-cassie" ]; then
  bazel build --host_force_python=PY3 examples/goldilocks_models:rom_control_real_robot 
else
  bazel build examples/goldilocks_models:rom_control_real_robot 
  bazel build director/...  
fi


