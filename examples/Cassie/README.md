# PD controller
From the base `utility/` directory:
1. Run GUI 
```
./bazel-bin/examples/Cassie/pd_controller_gui
```
2. Run lcm-spy 
```
./bazel-bin/lcmtypes/cassie-lcm-spy
```
3. Run PD controller
```
./bazel-bin/examples/Cassie/run_pd_controller
```
4. (To stream fake state information)
```
./bazel-bin/examples/Cassie/run_state_pub_test
```