# PD controller
From the base `utility/` directory:
1. Run GUI 
```
./bazel-bin/applications/cassie/pd_controller_gui
```
2. Run lcm-spy 
```
./bazel-bin/applications/cassie/lcmtypes/cassie-lcm-spy
```
3. Run PD controller
```
./bazel-bin/applications/cassie/run_pd_controller
```
4. (To stream fake state information)
```
./bazel-bin/applications/cassie/run_state_pub_test
```