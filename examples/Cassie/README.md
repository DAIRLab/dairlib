## PD controller tests
Launch bot-procman-sherrif using the cassie_test.pmd configuration file.
Currently, you must bein the root `dairlib` directory for this to work properly.
```
bot-procman-sherrif examples/Cassie/cassie_test.pmd
```
Note: if you've installed libbot2, using apt, `bot-procman-sherrif` is probably located in `/opt/libbot2/0.0.1.20180130/bin/` or a similar directory.

The following steps will launch a PD controller and simulation:
1. Options->Spawn local deputy (allows launching of processes)
2. Right-click and start `drake-director` and `state-visualizer`. It's a good idea to allow Director to open before launching the visualizer.
3. Right-click and start `pd-controller` and `simulator`