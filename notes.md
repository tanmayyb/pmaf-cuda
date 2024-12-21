
## Piping output

Piping
```
source /home/dev/ws_pmaf_cuda/pmaf-cuda/stubs/devel/setup.bash && source /home/dev/ws_pmaf_cuda/pmaf-cuda/pmaf-cuda/devel/setup.bash && roslaunch bimanual_planning_ros planning_moveit_dual_arms.launch > perf_$(date +"%F_%H:%M:%S").csv
```


## Profilling

planning_node.launch:

```
  <group ns="$(arg robot_id)/dual_panda_costp_controller">
    ...
    <arg name="time_now" default="$(eval eval ('_' + '_import_' + '_(\'datetime\')').datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))"/>
    ...

    # vanilla
    <node pkg="bimanual_planning_ros" type="panda_bimanual_control_node" name="bimanual_planning_node" output="screen"/>

    
    # callgrind support
    <node pkg="bimanual_planning_ros" type="panda_bimanual_control_node" name="bimanual_planning_node" output="screen" launch-prefix="valgrind --tool=callgrind --callgrind-out-file='callgrind.bimanual_planning_node.$(arg time_now)'"/>

    # vallgrind (RTFunctionTrace)
    <node pkg="bimanual_planning_ros" type="panda_bimanual_control_node" name="bimanual_planning_node" output="screen" launch-prefix="valgrind"/>


    # callgrind support - jumps
    <node pkg="bimanual_planning_ros" type="panda_bimanual_control_node" name="bimanual_planning_node" output="screen" launch-prefix="valgrind --tool=callgrind --dump-instr=yes --collect-jumps=yes --callgrind-out-file='callgrind.bimanual_planning_node.$(arg time_now)'"/>


  </group>

```

sampling:
```
sudo perf record -a --call-graph fp -- sleep 40 && sudo chmod +r perf.data
```


## Run

```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT
export QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT
cd $COPPELIASIM_ROOT && ./coppeliaSim.sh -h $PMAF_CUDA_ROOT/pmaf-cuda/src/bimanual_planning_ros/vrep_scenes/dual_arms.ttt

source /home/dev/ws_pmaf_cuda/pmaf-cuda/stubs/devel/setup.bash && source /home/dev/ws_pmaf_cuda/pmaf-cuda/pmaf-cuda/devel/setup.bash && roslaunch bimanual_planning_ros vrep_interface_dual_arms.launch task_sequence:=dual_arms_static3

source /home/dev/ws_pmaf_cuda/pmaf-cuda/stubs/devel/setup.bash && source /home/dev/ws_pmaf_cuda/pmaf-cuda/pmaf-cuda/devel/setup.bash && roslaunch bimanual_planning_ros planning_moveit_dual_arms.launch
```


## Building
```
cd $PMAF_CUDA_ROOT/pmaf-cuda
catkin clean --yes && catkin build
```



## Errors

kcachegrind error:
```
==145819== Warning: noted but unhandled ioctl 0x30000001 with no size/direction hints.
==145819==    This could cause spurious value errors to appear.
==145819==    See README_MISSING_SYSCALL_OR_IOCTL for guidance on writing a proper wrapper.
==145819== Warning: noted but unhandled ioctl 0x4b with no size/direction hints.
==145819==    This could cause spurious value errors to appear.
==145819==    See README_MISSING_SYSCALL_OR_IOCTL for guidance on writing a proper wrapper.
==145819== Warning: noted but unhandled ioctl 0x27 with no size/direction hints.
==145819==    This could cause spurious value errors to appear.
==145819==    See README_MISSING_SYSCALL_OR_IOCTL for guidance on writing a proper wrapper.
```



## Resources

- KCachegrind (Callgrind) explanation [1](https://www.youtube.com/watch?v=h-0HpCblt3A), [2](https://www.youtube.com/watch?v=iH-hDOuQfcY) 

- Flamegraph explanation [link](https://stackoverflow.com/questions/27842281/sunknown-events-in-nodejs-v8-flamegraph-using-perf-events/27867426#27867426)

- C profiler/tracer w/ timeline [link](https://stackoverflow.com/questions/77214379/c-profiler-tracer-with-timeline-view)

