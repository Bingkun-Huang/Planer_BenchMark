# Planer_BenchMark
Evaluation metrics for different planners

`roslaunch planners metrics_for_multi_agent.launch` for FLIQC 

`rroslaunch planners metrics_for_std_planners.launch pipeline:=XXX`for standard planners

For example:
`roslaunch planners metrics_for_std_planners.launch pipeline:=ompl ompl:=EST`

FOR installation CHOMP, please see https://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/stomp_planner/stomp_planner_tutorial.html
