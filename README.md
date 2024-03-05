# Coverage Pattern Service
Three Patterns of Waypoints giving to move_base from selected point area of Rviz

Package is refered from https://gitlab.com/Humpelstilzchen/path_coverage_ros/-/tree/master/scripts?ref_type=heads which is the planning of Trapezoidal_coverage where the area to cover is defined by "Publish Point" and the resulting waypoints are given to ROS navigation stack. Boustrophedon decomposition is included by reference.

The zigzag and vortical patterns are added so the package provides three patterns of path coverage. Although need to improve for better coverage, it works.

And for all points of area to be covered, the coverage checking(grid waypoints)running is added. 

Service Server with these pattern movements which is called by PublishPoint Tool of Rviz or predefined area points.

!!!Adjustment of global and local planner parameters is needed for better path planning results.
Currently, algorithms for Grid Map Occupied Cells Connection and Go and Stop Behaviour Control are being studied.
