# Coverage Pattern Service
Three Patterns of Waypoints giving to move_base from selected point area of Rviz

Package is refered from https://gitlab.com/Humpelstilzchen/path_coverage_ros/-/tree/master/scripts?ref_type=heads which is the planning of Trapezoidal_coverage where the area to cover is defined by "Publish Point" and the resulting waypoints are given to ROS navigation stack. Boustrophedon decomposition is included by reference.

The zigzag and vortical patterns are added so the package provides three patterns of path coverage. Although need to improve for better coverage, it works.
![1](https://github.com/theinge9/coverage_pattern/assets/138423325/4c05eb3d-4db0-4350-9e9b-b39e79eb3544)  
![2](https://github.com/theinge9/coverage_pattern/assets/138423325/924b0f46-56aa-4273-a7bf-2c6fd563dad7)
![3](https://github.com/theinge9/coverage_pattern/assets/138423325/f8567d4d-6585-4e1f-911b-65da5253ece3)

And for all points of area to be covered, the coverage checking(grid waypoints)running is added. 
![4](https://github.com/theinge9/coverage_pattern/assets/138423325/78c0b64d-ceab-41c4-99df-602d4d0029a7)

Service Server with these pattern movements which is called by PublishPoint Tool of Rviz or predefined area points.
![5](https://github.com/theinge9/coverage_pattern/assets/138423325/dc414461-077b-4dff-b182-7d3aebc6252c)

Currently, algorithms for Grid Map Occupied Cells Connection and Go and Stop Behaviour Control are being studied.
!!!Adjustment of global and local planner parameters is needed for better path planning results.
