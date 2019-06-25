make_octree_file('/home/pradeepr/Downloads/test_frame.stl', '/home/pradeepr/Downloads/test_frame.bt', 256);
%%
clear all;
fcc = FCLCollisionChecker();
%%
fcc.load_object_into_cache('/home/pradeepr/Downloads/test_frame.bt', 'part1');

robot_balls = [0 0 0]';
robot_balls_radii = [1.0];
fcc.load_robot_into_cache(robot_balls, robot_balls_radii);
%%
fcc.insert_object_into_environment('part1', [0 0 0]);
%%

fcc.insert_robot_into_environment();
%%
ret = fcc.query_collision();