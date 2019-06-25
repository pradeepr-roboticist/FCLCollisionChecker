% make_octree_file('/home/pradeepr/Downloads/test_frame.stl', '/home/pradeepr/Downloads/test_frame.bt', 256);
clear all;
global workspace_figure;
workspace_figure = WorkspaceFigure();
% 
% [v, f, n, ~] = stlReadBinary('/home/pradeepr/Downloads/test_frame.stl');
% h = stlPlot(v,f,'s',[0.5 0.5 0.5], 'FaceAlpha', 1);
mesh_path = '/home/pradeepr/Desktop/AVAStar/problems/amazon_vikas_demo';
fcl_collision_checker = FCLCollisionChecker();
iob = InteractiveObject(mesh_path, 'test_frame', 'part1', fcl_collision_checker);

% fcl_collision_checker.load_object_into_cache('/home/pradeepr/Downloads/test_frame.bt', 'part1');
% fcl_collision_checker.insert_object_into_environment('part1', [0 0 0]);

robot = iiwa7Robot();

%
q_query = zeros(1, 7);
result = robot.kinematics.collision_balls(q_query);
ball_centers = result.balls_centers;
B_sizes = result.ball_radii;
ball_link_memberships = result.ball_link_memberships;
B = [ball_centers.B1,ball_centers.B2,ball_centers.B3,ball_centers.B4,ball_centers.B5,ball_centers.B6,ball_centers.B7,ball_centers.B8,ball_centers.B9];
num_balls = size(B, 2);

fcl_collision_checker.load_robot_into_cache(B, B_sizes);
fcl_collision_checker.insert_robot_into_environment();
%
interactive_objs = [];

robot.edt_query_function = @(q) iiwa7_amazon_demo_collision_detection(q, robot.kinematics, fcl_collision_checker, interactive_objs, false); 

global robot_vis;
robot_vis = robot.display_class(robot, workspace_figure);
robot_vis.set_collision_func(robot.edt_query_function);


%%
% q_query = [0 -1.75 0 0 0 -0.45 0];
% result = robot.kinematics.collision_balls(q_query);
% ball_centers = result.balls_centers;
% B_sizes = result.ball_radii;
% B = [ball_centers.B1,ball_centers.B2,ball_centers.B3,ball_centers.B4,ball_centers.B5,ball_centers.B6,ball_centers.B7,ball_centers.B8,ball_centers.B9];
% 
% fcc.update_robot(B);

% ret = fcc.query_collision();
