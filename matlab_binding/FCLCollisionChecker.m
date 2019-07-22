% File              : FCLCollisionChecker.m
% Author            : Pradeep Rajendran <pradeepr.roboticist@gmail.com>
% Date              : 06.24.2019
% Last Modified Date: 06.24.2019
% Last Modified By  : Pradeep Rajendran <pradeepr.roboticist@gmail.com>

classdef FCLCollisionChecker < handle
    properties (Constant)
        NEW_CMD_ID = uint64(0);
        DELETE_CMD_ID = uint64(1);
        QUERY_DISTANCE_FROM_OBSTACLE_CMD_ID = uint64(2);
        QUERY_COLLISION_CMD_ID = uint64(3);
        QUERY_DISTANCE_CMD_ID = uint64(4);
        LOAD_OBJECT_CMD_ID = uint64(5);
        LOAD_ROBOT_CMD_ID = uint64(6);
        INSERT_OBJECT_CMD_ID = uint64(7);
        INSERT_ROBOT_CMD_ID = uint64(8);
        UPDATE_OBJECT_CMD_ID = uint64(9);
        UPDATE_ROBOT_CMD_ID = uint64(10);
        REMOVE_OBJECT_CMD_ID = uint64(11);
        REMOVE_ROBOT_CMD_ID = uint64(12);
    end
    properties (SetAccess = private, Hidden = true)
        objectHandle_old; % Handle to the underlying C++ class instance
        objectHandle_new;
    end
    methods
        %% Constructor - Create a new C++ class instance 
        function this = FCLCollisionChecker(varargin)
            this.objectHandle_new = fcl_collision_checker_interface_fcl_new(FCLCollisionChecker.NEW_CMD_ID, varargin{:});
            this.objectHandle_old = fcl_collision_checker_interface_fcl_old(FCLCollisionChecker.NEW_CMD_ID, varargin{:});
        end
        
        %% Destructor - Destroy the C++ class instance
        function delete(this)
            warning('Deleting FCLCollisionChecker instance');
            fcl_collision_checker_interface_fcl_new(FCLCollisionChecker.DELETE_CMD_ID, this.objectHandle_new);
            fcl_collision_checker_interface_fcl_old(FCLCollisionChecker.DELETE_CMD_ID, this.objectHandle_old);
        end

        %% Get distance data from given point
        function varargout = query_distance_from_obstacle(this, varargin)
            [varargout{1:nargout}] = fcl_collision_checker_interface_fcl_new(FCLCollisionChecker.QUERY_DISTANCE_FROM_OBSTACLE_CMD_ID, this.objectHandle_new, varargin{:});
        end
        %% Get collision data
        function varargout = query_collision(this, varargin)
            [varargout{1:nargout}] = fcl_collision_checker_interface_fcl_old(FCLCollisionChecker.QUERY_COLLISION_CMD_ID, this.objectHandle_old, varargin{:});
        end
        function varargout = query_distance(this, varargin)
            [varargout{1:nargout}] = fcl_collision_checker_interface_fcl_old(FCLCollisionChecker.QUERY_DISTANCE_CMD_ID, this.objectHandle_old, varargin{:});
        end
        %% Load object into cache
        function varargout = load_object_into_cache(this, varargin)
            if isempty(varargin{1}) || isempty(varargin{2})
                error('Need input: load_object_into_cache(filename, tag)');
            end
            [varargout{1:nargout}] = fcl_collision_checker_interface_fcl_new(FCLCollisionChecker.LOAD_OBJECT_CMD_ID, this.objectHandle_new, varargin{:});
            [varargout{1:nargout}] = fcl_collision_checker_interface_fcl_old(FCLCollisionChecker.LOAD_OBJECT_CMD_ID, this.objectHandle_old, varargin{:});
        end
        %% Insert object into environment
        function varargout = insert_object_into_environment(this, varargin)
            if isempty(varargin{1}) || isempty(varargin{2})
                error('Need input: insert_object_into_environment(tag, position)');
            end
            [varargout{1:nargout}] = fcl_collision_checker_interface_fcl_new(FCLCollisionChecker.INSERT_OBJECT_CMD_ID, this.objectHandle_new, varargin{:});
            [varargout{1:nargout}] = fcl_collision_checker_interface_fcl_old(FCLCollisionChecker.INSERT_OBJECT_CMD_ID, this.objectHandle_old, varargin{:});
        end
        %% Load robot into cache
        function varargout = load_robot_into_cache(this, varargin)
            [varargout{1:nargout}] = fcl_collision_checker_interface_fcl_new(FCLCollisionChecker.LOAD_ROBOT_CMD_ID, this.objectHandle_new, varargin{:});
            [varargout{1:nargout}] = fcl_collision_checker_interface_fcl_old(FCLCollisionChecker.LOAD_ROBOT_CMD_ID, this.objectHandle_old, varargin{:});
        end
        %% Insert robot into environment
        function varargout = insert_robot_into_environment(this, varargin)
            [varargout{1:nargout}] = fcl_collision_checker_interface_fcl_new(FCLCollisionChecker.INSERT_ROBOT_CMD_ID, this.objectHandle_new, varargin{:});
            [varargout{1:nargout}] = fcl_collision_checker_interface_fcl_old(FCLCollisionChecker.INSERT_ROBOT_CMD_ID, this.objectHandle_old, varargin{:});
        end
        %% Update object
        function varargout = update_object(this, varargin)
            [varargout{1:nargout}] = fcl_collision_checker_interface_fcl_new(FCLCollisionChecker.UPDATE_OBJECT_CMD_ID, this.objectHandle_new, varargin{:});
            [varargout{1:nargout}] = fcl_collision_checker_interface_fcl_old(FCLCollisionChecker.UPDATE_OBJECT_CMD_ID, this.objectHandle_old, varargin{:});
        end
        %% Update robot
        function varargout = update_robot(this, varargin)
            [varargout{1:nargout}] = fcl_collision_checker_interface_fcl_new(FCLCollisionChecker.UPDATE_ROBOT_CMD_ID, this.objectHandle_new, varargin{:});
            [varargout{1:nargout}] = fcl_collision_checker_interface_fcl_old(FCLCollisionChecker.UPDATE_ROBOT_CMD_ID, this.objectHandle_old, varargin{:});
        end
        %% Remove object
        function varargout = remove_object(this, varargin)
            [varargout{1:nargout}] = fcl_collision_checker_interface_fcl_new(FCLCollisionChecker.REMOVE_OBJECT_CMD_ID, this.objectHandle_new, varargin{:});
            [varargout{1:nargout}] = fcl_collision_checker_interface_fcl_old(FCLCollisionChecker.REMOVE_OBJECT_CMD_ID, this.objectHandle_old, varargin{:});
        end
        %% Remove robot
        function varargout = remove_robot(this, varargin)
            [varargout{1:nargout}] = fcl_collision_checker_interface_fcl_new(FCLCollisionChecker.REMOVE_ROBOT_CMD_ID, this.objectHandle_new, varargin{:});
            [varargout{1:nargout}] = fcl_collision_checker_interface_fcl_old(FCLCollisionChecker.REMOVE_ROBOT_CMD_ID, this.objectHandle_old, varargin{:});
        end
    end
end