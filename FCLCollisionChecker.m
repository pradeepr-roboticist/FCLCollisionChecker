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
        objectHandle; % Handle to the underlying C++ class instance
    end
    methods
        %% Constructor - Create a new C++ class instance 
        function this = FCLCollisionChecker(varargin)
            this.objectHandle = fcl_collision_checker_interface(FCLCollisionChecker.NEW_CMD_ID, varargin{:});
        end
        
        %% Destructor - Destroy the C++ class instance
        function delete(this)
            warning('Deleting FCLCollisionChecker instance');
            fcl_collision_checker_interface(FCLCollisionChecker.DELETE_CMD_ID, this.objectHandle);
        end

%         %% Update data
%         function varargout = update(this, varargin)
%             if isempty(varargin{1})
%                 error('Need input: update(new_flag, [x, y, z])');
%             end
%             [varargout{1:nargout}] = fcl_collision_checker_interface(FCLCollisionChecker.UPDATE_CMD_ID, this.objectHandle, varargin{:});
%         end
        %% Get distance data from given point
        function varargout = query_distance_from_obstacle(this, varargin)
%             if isempty(varargin{1})
%                 error('Need input: query([x, y, z])');
%             end
            [varargout{1:nargout}] = fcl_collision_checker_interface(FCLCollisionChecker.QUERY_DISTANCE_FROM_OBSTACLE_CMD_ID, this.objectHandle, varargin{:});
        end
        %% Get distance data
        function varargout = query_collision(this, varargin)
%             if isempty(varargin{1})
%                 error('Need input: query([x, y, z])');
%             end
            [varargout{1:nargout}] = fcl_collision_checker_interface(FCLCollisionChecker.QUERY_COLLISION_CMD_ID, this.objectHandle, varargin{:});
        end
        function varargout = query_distance(this, varargin)
%             if isempty(varargin{1})
%                 error('Need input: query([x, y, z])');
%             end
            [varargout{1:nargout}] = fcl_collision_checker_interface(FCLCollisionChecker.QUERY_DISTANCE_CMD_ID, this.objectHandle, varargin{:});
        end
        %% Load object into cache
        function varargout = load_object_into_cache(this, varargin)
            if isempty(varargin{1}) || isempty(varargin{2})
                error('Need input: load_object_into_cache(filename, tag)');
            end
            [varargout{1:nargout}] = fcl_collision_checker_interface(FCLCollisionChecker.LOAD_OBJECT_CMD_ID, this.objectHandle, varargin{:});
        end
        %% Insert object into environment
        function varargout = insert_object_into_environment(this, varargin)
            if isempty(varargin{1}) || isempty(varargin{2})
                error('Need input: insert_object_into_environment(tag, position)');
            end
            [varargout{1:nargout}] = fcl_collision_checker_interface(FCLCollisionChecker.INSERT_OBJECT_CMD_ID, this.objectHandle, varargin{:});
        end
        %% Load robot into cache
        function varargout = load_robot_into_cache(this, varargin)
            if isempty(varargin{1})
                error('Need input: load_robot_into_cache(tag, ...)');
            end
            [varargout{1:nargout}] = fcl_collision_checker_interface(FCLCollisionChecker.LOAD_ROBOT_CMD_ID, this.objectHandle, varargin{:});
        end
        %% Insert robot into environment
        function varargout = insert_robot_into_environment(this, varargin)
%             if isempty(varargin{1})
%                 error('Need input: insert_robot_into_environment()');
%             end
            [varargout{1:nargout}] = fcl_collision_checker_interface(FCLCollisionChecker.INSERT_ROBOT_CMD_ID, this.objectHandle, varargin{:});
        end
        %% Update object
        function varargout = update_object(this, varargin)
%             if isempty(varargin{1})
%                 error('Need input: insert_robot_into_environment()');
%             end
            [varargout{1:nargout}] = fcl_collision_checker_interface(FCLCollisionChecker.UPDATE_OBJECT_CMD_ID, this.objectHandle, varargin{:});
        end
        %% Update robot
        function varargout = update_robot(this, varargin)
%             if isempty(varargin{1})
%                 error('Need input: insert_robot_into_environment()');
%             end
            [varargout{1:nargout}] = fcl_collision_checker_interface(FCLCollisionChecker.UPDATE_ROBOT_CMD_ID, this.objectHandle, varargin{:});
        end
        %% Remove object
        function varargout = remove_object(this, varargin)
%             if isempty(varargin{1})
%                 error('Need input: insert_robot_into_environment()');
%             end
            [varargout{1:nargout}] = fcl_collision_checker_interface(FCLCollisionChecker.REMOVE_OBJECT_CMD_ID, this.objectHandle, varargin{:});
        end
        %% Remove robot
        function varargout = remove_robot(this, varargin)
%             if isempty(varargin{1})
%                 error('Need input: insert_robot_into_environment()');
%             end
            [varargout{1:nargout}] = fcl_collision_checker_interface(FCLCollisionChecker.REMOVE_ROBOT_CMD_ID, this.objectHandle, varargin{:});
        end
    end
end
