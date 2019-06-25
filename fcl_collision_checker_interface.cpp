// Matlab stuff
#include "mex.h"
#include "matrix.h"

// My wrapper
#include "FCLCollisionChecker.h"

#include "class_handle.hpp"

#include <iostream>
using namespace fcl;

const size_t NEW_CMD_ID = 0;
const size_t DELETE_CMD_ID = 1;
const size_t UPDATE_CMD_ID = 2;
const size_t QUERY_CMD_ID = 3;
const size_t LOAD_OBJECT_CMD_ID = 4;
const size_t LOAD_ROBOT_CMD_ID = 5;
const size_t INSERT_OBJECT_CMD_ID = 6;
const size_t INSERT_ROBOT_CMD_ID = 7;
const size_t UPDATE_OBJECT_CMD_ID = 8;
const size_t UPDATE_ROBOT_CMD_ID = 9;

fcl::Vec3f read_point_from_matlab(const mxArray* point_ptr)
{
    const mxDouble *const loc = mxGetDoubles(point_ptr);
    return fcl::Vec3f(loc[0], loc[1], loc[2]);
}
std::vector<fcl::Vec3f> read_points_from_matlab(const mxArray* points_ptr)
{
    std::vector<fcl::Vec3f> position;
    const mxDouble *const position_ptr = mxGetDoubles(points_ptr);
    const size_t point_dim = mxGetM(points_ptr);
    const size_t num_points = mxGetN(points_ptr);
    if (point_dim != 3)
    {
        mexErrMsgTxt("Wrong dimensions for input points: points should be 3 x N");
    }
    position.reserve(num_points);
    size_t pos_idx = 0;
    for (size_t k = 0; k < num_points; k++)
    {
        position.push_back(fcl::Vec3f(position_ptr[pos_idx], position_ptr[pos_idx+1], position_ptr[pos_idx+2]));
        pos_idx += 3;
    }
    return position;
}
std::vector<double> read_array_from_matlab(const mxArray* array_ptr)
{
    const size_t rows_dim = mxGetM(array_ptr);
    const size_t cols_dim = mxGetN(array_ptr);
    const size_t num_values = std::max(rows_dim, cols_dim);
    const mxDouble *const dbl_ptr = mxGetDoubles(array_ptr);
    std::vector<double> array;
    array.reserve(num_values);
    for (size_t k = 0; k < num_values; ++k)
    {
        array.push_back(dbl_ptr[k]);
    }
    // if (point_dim != 3)
    // {
    //     mexErrMsgTxt("Wrong dimensions for input points: points should be 3 x N");
    // }
    return array;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    
    if (nrhs < 1)
    {
        mexErrMsgTxt("First input should be a command id of type uint64.");
    }
    // Get the command id
    const size_t cmd_id = (size_t) mxGetScalar(prhs[0]);

    // New
    if ( NEW_CMD_ID == cmd_id ) {
        // Check parameters
        if (nlhs != 1)
            mexErrMsgTxt("New: One output expected.");
        // Return a handle to a new C++ instance
        plhs[0] = convertPtr2Mat<FCLCollisionChecker>(new FCLCollisionChecker);
        return;
    }

    // Delete
    if ( DELETE_CMD_ID == cmd_id ) {
        // Destroy the C++ object
        destroyObject<FCLCollisionChecker>(prhs[1]);
        // Warn if other commands were ignored
        if (nlhs != 0 || nrhs != 2)
            mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
        return;
    }

    // Get the class instance pointer from the second input
    FCLCollisionChecker *cc_instance = convertMat2Ptr<FCLCollisionChecker>(prhs[1]);


    if ( QUERY_CMD_ID == cmd_id ) {
        // Check parameters
        // if (nlhs < 0 || nrhs < 1)
        //     mexWarnMsgTxt("Query: Unexpected arguments.");
        
        // // Get points
        // const mxArray* points = prhs[2];
        // const mxDouble *const loc = mxGetDoubles(points);
        // const size_t rows = mxGetM(points);
        // const size_t num_points = mxGetN(points);
        // if (rows != 3)
        // {
        //     mexErrMsgTxt("Wrong dimensions for input points: points should be 3 x N");
        // }

        // std::vector<octomap::point3d> closest_obstacles(num_points);
        // std::vector<float> distances(num_points);

        // size_t coord_counter = 0;
        // for (size_t k = 0; k < num_points; k++)
        // {
        //     const octomap::point3d query(loc[coord_counter+0], loc[coord_counter+1], loc[coord_counter+2]);
        //     coord_counter += 3;
        //     float distance = -1;
        //     octomap::point3d closest_obstacle;
        //     cc_instance->get_distance_and_closest_obstacle(query, distance, closest_obstacle);

        //     closest_obstacles[k] = closest_obstacle;
        //     distances[k] = distance;
        // }

        // if (nlhs == 2)
        // {
        //     plhs[0] = mxCreateNumericMatrix(1, num_points, mxDOUBLE_CLASS, mxREAL);
        //     mxDouble* ptr_dist = mxGetDoubles(plhs[0]);
        //     plhs[1] = mxCreateNumericMatrix(3, num_points, mxDOUBLE_CLASS, mxREAL);
        //     mxDouble* ptr_closest_obst = mxGetDoubles(plhs[1]);
        //     // ptr_dist[0] = (double) distance;
        //     size_t n = 0;
        //     for (size_t k = 0; k < num_points; k++)
        //     {
        //         ptr_dist[k] = distances[k];
        //         ptr_closest_obst[n++] = (double) closest_obstacles[k].x();
        //         ptr_closest_obst[n++] = (double) closest_obstacles[k].y();
        //         ptr_closest_obst[n++] = (double) closest_obstacles[k].z();
        //     }

        // }

        if (nlhs == 1)
        {
            std::cout << "MEX api side" << std::endl;
            plhs[0] = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
            mxDouble* coll_val = mxGetDoubles(plhs[0]);
            *coll_val = cc_instance->query_collision()? 1:0;
        }
        return;
    }
    if ( LOAD_OBJECT_CMD_ID == cmd_id ) {
        // Check parameters
        // if (nlhs < 0 || nrhs < 3)
        //     mexWarnMsgTxt("Insert: Unexpected arguments.");
        
        // Get filename 
        char* filename = mxArrayToString(prhs[2]);
        char* tag = mxArrayToString(prhs[3]);
        // std::cout << "\nLoading file : " << filename << std::endl;
        cc_instance->load_object_into_cache(filename, tag);
        mxFree(filename); // conform to documentation requirements : https://www.mathworks.com/help/matlab/matlab_external/passing-strings-1.html
        mxFree(tag); // conform to documentation requirements : https://www.mathworks.com/help/matlab/matlab_external/passing-strings-1.html
        return;
    }
    if ( INSERT_OBJECT_CMD_ID == cmd_id ) {
        // Check parameters
        // if (nlhs < 0 || nrhs < 3)
        //     mexWarnMsgTxt("Insert: Unexpected arguments.");
        
        // Get filename 
        char* tag = mxArrayToString(prhs[2]);
        cc_instance->insert_object_into_environment(tag, read_point_from_matlab(prhs[3]));
        mxFree(tag); // conform to documentation requirements : https://www.mathworks.com/help/matlab/matlab_external/passing-strings-1.html
        return;
    }
    if ( UPDATE_OBJECT_CMD_ID == cmd_id ) {
        // Check parameters
        // if (nlhs < 0 || nrhs < 3)
        //     mexWarnMsgTxt("Insert: Unexpected arguments.");
        
        char* tag = mxArrayToString(prhs[2]);
        cc_instance->update_object(tag, read_point_from_matlab(prhs[3]));
        return;
    }

    if ( LOAD_ROBOT_CMD_ID == cmd_id ) {
        // Check parameters
        // if (nlhs < 0 || nrhs < 3)
        //     mexWarnMsgTxt("Insert: Unexpected arguments.");

        std::vector<fcl::Vec3f> position = read_points_from_matlab(prhs[2]);
        std::vector<double> radii = read_array_from_matlab(prhs[3]);
        cc_instance->load_robot_into_cache(position, radii);
        return;
    }
    if ( INSERT_ROBOT_CMD_ID == cmd_id ) {
        // Check parameters
        // if (nlhs < 0 || nrhs < 3)
        //     mexWarnMsgTxt("Insert: Unexpected arguments.");
        
        // void load_robot_into_cache(const char* tag, const std::vector<fcl::Vec3f>& position, const std::vector<double>& radii)
        // Get filename
        cc_instance->insert_robot_into_environment();
        return;
    }
    if ( UPDATE_ROBOT_CMD_ID == cmd_id ) {
        // Check parameters
        // if (nlhs < 0 || nrhs < 3)
        //     mexWarnMsgTxt("Insert: Unexpected arguments.");
        
        // void load_robot_into_cache(const char* tag, const std::vector<fcl::Vec3f>& position, const std::vector<double>& radii)
        // Get filename
        std::vector<fcl::Vec3f> position = read_points_from_matlab(prhs[2]);
        cc_instance->update_robot(position);
        return;
    }

    return;
    // Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}

