// Matlab stuff
#include "mex.h"
#include "matrix.h"

// My wrapper
#include "FCLCollisionChecker.h"

#include "class_handle.hpp"

#include <iostream>
#include <algorithm>
using namespace fcl;

const size_t NEW_CMD_ID = 0;
const size_t DELETE_CMD_ID = 1;
const size_t QUERY_DISTANCE_FROM_OBSTACLE_CMD_ID = 2;
const size_t QUERY_COLLISION_CMD_ID = 3;
const size_t QUERY_DISTANCE_CMD_ID = 4;
const size_t LOAD_OBJECT_CMD_ID = 5;
const size_t LOAD_ROBOT_CMD_ID = 6;
const size_t INSERT_OBJECT_CMD_ID = 7;
const size_t INSERT_ROBOT_CMD_ID = 8;
const size_t UPDATE_OBJECT_CMD_ID = 9;
const size_t UPDATE_ROBOT_CMD_ID = 10;
const size_t REMOVE_OBJECT_CMD_ID = 11;
const size_t REMOVE_ROBOT_CMD_ID = 12;

fcl::Vec3f read_point_from_matlab(const mxArray* point_ptr)
{
    const mxDouble *const loc = mxGetDoubles(point_ptr);
    return fcl::Vec3f(loc[0], loc[1], loc[2]);
}
std::pair<fcl::Matrix3f, fcl::Vec3f> read_transform_matrix_from_matlab(const mxArray* mat_ptr)
{
    const mxDouble *const data_ptr = mxGetDoubles(mat_ptr);
    const size_t row_dim = mxGetM(mat_ptr);
    const size_t col_dim = mxGetN(mat_ptr);
    if (row_dim != 4 || col_dim != 4)
    {
        mexErrMsgTxt("Wrong dimensions. Expected 4x4 matrix.");
    }
    fcl::Vec3f position(data_ptr[12], data_ptr[13], data_ptr[14]);
    // fcl::Matrix3f rotation({data_ptr[0], data_ptr[1], data_ptr[2]}, {data_ptr[4], data_ptr[5], data_ptr[6]}, {data_ptr[8], data_ptr[9], data_ptr[10]});
    fcl::Matrix3f rotation({data_ptr[0], data_ptr[4], data_ptr[4]}, {data_ptr[1], data_ptr[5], data_ptr[9]}, {data_ptr[2], data_ptr[6], data_ptr[10]}); // specified row wise [A;B;C] in matlab syntax
    return std::pair<fcl::Matrix3f, fcl::Vec3f>(rotation, position);
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

    if ( QUERY_DISTANCE_FROM_OBSTACLE_CMD_ID == cmd_id )
    {
        if (nlhs == 1)
        {
            auto point_list = read_points_from_matlab(prhs[2]);
            plhs[0] = mxCreateNumericMatrix(1, point_list.size(), mxDOUBLE_CLASS, mxREAL);
            auto dist_list = cc_instance->query_obstacle_distance_from_point(point_list);
            mxDouble* dist_ptr = mxGetDoubles(plhs[0]);
            std::copy(dist_list.begin(), dist_list.end(), dist_ptr);
        }
        return;
    }

    if ( QUERY_COLLISION_CMD_ID == cmd_id ) {
        // Check parameters
        // if (nlhs < 0 || nrhs < 1)
        //     mexWarnMsgTxt("Query: Unexpected arguments.");
        if (nlhs == 1)
        {
            // plhs[0] = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
            // mxDouble* coll_val = mxGetDoubles(plhs[0]);
            // *coll_val = cc_instance->query_collision()? 1:0;
            plhs[0] = mxCreateLogicalMatrix(1, 1);
            mxLogical* coll_val = mxGetLogicals(plhs[0]);
            *coll_val = cc_instance->query_collision();
        }
        return;
    }

    if ( QUERY_DISTANCE_CMD_ID == cmd_id ) {
        // Check parameters
        // if (nlhs < 0 || nrhs < 1)
        //     mexWarnMsgTxt("Query: Unexpected arguments.");
        if (nlhs == 1)
        {
            plhs[0] = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
            mxDouble* dist_val = mxGetDoubles(plhs[0]);
            *dist_val = cc_instance->query_distance();
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
        auto R_T = read_transform_matrix_from_matlab(prhs[3]);
        cc_instance->update_object(tag, R_T.first, R_T.second);
        mxFree(tag);
        return;
    }
    if ( REMOVE_OBJECT_CMD_ID == cmd_id ) {
        // Check parameters
        // if (nlhs < 0 || nrhs < 3)
        //     mexWarnMsgTxt("Insert: Unexpected arguments.");
        char* tag = mxArrayToString(prhs[2]);
        cc_instance->remove_object_from_environment(tag);
        mxFree(tag);
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
        cc_instance->insert_robot_into_environment();
        return;
    }
    if ( UPDATE_ROBOT_CMD_ID == cmd_id ) {
        // Check parameters
        // if (nlhs < 0 || nrhs < 3)
        //     mexWarnMsgTxt("Insert: Unexpected arguments.");
        std::vector<fcl::Vec3f> position = read_points_from_matlab(prhs[2]);
        cc_instance->update_robot(position);
        return;
    }
    if ( REMOVE_ROBOT_CMD_ID == cmd_id ) {
        // Check parameters
        // if (nlhs < 0 || nrhs < 3)
        //     mexWarnMsgTxt("Insert: Unexpected arguments.");
        cc_instance->remove_robot_from_environment();
        return;
    }

    return;
    // Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}

