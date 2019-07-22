// Copyright (C) 2019 Pradeep Rajendran
// 
// This file is part of FCLCollisionChecker.
// 
// FCLCollisionChecker is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// FCLCollisionChecker is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with FCLCollisionChecker.  If not, see <http://www.gnu.org/licenses/>.

/**
 * File              : fcl_collision_checker_interface.cpp
 * Author            : Pradeep Rajendran <pradeepr.roboticist@gmail.com>
 * Date              : 06.24.2019
 * Last Modified Date: 06.24.2019
 * Last Modified By  : Pradeep Rajendran <pradeepr.roboticist@gmail.com>
 */

// Matlab stuff
#include "mex.h"
#include "matrix.h"

// My wrapper
#include "FCLCollisionChecker.h"

#include "class_handle.hpp"

#include <iostream>
#include <algorithm>
using namespace fcl;


#ifdef FCL_NEW
using S = double;
#endif

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


#ifdef FCL_NEW
void write_fcl_points_to_matlab(mxArray const* const points_ptr, std::vector<fcl::Vector3<S>> const& closest_point_list)
#elif FCL_OLD
void write_fcl_points_to_matlab(mxArray const* const points_ptr, std::vector<fcl::Vec3f> const& closest_point_list)
#endif
{
    mxDouble *const dbl_ptr = mxGetDoubles(points_ptr);
    //column major
    auto write_single_vec = [&dbl_ptr](const int idx, auto const &pt) -> int {
        dbl_ptr[idx+0] = pt[0];
        dbl_ptr[idx+1] = pt[1];
        dbl_ptr[idx+2] = pt[2];
        // std::cout << "\n Writing : " << pt[0] << "," << pt[1] << "," << pt[2] << std::endl;
        return idx + 3;
    };
    int i = 0;
    for (auto it = closest_point_list.cbegin(); it != closest_point_list.cend(); ++it)
    {
        i = write_single_vec(i, *it);
    }
}

#ifdef FCL_NEW
fcl::Vector3<S> read_point_from_matlab(mxArray const* point_ptr)
{
    const mxDouble *const loc = mxGetDoubles(point_ptr);
    return fcl::Vector3<S>(loc[0], loc[1], loc[2]);
}
#elif FCL_OLD
fcl::Vec3f read_point_from_matlab(mxArray const* point_ptr)
{
    const mxDouble *const loc = mxGetDoubles(point_ptr);
    return fcl::Vec3f(loc[0], loc[1], loc[2]);
}
#endif

#ifdef FCL_NEW
std::pair<fcl::Matrix3<S>, fcl::Vector3<S>> read_transform_matrix_from_matlab(const mxArray* mat_ptr)
#elif FCL_OLD
std::pair<fcl::Matrix3f, fcl::Vec3f> read_transform_matrix_from_matlab(const mxArray* mat_ptr)
#endif
{
    const mxDouble *const data_ptr = mxGetDoubles(mat_ptr);
    const size_t row_dim = mxGetM(mat_ptr);
    const size_t col_dim = mxGetN(mat_ptr);
    if (row_dim != 4 || col_dim != 4)
    {
        mexErrMsgTxt("Wrong dimensions. Expected 4x4 matrix.");
    }
#ifdef FCL_NEW
    fcl::Vector3<S> position(data_ptr[12], data_ptr[13], data_ptr[14]);
    fcl::Matrix3<S> rotation;
    rotation(0,0) = data_ptr[0];
    rotation(0,1) = data_ptr[1];
    rotation(0,2) = data_ptr[2];
    
    rotation(1,0) = data_ptr[4];
    rotation(1,1) = data_ptr[5];
    rotation(1,2) = data_ptr[6]; 
    
    rotation(2,0) = data_ptr[8];
    rotation(2,1) = data_ptr[9];
    rotation(2,2) = data_ptr[10];
    return std::pair<fcl::Matrix3<S>, fcl::Vector3<S>>(rotation, position);
#elif FCL_OLD
    fcl::Vec3f position(data_ptr[12], data_ptr[13], data_ptr[14]);
    fcl::Matrix3f rotation({data_ptr[0], data_ptr[4], data_ptr[4]}, {data_ptr[1], data_ptr[5], data_ptr[9]}, {data_ptr[2], data_ptr[6], data_ptr[10]}); // specified row wise [A;B;C] in matlab syntax
    return std::pair<fcl::Matrix3f, fcl::Vec3f>(rotation, position);
#endif
}

#ifdef FCL_NEW
std::vector<fcl::Vector3<S>> read_points_from_matlab(const mxArray* points_ptr)
{
    std::vector<fcl::Vector3<S>> position;
#elif FCL_OLD
std::vector<fcl::Vec3f> read_points_from_matlab(const mxArray* points_ptr)
{
    std::vector<fcl::Vec3f> position;
#endif
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
#ifdef FCL_NEW
        position.push_back(fcl::Vector3<S>(position_ptr[pos_idx], position_ptr[pos_idx+1], position_ptr[pos_idx+2]));
#elif FCL_OLD
        position.push_back(fcl::Vec3f(position_ptr[pos_idx], position_ptr[pos_idx+1], position_ptr[pos_idx+2]));
#endif
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
        // if (nlhs == 1)
        // {
        auto point_list = read_points_from_matlab(prhs[2]);
        auto num_points = point_list.size();
        plhs[0] = mxCreateNumericMatrix(1, num_points, mxDOUBLE_CLASS, mxREAL);
        plhs[1] = mxCreateNumericMatrix(3, num_points, mxDOUBLE_CLASS, mxREAL);
        std::vector<double> dist_list;
#ifdef FCL_NEW
        std::vector<fcl::Vector3<S>> closest_point_list;
#elif FCL_OLD
        std::vector<fcl::Vec3f> closest_point_list;
#endif
        dist_list.reserve(num_points);
        closest_point_list.reserve(num_points);
        cc_instance->query_obstacle_distance_from_point(point_list, dist_list, closest_point_list);

        // Copy over data to matlab
        mxDouble* dist_ptr = mxGetDoubles(plhs[0]);
        std::copy(dist_list.begin(), dist_list.end(), dist_ptr);
        write_fcl_points_to_matlab(plhs[1], closest_point_list);
        // }
        return;
    }

    if ( QUERY_COLLISION_CMD_ID == cmd_id ) {
        // Check parameters
        // if (nlhs < 0 || nrhs < 1)
        //     mexWarnMsgTxt("Query: Unexpected arguments.");
        if (nlhs == 2)
        {
            std::vector<int> robot_ball_ids;
            plhs[0] = mxCreateLogicalMatrix(1, 1);
            
            mxLogical* coll_val = mxGetLogicals(plhs[0]);

            int num_contacts_max = 1;
            if (nrhs == 3) num_contacts_max = mxGetScalar(prhs[2]); // get max number of contacts (to exhaustively perform collision detection)
            cc_instance->query_collision(coll_val, robot_ball_ids, num_contacts_max);

            // std::cout << "IDs size: " << robot_ball_ids.size() << std::endl;
            plhs[1] = mxCreateNumericMatrix(1, robot_ball_ids.size(), mxINT32_CLASS, mxREAL);
            mxInt32* robot_ball_id_ptr = mxGetInt32s(plhs[1]);
            std::copy(robot_ball_ids.begin(), robot_ball_ids.end(), robot_ball_id_ptr);
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

#ifdef FCL_NEW
        std::vector<fcl::Vector3<S>> position = read_points_from_matlab(prhs[2]);
#elif FCL_OLD
        std::vector<fcl::Vec3f> position = read_points_from_matlab(prhs[2]);
#endif
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
#ifdef FCL_NEW
        std::vector<fcl::Vector3<S>> position = read_points_from_matlab(prhs[2]);
#elif FCL_OLD
        std::vector<fcl::Vec3f> position = read_points_from_matlab(prhs[2]);
#endif
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

