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


#include "FCLCollisionChecker.h"

int main(int argc, char** argv)
{
    FCLCollisionChecker fcl;

    // Load an object into memory
    fcl.load_object_into_cache("../data/geom_test.bt", "test_part");

    // Place an object into environment
    auto position = fcl::Vec3f(0, 0, 0);
    fcl.insert_object_into_environment("test_part", position);

    // Update its transform 
    // Specified row wise [A;B;C] in matlab syntax
    auto R = fcl::Matrix3f({1, 0, 0},  // A
                           {0, 1, 0},  // B
                           {0, 0, 1}); // C 

    auto t = fcl::Vec3f(0, 0, 0);
    fcl.update_object("test_part", R, t);

    // Load robot balls into cache
    auto robot_ball_positions = std::vector<fcl::Vec3f>(3);
    robot_ball_positions[0] = fcl::Vec3f(0, 0, 0);
    robot_ball_positions[1] = fcl::Vec3f(0, 0, 0.5);
    robot_ball_positions[2] = fcl::Vec3f(0, 0, 1);

    auto robot_ball_radii = std::vector<double>(3);
    robot_ball_radii[0] = 0.1;
    robot_ball_radii[1] = 0.1;
    robot_ball_radii[2] = 0.1;

    fcl.load_robot_into_cache(robot_ball_positions, robot_ball_radii);

    // Insert robot into environment
    fcl.insert_robot_into_environment();

    // Perform collision detection between robot balls and environment
    {
        std::vector<int> colliding_ball_ids;
        bool collision_flag = false;
        int num_contacts = 1;
        fcl.query_collision(&collision_flag, colliding_ball_ids, num_contacts);

        std::string msg = collision_flag ? "Collision" : "No collision";
        std::cout << msg << " detected " << std::endl;
    }

    {
        // Move robot to new position
        auto robot_ball_positions_new = std::vector<fcl::Vec3f>(3);
        robot_ball_positions_new[0] = fcl::Vec3f(0, 0.3, 0);
        robot_ball_positions_new[1] = fcl::Vec3f(0, 0.3, 0.3);
        robot_ball_positions_new[2] = fcl::Vec3f(0, 0.3, 1);
        fcl.update_robot(robot_ball_positions_new);

        // Perform collision detection between robot balls and environment
        std::vector<int> colliding_ball_ids;
        bool collision_flag = false;
        int num_contacts = 1;
        fcl.query_collision(&collision_flag, colliding_ball_ids, num_contacts);

        std::string msg = collision_flag ? "Collision" : "No collision";
        std::cout << msg << " detected " << std::endl;
    }

    // Tear down everything (not needed, but good to know)
    fcl.remove_object_from_environment("test_part");
    fcl.remove_robot_from_environment();
    return 0;
}

/*
typedef Map<Matrix<double, 3, Dynamic> > Centers;

inline size_t get_ball_centers_per_link(const size_t offset, const size_t N, const Matrix4d transform, Centers& output) const
{
    Matrix<double, 4, Dynamic> l(4, N);
    for (size_t k=0; k<N; k++)
    {
        Vector4d tmp;
        Matrix<double, 1, 1> one = Matrix<double, 1, 1>::Ones();
        tmp << ball_centers_nominal[offset+k], one;
        l.col(k) = tmp;

    }
    // std::cout << "\n L : \n " << l << std::endl; 
    // std::cout << "\n ### \n ";

    Matrix<double, 4, Dynamic> out(4, N);
    out = transform * l;
    output = out.topRows(3);

    return offset + N;
}

inline size_t set_ball_nominal_centers_per_link_from_file(const size_t offset, const string filename)
{
    // std::cout << "\n Trying to read file " << filename << std::endl;
    std::ifstream file(filename.c_str(), std::ios_base::in);
    size_t k = 0;
    size_t n = offset;
    double x, y, z, rad;
    while (file >> x >> y >> z >> rad)
    {
        // std::cout << "\nX: " << x << ", Y: " << y << ", Z: " << z << ", R: " << rad << std::endl; 
        ball_centers_nominal.push_back(Vector3d(x, y, z));
        ball_sizes.push_back(rad);
        // std::cout << "\n BC : \n " << ball_centers_nominal[n] << ", off : " << offset << ", k : " << k << ", n : " << n << std::endl; 
        k = k + 1;
        n = offset + k;
    }
    std::cout << "\n Read " << k << " balls from " << filename << std::endl;
    return n;
}
inline void get_ball_centers(const HomTransMat& M0,
                                const HomTransMat& M1,
                                const HomTransMat& M2,
                                const HomTransMat& M3,
                                const HomTransMat& M4,
                                const HomTransMat& M5,
                                const HomTransMat& M6,
                                const HomTransMat& M7,
                                const HomTransMat& M8,
                                Centers& B0,
                                Centers& B1,
                                Centers& B2,
                                Centers& B3,
                                Centers& B4,
                                Centers& B5,
                                Centers& B6,
                                Centers& B7,
                                Centers& B8
                                ) const
{
    // Use forward kinematics to figure out ball centers of link 1
    const Matrix4d T0 = M0 * PCM0;
    const Matrix4d T1 = M1 * PCM1;
    const Matrix4d T2 = M2 * PCM2;
    const Matrix4d T3 = M3 * PCM3;
    const Matrix4d T4 = M4 * PCM4;
    const Matrix4d T5 = M5 * PCM5;
    const Matrix4d T6 = M6 * PCM6;
    const Matrix4d T7 = M7 * PCM7;
    const Matrix4d T8 = M8 * PCM8;

    size_t offset = 0;
    offset = get_ball_centers_per_link(offset , N0, T0, B0);
    offset = get_ball_centers_per_link(offset , N1, T1, B1);
    offset = get_ball_centers_per_link(offset , N2, T2, B2);
    offset = get_ball_centers_per_link(offset , N3, T3, B3);
    offset = get_ball_centers_per_link(offset , N4, T4, B4);
    offset = get_ball_centers_per_link(offset , N5, T5, B5);
    offset = get_ball_centers_per_link(offset , N6, T6, B6);
    offset = get_ball_centers_per_link(offset , N7, T7, B7);
    offset = get_ball_centers_per_link(offset , N8, T8, B8);

}

 */