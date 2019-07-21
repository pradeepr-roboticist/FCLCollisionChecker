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
 * File              : FCLCollisionChecker.h
 * Author            : Pradeep Rajendran <pradeepr.roboticist@gmail.com>
 * Date              : 06.24.2019
 * Last Modified Date: 06.24.2019
 * Last Modified By  : Pradeep Rajendran <pradeepr.roboticist@gmail.com>
 */

#include <iostream>
#include <algorithm>
#include <iterator>
#include <limits>
#include <memory>
#include <string>

#if defined(FCL_NEW) || defined(FCL_OLD)
#else
#error Please define either FCL_NEW or FCL_OLD
#endif

// Octomap stuff
#include <octomap/octomap.h>


// FCL stuff
#ifdef FCL_NEW
#include "fcl/config.h"
#include "fcl/geometry/octree/octree.h"                                                         
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/distance.h"
#include "fcl/broadphase/broadphase_bruteforce.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#elif FCL_OLD
#include "fcl/config.h"
#include "fcl/octree.h"                                                         
#include "fcl/traversal/traversal_node_octree.h"                                
#include "fcl/collision.h"                                                      
#include "fcl/broadphase/broadphase.h"                                          
#include "fcl/shape/geometric_shape_to_BVH_model.h"                             
#include "fcl/math/transform.h" 
#include "fcl/collision_data.h"                                                      
#include "fcl/collision_object.h"                                                      
#include "fcl/distance.h"                                                      
#include "fcl/data_types.h"
#endif

// #define DEBUG

#ifdef FCL_NEW
using S = double;
#endif

#ifdef FCL_NEW
template <typename S>
#endif
struct CollisionData
{
  CollisionData()
  {
    done = false;
    collision_object_a = nullptr;
    collision_object_b = nullptr;
  }

  /// @brief Collision request
#ifdef FCL_NEW
  fcl::CollisionRequest<S> request;
  /// @brief Collision result
  fcl::CollisionResult<S> result;
#elif FCL_OLD
  fcl::CollisionRequest request;
  /// @brief Collision result
  fcl::CollisionResult result;
#endif

  /// @brief Whether the collision iteration can stop
  bool done;
#ifdef FCL_NEW
  fcl::CollisionObject<S>* collision_object_a = nullptr;
  fcl::CollisionObject<S>* collision_object_b = nullptr;
#elif FCL_OLD
  fcl::CollisionObject* collision_object_a = nullptr;
  fcl::CollisionObject* collision_object_b = nullptr;
#endif
};

#ifdef FCL_NEW
template <typename S>
#endif
struct DistanceData
{
  DistanceData()
  {
    done = false;
  }

#ifdef FCL_NEW
  fcl::DistanceRequest<S> request;
  fcl::DistanceResult<S> result;
#elif FCL_OLD
  fcl::DistanceRequest request;
  fcl::DistanceResult result;
#endif

  /// @brief Whether the distance iteration can stop
  bool done;

};

#ifdef FCL_NEW
template <typename S>
bool defaultDistanceFunction_(fcl::CollisionObject<S>* o1, fcl::CollisionObject<S>* o2, void* cdata_, S& dist)
{
  DistanceData<S>* cdata = static_cast<DistanceData<S>*>(cdata_);
  const fcl::DistanceRequest<S>& request = cdata->request;
//   std::cout << "Requesting points : " << request.enable_nearest_points << std::endl;

  fcl::DistanceResult<S>& result = cdata->result;
#elif FCL_OLD
bool defaultDistanceFunction_(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_, double& dist)
{
  DistanceData* cdata = static_cast<DistanceData*>(cdata_);
  const fcl::DistanceRequest& request = cdata->request;
//   std::cout << "Requesting points : " << request.enable_nearest_points << std::endl;

  fcl::DistanceResult& result = cdata->result;
#endif

  if(cdata->done) { dist = result.min_distance; return true; }

  fcl::distance(o1, o2, request, result);
  
  dist = result.min_distance;

  if(dist <= 0) return true; // in collision or in touch

  return cdata->done;
}

#ifdef FCL_NEW
template <typename S>
bool defaultCollisionFunction_(fcl::CollisionObject<S>* o1, fcl::CollisionObject<S>* o2, void* cdata_)
{
#elif FCL_OLD
bool defaultCollisionFunction_(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_)
{
#endif
    // std::cout << "Came here: o1 : \n" << o1->getTranslation() << " , o2: \n" << o2->getTranslation() << std::endl;
    // CollisionData<S> *cdata = static_cast<CollisionData<S> *>(cdata_);
    // const fcl::CollisionRequest<S>& request = cdata->request;
    // fcl::CollisionResult<S>& result = cdata->result;

    // if(cdata->done) return true;

    // fcl::collide(o1, o2, request, result);

    // if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
    //     cdata->done = true;

    // return cdata->done;
#ifdef FCL_NEW
  auto* cdata = static_cast<CollisionData<S>*>(cdata_);
#elif FCL_OLD
  auto* cdata = static_cast<CollisionData*>(cdata_);
#endif
  const auto& request = cdata->request;
  auto& result = cdata->result;

  if(cdata->done) return true;

  fcl::collide(o1, o2, request, result);

  if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
  {
    cdata->done = true;
    cdata->collision_object_a = o1;
    cdata->collision_object_b = o2;
  }

  return cdata->done;
}


class FCLCollisionChecker
{
    private:
        float resolution_;

#ifdef FCL_NEW
        std::unique_ptr<fcl::DynamicAABBTreeCollisionManager<S>> env_object_manager_;
        std::unordered_map<std::string, std::shared_ptr<fcl::OcTree<S>>> object_octomap_cache_;
        std::unordered_map<std::string, std::shared_ptr<fcl::CollisionObject<S>>> env_collision_object_cache_;
        std::unique_ptr<fcl::DynamicAABBTreeCollisionManager<S>> robot_object_manager_;
        std::vector<std::shared_ptr<fcl::CollisionObject<S>>> robot_collision_object_cache_;
        std::shared_ptr<fcl::CollisionObject<S>> small_sphere_; // this is used to obtain the distance to closest obstacle
#elif FCL_OLD
        std::unique_ptr<fcl::DynamicAABBTreeCollisionManager> env_object_manager_;
        std::unordered_map<std::string, std::shared_ptr<fcl::OcTree>> object_octomap_cache_;
        std::unordered_map<std::string, std::shared_ptr<fcl::CollisionObject>> env_collision_object_cache_;
        std::unique_ptr<fcl::DynamicAABBTreeCollisionManager> robot_object_manager_;
        std::vector<std::shared_ptr<fcl::CollisionObject>> robot_collision_object_cache_;
        std::shared_ptr<fcl::CollisionObject> small_sphere_; // this is used to obtain the distance to closest obstacle
#endif
        double small_sphere_radius_;

#ifdef FCL_NEW
        std::shared_ptr<fcl::OcTree<S>> read_from_file_(const char* filename)
#elif FCL_OLD
        std::shared_ptr<fcl::OcTree> read_from_file_(const char* filename)
#endif
        {
            // This resolution does not seem to matter if a tree is read from
            // file.
            ///////////////////// LOAD OCTREE FROM FILE ////////////////////////////////
            resolution_ = 0.0001;
            // Following the words of wisdom from Deb Haldar https://www.acodersjourney.com/top-10-dumb-mistakes-avoid-c-11-smart-pointers/
            auto tree = std::make_shared<octomap::OcTree>(resolution_);
            tree->readBinary(filename);
            std::cout<<"Read in tree from "<< filename <<" with "<<tree->getNumLeafNodes()<<" leaves."<<std::endl;
            double x,y,z;
            tree->getMetricMin(x,y,z);
            octomap::point3d min(x,y,z);
            std::cout<<"Metric min: "<<x<<","<<y<<","<<z<<std::endl;
            tree->getMetricMax(x,y,z);
            octomap::point3d max(x,y,z);
            std::cout<<"Metric max: "<<x<<","<<y<<","<<z<<std::endl;
            const double map_res = tree->getResolution();
            std::cout<<"Map resolution: "<< map_res <<std::endl;
            std::cout << "Load complete" << std::endl;
            ///////////////////////////////////////////////////////////////////////////

#ifdef FCL_NEW
            auto tree_fcl = std::make_shared<fcl::OcTree<S>>(tree);
#elif FCL_OLD
            auto tree_fcl = std::make_shared<fcl::OcTree>(tree);
#endif
            return tree_fcl;
        }

    public:
        FCLCollisionChecker()
        {
#ifdef FCL_NEW
            env_object_manager_ = std::make_unique<fcl::DynamicAABBTreeCollisionManager<S>>();
            robot_object_manager_ = std::make_unique<fcl::DynamicAABBTreeCollisionManager<S>>();
#elif FCL_OLD
            env_object_manager_ = std::make_unique<fcl::DynamicAABBTreeCollisionManager>();
            robot_object_manager_ = std::make_unique<fcl::DynamicAABBTreeCollisionManager>();
#endif

            env_object_manager_->setup();
            robot_object_manager_->setup();

            small_sphere_radius_ = 0.01;
#ifdef FCL_NEW
            auto sphere = std::make_shared<fcl::Sphere<S>>(small_sphere_radius_);
            small_sphere_ = std::make_shared<fcl::CollisionObject<S>>(sphere);
#elif FCL_OLD
            auto sphere = std::make_shared<fcl::Sphere>(small_sphere_radius_);
            small_sphere_ = std::make_shared<fcl::CollisionObject>(sphere);
#endif
        }
        ~FCLCollisionChecker()
        {
            std::cout << "FCLCollisionChecker destroyed" << std::endl;
        }

        void load_object_into_cache(const char* filename, const char* tag)
        {
            if (0 == object_octomap_cache_.count(tag))
            {
                auto tree_fcl = read_from_file_(filename);
                std::cout << "Loading object into cache with tag \"" << tag << "\"" << std::endl;
                object_octomap_cache_[tag] = tree_fcl;
            }
            else
            {
                std::cout << "Object with same name already loaded. Ignoring command." << std::endl;
            }
            
        }

#ifdef FCL_NEW
        void insert_object_into_environment(const char* tag, const fcl::Vector3<S>& position)
#elif FCL_OLD
        void insert_object_into_environment(const char* tag, const fcl::Vec3f& position)
#endif
        {
//             #ifdef DEBUG
//             std::cout << "Trying to insert \"" << tag << "\""
//                       << " at \n " << position << std::endl;
//             #endif
            auto tree_fcl = object_octomap_cache_[tag];
// #ifdef FCL_NEW
//             fcl::Transform3<S> tf0;
//             tf0.translation() = fcl::Vector3<S>(position[0], position[1], position[2]);
//             #ifdef DEBUG
//             std::cout << "TF0 x: " << tf0(0, 3) << std::endl;
//             std::cout << "TF0 y: " << tf0(1, 3) << std::endl;
//             std::cout << "TF0 z: " << tf0(2, 3) << std::endl;
//             #endif
// #elif FCL_OLD
//             fcl::Transform3f tf0;
//             tf0.setTranslation(fcl::Vec3f(position[0], position[1], position[2]));
// #endif
            // Based on the discussion at https://stackoverflow.com/questions/13403490/passing-shared-ptrderived-as-shared-ptrbase
            // and the reference at http://www.cplusplus.com/reference/memory/static_pointer_cast/
#ifdef FCL_NEW
            auto geom = std::static_pointer_cast<fcl::CollisionGeometry<S>>(tree_fcl);
            std::shared_ptr<fcl::CollisionObject<S>> env_obj = std::make_shared<fcl::CollisionObject<S>>(geom);
            auto R = fcl::Matrix3<S>::Identity();
#elif FCL_OLD
            auto geom = std::static_pointer_cast<fcl::CollisionGeometry>(tree_fcl);
            std::shared_ptr<fcl::CollisionObject> env_obj = std::make_shared<fcl::CollisionObject>(geom);
            auto R = fcl::Matrix3f();
            R.setIdentity();
#endif
            
            env_obj->setTranslation(position);
            env_obj->setRotation(R);
            
            env_collision_object_cache_[tag] = env_obj;
            env_object_manager_->registerObject(env_obj.get());
            std::cout << "Inserted \"" << tag << "\"" << std::endl;
        }
        void remove_object_from_environment(const char* tag)
        {
            if (0 == env_collision_object_cache_.count(tag))
            {
                // object does not exist
                std::cout << "Object \"" << tag << "\" does not exist."<< std::endl;
            }
            else
            {
                auto env_obj = env_collision_object_cache_[tag];
                env_object_manager_->unregisterObject(env_obj.get());
                env_collision_object_cache_.erase(tag);
                std::cout << "Removed object \"" << tag << "\"" << std::endl;
            }
            
        }
#ifdef FCL_NEW
        void update_object(const char* tag, const fcl::Matrix3<S>& R, const fcl::Vector3<S>& t)
#elif FCL_OLD
        void update_object(const char* tag, const fcl::Matrix3f& R, const fcl::Vec3f& t)
#endif
        {
            auto env_obj = env_collision_object_cache_[tag];
            // std::cout << "Current transform of \"" << tag << "\"" << " t : " << env_obj->getTranslation() << std::endl;
            // std::cout << "Current transform of \"" << tag << "\"" << " R : " << env_obj->getRotation() << std::endl;
            env_obj->setTranslation(t);
            env_obj->setRotation(R);
            // env_obj->setTransform(...); not working
            // std::cout << "Setting transform of \"" << tag << "\"" << " t : " << env_obj->getTranslation() << std::endl;
            // std::cout << "Setting transform of \"" << tag << "\"" << " R : " << env_obj->getRotation() << std::endl;
            std::cout << "Transform set for \"" << tag << "\"" << std::endl;
            // env_object_manager_->unregisterObject(env_obj.get());
        }


#ifdef FCL_NEW
        void load_robot_into_cache(const std::vector<fcl::Vector3<S>>& position, const std::vector<double>& radii)
#elif FCL_OLD
        void load_robot_into_cache(const std::vector<fcl::Vec3f>& position, const std::vector<double>& radii)
#endif
        {
            robot_collision_object_cache_.clear();
            robot_collision_object_cache_.reserve(position.size());
            for (int k = 0; k < position.size(); ++k)
            {
#ifdef FCL_NEW
                fcl::Transform3<S> tf0;
                tf0.translation() = position[k];
                auto sphere_k = std::make_shared<fcl::Sphere<S>>(radii[k]);
                std::shared_ptr<fcl::CollisionObject<S>> rob_obj = std::make_shared<fcl::CollisionObject<S>>(sphere_k, tf0);
#elif FCL_OLD
                fcl::Transform3f tf0;
                tf0.setTranslation(position[k]);
                auto sphere_k = std::make_shared<fcl::Sphere>(radii[k]);
                std::shared_ptr<fcl::CollisionObject> rob_obj = std::make_shared<fcl::CollisionObject>(sphere_k, tf0);
#endif
                robot_collision_object_cache_.push_back(rob_obj);
            }
            std::cout << "Loaded " << robot_collision_object_cache_.size() << " robot balls " << std::endl;
        }

        void insert_robot_into_environment()
        {
            for (auto entry : robot_collision_object_cache_)
            {
                robot_object_manager_->registerObject(entry.get());
            }
            std::cout << "Inserted robot."<< std::endl;
        }

        void remove_robot_from_environment()
        {
            for (auto entry : robot_collision_object_cache_)
            {
                robot_object_manager_->unregisterObject(entry.get());
            }
            robot_collision_object_cache_.clear();
            std::cout << "Removed robot."<< std::endl;
        }

#ifdef FCL_NEW
        void update_robot(const std::vector<fcl::Vector3<S>>& positions_new)
#elif FCL_OLD
        void update_robot(const std::vector<fcl::Vec3f>& positions_new)
#endif
        {
            if (positions_new.size() != robot_collision_object_cache_.size())
            {
                std::cout << "Can't update robot. Collision object size mismatch." << std::endl;
            }
            else
            {
                for (size_t k = 0; k < robot_collision_object_cache_.size(); ++k)
                {
                    robot_collision_object_cache_[k]->setTranslation(positions_new[k]);
                    #ifdef DEBUG
                    std::cout << "Updated robot ball " << k << " pos: " << positions_new[k] << std::endl;
                    #endif
                }
                    #ifdef DEBUG
                    std::cout << "Updated robot."<< std::endl;
                    #endif
            }
        }

#ifdef FCL_NEW
        void query_obstacle_distance_from_point(const std::vector<fcl::Vector3<S>>& point_list, std::vector<double>& smallest_dists, std::vector<fcl::Vector3<S>>& closest_points)
#elif FCL_OLD
        void query_obstacle_distance_from_point(const std::vector<fcl::Vec3f>& point_list, std::vector<double>& smallest_dists, std::vector<fcl::Vec3f>& closest_points)
#endif
        {
            for (auto it = point_list.begin(); it != point_list.end(); ++it)
            {
                double dist;
#ifdef FCL_NEW
                fcl::Vector3<S> point_a, point_b;
#elif FCL_OLD
                fcl::Vec3f point_a, point_b;
#endif
                query_obstacle_distance_from_point_helper(*it, dist, point_a, point_b);
                closest_points.push_back(point_a);
                smallest_dists.push_back(dist);
            }
            // auto func = [this](auto const &pt) -> auto { return this->query_obstacle_distance_from_point(pt); };
            // std::transform(point_list.begin(), point_list.end(), std::back_inserter(ret), func);
        }

#ifdef FCL_NEW
        void query_obstacle_distance_from_point_helper(const fcl::Vector3<S>& point, double& smallest_dist, fcl::Vector3<S>& point_a, fcl::Vector3<S>& point_b)
        {
            DistanceData<S> cdata;
#elif FCL_OLD
        void query_obstacle_distance_from_point_helper(const fcl::Vec3f& point, double& smallest_dist, fcl::Vec3f& point_a, fcl::Vec3f& point_b)
        {
            DistanceData cdata;
#endif
            small_sphere_->setTranslation(point);
            cdata.request.enable_nearest_points = true;
            env_object_manager_->distance(small_sphere_.get(), (void *)&cdata, defaultDistanceFunction_);

            smallest_dist = cdata.result.min_distance + small_sphere_radius_;
            point_a = cdata.result.nearest_points[0];
            point_b = cdata.result.nearest_points[1];

            // std::cout << "Point A: " << point_a << std::endl;
            // std::cout << "Point B: " << point_b << std::endl;
            #ifdef DEBUG
            std::cout << "Closest obstacle distance: " << cdata.result.min_distance << std::endl;
            #endif
        }

        double query_distance()
        {
#ifdef FCL_NEW
            DistanceData<S> cdata;
#elif FCL_OLD
            DistanceData cdata;
#endif
            robot_object_manager_->distance(env_object_manager_.get(), (void *)&cdata, defaultDistanceFunction_);
            #ifdef DEBUG
            std::cout << "Min distance: " << cdata.result.min_distance << std::endl;
            #endif
            return cdata.result.min_distance;
        }
        
        void query_collision(bool* collision_flag, int32_t* robot_ball_id)
        {
#ifdef FCL_NEW
            CollisionData<S> cdata;
#elif FCL_OLD
            CollisionData cdata;
#endif
            cdata.request.num_max_contacts = 1;
            robot_object_manager_->collide(env_object_manager_.get(), (void *)&cdata, defaultCollisionFunction_);

            // fcl::Transform3d tf1;
            // tf1.translation() = fcl::Vector3<S>(0, 0, 0);
            // auto sphere1 = std::make_shared<fcl::Sphere<S>>(0.35);
            // auto obj1 = std::make_shared<fcl::CollisionObject<S>>(sphere1, tf1);
            // std::cout << " We know that 0\n" << obj1->getTranslation() << std::endl;
            // auto tmp = std::make_unique<fcl::DynamicAABBTreeCollisionManager<S>>();
            // tmp->setup();
            // tmp->registerObject(obj1.get());

            // fcl::Transform3d tf0;
            // tf0.translation() = fcl::Vector3<S>(0.5, 0, 0);

            // auto sphere = std::make_shared<fcl::Sphere<S>>(0.2);
            // auto obj0 = std::make_shared<fcl::CollisionObject<S>>(sphere, tf0);
            // std::cout << " We know that 0\n" << obj0->getTranslation() << std::endl;
            // tmp->collide(obj0.get(), (void *)&cdata, defaultCollisionFunction_);
            // robot_object_manager_->collide(obj1.get(), (void *)&cdata, defaultCollisionFunction_);

            *collision_flag = cdata.result.isCollision();
            *robot_ball_id = -1;
            if (true == cdata.result.isCollision())
            {
              // std::cout << cdata.collision_object_a << std::endl;
              for (auto it = env_collision_object_cache_.begin(); it != env_collision_object_cache_.end(); ++it)
              {
                if ((it->second.get() == cdata.collision_object_a) || (it->second.get() == cdata.collision_object_b))
                {
                  // std::cout << "Object: " << it->first << std::endl;
                }
              }
              for (auto it = robot_collision_object_cache_.begin(); it != robot_collision_object_cache_.end(); ++it)
              {
                if ((it->get() == cdata.collision_object_a) || (it->get() == cdata.collision_object_b))
                {
                  // std::cout << "Object: " << it - robot_collision_object_cache_.begin() << std::endl;
                  // std::cout << "Location: " << it->get()->getTranslation() << std::endl;
                  *robot_ball_id = it - robot_collision_object_cache_.begin();
                }
              }
            }
#ifdef DEBUG
            std::cout << "In query collision" << std::endl;
            if (cdata.result.isCollision())
            {
                std::cout << "Has collision" << std::endl;
            }
            else
            {
                std::cout << "No collision" << std::endl;
            }
#endif
        }

};