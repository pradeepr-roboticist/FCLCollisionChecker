/**
 * File              : FCLCollisionChecker.h
 * Author            : Pradeep Rajendran <pradeepr.roboticist@gmail.com>
 * Date              : 06.24.2019
 * Last Modified Date: 06.24.2019
 * Last Modified By  : Pradeep Rajendran <pradeepr.roboticist@gmail.com>
 */

#include <iostream>
#include <limits>
#include <memory>
#include <string>

// Octomap stuff
#include <octomap/octomap.h>


// FCL stuff
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

struct CollisionData
{
  CollisionData()
  {
    done = false;
  }

  /// @brief Collision request
  fcl::CollisionRequest request;

  /// @brief Collision result
  fcl::CollisionResult result;

  /// @brief Whether the collision iteration can stop
  bool done;
};
struct DistanceData
{
  DistanceData()
  {
    done = false;
  }

  /// @brief Distance request
  fcl::DistanceRequest request;

  /// @brief Distance result
  fcl::DistanceResult result;

  /// @brief Whether the distance iteration can stop
  bool done;

};

bool defaultDistanceFunction_(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_, fcl::FCL_REAL& dist)
{
  DistanceData* cdata = static_cast<DistanceData*>(cdata_);
  const fcl::DistanceRequest& request = cdata->request;
  fcl::DistanceResult& result = cdata->result;

  if(cdata->done) { dist = result.min_distance; return true; }

  fcl::distance(o1, o2, request, result);
  
  dist = result.min_distance;

  if(dist <= 0) return true; // in collision or in touch

  return cdata->done;
}

bool defaultCollisionFunction_(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_)
{
    CollisionData* cdata = static_cast<CollisionData*>(cdata_);
    const fcl::CollisionRequest& request = cdata->request;
    fcl::CollisionResult& result = cdata->result;

    if(cdata->done) return true;

    fcl::collide(o1, o2, request, result);

    if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
        cdata->done = true;

    return cdata->done;
}

class FCLCollisionChecker
{
    private:
        float resolution_;

        // std::unique_ptr<fcl::DynamicAABBTreeCollisionManager> env_object_manager_;
        std::shared_ptr<fcl::DynamicAABBTreeCollisionManager> env_object_manager_;
        std::unordered_map<std::string, std::shared_ptr<fcl::OcTree>> object_octomap_cache_;
        std::unordered_map<std::string, std::shared_ptr<fcl::CollisionObject>> env_collision_object_cache_;

        // std::unique_ptr<fcl::DynamicAABBTreeCollisionManager> robot_object_manager_;
        std::shared_ptr<fcl::DynamicAABBTreeCollisionManager> robot_object_manager_;
        std::vector<std::shared_ptr<fcl::CollisionObject>> robot_collision_object_cache_;


        std::shared_ptr<fcl::OcTree> read_from_file_(const char* filename)
        {
            // This resolution does not seem to matter if a tree is read from
            // file.
            ///////////////////// LOAD OCTREE FROM FILE ////////////////////////////////
            resolution_ = 0.01;
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

            auto tree_fcl = std::make_shared<fcl::OcTree>(tree);
            return tree_fcl;
        }

    public:
        FCLCollisionChecker()
        {
            env_object_manager_ = std::make_unique<fcl::DynamicAABBTreeCollisionManager>();
            env_object_manager_->setup();
            robot_object_manager_ = std::make_unique<fcl::DynamicAABBTreeCollisionManager>();
            robot_object_manager_->setup();
        }
        ~FCLCollisionChecker()
        {
            std::cout << "FCLCollisionChecker destroyed" << std::endl;
            // if (nullptr != tree_)
            // {
            //     std::cout << "Attempting to delete .... " << std::endl;
            //     delete tree_;
            // }
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

        void insert_object_into_environment(const char* tag, const fcl::Vec3f& position)
        {
            std::cout << "Trying to insert \"" << tag << "\""
                      << " at " << position << std::endl;
            auto tree_fcl = object_octomap_cache_[tag];
            fcl::Transform3f tf0;
            tf0.setTranslation(fcl::Vec3f(position[0], position[1], position[2]));

            // fcl::CollisionObject *env_obj = new fcl::CollisionObject(geom, tf0);

            // auto geom = std::shared_ptr<fcl::CollisionGeometry>(tree_fcl.get());
            // std::shared_ptr<fcl::CollisionObject> env_obj = std::make_shared<fcl::CollisionObject>(geom, tf0);

            // Based on the discussion at https://stackoverflow.com/questions/13403490/passing-shared-ptrderived-as-shared-ptrbase
            // and the reference at http://www.cplusplus.com/reference/memory/static_pointer_cast/
            auto geom = std::static_pointer_cast<fcl::CollisionGeometry>(tree_fcl);
            std::shared_ptr<fcl::CollisionObject> env_obj = std::make_shared<fcl::CollisionObject>(geom, tf0);

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
        void update_object(const char* tag, const fcl::Matrix3f& R, const fcl::Vec3f& t)
        {
            auto env_obj = env_collision_object_cache_[tag];
            std::cout << "Current transform of \"" << tag << "\"" << " t : " << env_obj->getTranslation() << std::endl;
            std::cout << "Current transform of \"" << tag << "\"" << " R : " << env_obj->getRotation() << std::endl;
            env_obj->setTranslation(t);
            env_obj->setRotation(R);
            // env_obj->setTransform(...); not working
            std::cout << "Setting transform of \"" << tag << "\"" << " t : " << env_obj->getTranslation() << std::endl;
            std::cout << "Setting transform of \"" << tag << "\"" << " R : " << env_obj->getRotation() << std::endl;
            // env_object_manager_->unregisterObject(env_obj.get());
        }


        void load_robot_into_cache(const std::vector<fcl::Vec3f>& position, const std::vector<double>& radii)
        {
            robot_collision_object_cache_.clear();
            robot_collision_object_cache_.reserve(position.size());
            for (int k = 0; k < position.size(); ++k)
            {
                fcl::Transform3f tf0;
                tf0.setTranslation(position[k]);
                // sphere_k = std::shared_ptr<fcl::Sphere> s0(new fcl::Sphere(1.8));
                auto sphere_k = std::make_shared<fcl::Sphere>(radii[k]);
                std::shared_ptr<fcl::CollisionObject> rob_obj = std::make_shared<fcl::CollisionObject>(sphere_k, tf0);

                // std::string const name = std::string(tag) + std::to_string(k);
                std::cout << "Loading robot ball " << std::to_string(k) << std::endl;
                robot_collision_object_cache_.push_back(rob_obj);
            }
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
        void update_robot(const std::vector<fcl::Vec3f>& positions_new)
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
                }
                std::cout << "Updated robot."<< std::endl;
            }
        }

        double query_distance()
        {
            std::cout << "In query distance" << std::endl;

            DistanceData cdata;
            robot_object_manager_->distance(env_object_manager_.get(), (void *)&cdata, defaultDistanceFunction_);
            std::cout << "Min distance: " << cdata.result.min_distance << std::endl;
            return cdata.result.min_distance;
        }
        
        bool query_collision()
        {
            std::cout << "In query collision" << std::endl;

            CollisionData cdata;
            cdata.request.num_max_contacts = 1;
            robot_object_manager_->collide(env_object_manager_.get(), (void *)&cdata, defaultCollisionFunction_);

            bool const ret_val = cdata.result.isCollision();
            if (cdata.result.isCollision())
            {
                std::cout << "Has collision" << std::endl;
            }
            else
            {
                std::cout << "No collision" << std::endl;
            }
            
            return ret_val;
        }

        // void load_map(const char* filename)
        // {

        //     std::shared_ptr<fcl::OcTree> tree_fcl = std::shared_ptr<fcl::OcTree>(new fcl::OcTree(tree_));
        //     fcl::CollisionObject tree_obj((std::shared_ptr<fcl::CollisionGeometry>(tree_fcl)));
        //     fcl::DynamicAABBTreeCollisionManager* manager = new fcl::DynamicAABBTreeCollisionManager();
        //     // manager->octree_as_geometry_collide = true;

        //     // std::shared_ptr<fcl::Sphere> s0(new fcl::Sphere(1.8));
        //     fcl::Transform3f tf0;
        //     tf0.setTranslation(fcl::Vec3f(0, 0, 0.25));
        //     // fcl::CollisionObject cob0(s0, tf0);
        //     fcl::CollisionObject cob0((std::shared_ptr<fcl::CollisionGeometry>(tree_fcl)), tf0);
        //     std::vector<fcl::CollisionObject *> cobs;
        //     cobs.push_back(&cob0);
        //     manager->registerObjects(cobs);
        //     manager->setup();

        //     // std::shared_ptr<Sphere> s1(new Sphere(1.0));
        //     // Transform3f tf1;
        //     // tf1.setTranslation(Vec3f(2.1, 0, 0));
        //     // CollisionObject cob1(s1, tf1);

        //     CollisionData cdata;
        //     cdata.request.num_max_contacts = 1;
        //     manager->collide(&tree_obj, (void *)&cdata, defaultCollisionFunction_);

        //     if (cdata.result.isCollision())
        //     {
        //         std::cout << "Has collision" << std::endl;
        //     }
        //     else
        //     {
        //         std::cout << "No collision" << std::endl;
        //     }
        //     // delete tree_fcl;
        //     // delete tree_;
        //     // delete manager; // this seems to delete registered objects
        // }

        // void get_distance_and_closest_obstacle(const octomap::point3d &query, float &distance, octomap::point3d &closest_obstacle) const
        // {
        // }
        // void update_node(const bool value, const float x, const float y, const float z)
        // {
        // }
};