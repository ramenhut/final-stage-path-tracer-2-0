/*
//
// Copyright (c) 1998-2019 Joe Bertolami. All Right Reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//   AND ANY EXPRESS OR IMPLIED WARRANTIES, CLUDG, BUT NOT LIMITED TO, THE
//   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//   ARE DISCLAIMED.  NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//   LIABLE FOR ANY DIRECT, DIRECT, CIDENTAL, SPECIAL, EXEMPLARY, OR
//   CONSEQUENTIAL DAMAGES (CLUDG, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
//   GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSESS TERRUPTION)
//   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER  CONTRACT, STRICT
//   LIABILITY, OR TORT (CLUDG NEGLIGENCE OR OTHERWISE) ARISG  ANY WAY  OF THE
//   USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Additional Information:
//
//   For more information, visit http://www.bertolami.com.
//
*/

#ifndef __SCENE_H__
#define __SCENE_H__

#include <map>
#include <string>

#include "camera.h"
#include "frame.h"
#include "material.h"
#include "math/base.h"
#include "mesh.h"
#include "object.h"

namespace base {

const uint32 kMaxSubdivisionDepth = 2;

class SceneBvhNode
    : public BaseBvhNode<::std::vector<Object>*, ObjectCollision> {
 public:
  // Copy ctor that initializes the node.
  explicit SceneBvhNode(SceneBvhNode* parent_node);
  // Called by parent nodes to insert objects by index into a node.
  void AddObject(uint32 index);
  // Checks if the node requires subdivision and, if necessary, allocates
  // child nodes and initiates recursive subdivision.
  void Subdivide() override;
  // Traces a ray through the node and returns collision information.
  bool Trace(const ray& trajectory, ObjectCollision* hit_info) const override;

 protected:
  friend class SceneBvh;
  SceneBvhNode(::std::vector<::std::unique_ptr<Object>>* data_source,
               uint32 max_tree_depth = kMaxSubdivisionDepth);
  // Called to allocate a node at children_[index] with type according to a
  // derived class.
  virtual void AllocateChild(int32 index) override;
  // External list of objects referenced by this node.
  ::std::vector<::std::unique_ptr<Object>>* tree_objects_;
  // Object indices directly managed by this node.
  ::std::vector<uint32> object_indices_;
  // Defines the maximum allowable depth for this tree. This is
  // computed based on the number of objects in the scene.
  uint32 max_tree_depth_;
};

class SceneBvh {
  // Root node of the tree.
  ::std::unique_ptr<SceneBvhNode> root_node_;
  // External list of objects covered by this node.
  ::std::vector<::std::unique_ptr<Object>>* tree_objects_;
  // Defines the maximum allowable depth for this tree. This is
  // computed based on the number of objects in the scene.
  uint32 max_tree_depth_;

 public:
  const vector3 GetCenter() const;
  void BuildBvh(::std::vector<::std::unique_ptr<Object>>* data_source,
                uint32 max_tree_depth = kMaxSubdivisionDepth);
  bool Trace(const ray& trajectory, ObjectCollision* hit_info) const;
};

class Scene {
 public:
  Scene();
  // Loads a scene description from a file (.scene format).
  bool LoadScene(const ::std::string& filename);
  // Loads a mesh object from a file and adds it to the scene.
  // Returns a pointer to the newly added object.
  MeshObject* AddMeshObject(const ::std::string& filename,
                            bool invert_normals = false,
                            const vector3& translation = vector3(0, 0, 0),
                            const vector3& scale = vector3(1, 1, 1),
                            // <x, y, z> is the axis, <w> is the angle.
                            const vector4& rotation = vector4(0, 0, 0, 0));
  // Adds a spherical object to the scene. Returns a pointer to the new object.
  SphericalObject* AddSphericalObject(const vector3& origin, float32 radius);
  // Adds a disc object to the scene. Returns a pointer to the new object.
  DiscObject* AddDiscObject(const vector3& origin, const vector3& normal,
                            float32 radius);
  // Adds a planar object to the scene. Returns a pointer to the new object.
  PlanarObject* AddPlanarObject(const plane& data);
  // Adds a cubic object to the scene. Returns a pointer to the new object.
  CuboidObject* AddCuboidObject(const vector3& origin, float32 width,
                                float32 height, float32 depth);
  // Adds a quad object based on origin, normal, and dimensions.
  QuadObject* AddQuadObject(const vector3& origin, const vector3& normal,
                            float32 width, float32 height);
  // Adds a quad object based on an upper left position, a right-u vector,
  // and a down-v vector.
  QuadObject* AddQuadObject(const vector3& position, const vector3& u,
                            const vector3& v);
  // Sets the default sky material for the scene.
  void SetSkyMaterial(::std::shared_ptr<LightMaterial> material);
  // Retrieves the sky material.
  LightMaterial* GetSkyMaterial() { return sky_material_.get(); }

  // Traces a ray through the scene and determines collision info.
  // Returns true if a collision was detected. False otherwise.
  bool Trace(const ray& trajectory, ObjectCollision* hit_info);
  // Returns the sky color given a view direction.
  const vector3 SampleSky(uint32 depth, const vector3& view);
  // Builds a bvh from the list of allocated scene objects. If the scene
  // contains at least 2 objects, the scene bvh will be used for tracing.
  void Optimize();
  // Returns the number of cameras preallocated in the scene.
  uint32 GetCameraCount() { return camera_list_.size(); }
  // Returns a pointer to a scene camera by index.
  Camera* GetCamera(uint32 index);

 private:
  // The sky material.
  ::std::shared_ptr<LightMaterial> sky_material_;
  // List of cameras that enables scene files to define camera sets. The
  // consumer of this class is still responsible for selecting which camera
  // (if any) to use during a trace.
  ::std::vector<Camera> camera_list_;
  // List of objects in the scene.
  ::std::vector<::std::unique_ptr<Object>> object_list_;
  // The acceleration structure for the scene. Used to speed up traces.
  SceneBvh object_tree_;
  // Indicates whether the object_tree_ should be used for tracing. Adding
  // or moving objects in the object_list_ will invalidate the tree until
  // the next call to Optimize().
  bool is_tree_valid_;

  // Scene file parsing
  void ParseMaterial(
      const char* material_name, ::std::ifstream* input_file,
      ::std::map<::std::string, ::std::shared_ptr<DiffuseMaterial>>*
          material_list);
  void ParseSphere(
      ::std::ifstream* input_file,
      ::std::map<::std::string, ::std::shared_ptr<DiffuseMaterial>>*
      material_list);
  void ParseCamera(
      ::std::ifstream* input_file);
  void ParseSky(
      ::std::ifstream* input_file,
      ::std::map<::std::string, ::std::shared_ptr<DiffuseMaterial>>*
      material_list);
  void ParseQuad(
      ::std::ifstream* input_file,
      ::std::map<::std::string, ::std::shared_ptr<DiffuseMaterial>>*
      material_list);
  void ParseCuboid(
      ::std::ifstream* input_file,
      ::std::map<::std::string, ::std::shared_ptr<DiffuseMaterial>>*
      material_list);
  void ParseMesh(
      ::std::ifstream* input_file,
      ::std::map<::std::string, ::std::shared_ptr<DiffuseMaterial>>*
      material_list);
};

}  // namespace base

#endif  // __SCENE_H__