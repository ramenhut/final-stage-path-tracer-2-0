
#include "scene.h"

#include <algorithm>
#include <fstream>
#include "math/intersect.h"
#include "math/random.h"

namespace base {

const uint32 kMaxObjectCountPerNode = 2;

SceneBvhNode::SceneBvhNode(
    ::std::vector<::std::unique_ptr<Object>>* data_source,
    uint32 max_tree_depth) {
  tree_objects_ = data_source;
  max_tree_depth_ = max_tree_depth;
}

SceneBvhNode::SceneBvhNode(SceneBvhNode* parent_node) {
  depth_ = parent_node->depth_ + 1;
  parent_ = parent_node;
  is_leaf_node_ = true;
  tree_objects_ = parent_node->tree_objects_;
}

void SceneBvhNode::AllocateChild(int32 index) {
  children_[index].reset(new SceneBvhNode(this));
}

void SceneBvhNode::AddObject(uint32 index) { object_indices_.push_back(index); }

void SceneBvhNode::Subdivide() {
  if (depth_ >= max_tree_depth_) {
    return;
  }

  // This helps us prune sparse subdivision paths.
  if (depth_ >= min(max_tree_depth_ - depth_,
                    (log(object_indices_.size()) / log(8) + 0.5f) - 2.0f)) {
    return;
  }

  if (object_indices_.size() > kMaxObjectCountPerNode) {
    // Configure bounds for the child nodes that correspond to
    // quadrants of the current node's bounds.
    ConfigureChildren();

    // Compare all of the parent node's objects against the newly
    // created child. Add objects to the child that intersect it.
    for (uint32 j = 0; j < object_indices_.size(); j++) {
      uint32 object_index = object_indices_.at(j);
      Object* obj = tree_objects_->at(object_index).get();
      for (uint32 i = 0; i < 8; i++) {
        if (bounds_intersect_bounds(obj->GetBounds(),
                                    children_[i]->GetBounds())) {
          SceneBvhNode* node = static_cast<SceneBvhNode*>(children_[i].get());
          node->AddObject(object_index);
        }
      }
    }

    // We no longer need the current node to hold any object indices as
    // its children have fully taken them over.
    object_indices_.clear();

    // Give the newly constructed children a chance to subdivide.
    // For a multithreaded implementation we'll simply queue the
    // subdivide operation.
    for (uint8 j = 0; j < 8; j++) {
      children_[j]->Subdivide();
    }
  }
}

bool SceneBvhNode::Trace(const ray& trajectory,
                         ObjectCollision* hit_info) const {
  collision node_hit;
  if (!ray_intersect_bounds(aabb_, trajectory, &node_hit) ||
      node_hit.param > hit_info->param) {
    return false;
  }

  bool trace_result = false;

  if (IsLeafNode()) {
    ObjectCollision temp_obj_hit;
    // Traverse objects and return closest hit (if any)
    for (uint32 i = 0; i < object_indices_.size(); i++) {
      uint32 object_index = object_indices_.at(i);
      Object* obj = tree_objects_->at(object_index).get();
      if (obj->Trace(trajectory, &temp_obj_hit)) {
        if (temp_obj_hit.param < hit_info->param) {
          *hit_info = temp_obj_hit;
          trace_result = true;
        }
      }
    }
  } else {
    return TraceInternal(node_hit, trajectory, hit_info);
  }
  return trace_result;
}

void SceneBvh::BuildBvh(::std::vector<::std::unique_ptr<Object>>* data_source,
                        uint32 max_tree_depth) {
  if (!data_source->size()) {
    return;
  }

  tree_objects_ = data_source;
  max_tree_depth_ = max_tree_depth;
  root_node_.reset(new SceneBvhNode(data_source));

  bounds root_bounds;
  for (uint32 i = 0; i < data_source->size(); i++) {
    root_bounds += data_source->at(i)->GetBounds();
  }
  root_node_->SetBounds(root_bounds);

  for (uint32 i = 0; i < data_source->size(); i++) {
    root_node_->AddObject(i);
  }

  root_node_->Subdivide();
}

bool SceneBvh::Trace(const ray& trajectory, ObjectCollision* hit_info) const {
  if (root_node_.get()) {
    return root_node_->Trace(trajectory, hit_info);
  }
  return false;
}

const vector3 SceneBvh::GetCenter() const {
  if (root_node_.get()) {
    return root_node_->aabb_.query_center();
  }
  return vector3();
}

Scene::Scene() : is_tree_valid_(false) {
  sky_material_.reset(new LightMaterial(vector3(0, 0, 0)));
}

Camera* Scene::GetCamera(uint32 index) {
  if (index >= camera_list_.size()) {
    return nullptr;
  }
  return &camera_list_[index];
}

void Scene::SetSkyMaterial(::std::shared_ptr<LightMaterial> material) {
  sky_material_ = material;
}

const vector3 Scene::SampleSky(uint32 depth, const vector3& view) {
  vector2 tex_coords = sphere_map_texcoords(view);
  return sky_material_->Sample(depth, vector3(), vector3(), view, vector3(),
                               vector3(), vector3(), vector3(), tex_coords) *
         3.0;
}

MeshObject* Scene::AddMeshObject(const ::std::string& filename,
                                 bool invert_normals,
                                 const vector3& translation,
                                 const vector3& scale,
                                 const vector4& rotation) {
  is_tree_valid_ = false;
  object_list_.emplace_back(
      new MeshObject(filename, invert_normals, translation, scale, rotation));
  return reinterpret_cast<MeshObject*>(object_list_.back().get());
}

SphericalObject* Scene::AddSphericalObject(const vector3& origin,
                                           float32 radius) {
  is_tree_valid_ = false;
  object_list_.emplace_back(new SphericalObject(origin, radius));
  return reinterpret_cast<SphericalObject*>(object_list_.back().get());
}

PlanarObject* Scene::AddPlanarObject(const plane& data) {
  is_tree_valid_ = false;
  object_list_.emplace_back(new PlanarObject(data));
  return reinterpret_cast<PlanarObject*>(object_list_.back().get());
}

DiscObject* Scene::AddDiscObject(const vector3& origin, const vector3& normal,
                                 float32 radius) {
  is_tree_valid_ = false;
  object_list_.emplace_back(new DiscObject(origin, normal, radius));
  return reinterpret_cast<DiscObject*>(object_list_.back().get());
}

CuboidObject* Scene::AddCuboidObject(const vector3& origin, float32 width,
                                     float32 height, float32 depth) {
  is_tree_valid_ = false;
  object_list_.emplace_back(new CuboidObject(origin, width, height, depth));
  return reinterpret_cast<CuboidObject*>(object_list_.back().get());
}

QuadObject* Scene::AddQuadObject(const vector3& origin, const vector3& normal,
                                 float32 width, float32 height) {
  is_tree_valid_ = false;
  object_list_.emplace_back(new QuadObject(origin, normal, width, height));
  return reinterpret_cast<QuadObject*>(object_list_.back().get());
}

QuadObject* Scene::AddQuadObject(const vector3& position, const vector3& u,
                                 const vector3& v) {
  is_tree_valid_ = false;
  object_list_.emplace_back(new QuadObject(position, u, v));
  return reinterpret_cast<QuadObject*>(object_list_.back().get());
}

void Scene::Optimize() {
  is_tree_valid_ = false;
  // Compute the ideal maximum depth based on the scene object count.
  // If this is non-zero, move forward with scene tree construction.
  int32 ideal_depth = (log(object_list_.size()) / log(8) + 0.5) - 2;
  if (ideal_depth > 0) {
    object_tree_.BuildBvh(&object_list_, ideal_depth);
    is_tree_valid_ = true;
  }
}

bool Scene::Trace(const ray& trajectory, ObjectCollision* hit_info) {
  bool collision_detected = false;

  if (!is_tree_valid_) {
    for (auto& i : object_list_) {
      collision_detected |= i.get()->Trace(trajectory, hit_info);
    }
  } else {
    collision_detected |= object_tree_.Trace(trajectory, hit_info);
  }

  plane collision_plane =
      calculate_plane(hit_info->surface_normal, hit_info->point);
  // If we've struck a back facing surface then invert our normal and
  // flag this collision as internal. This provides our materials with
  // a consistent orientation while also allowing them to handle internal
  // collisions appropriately.
  if (plane_distance(collision_plane, trajectory.start) < 0.0) {
    hit_info->surface_normal *= -1.0;
    hit_info->is_internal = true;
  }
  return collision_detected;
}

void Scene::ParseMaterial(
    const char* material_name, ::std::ifstream* input_file,
    ::std::map<::std::string, ::std::shared_ptr<DiffuseMaterial>>*
        material_list) {
  vector3 color;
  vector3 emission;
  float32 metallic = 0;
  float32 roughness = 0;
  float32 refraction_index = 1;
  float32 texture_scale = 1.0;
  int32 brdf = 0;
  float32 frostiness = 0.0;
  float32 reflectivity = 0.1;
  char texture_name[MAX_PATH] = {0};
  ::std::string input_line;

  while (getline(*input_file, input_line)) {
    if (strchr(input_line.c_str(), '}')) break;

    sscanf(input_line.c_str(), " color %f %f %f", &color.x, &color.y, &color.z);
    sscanf(input_line.c_str(), " emission %f %f %f", &emission.x, &emission.y,
           &emission.z);
    sscanf(input_line.c_str(), " metallic %f", &metallic);
    sscanf(input_line.c_str(), " roughness %f", &roughness);
    sscanf(input_line.c_str(), " index %f", &refraction_index);
    sscanf_s(input_line.c_str(), " texture %s", texture_name, MAX_PATH);
    sscanf(input_line.c_str(), " texture_scale %f", &texture_scale);
    sscanf(input_line.c_str(), " brdf %i", &brdf);
    sscanf(input_line.c_str(), " frostiness %f", &frostiness);
    sscanf(input_line.c_str(), " reflectivity %f", &reflectivity);
  }

  ::std::shared_ptr<DiffuseMaterial> material;

  if (emission.x || emission.y || emission.z) {
    material = ::std::make_shared<::base::LightMaterial>(emission);
  } else if (roughness) {
    material = ::std::make_shared<::base::CeramicMaterial>(color, roughness);
  } else if (metallic) {
    if (metallic == 1.0) {
      material = ::std::make_shared<::base::MirrorMaterial>(color);
    } else {
      material = ::std::make_shared<::base::MetalMaterial>(color, metallic);
    }
  } else if (brdf == 1) {
    material = ::std::make_shared<::base::LiquidMaterial>(
        color, refraction_index, reflectivity);
  } else if (brdf == 2) {
    material = ::std::make_shared<::base::GlassMaterial>(
        color, refraction_index, reflectivity, frostiness);
  } else {
    material = ::std::make_shared<::base::DiffuseMaterial>(color);
  }

  if (strlen(texture_name) && strcmp(texture_name, "None") != 0) {
    material->LoadDiffuseTexture(texture_name, texture_scale);
  }

  (*material_list)[::std::string(material_name)] = material;
}

void Scene::ParseSphere(
    ::std::ifstream* input_file,
    ::std::map<::std::string, ::std::shared_ptr<DiffuseMaterial>>*
        material_list) {
  float32 radius;
  vector3 position;
  ::std::string input_line;
  char material_name[MAX_PATH] = {0};

  while (getline(*input_file, input_line)) {
    if (strchr(input_line.c_str(), '}')) break;
    sscanf_s(input_line.c_str(), " material %s", material_name, MAX_PATH);
    sscanf(input_line.c_str(), " position %f %f %f", &position.x, &position.y,
           &position.z);
    sscanf(input_line.c_str(), " radius %f", &radius);
  }

  ::std::string object_material(material_name);
  ::base::Object* sphere_object = AddSphericalObject(position, radius);

  if (sphere_object) {
    if (object_material.length() && material_list->count(object_material)) {
      sphere_object->SetMaterial((*material_list)[object_material]);
    }
  }
}

void Scene::ParseCamera(::std::ifstream* input_file) {
  Camera scene_camera;
  vector3 position = scene_camera.origin;
  vector3 target = scene_camera.target;
  float32 fov = scene_camera.fov_y;
  float32 aperture = scene_camera.aperture_size;
  float32 focal_depth = scene_camera.focal_depth;
  ::std::string input_line;

  while (getline(*input_file, input_line)) {
    if (strchr(input_line.c_str(), '}')) break;

    sscanf(input_line.c_str(), " position %f %f %f", &position.x, &position.y,
           &position.z);
    sscanf(input_line.c_str(), " target %f %f %f", &target.x, &target.y,
           &target.z);
    sscanf(input_line.c_str(), " fov %f", &fov);
    sscanf(input_line.c_str(), " aperture %f", &aperture);
    sscanf(input_line.c_str(), " focal_depth %f", &focal_depth);
  }

  scene_camera.origin = position;
  scene_camera.target = target;
  scene_camera.fov_y = fov;
  scene_camera.aperture_size = aperture;
  scene_camera.focal_depth = focal_depth;
  camera_list_.push_back(scene_camera);
}

void Scene::ParseSky(
    ::std::ifstream* input_file,
    ::std::map<::std::string, ::std::shared_ptr<DiffuseMaterial>>*
        material_list) {
  ::std::string input_line;
  char material_name[MAX_PATH] = {0};

  while (getline(*input_file, input_line)) {
    if (strchr(input_line.c_str(), '}')) break;
    sscanf_s(input_line.c_str(), " material %s", material_name, MAX_PATH);
  }

  ::std::string object_material(material_name);

  if (object_material.length() && material_list->count(object_material)) {
    SetSkyMaterial(std::static_pointer_cast<LightMaterial>(
        (*material_list)[object_material]));
  }
}

void Scene::ParseQuad(
    ::std::ifstream* input_file,
    ::std::map<::std::string, ::std::shared_ptr<DiffuseMaterial>>*
        material_list) {
  vector3 position;
  vector3 normal;
  float32 width;
  float32 height;
  ::std::string input_line;
  char quad_material[MAX_PATH] = {0};

  while (getline(*input_file, input_line)) {
    if (strchr(input_line.c_str(), '}')) break;
    sscanf_s(input_line.c_str(), " material %s", quad_material, MAX_PATH);
    sscanf(input_line.c_str(), " position %f %f %f", &position.x, &position.y,
           &position.z);
    sscanf(input_line.c_str(), " normal %f %f %f", &normal.x, &normal.y,
           &normal.z);
    sscanf(input_line.c_str(), " width %f", &width);
    sscanf(input_line.c_str(), " height %f", &height);
  }

  ::std::string object_material(quad_material);
  ::base::Object* quad_object = AddQuadObject(position, normal, width, height);

  if (quad_object) {
    if (object_material.length() && material_list->count(object_material)) {
      quad_object->SetMaterial((*material_list)[object_material]);
    }
  }
}

void Scene::ParseCuboid(
    ::std::ifstream* input_file,
    ::std::map<::std::string, ::std::shared_ptr<DiffuseMaterial>>*
        material_list) {
  vector3 position;
  float32 width;
  float32 height;
  float32 depth;
  vector4 local_rotation;
  ::std::string input_line;
  char cuboid_material[MAX_PATH] = {0};

  while (getline(*input_file, input_line)) {
    if (strchr(input_line.c_str(), '}')) break;
    sscanf_s(input_line.c_str(), " material %s", cuboid_material, MAX_PATH);
    sscanf(input_line.c_str(), " position %f %f %f", &position.x, &position.y,
           &position.z);
    sscanf(input_line.c_str(), " width %f", &width);
    sscanf(input_line.c_str(), " height %f", &height);
    sscanf(input_line.c_str(), " depth %f", &depth);
    sscanf(input_line.c_str(), " rotation %f %f %f %f", &local_rotation.x,
           &local_rotation.y, &local_rotation.z, &local_rotation.w);
  }

  ::std::string object_material(cuboid_material);
  ::base::CuboidObject* cuboid_object =
      AddCuboidObject(position, width, height, depth);

  if (cuboid_object) {
    if (object_material.length() && material_list->count(object_material)) {
      cuboid_object->SetMaterial((*material_list)[object_material]);
    }
    cuboid_object->Rotate(vector3(local_rotation), local_rotation.w);
  }
}

void Scene::ParseMesh(
    ::std::ifstream* input_file,
    ::std::map<::std::string, ::std::shared_ptr<DiffuseMaterial>>*
        material_list) {
  ::std::string input_line;
  char mesh_filename[MAX_PATH] = {0};
  char mesh_material[MAX_PATH] = {0};
  vector3 local_translation;
  vector3 local_scale(1, 1, 1);
  vector4 local_rotation;

  while (getline(*input_file, input_line)) {
    if (strchr(input_line.c_str(), '}')) break;

    sscanf_s(input_line.c_str(), " file %s", mesh_filename, MAX_PATH);
    sscanf_s(input_line.c_str(), " material %s", mesh_material, MAX_PATH);
    sscanf(input_line.c_str(), " translation %f %f %f", &local_translation.x,
           &local_translation.y, &local_translation.z);
    sscanf(input_line.c_str(), " scale %f %f %f", &local_scale.x,
           &local_scale.y, &local_scale.z);
    sscanf(input_line.c_str(), " rotation %f %f %f %f", &local_rotation.x,
           &local_rotation.y, &local_rotation.z, &local_rotation.w);
  }

  if (strlen(mesh_filename)) {
    ::std::string object_material(mesh_material);
    ::base::Object* mesh_object =
        AddMeshObject(::std::string(mesh_filename), false, local_translation,
                      local_scale, local_rotation);
    if (mesh_object) {
      if (object_material.length() && material_list->count(object_material)) {
        mesh_object->SetMaterial((*material_list)[object_material]);
      }
    }
  }
}

bool Scene::LoadScene(const ::std::string& filename) {
  ::std::map<::std::string, ::std::shared_ptr<DiffuseMaterial>> material_list;
  ::std::ifstream input_file(filename, ::std::ios::in);

  if (!input_file.is_open()) {
    printf("Failed to read scene file %s.\n", filename.c_str());
    return false;
  }

  ::std::string input_line;
  while (getline(input_file, input_line)) {
    if (input_line[0] == '#') continue;

    char material_name[MAX_PATH] = {0};

    if (sscanf_s(input_line.c_str(), " material %s", material_name, MAX_PATH) ==
        1) {
      ParseMaterial(material_name, &input_file, &material_list);
    } else if (strstr(input_line.c_str(), "sphere")) {
      ParseSphere(&input_file, &material_list);
    } else if (strstr(input_line.c_str(), "camera")) {
      ParseCamera(&input_file);
    } else if (strstr(input_line.c_str(), "sky")) {
      ParseSky(&input_file, &material_list);
    } else if (strstr(input_line.c_str(), "quad")) {
      ParseQuad(&input_file, &material_list);
    } else if (strstr(input_line.c_str(), "cuboid")) {
      ParseCuboid(&input_file, &material_list);
    } else if (strstr(input_line.c_str(), "mesh")) {
      ParseMesh(&input_file, &material_list);
    }
  }
  input_file.close();

  Optimize();

  printf("Scene file %s loaded successfully.\n", filename.c_str());

  return true;
}

}  // namespace base