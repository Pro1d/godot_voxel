#ifndef VOXEL_GENERATOR_ISLAND_H
#define VOXEL_GENERATOR_ISLAND_H

#include "../../storage/voxel_buffer.h"
#include "../voxel_generator.h"

#include <modules/opensimplex/open_simplex_noise.h>

// for INT / FLOAT / ...
#define PARAMETER_SET_GET_DEF(TYPE, NAME) \
  TYPE VoxelGeneratorIsland::get_##NAME() const { \
    RWLockRead rlock(_parameters_lock); \
    return _parameters.NAME; \
  } \
  void VoxelGeneratorIsland::set_##NAME(TYPE NAME) { \
    RWLockWrite wlock(_parameters_lock); \
    _parameters.NAME = NAME; \
  }

#define PARAMETER_SET_GET_DECL(TYPE, NAME) \
  TYPE get_##NAME() const; \
  void set_##NAME(TYPE NAME);

#define PARAMETER_BIND(TYPE, NAME) \
	ClassDB::bind_method(D_METHOD("set_"#NAME, #NAME), &VoxelGeneratorIsland::set_##NAME); \
	ClassDB::bind_method(D_METHOD("get_"#NAME), &VoxelGeneratorIsland::get_##NAME); \
	ADD_PROPERTY(PropertyInfo(Variant::TYPE, #NAME), "set_"#NAME, "get_"#NAME);

// for Resource: Curve, OpenSimplexNoise, ...
#define RES_PARAMETER_SET_GET_DEF(TYPE, NAME) \
  Ref<TYPE> VoxelGeneratorIsland::get_##NAME() const { \
    RWLockRead rlock(_parameters_lock); \
    return _parameters.NAME; \
  } \
  void VoxelGeneratorIsland::set_##NAME(Ref<TYPE> NAME) { \
    if (_ref_##NAME == NAME) { \
      return; \
    } \
    if (_ref_##NAME.is_valid()) { \
      _ref_##NAME->disconnect(CoreStringNames::get_singleton()->changed, this, "_on_"#NAME"_changed"); \
    } \
    _ref_##NAME = NAME; \
    if (_ref_##NAME.is_valid()) { \
      _ref_##NAME->connect(CoreStringNames::get_singleton()->changed, this, "_on_"#NAME"_changed"); \
      RWLockWrite wlock(_parameters_lock); \
      _parameters.NAME = _ref_##NAME->duplicate(); \
      _on_res_parameter_updated(_parameters.NAME); \
    } \
    else { \
      RWLockWrite wlock(_parameters_lock); \
      _parameters.NAME.unref(); \
    } \
  } \
  void VoxelGeneratorIsland::_on_##NAME##_changed() { \
    ERR_FAIL_COND(_ref_##NAME.is_null()); \
    RWLockWrite wlock(_parameters_lock); \
    _parameters.NAME = _ref_##NAME->duplicate(); \
    _on_res_parameter_updated(_parameters.NAME); \
  }

#define RES_PARAMETER_SET_GET_DECL(TYPE, NAME) \
  Ref<TYPE> get_##NAME() const; \
  void set_##NAME(Ref<TYPE> NAME); \
  void _on_##NAME##_changed(); \
  Ref<TYPE> _ref_##NAME;

#define RES_PARAMETER_BIND(TYPE, NAME) \
	ClassDB::bind_method(D_METHOD("set_"#NAME, #NAME), &VoxelGeneratorIsland::set_##NAME); \
	ClassDB::bind_method(D_METHOD("get_"#NAME), &VoxelGeneratorIsland::get_##NAME); \
  ClassDB::bind_method(D_METHOD("_on_"#NAME"_changed"), &VoxelGeneratorIsland::_on_##NAME##_changed); \
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, #NAME, PROPERTY_HINT_RESOURCE_TYPE, #TYPE), "set_"#NAME, "get_"#NAME);

class VoxelGeneratorIsland : public VoxelGenerator {
  GDCLASS(VoxelGeneratorIsland, VoxelGenerator);
public:
  VoxelGeneratorIsland();
  ~VoxelGeneratorIsland();

  Result generate_block(VoxelBlockRequest& input) override;

  int get_used_channels_mask() const override;

  PARAMETER_SET_GET_DECL(int, steps_mask);
  PARAMETER_SET_GET_DECL(float, voxel_scale);
  PARAMETER_SET_GET_DECL(float, sea_height);
  PARAMETER_SET_GET_DECL(float, max_height);
  PARAMETER_SET_GET_DECL(float, radius);
  PARAMETER_SET_GET_DECL(float, radius_cutoff_power);
  PARAMETER_SET_GET_DECL(float, radius_cutoff_scale);
  PARAMETER_SET_GET_DECL(float, ground_ratio);
  RES_PARAMETER_SET_GET_DECL(OpenSimplexNoise, biome_noise);
  RES_PARAMETER_SET_GET_DECL(OpenSimplexNoise, height_noise);
  RES_PARAMETER_SET_GET_DECL(Curve, curve_biome0);
  RES_PARAMETER_SET_GET_DECL(Curve, curve_biome1);

private:
  void _on_res_parameter_updated(Ref<Curve>& ref) { ref->bake(); }
  void _on_res_parameter_updated(Ref<OpenSimplexNoise>& ref) {}

  static void _bind_methods();

  struct Parameters {
    int steps_mask;
    float voxel_scale;
    float sea_height;
    float max_height;
    float radius;
    float radius_cutoff_power;
    float radius_cutoff_scale;
    float ground_ratio;
    Ref<OpenSimplexNoise> biome_noise;
    Ref<OpenSimplexNoise> height_noise;
    Ref<Curve> curve_biome0;
    Ref<Curve> curve_biome1;
    // curves[biome] (extrapolate curves tangeante under sea)
    // ?rock noise

    float read_height_noise(float x, float z) const;
  };


  Parameters _parameters;
  RWLock _parameters_lock;
};

#endif // VOXEL_GENERATOR_WAVES_H
