#include "voxel_generator_island.h"
#include <core/core_string_names.h>
#include <core/engine.h>

VoxelGeneratorIsland::VoxelGeneratorIsland() {
  _parameters.steps_mask = 0xffffffff;
  _parameters.voxel_scale = 1.f;
  _parameters.sea_height = 0.f;
  _parameters.max_height = 160.f;
  _parameters.radius = 512.f;
  _parameters.radius_cutoff_power = 5;
  _parameters.radius_cutoff_scale = 1.f;
  _parameters.ground_ratio = 0.1f;
#ifdef TOOLS_ENABLED
	if (Engine::get_singleton()->is_editor_hint()) {
		// Have one by default in editor
		Ref<OpenSimplexNoise> noise;
		noise.instance();
		set_height_noise(noise);
	}
#endif
}

VoxelGeneratorIsland::~VoxelGeneratorIsland() {
}

float VoxelGeneratorIsland::Parameters::read_height_noise(float x, float z) const {
  if (height_noise.is_null())
    return 0.5f;
  static const float open_simplex_rescale = 1.8f;
  static const float open_simplex_period_scale = 1.f / 4.f;
  return height_noise->get_noise_2d(x * open_simplex_period_scale, z * open_simplex_period_scale) * 0.5f * open_simplex_rescale + 0.5f;
}

struct Affine {
  float a, b;
  float operator()(float x) const { return a * x + b; }
};

Affine negative_extrapolation(Ref<Curve>& curve) {
  if (curve.is_null())
    return Affine{0.f, 0.f};
  const float dx = 1.f / curve->get_bake_resolution();
  const float y1 = curve->interpolate_baked(0.f);
  const float y2 = curve->interpolate_baked(dx);
  return Affine{(y2 - y1) / dx, y1};
}

float apply_height_curve(float h, Ref<Curve>& curve, Affine const& negative_extrapolate) {
  if (curve.is_null())
    return h;
  return h > 0.f ? curve->interpolate_baked(h) : negative_extrapolate(h);
}

Affine ground_ratio_function(float ground_ratio) {
  // y = (x - ground_ratio) / (1 - ground_ratio)
  // y = x / (1 - gr) - gr / (1 - gr)
  return Affine{1.f / (1.f - ground_ratio), -ground_ratio / (1.f - ground_ratio)};
}
#define IFSM() if (((1<<step++) & params.steps_mask) != 0)
VoxelGenerator::Result VoxelGeneratorIsland::generate_block(VoxelBlockRequest& input) {
  Parameters params;
  {
    RWLockRead rlock(_parameters_lock);
    params = _parameters;
  }

  // params
  VoxelBufferInternal& out_buffer = input.voxel_buffer;
  const Vector3i origin = input.origin_in_voxels;
  const Vector3i size = out_buffer.get_size();
  const int stride = 1 << input.lod;
  const int channel = VoxelBufferInternal::CHANNEL_SDF;

  // pre-compute
  const Affine neg_curve_extrap0 = negative_extrapolation(params.curve_biome0);
  //const Affine neg_curve_extrap1 = negative_extrapolation(params.curve_biome1);
  const Affine ground_ratio = ground_ratio_function(params.ground_ratio);
  const float inv_sq_radius = 1.f / (params.radius * params.radius);
  const auto height_func = [&](int gx, int gz) {
    const float fx = gx * params.voxel_scale;
    const float fz = gz * params.voxel_scale;
    const float sq_xz_dist = fx * fx + fz * fz;
    const float norm_sq_dist = sq_xz_dist * inv_sq_radius;
    const float island_shape = Math::pow(norm_sq_dist, params.radius_cutoff_power / 2) * params.radius_cutoff_scale;
    float h = 1.f;
    int step = 0;
    IFSM() h = params.read_height_noise(fx, fz);
    IFSM() h = h - island_shape; 
    IFSM() h = ground_ratio(h);
    IFSM() h = apply_height_curve(h, params.curve_biome0, neg_curve_extrap0);
    h = h * params.max_height;
    return h;
  };
  const auto get_sdf = [&](float y, float c, float n) {
    if (c < n ? !(c < y && y < n) : !(n < y && y < c))
      return y > c ? 1.f : -1.f;
    //const float sign = y > c ? 1.f : -1.f;
    const Vector2 slope_t{1, n - c};
    const Vector2 up{0, y - c}; // n*y-n*c-c*y+c*c = up.dot(slope_t) <= 0)
    const float dist_to_slope = slope_t.cross(up) / slope_t.length();
    return dist_to_slope;
  };
  int gz = origin.z;
  for (int z = 0; z < size.z; ++z, gz += stride) {
    //const float fz = gz * params.voxel_scale;
    int gx = origin.x;
    for (int x = 0; x < size.x; ++x, gx += stride) {
      //const float fx = gx * params.voxel_scale;
      int gy = origin.y;
      const float c = height_func(gx, gz);
      const float xn = height_func(gx-1, gz);
      const float xp = height_func(gx+1, gz);
      const float zn = height_func(gx, gz-1);
      const float zp = height_func(gx, gz+1);
      for (int y = 0; y < size.y; ++y, gy += stride) {
        const float fy = gy * params.voxel_scale - params.sea_height;
        //const float sdf = fy - h;
        float sdf = fy - c, sdf_n;
        sdf_n = get_sdf(fy, c, xn); if (fabs(sdf_n) < fabs(sdf)) sdf = sdf_n;
        sdf_n = get_sdf(fy, c, xp); if (fabs(sdf_n) < fabs(sdf)) sdf = sdf_n;
        sdf_n = get_sdf(fy, c, zn); if (fabs(sdf_n) < fabs(sdf)) sdf = sdf_n;
        sdf_n = get_sdf(fy, c, zp); if (fabs(sdf_n) < fabs(sdf)) sdf = sdf_n;
        const float clamped_sdf = clamp(sdf, -1.f, 1.f);
        out_buffer.set_voxel_f(clamped_sdf, x, y, z, channel);
      } // for y
    } // for x
  } // for z
  out_buffer.compress_uniform_channels();
  VoxelGenerator::Result result;
  return result;
}

int VoxelGeneratorIsland::get_used_channels_mask() const {
  return (1 << VoxelBufferInternal::CHANNEL_SDF);
}

PARAMETER_SET_GET_DEF(int, steps_mask);
PARAMETER_SET_GET_DEF(float, voxel_scale);
PARAMETER_SET_GET_DEF(float, sea_height);
PARAMETER_SET_GET_DEF(float, max_height);
PARAMETER_SET_GET_DEF(float, radius);
PARAMETER_SET_GET_DEF(float, radius_cutoff_power);
PARAMETER_SET_GET_DEF(float, radius_cutoff_scale);
PARAMETER_SET_GET_DEF(float, ground_ratio);
RES_PARAMETER_SET_GET_DEF(OpenSimplexNoise, biome_noise);
RES_PARAMETER_SET_GET_DEF(OpenSimplexNoise, height_noise);
RES_PARAMETER_SET_GET_DEF(Curve, curve_biome0);
RES_PARAMETER_SET_GET_DEF(Curve, curve_biome1);

void VoxelGeneratorIsland::_bind_methods() {
  PARAMETER_BIND(INT, steps_mask);
  PARAMETER_BIND(REAL, voxel_scale);
  PARAMETER_BIND(REAL, sea_height);
  PARAMETER_BIND(REAL, max_height);
  PARAMETER_BIND(REAL, radius);
  PARAMETER_BIND(REAL, radius_cutoff_power);
  PARAMETER_BIND(REAL, radius_cutoff_scale);
  PARAMETER_BIND(REAL, ground_ratio);
  RES_PARAMETER_BIND(OpenSimplexNoise, biome_noise);
  RES_PARAMETER_BIND(OpenSimplexNoise, height_noise);
  RES_PARAMETER_BIND(Curve, curve_biome0);
  RES_PARAMETER_BIND(Curve, curve_biome1);
}
