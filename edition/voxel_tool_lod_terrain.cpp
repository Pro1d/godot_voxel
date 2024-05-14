#include "voxel_tool_lod_terrain.h"
#include "../constants/voxel_string_names.h"
#include "../terrain/voxel_lod_terrain.h"
#include "../util/funcs.h"
#include "../util/godot/funcs.h"
#include "../util/island_finder.h"
#include "../util/voxel_raycast.h"
#include "funcs.h"

#include <scene/3d/collision_shape.h>
#include <scene/3d/mesh_instance.h>
#include <scene/3d/physics_body.h>
#include <scene/main/timer.h>

VoxelToolLodTerrain::VoxelToolLodTerrain(VoxelLodTerrain *terrain) :
		_terrain(terrain) {
	ERR_FAIL_COND(terrain == nullptr);
	// At the moment, only LOD0 is supported.
	// Don't destroy the terrain while a voxel tool still references it

	_crease_noise.instance();
	_crease_noise->set_octaves(1);
	_crease_noise->set_period(4.0);
	_edition_speed = 0.2f;
	_edition_normal = Vector3(0, 1, 0);
}

bool VoxelToolLodTerrain::is_area_editable(const Box3i &box) const {
	ERR_FAIL_COND_V(_terrain == nullptr, false);
	return _terrain->is_area_editable(box);
}

template <typename Volume_F>
float get_sdf_interpolated(const Volume_F &f, Vector3 pos) {
	const Vector3i c = Vector3i::from_floored(pos);

	const float s000 = f(Vector3i(c.x, c.y, c.z));
	const float s100 = f(Vector3i(c.x + 1, c.y, c.z));
	const float s010 = f(Vector3i(c.x, c.y + 1, c.z));
	const float s110 = f(Vector3i(c.x + 1, c.y + 1, c.z));
	const float s001 = f(Vector3i(c.x, c.y, c.z + 1));
	const float s101 = f(Vector3i(c.x + 1, c.y, c.z + 1));
	const float s011 = f(Vector3i(c.x, c.y + 1, c.z + 1));
	const float s111 = f(Vector3i(c.x + 1, c.y + 1, c.z + 1));

	return interpolate(s000, s100, s101, s001, s010, s110, s111, s011, fract(pos));
}

// Binary search can be more accurate than linear regression because the SDF can be inaccurate in the first place.
// An alternative would be to polygonize a tiny area around the middle-phase hit position.
// `d1` is how far from `pos0` along `dir` the binary search will take place.
// The segment may be adjusted internally if it does not contain a zero-crossing of the
template <typename Volume_F>
float approximate_distance_to_isosurface_binary_search(
		const Volume_F &f, Vector3 pos0, Vector3 dir, float d1, int iterations) {
	float d0 = 0.f;
	float sdf0 = get_sdf_interpolated(f, pos0);
	// The position given as argument may be a rough approximation coming from the middle-phase,
	// so it can be slightly below the surface. We can adjust it a little so it is above.
	for (int i = 0; i < 4 && sdf0 < 0.f; ++i) {
		d0 -= 0.5f;
		sdf0 = get_sdf_interpolated(f, pos0 + dir * d0);
	}

	float sdf1 = get_sdf_interpolated(f, pos0 + dir * d1);
	for (int i = 0; i < 4 && sdf1 > 0.f; ++i) {
		d1 += 0.5f;
		sdf1 = get_sdf_interpolated(f, pos0 + dir * d1);
	}

	if ((sdf0 > 0) != (sdf1 > 0)) {
		// Binary search
		for (int i = 0; i < iterations; ++i) {
			const float dm = 0.5f * (d0 + d1);
			const float sdf_mid = get_sdf_interpolated(f, pos0 + dir * dm);

			if ((sdf_mid > 0) != (sdf0 > 0)) {
				sdf1 = sdf_mid;
				d1 = dm;
			} else {
				sdf0 = sdf_mid;
				d0 = dm;
			}
		}
	}

	// Pick distance closest to the surface
	if (Math::abs(sdf0) < Math::abs(sdf1)) {
		return d0;
	} else {
		return d1;
	}
}

Ref<VoxelRaycastResult> VoxelToolLodTerrain::raycast(
		Vector3 pos, Vector3 dir, float max_distance, uint32_t collision_mask) {
	// TODO Transform input if the terrain is rotated
	// TODO Implement broad-phase on blocks to minimize locking and increase performance
	// TODO Implement reverse raycast? (going from inside ground to air, could be useful for undigging)

	struct RaycastPredicate {
		const VoxelLodTerrain *terrain;

		bool operator()(Vector3i pos) {
			// This is not particularly optimized, but runs fast enough for player raycasts
			const uint64_t raw_value = terrain->get_voxel(pos, VoxelBufferInternal::CHANNEL_SDF, 0xfffe);
			// TODO Format should be accessible from terrain
			const float sdf = u16_to_norm(raw_value);
			return sdf < 0;
		}
	};

	Ref<VoxelRaycastResult> res;

	// We use grid-raycast as a middle-phase to roughly detect where the hit will be
	RaycastPredicate predicate = { _terrain };
	Vector3i hit_pos;
	Vector3i prev_pos;
	float hit_distance;
	float hit_distance_prev;
	// Voxels polygonized using marching cubes influence a region centered on their lower corner,
	// and extend up to 0.5 units in all directions.
	//
	//   o--------o--------o
	//   | A      |     B  |  Here voxel B is full, voxels A, C and D are empty.
	//   |       xxx       |  Matter will show up at the lower corner of B due to interpolation.
	//   |     xxxxxxx     |
	//   o---xxxxxoxxxxx---o
	//   |     xxxxxxx     |
	//   |       xxx       |
	//   | C      |     D  |
	//   o--------o--------o
	//
	// `voxel_raycast` operates on a discrete grid of cubic voxels, so to account for the smooth interpolation,
	// we may offset the ray so that cubes act as if they were centered on the filtered result.
	const Vector3 offset(0.5, 0.5, 0.5);
	if (voxel_raycast(pos + offset, dir, predicate, max_distance, hit_pos, prev_pos, hit_distance, hit_distance_prev)) {
		// Approximate surface

		float d = hit_distance;

		if (_raycast_binary_search_iterations > 0) {
			// This is not particularly optimized, but runs fast enough for player raycasts
			struct VolumeSampler {
				const VoxelLodTerrain *terrain;

				inline float operator()(const Vector3i &pos) const {
					const uint64_t raw_value = terrain->get_voxel(pos, VoxelBufferInternal::CHANNEL_SDF, 0xfffe);
					// TODO Format should be accessible from terrain
					const float sdf = u16_to_norm(raw_value);
					return sdf;
				}
			};

			VolumeSampler sampler{ _terrain };
			d = hit_distance_prev + approximate_distance_to_isosurface_binary_search(sampler,
											pos + dir * hit_distance_prev,
											dir, hit_distance - hit_distance_prev,
											_raycast_binary_search_iterations);
		}

		res.instance();
		res->position = hit_pos;
		res->previous_position = prev_pos;
		res->normal = get_gradient(pos + dir * d);
		res->distance_along_ray = d;
	}

	return res;
}

struct SphereBrush {
	Vector3 center;
	float radius;
	//   ^ result
	// r |,
	//   |  * ,
	//   |      * ,
	//   +----------*-,-----------> dist
	//               r  * ,
	float operator()(Vector3 const& pos) const { return radius - center.distance_to(pos); }
};

// Voxels in a 3x3x3 cube neighborhood
static const std::array<Vector3i, 6> side_neighbors{
	Vector3i(1, 0, 0), Vector3i(-1, 0, 0),
	Vector3i(0, 1, 0), Vector3i(0, -1, 0),
	Vector3i(0, 0, 1), Vector3i(0, 0, -1),
};
enum {XP = 0, XN, YP, YN, ZP, ZN};
static const std::array<Vector3i, 12> edge_neighbors{
	Vector3i(1, 1, 0), Vector3i(0, 1, -1), Vector3i(-1, 1, 0),
	Vector3i(1, -1, 0), Vector3i(0, 1, 1), Vector3i(-1, -1, 0),
	Vector3i(1, 0, 1), Vector3i(0, -1, -1), Vector3i(-1, 0, 1),
	Vector3i(1, 0, -1), Vector3i(0, -1, 1), Vector3i(-1, 0, -1),
};
static const std::array<Vector3i, 8> corner_neighbors{
	Vector3i(1, 1, 1), Vector3i(1, -1, 1),
	Vector3i(1, 1, -1), Vector3i(1, -1, -1),
	Vector3i(-1, 1, 1), Vector3i(-1, -1, 1),
	Vector3i(-1, 1, -1), Vector3i(-1, -1, -1),
};

void VoxelToolLodTerrain::do_sphere(Vector3 center, float radius) {
	VOXEL_PROFILE_SCOPE();
	ERR_FAIL_COND(_terrain == nullptr);

	const Vector3 r = Vector3(radius, radius, radius);
	const Vector3i min_inclusive = Vector3i::from_floored(center - r);
	const Vector3i max_exclusive = Vector3i::from_ceiled(center + r) + Vector3i(1);
	const Box3i box_raw = Box3i::from_min_max(min_inclusive, max_exclusive);
	const Box3i box_read = box_raw.padded(1).clipped(_terrain->get_voxel_bounds());
	const Box3i box_edit = box_read.padded(-1);
	if (!is_area_editable(box_read)) {
		PRINT_VERBOSE("Area not editable");
		return;
	}
	static constexpr unsigned channel = VoxelBufferInternal::CHANNEL_SDF;

	_voxels_src.resize(box_read.size);
	_voxels_dst.resize(box_read.size);

	// Copy voxel terrain to src buffer
	_terrain->get_voxel_data_map_lod0().read_box(
			box_read, channel, [&](Vector3i const& pos_glob, uint16_t u16_sdf) {
				const Vector3i pos_read = pos_glob - box_read.pos;
				_voxels_src[pos_read] = u16_to_norm(u16_sdf);
			});
	//_terrain->copy(box_read.pos, _voxels_src, 1 << VoxelBufferInternal::CHANNEL_SDF);

	// Edit with brush
	const auto edit = [&](auto&& action) {
		box_read.for_each_cell_zxy([&](Vector3i const& pos_glob) {
				const Vector3i pos_read = pos_glob - box_read.pos;
				if (box_edit.contains(pos_glob))
					_voxels_dst[pos_read] = action(pos_glob, pos_read);
				else
					_voxels_dst[pos_read] = _voxels_src[pos_read];
		});
	};

	const auto has_opposite_neighbor = [&](float sdf_center, Vector3i const& pos_read) {
		union FI { float f; uint32_t i; };
		const bool sign_center = FI{sdf_center}.i >> 31;
		const Vector3i min = pos_read - Vector3i(1);
		const Vector3i max = pos_read + Vector3i(1);
		for (Vector3i p = min; p.z <= max.z; ++p.z)
			for (p.x = min.x; p.x <= max.x; ++p.x)
				for (p.y = min.y; p.y <= max.y; ++p.y)
					if ((FI{_voxels_dst[p]}.i >> 31) != sign_center)
						return true;
		return false;
	};

	// clean voxels after edition
	const auto normalize_and_write = [&] {
		_terrain->write_box(box_edit, channel, [&](Vector3i const& pos_glob, uint16_t) {
			const Vector3i pos_read = pos_glob - box_read.pos;
			const float edited_sdf = _voxels_dst[pos_read];
			if (has_opposite_neighbor(edited_sdf, pos_read))
				return norm_to_u16(edited_sdf);
			else
				return norm_to_u16(edited_sdf > 0 ? 1.f : -1.f);
		});
	};

	SphereBrush brush{center, radius};

	switch (_mode) {
		case MODE_ADD: {
			edit([&](Vector3i const& pos, Vector3i const& pos_read) {
					const float weight = clamp(brush(pos.to_vec3()) / brush.radius, 0.f, 1.f);
					const float w = weight > 0 ? 1 - (1 - weight) * 0.85f : 0;
					const float sdf = w * _edition_speed;
					return _voxels_src[pos_read] - sdf;
				});
			normalize_and_write();
		} break;

		case MODE_REMOVE: {
			edit([&](Vector3i const& pos, Vector3i const& pos_read) {
					const float weight = clamp(brush(pos.to_vec3()) / brush.radius, 0.f, 1.f);
					const float w = weight > 0 ? 1 - (1 - weight) * 0.85f : 0;
					const float sdf = w * _edition_speed;
					return _voxels_src[pos_read] + sdf;
				});
			normalize_and_write();
		} break;

		case MODE_SMOOTH: {
			edit([&](Vector3i const& pos, Vector3i const& pos_read) {
					const float weight = smoothstep(0.f, brush.radius * 0.2f, brush(pos.to_vec3())) * _edition_speed;
					const float sdf = _voxels_src[pos_read];
					const auto sum = [&](auto&& N) {
						float s = 0.f;
						for (Vector3i const& n : N)
							s += _voxels_src[pos_read + n];
						return s;
					};
					const float mean = sum(side_neighbors) * (2.f / 28.f) +
														 sum(edge_neighbors) * (1.f / 28.f) +
														 sum(corner_neighbors) * (.5f / 28.f);
					return sdf * (1 - weight) + mean * weight;
				});
			normalize_and_write();
		} break;

		case MODE_CREASE: {
			edit([&](Vector3i const& pos, Vector3i const& pos_read) {
					const float weight = smoothstep(0.f, brush.radius * 0.2f, brush(pos.to_vec3())) * _edition_speed;
					const float sdf = weight * _crease_noise->get_noise_3d(pos.x, pos.y, pos.z) * 0.1f;
					return _voxels_src[pos_read] + sdf;
				});
			normalize_and_write();
		} break;

		case MODE_PLANE: {
			edit([&](Vector3i const& pos, Vector3i const& pos_read) {
					const float weight = smoothstep(0.f, brush.radius * 0.2f, brush(pos.to_vec3()));
					const float dot = _edition_normal.dot(pos.to_vec3() - brush.center);
					const float normal_weight = clamp(dot + 1, 0.f, 1.f) * 0.3 * _edition_speed;
					const float sdf = weight * normal_weight;
					return _voxels_src[pos_read] + sdf;
				});
			normalize_and_write();
		} break;

		case MODE_LEVEL: {
			edit([&](Vector3i const& pos, Vector3i const& pos_read) {
					const float weight = smoothstep(0.f, brush.radius * 0.2f, brush(pos.to_vec3()));
					const float dot = _edition_normal.dot(pos.to_vec3() - brush.center);
					const float normal_weight = clamp(dot,-1.f, 0.f) * 0.3 * _edition_speed;
					const float sdf = weight * normal_weight;
					return _voxels_src[pos_read] + sdf;
				});
			normalize_and_write();
		} break;

		case MODE_TEXTURE_PAINT: {
			_terrain->write_box(box_edit, VoxelBufferInternal::CHANNEL_WEIGHTS,
					SimpleTextureBlendSphereOp{ center, radius, _texture_params });
		} break;

		default:
			ERR_PRINT("Unknown mode");
			break;
	}
}

void VoxelToolLodTerrain::copy(Vector3i pos, Ref<VoxelBuffer> dst, uint8_t channels_mask) const {
	ERR_FAIL_COND(_terrain == nullptr);
	ERR_FAIL_COND(dst.is_null());
	if (channels_mask == 0) {
		channels_mask = (1 << _channel);
	}
	_terrain->copy(pos, dst->get_buffer(), channels_mask);
}

Vector3 VoxelToolLodTerrain::get_gradient(Vector3 pos) const {
	const float delta = 1.0f;

	const float xp = get_voxel_f_interpolated(pos + Vector3(+delta, 0, 0));
	const float xn = get_voxel_f_interpolated(pos + Vector3(-delta, 0, 0));
	const float yp = get_voxel_f_interpolated(pos + Vector3(0, +delta, 0));
	const float yn = get_voxel_f_interpolated(pos + Vector3(0, -delta, 0));
	const float zp = get_voxel_f_interpolated(pos + Vector3(0, 0, +delta));
	const float zn = get_voxel_f_interpolated(pos + Vector3(0, 0, -delta));

	return Vector3(xp - xn, yp - yn, zp - zn).normalized();
}

float VoxelToolLodTerrain::get_voxel_f_interpolated(Vector3 position) const {
	ERR_FAIL_COND_V(_terrain == nullptr, 0);
	const int channel = get_channel();
	const VoxelLodTerrain *terrain = _terrain;
	// TODO Optimization: is it worth a making a fast-path for this?
	return get_sdf_interpolated([terrain, channel](Vector3i ipos) {
		const uint64_t raw_value = terrain->get_voxel(ipos, VoxelBufferInternal::CHANNEL_SDF, 0);
		// TODO Format should be accessible from terrain
		const float sdf = u16_to_norm(raw_value);
		return sdf;
	},
			position);
}

uint64_t VoxelToolLodTerrain::_get_voxel(Vector3i pos) const {
	ERR_FAIL_COND_V(_terrain == nullptr, 0);
	return _terrain->get_voxel(pos, _channel, 0);
}

float VoxelToolLodTerrain::_get_voxel_f(Vector3i pos) const {
	ERR_FAIL_COND_V(_terrain == nullptr, 0);
	const uint64_t raw_value = _terrain->get_voxel(pos, _channel, 0);
	// TODO Format should be accessible from terrain
	return u16_to_norm(raw_value);
}

void VoxelToolLodTerrain::_set_voxel(Vector3i pos, uint64_t v) {
	ERR_FAIL_COND(_terrain == nullptr);
	_terrain->try_set_voxel_without_update(pos, _channel, v);
}

void VoxelToolLodTerrain::_set_voxel_f(Vector3i pos, float v) {
	ERR_FAIL_COND(_terrain == nullptr);
	// TODO Format should be accessible from terrain
	_terrain->try_set_voxel_without_update(pos, _channel, norm_to_u16(v));
}

void VoxelToolLodTerrain::_post_edit(const Box3i &box) {
	ERR_FAIL_COND(_terrain == nullptr);
	_terrain->post_edit_area(box);
}

int VoxelToolLodTerrain::get_raycast_binary_search_iterations() const {
	return _raycast_binary_search_iterations;
}

void VoxelToolLodTerrain::set_raycast_binary_search_iterations(int iterations) {
	_raycast_binary_search_iterations = clamp(iterations, 0, 16);
}

// Turns floating chunks of voxels into rigidbodies:
// Detects separate groups of connected voxels within a box. Each group fully contained in the box is removed from
// the source volume, and turned into a rigidbody.
// This is one way of doing it, I don't know if it's the best way (there is rarely a best way)
// so there are probably other approaches that could be explored in the future, if they have better performance
static Array separate_floating_chunks(VoxelTool &voxel_tool, Box3i world_box, Node *parent_node, Transform transform,
		Ref<VoxelMesher> mesher, Array materials) {
	VOXEL_PROFILE_SCOPE();

	// Checks
	ERR_FAIL_COND_V(mesher.is_null(), Array());
	ERR_FAIL_COND_V(parent_node == nullptr, Array());

	// Copy source data

	// TODO Do not assume channel, at the moment it's hardcoded for smooth terrain
	static const int channels_mask = (1 << VoxelBufferInternal::CHANNEL_SDF);
	static const int main_channel = VoxelBufferInternal::CHANNEL_SDF;

	// TODO We should be able to use `VoxelBufferInternal`, just needs some things exposed
	Ref<VoxelBuffer> source_copy_buffer_ref;
	{
		VOXEL_PROFILE_SCOPE_NAMED("Copy");
		source_copy_buffer_ref.instance();
		source_copy_buffer_ref->create(world_box.size.x, world_box.size.y, world_box.size.z);
		voxel_tool.copy(world_box.pos, source_copy_buffer_ref, channels_mask);
	}
	VoxelBufferInternal &source_copy_buffer = source_copy_buffer_ref->get_buffer();

	// Label distinct voxel groups

	static thread_local std::vector<uint8_t> ccl_output;
	ccl_output.resize(world_box.size.volume());

	unsigned int label_count = 0;

	{
		VOXEL_PROFILE_SCOPE_NAMED("CCL scan");
		IslandFinder island_finder;
		island_finder.scan_3d(
				Box3i(Vector3i(), world_box.size), [&source_copy_buffer](Vector3i pos) {
					// TODO Can be optimized further with direct access
					return source_copy_buffer.get_voxel_f(pos.x, pos.y, pos.z, main_channel) < 0.f;
				},
				to_span(ccl_output), &label_count);
	}

	struct Bounds {
		Vector3i min_pos;
		Vector3i max_pos; // inclusive
		bool valid = false;
	};

	// Compute bounds of each group

	std::vector<Bounds> bounds_per_label;
	{
		VOXEL_PROFILE_SCOPE_NAMED("Bounds calculation");

		// Adding 1 because label 0 is the index for "no label"
		bounds_per_label.resize(label_count + 1);

		unsigned int ccl_index = 0;
		for (int z = 0; z < world_box.size.z; ++z) {
			for (int x = 0; x < world_box.size.x; ++x) {
				for (int y = 0; y < world_box.size.y; ++y) {
					CRASH_COND(ccl_index >= ccl_output.size());
					const uint8_t label = ccl_output[ccl_index];
					++ccl_index;

					if (label == 0) {
						continue;
					}

					CRASH_COND(label >= bounds_per_label.size());
					Bounds &bounds = bounds_per_label[label];

					if (bounds.valid == false) {
						bounds.min_pos = Vector3i(x, y, z);
						bounds.max_pos = bounds.min_pos;
						bounds.valid = true;

					} else {
						if (x < bounds.min_pos.x) {
							bounds.min_pos.x = x;
						} else if (x > bounds.max_pos.x) {
							bounds.max_pos.x = x;
						}

						if (y < bounds.min_pos.y) {
							bounds.min_pos.y = y;
						} else if (y > bounds.max_pos.y) {
							bounds.max_pos.y = y;
						}

						if (z < bounds.min_pos.z) {
							bounds.min_pos.z = z;
						} else if (z > bounds.max_pos.z) {
							bounds.max_pos.z = z;
						}
					}
				}
			}
		}
	}

	// Eliminate groups that touch the box border,
	// because that means we can't tell if they are truly hanging in the air or attached to land further away

	const Vector3i lbmax = world_box.size - Vector3i(1);
	for (unsigned int label = 1; label < bounds_per_label.size(); ++label) {
		CRASH_COND(label >= bounds_per_label.size());
		Bounds &local_bounds = bounds_per_label[label];
		ERR_CONTINUE(!local_bounds.valid);

		if (
				local_bounds.min_pos.x == 0 ||
				local_bounds.min_pos.y == 0 ||
				local_bounds.min_pos.z == 0 ||
				local_bounds.max_pos.x == lbmax.x ||
				local_bounds.max_pos.y == lbmax.y ||
				local_bounds.max_pos.z == lbmax.z) {
			//
			local_bounds.valid = false;
		}
	}

	// Create voxel buffer for each group

	struct InstanceInfo {
		Ref<VoxelBuffer> voxels;
		Vector3i world_pos;
		unsigned int label;
	};
	std::vector<InstanceInfo> instances_info;

	const int min_padding = 2; //mesher->get_minimum_padding();
	const int max_padding = 2; //mesher->get_maximum_padding();

	{
		VOXEL_PROFILE_SCOPE_NAMED("Extraction");

		for (unsigned int label = 1; label < bounds_per_label.size(); ++label) {
			CRASH_COND(label >= bounds_per_label.size());
			const Bounds local_bounds = bounds_per_label[label];

			if (!local_bounds.valid) {
				continue;
			}

			const Vector3i world_pos = world_box.pos + local_bounds.min_pos - Vector3i(min_padding);
			const Vector3i size = local_bounds.max_pos - local_bounds.min_pos + Vector3i(1 + max_padding + min_padding);

			// TODO We should be able to use `VoxelBufferInternal`, just needs some things exposed
			Ref<VoxelBuffer> buffer_ref;
			buffer_ref.instance();
			buffer_ref->create(size.x, size.y, size.z);

			// Read voxels from the source volume
			voxel_tool.copy(world_pos, buffer_ref, channels_mask);

			VoxelBufferInternal &buffer = buffer_ref->get_buffer();

			// Cleanup padding borders
			const Box3i inner_box(Vector3i(min_padding), buffer.get_size() - Vector3i(min_padding + max_padding));
			Box3i(Vector3i(), buffer.get_size())
					.difference(inner_box, [&buffer](Box3i box) {
						buffer.fill_area_f(1.f, box.pos, box.pos + box.size, main_channel);
					});

			// Filter out voxels that don't belong to this label
			for (int z = local_bounds.min_pos.z; z <= local_bounds.max_pos.z; ++z) {
				for (int x = local_bounds.min_pos.x; x <= local_bounds.max_pos.x; ++x) {
					for (int y = local_bounds.min_pos.y; y <= local_bounds.max_pos.y; ++y) {
						const unsigned int ccl_index = Vector3i(x, y, z).get_zxy_index(world_box.size);
						CRASH_COND(ccl_index >= ccl_output.size());
						const uint8_t label2 = ccl_output[ccl_index];

						if (label2 != 0 && label != label2) {
							buffer.set_voxel_f(1.f,
									min_padding + x - local_bounds.min_pos.x,
									min_padding + y - local_bounds.min_pos.y,
									min_padding + z - local_bounds.min_pos.z, main_channel);
						}
					}
				}
			}

			instances_info.push_back(InstanceInfo{ buffer_ref, world_pos, label });
		}
	}

	// Erase voxels from source volume.
	// Must be done after we copied voxels from it.

	{
		VOXEL_PROFILE_SCOPE_NAMED("Erasing");

		voxel_tool.set_channel(main_channel);

		for (unsigned int instance_index = 0; instance_index < instances_info.size(); ++instance_index) {
			CRASH_COND(instance_index >= instances_info.size());
			const InstanceInfo info = instances_info[instance_index];
			ERR_CONTINUE(info.voxels.is_null());

			voxel_tool.sdf_stamp_erase(info.voxels, info.world_pos);
		}
	}

	// Create instances

	Array nodes;

	{
		VOXEL_PROFILE_SCOPE_NAMED("Remeshing and instancing");

		for (unsigned int instance_index = 0; instance_index < instances_info.size(); ++instance_index) {
			CRASH_COND(instance_index >= instances_info.size());
			const InstanceInfo info = instances_info[instance_index];
			ERR_CONTINUE(info.voxels.is_null());

			CRASH_COND(info.label >= bounds_per_label.size());
			const Bounds local_bounds = bounds_per_label[info.label];
			ERR_CONTINUE(!local_bounds.valid);

			// DEBUG
			// print_line(String("--- Instance {0}").format(varray(instance_index)));
			// for (int z = 0; z < info.voxels->get_size().z; ++z) {
			// 	for (int x = 0; x < info.voxels->get_size().x; ++x) {
			// 		String s;
			// 		for (int y = 0; y < info.voxels->get_size().y; ++y) {
			// 			float sdf = info.voxels->get_voxel_f(x, y, z, VoxelBuffer::CHANNEL_SDF);
			// 			if (sdf < -0.1f) {
			// 				s += "X ";
			// 			} else if (sdf < 0.f) {
			// 				s += "x ";
			// 			} else {
			// 				s += "- ";
			// 			}
			// 		}
			// 		print_line(s);
			// 	}
			// 	print_line("//");
			// }

			const Transform local_transform(Basis(), info.world_pos.to_vec3());

			for (int i = 0; i < materials.size(); ++i) {
				Ref<ShaderMaterial> sm = materials[i];
				if (sm.is_valid() &&
						sm->get_shader().is_valid() &&
						sm->get_shader()->has_param(VoxelStringNames::get_singleton()->u_block_local_transform)) {
					// That parameter should have a valid default value matching the local transform relative to the volume,
					// which is usually per-instance, but in Godot 3 we have no such feature, so we have to duplicate.
					sm = sm->duplicate(false);
					sm->set_shader_param(VoxelStringNames::get_singleton()->u_block_local_transform, local_transform);
					materials[i] = sm;
				}
			}

			Ref<Mesh> mesh = mesher->build_mesh(info.voxels, materials);
			// The mesh is not supposed to be null,
			// because we build these buffers from connected groups that had negative SDF.
			ERR_CONTINUE(mesh.is_null());

			if (is_mesh_empty(mesh)) {
				continue;
			}

			// DEBUG
			// {
			// 	Ref<VoxelBlockSerializer> serializer;
			// 	serializer.instance();
			// 	Ref<StreamPeerBuffer> peer;
			// 	peer.instance();
			// 	serializer->serialize(peer, info.voxels, false);
			// 	String fpath = String("debug_data/split_dump_{0}.bin").format(varray(instance_index));
			// 	FileAccess *f = FileAccess::open(fpath, FileAccess::WRITE);
			// 	PoolByteArray bytes = peer->get_data_array();
			// 	PoolByteArray::Read bytes_read = bytes.read();
			// 	f->store_buffer(bytes_read.ptr(), bytes.size());
			// 	f->close();
			// 	memdelete(f);
			// }

			// TODO Option to make multiple convex shapes
			// TODO Use the fast way. This is slow because of the internal TriangleMesh thing.
			// TODO Don't create a body if the mesh has no triangles
			Ref<Shape> shape = mesh->create_convex_shape();
			ERR_CONTINUE(shape.is_null());
			CollisionShape *collision_shape = memnew(CollisionShape);
			collision_shape->set_shape(shape);
			// Center the shape somewhat, because Godot is confusing node origin with center of mass
			const Vector3i size = local_bounds.max_pos - local_bounds.min_pos + Vector3i(1 + max_padding + min_padding);
			const Vector3 offset = -size.to_vec3() * 0.5f;
			collision_shape->set_translation(offset);

			RigidBody *rigid_body = memnew(RigidBody);
			rigid_body->set_transform(transform * local_transform.translated(-offset));
			rigid_body->add_child(collision_shape);
			rigid_body->set_mode(RigidBody::MODE_KINEMATIC);

			// Switch to rigid after a short time to workaround clipping with terrain,
			// because colliders are updated asynchronously
			Timer *timer = memnew(Timer);
			timer->set_wait_time(0.2);
			timer->set_one_shot(true);
			timer->connect("timeout", rigid_body, "set_mode", varray(RigidBody::MODE_RIGID));
			// Cannot use start() here because it requires to be inside the SceneTree,
			// and we don't know if it will be after we add to the parent.
			timer->set_autostart(true);
			rigid_body->add_child(timer);

			MeshInstance *mesh_instance = memnew(MeshInstance);
			mesh_instance->set_mesh(mesh);
			mesh_instance->set_translation(offset);
			rigid_body->add_child(mesh_instance);

			parent_node->add_child(rigid_body);

			nodes.append(rigid_body);
		}
	}

	return nodes;
}

Array VoxelToolLodTerrain::separate_floating_chunks(AABB world_box, Node *parent_node) {
	ERR_FAIL_COND_V(_terrain == nullptr, Array());
	ERR_FAIL_COND_V(!is_valid_size(world_box.size), Array());
	Ref<VoxelMesher> mesher = _terrain->get_mesher();
	Array materials;
	materials.append(_terrain->get_material());
	const Box3i int_world_box(Vector3i::from_floored(world_box.position), Vector3i::from_ceiled(world_box.size));
	return ::separate_floating_chunks(
			*this, int_world_box, parent_node, _terrain->get_global_transform(), mesher, materials);
}

float VoxelToolLodTerrain::get_crease_noise_period() const {
	return _crease_noise->get_period();
}
void VoxelToolLodTerrain::set_crease_noise_period(float period) {
	_crease_noise->set_period(period);
}
float VoxelToolLodTerrain::get_edition_speed() const {
	return _edition_speed;
}
void VoxelToolLodTerrain::set_edition_speed(float speed) {
	_edition_speed = speed;
}
Vector3 VoxelToolLodTerrain::get_edition_normal() const {
	return _edition_normal;
}
void VoxelToolLodTerrain::set_edition_normal(Vector3 const& normal) {
	_edition_normal = normal;
}

void VoxelToolLodTerrain::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_raycast_binary_search_iterations", "iterations"),
			&VoxelToolLodTerrain::set_raycast_binary_search_iterations);
	ClassDB::bind_method(D_METHOD("get_raycast_binary_search_iterations"),
			&VoxelToolLodTerrain::get_raycast_binary_search_iterations);
	ClassDB::bind_method(D_METHOD("get_voxel_f_interpolated", "position"),
			&VoxelToolLodTerrain::get_voxel_f_interpolated);
	ClassDB::bind_method(D_METHOD("separate_floating_chunks", "box", "parent_node"),
			&VoxelToolLodTerrain::separate_floating_chunks);
	ClassDB::bind_method(D_METHOD("set_crease_noise_period", "period"), &VoxelToolLodTerrain::set_crease_noise_period);
	ClassDB::bind_method(D_METHOD("get_crease_noise_period"), &VoxelToolLodTerrain::get_crease_noise_period);
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "crease_noise_period"), "set_crease_noise_period", "get_crease_noise_period");
	ClassDB::bind_method(D_METHOD("set_edition_speed", "speed"), &VoxelToolLodTerrain::set_edition_speed);
	ClassDB::bind_method(D_METHOD("get_edition_speed"), &VoxelToolLodTerrain::get_edition_speed);
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "edition_speed"), "set_edition_speed", "get_edition_speed");
	ClassDB::bind_method(D_METHOD("set_edition_normal", "normal"), &VoxelToolLodTerrain::set_edition_normal);
	ClassDB::bind_method(D_METHOD("get_edition_normal"), &VoxelToolLodTerrain::get_edition_normal);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "edition_normal"), "set_edition_normal", "get_edition_normal");
}
