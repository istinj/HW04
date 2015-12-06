#include "animation.h"
#include "tesselation.h"

// compute the frame from an animation
frame3f animate_compute_frame(FrameAnimation* animation, int time) {
    // grab keyframe interval
    auto interval = 0;
    for(auto t : animation->keytimes) if(time < t) break; else interval++;
    interval--;
    // get translation and rotation matrices
    auto t = float(time-animation->keytimes[interval])/float(animation->keytimes[interval+1]-animation->keytimes[interval]);
    auto m_t = translation_matrix(animation->translation[interval]*(1-t)+animation->translation[interval+1]*t);
    auto m_rz = rotation_matrix(animation->rotation[interval].z*(1-t)+animation->rotation[interval+1].z*t,z3f);
    auto m_ry = rotation_matrix(animation->rotation[interval].y*(1-t)+animation->rotation[interval+1].y*t,y3f);
    auto m_rx = rotation_matrix(animation->rotation[interval].x*(1-t)+animation->rotation[interval+1].x*t,x3f);
    // compute combined xform matrix
    auto m = m_t * m_rz * m_ry * m_rx;
    // return the transformed frame
    return transform_frame(m, animation->rest_frame);
}

// update mesh frames for animation
void animate_frame(Scene* scene) {
    // YOUR CODE GOES HERE ---------------------
	// foreach mesh
	for (auto mesh_ : scene->meshes)
	{
		// if not animation, continue
		if (mesh_->animation == nullptr)
			continue;

		// update frame 
		frame3f new_frame_mesh = animate_compute_frame(mesh_->animation, scene->animation->time);
		mesh_->frame = new_frame_mesh;
	}

    // foreach surface
	for (auto surf_ : scene->surfaces)
	{
		// if not animation, continue
		if (surf_->animation == nullptr)
			continue;

		// update frame
		frame3f new_frame_surf = animate_compute_frame(surf_->animation, scene->animation->time);
		surf_->frame = new_frame_surf;

		// update the _display_mesh 
		surf_->_display_mesh->frame = new_frame_surf;
	}
}

// skinning scene
void animate_skin(Scene* scene) {
    // YOUR CODE GOES HERE ---------------------
    // foreach mesh
	for (auto mesh_ : scene->meshes)
	{
		// if no skinning, continue
		if (mesh_->skinning == nullptr)
			continue;

		// temp vectors for pos and norm
		vec3f temp_pos, temp_norm;

		// foreach vertex index
		for (int v_index = 0; v_index < mesh_->pos.size(); v_index++)
		{
			// set pos/norm to zero
			mesh_->pos[v_index] = zero3f;
			mesh_->norm[v_index] = zero3f;

			// for each bone slot (0..3)
			for (int b_index = 0; b_index < 4; b_index++)
			{
				// get bone weight and index
				int bone_id = mesh_->skinning->bone_ids.at(v_index)[b_index];
				float bone_w = mesh_->skinning->bone_weights.at(v_index)[b_index];

				// if index < 0, continue
				if (bone_id < 0)
					continue;

				// grab bone xform
				mat4f bone_xf = mesh_->skinning->bone_xforms[scene->animation->time][bone_id];

				// update position and normal 
				temp_pos = bone_w * transform_point(bone_xf, mesh_->skinning->rest_pos[v_index]);
				temp_norm = bone_w * transform_normal(bone_xf, mesh_->skinning->rest_norm[v_index]);
				mesh_->pos[v_index] += temp_pos;
				mesh_->norm[v_index] += temp_norm;
			}

			// normalize normal  
			mesh_->norm[v_index] = normalize(mesh_->norm[v_index]);
		}
	}
}

// particle simulation
void simulate(Scene* scene) {
    // YOUR CODE GOES HERE ---------------------
    // for each mesh
	for (auto mesh_ : scene->meshes)
	{
		// skip if no simulation
		if (mesh_->simulation == nullptr)
			continue;

        // compute time per step
		auto stepTime = scene->animation->dt / scene->animation->simsteps;

        // foreach simulation steps
		for (int j = 0; j < scene->animation->simsteps; j++)
		{
			// compute extenal forces (gravity)
			auto gravity = scene->animation->gravity;

			for (int k = 0; k < mesh_->simulation->force.size(); k++)
			{
				mesh_->simulation->force[k] = mesh_->simulation->mass[k] * gravity;
			}

            // for each spring, compute spring force on points
			for (auto spring_ : mesh_->simulation->springs)
			{
				// compute spring distance and length
				float l_r = spring_.restlength;
				float l_s = length(mesh_->pos[spring_.ids.y] - mesh_->pos[spring_.ids.x]);
				vec3f spr_dir = normalize(mesh_->pos[spring_.ids.y] - mesh_->pos[spring_.ids.x]);

                // compute static force
				vec3f static_F = spring_.ks * (l_s - l_r) * spr_dir;

                // accumulate static force on points
				mesh_->simulation->force[spring_.ids.x] += static_F;
				mesh_->simulation->force[spring_.ids.y] -= static_F;

				// compute dynamic force
				vec3f v_s = mesh_->simulation->vel[spring_.ids.y] - mesh_->simulation->vel[spring_.ids.x];
				vec3f dynamic_F = spring_.kd * dot(v_s, spr_dir) * spr_dir;

                // accumulate dynamic force on points
				mesh_->simulation->force[spring_.ids.x] += dynamic_F;
				mesh_->simulation->force[spring_.ids.y] -= dynamic_F;
			}

            // newton laws 
            // if pinned, skip
			for (int i = 0; i < mesh_->pos.size(); i++)
			{
				if (!mesh_->simulation->pinned[i])
				{
					// acceleration
					vec3f acc = mesh_->simulation->force[i] / mesh_->simulation->mass[i];

					// update velocity and positions using Euler's method
					mesh_->pos[i] += (mesh_->simulation->vel[i] * stepTime) + (acc * sqr(stepTime) * 0.5f);
					mesh_->simulation->vel[i] += acc * stepTime;

					// for each mesh, check for collision
					for (auto coll_ : scene->surfaces)
					{
						// compute inside tests
						// if quad
						if (coll_->isquad)
						{
							// compute local poisition
							auto local_pos = transform_point_inverse(coll_->frame, mesh_->pos[i]);

							// perform inside test
							if (local_pos.z < 0 &&
								local_pos.x > -(coll_->radius) &&
								local_pos.x < (coll_->radius) &&
								local_pos.y > -(coll_->radius) &&
								local_pos.y < (coll_->radius))
							{
								// if inside, set position and normal
								mesh_->pos[i] = transform_point(coll_->frame, vec3f(local_pos.x, local_pos.y, 0));
								vec3f n_coll = coll_->frame.z;

								// velocity update
								vec3f v_prev = mesh_->simulation->vel[i];
								float dump_p = scene->animation->bounce_dump.x;
								float dump_o = scene->animation->bounce_dump.y;
								mesh_->simulation->vel[i] = ((v_prev - dot(n_coll, v_prev) * n_coll) * (1 - dump_p) +
									((-1) * dot(n_coll, v_prev) * n_coll) * (1 - dump_o));
							}
						}

						// else sphere
						else
						{
							vec3f centre = coll_->frame.o;
							float distance = length(mesh_->pos[i] - centre);

							// inside test
							if (distance < coll_->radius)
							{
								// if inside, set position and normal
								mesh_->pos[i] = (coll_->radius * normalize(mesh_->pos[i] - centre)) + centre;
								vec3f n_coll = normalize(mesh_->pos[i] - centre);

								// velocity update
								vec3f v_prev = mesh_->simulation->vel[i];
								float dump_p = scene->animation->bounce_dump.x;
								float dump_o = scene->animation->bounce_dump.y;
								mesh_->simulation->vel[i] = ((v_prev - dot(n_coll, v_prev) * n_coll) * (1 - dump_p) +
									((-1) * dot(n_coll, v_prev) * n_coll) * (1 - dump_o));
							}
						}
						// if inside
						// set particle position
						// update velocity

						// Already done.
					}
				}
				else continue;
			}
		}

        // smooth normals if it has triangles or quads
		if (mesh_->quad.size() != 0 || mesh_->triangle.size() != 0)
			smooth_normals(mesh_);
	}
        
}

// scene reset
void animate_reset(Scene* scene) {
    scene->animation->time = 0;
    for(auto mesh : scene->meshes) {
        if(mesh->animation) {
            mesh->frame = mesh->animation->rest_frame;
        }
        if(mesh->skinning) {
            mesh->pos = mesh->skinning->rest_pos;
            mesh->norm = mesh->skinning->rest_norm;
        }
        if(mesh->simulation) {
            mesh->pos = mesh->simulation->init_pos;
            mesh->simulation->vel = mesh->simulation->init_vel;
            mesh->simulation->force.resize(mesh->simulation->init_pos.size());
        }
    }
}

// scene update
void animate_update(Scene* scene) {
    if(scene->animation->time >= scene->animation->length-1) {
        if(scene->animation->loop) animate_reset(scene);
        else return;
    } else scene->animation->time ++;
    animate_frame(scene);
    if(! scene->animation->gpu_skinning) animate_skin(scene);
    simulate(scene);
}
