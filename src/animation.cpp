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
			}
			// updating pos and normals
			mesh_->pos[v_index] += temp_pos;
			// normalize normal  
			mesh_->norm[v_index] += normalize(temp_norm);
		}
	}
}

// particle simulation
void simulate(Scene* scene) {
    // YOUR CODE GOES HERE ---------------------
    // for each mesh
        // skip if no simulation
        // compute time per step
        // foreach simulation steps
            // compute extenal forces (gravity)
            // for each spring, compute spring force on points
                // compute spring distance and length
                // compute static force
                // accumulate static force on points
                // compute dynamic force
                // accumulate dynamic force on points
            // newton laws
                // if pinned, skip
                // acceleration
                // update velocity and positions using Euler's method
                // for each mesh, check for collision
                    // compute inside tests
                    // if quad
                        // compute local poisition
                        // perform inside test
                            // if inside, set position and normal
                        // else sphere
                        // inside test
                            // if inside, set position and normal
                    // if inside
                        // set particle position
                        // update velocity
        // smooth normals if it has triangles or quads
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
