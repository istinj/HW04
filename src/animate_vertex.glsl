#version 120

attribute vec3 vertex_pos;          // vertex position (in mesh coordinate frame)
attribute vec3 vertex_norm;         // vertex normal   (in mesh coordinate frame)
attribute vec2 vertex_texcoord;     // vertex texture coordinate

uniform mat4 mesh_frame;            // mesh frame (as a matrix)
uniform mat4 camera_frame_inverse;  // inverse of the camera frame (as a matrix)
uniform mat4 camera_projection;     // camera projection

varying vec3 pos;                   // [to fragment shader] vertex position (in world coordinate)
varying vec3 norm;                  // [to fragment shader] vertex normal (in world coordinate)
varying vec2 texcoord;              // [to fragment shader] vertex texture coordinate

uniform bool skin_enabled;                  // skinning
uniform mat4 skin_bone_xforms[48];          // bone xform
attribute vec4 vertex_skin_bone_ids;        // skin bone indices
attribute vec4 vertex_skin_bone_weights;    // skin weights

// main function
void main() {
    // apply skinning if necessary
    if(skin_enabled) {
        // YOUR CODE GOES HERE ---------------------
        // (only for extra credit)
        // this is a placeholder
        pos = vec3(0,0,0);
        norm = vec3(0,0,0);

        for (int b_slot = 0; b_slot < 4; b_slot++)
        {
            int bone_id = vertex_skin_bone_ids[b_slot];
            float bone_w = vertex_skin_bone_weights[b_slot];
            if (bone_id < 0)
                continue;
            mat4 bone_xform_matrix = skin_bone_xforms[bone_id];
            pos += (bone_w * bone_xform_matrix * vec4(vertex_pos, 1)).xyz;
            norm += (bone_w * bone_xform_matrix * vec4(vertex_norm, 0)).xyz;
        }
    } else {
        pos = vertex_pos;
        norm = vertex_norm;
    }
    // compute pos and normal in world space and set up variables for fragment shader (use mesh_frame)
    pos = (mesh_frame * vec4(pos,1)).xyz / (mesh_frame * vec4(pos,1)).w;
    norm = (mesh_frame * vec4(norm,0)).xyz;
    // copy texture coordinates down
    texcoord = vertex_texcoord;
    // project vertex position to gl_Position using mesh_frame, camera_frame_inverse and camera_projection
    gl_Position = camera_projection * camera_frame_inverse * vec4(pos,1);
}
