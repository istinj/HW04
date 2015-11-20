# FCG_03
Homework 03 of Foundation of Computer Graphics

## Framework Overview
We suggest you use our framework to create your renderer. We have removed from the code all the function implementations your will need to provide, but we have left the function declarations which can aid you in planning your solution. All code in the framework is documented, so please read the documentation for further information. Following is a brief description of the content of the framework.

- *common.h* includes the basic files from the standard library and contains general utilities such as print messages, handle errors, and enable Python-style foreach loops;
- *vmath.h* includes various math functions from the standard library, and math types specific to graphics; vecXXs are 2d, 3d and 4d tuples, both float and integerers, with related arithmetic options and functions - you should use this type for point, vectors, colors, etc.; frame3fs are 3d frames with transformations to and from the frame for points, vectors, normals, etc.; mat4f defines a 4x4 matrix with matrix-vector operations and functions to create transform matrices and convert frames
- *image.h/image.cpp* defines a color image, with pixel access operations and image loading/saving operations
- *lodepng.h/lodepng.cpp* provide support for the PNG file format
- *json.h/json.cpp/picojson.h* provide support for the JSON file format
- *scene.h/scene.cpp* defines the scene data structure and provide JSON scene loading
- *tesselation.h/tesselation.cpp*: implements smooth curves and surfaces
- *animation.h/animation.cpp*: implements animation and simulation: **your code goes here**
- *animate.cpp* implements the OpenGL renderer and the interaction
- *animate_vertex.glsl/animate_fragment.glsl*: vertex and fragment shaders: **your extra credit code goes here**.

In this homework, scenes are becoming more complex. A Scene is comprised of a Camera, and a list of Meshes, a list of Surfaces and a list of Lights. The Camera is defined by its frame, the size and distance of the image plane and the focus distance (used for interaction). Each Mesh is a collection of either points, lines or triangles and quads, centered with respect to its frame, and colored according to a Blinn-Phong Material with diffuse, specular coefficients. Each Mesh is represented as an indexed polygonal mesh, with vertex position normals and texture coordinates. Each spline segment is the reference to the four vertices of the Bezier. Each Surface can be either a quad or a sphere, as before. Each Light is a point light centered with respect to its frame and with given intensity. The scene also includes the background color, the ambient illumination, the image resolution and the samples per pixel.

Animation data is store in auxiliary structures, in Meshes, Surfaces and the Scene. The scene animation data includes the current time (measured a frame number), the total animation length, the time in seconds for each frame dt, the number of simulation steps simsteps for each frame, the gravity acceleration and the dumpening coefficients for the bounce_bump (parallel, ortho respectively).

The Mesh animation data is divided into separate structures. If a structure is missing, just skip that mesh. MeshAnimation contains keyframe data for the mesh frame, including the initial frame restframe, the times for each keyframe keytimes, and values for translation and rotation. MeshSkinning contains skinning data, including rest position restpos and normals restnorm, per-vertex bone indices boneids and weights boneweights. We decide that maximum 4 bonces can be active at any given time, and indicated with -1 if a bone index is unused. Bone animation is included in bonexforms which stored transform matrices for each bone and time frame. MeshSimulation defines simulation data, storing initial position and velocities initpos, initvel, mass and whether a point is locked into its position pinned (just skip these in the simulation loop). During simulation, just update the vertex position in mesh->pos and the velocities in mesh->simulation->vel. Finally, we inlude a list of springs, each with vertex indices ids, rest length restlength and static ks and dynamic kd conefficients. We also include collision data in MeshCollision that represents the simple primitive, sphere or quad, a Mesh is associated with.

In this homework, model geometry is read from RAW files. This is a trivial file format we built for the course to make parsing trivial in C++. This geometry is stored in the models directory.

We provide very simple interactions for your viewer. Clicking and moving the mouse lets you rotate the model. If you want to save, please restart the program to avoid issues, and press s. For debugging purposes, it is helpful to see face edges. You can do so with wireframing, enabled with w. Animation is enabled by hitting space and GPU skinning is enabled with g (only for extra credit work). We also provide version of the test cases that only include 1/3 of the frames; these are the ones you should use to compute the images.

Since we perform a lot of computation, we suggest you compile in Release mode. You Debug mode only when deemed stricly necessary. You can also modify the scenes, including the amount of samples while debugging.

## Requirements

You are to implement the code left blank in animation.cpp. You will implement these features.

1. *Keyframe interpolation* (animation.cpp#animateframe). Implement keyframe interpolation of the Mesh frame. In this homework, linearly interpolate translation and rotation angles stored in the keyframe, based on the key times. The final transform matrix is written as translate * rotatez * rotatey * rotatex and should be multipled by the restframe to obtain the current mesh->frame.

2. *Skinning* (animation.cpp#animateSkin). Implement mesh skinning by computing both position pos and normals norm based on the bone transforms, skin weights and rest positions restpos and normals restnorm. The various variables are described above and in the code.

3. *Simulation* (animation.cpp#simulate). Implement a particle-based simulator for particle systems, cloth and soft bodies.
  - Implement particle dynamics, by first setting the force based on gravity and particle mass, and then using Euler integration to advance the particles. Remember that the simulation is performed simsteps times, each of which advancing the time by *dt/simsteps*.
  - Add collision after Euler integration, by first checking if a particle is inside a Mesh (using the proxy collision data), and then adjusting position and velocity.
  - Add springs to the force computation for cloth and soft bodies



## Hints

We suggest to implement the renderer following the steps presented above. To debug code, you can use the step by step advancing.

## Extra Credit

Implement *skinning* on the *GPU*. You can implement the same exact algorithm. Just pass the arrays to the vertex shader and execute skinning there.