# 3D Graphics Physics Engine OpenGL
 
This is a 3D graphics and physics engine coded from scratch (C++, and OpenGL points, lines, and triangles). All the libraries including the vector math and matrix math libraries were all written from scratch. Every object is like it's own coordinate system. Points and lines are grouped into triangles, then triangles are grouped into meshes (shapes). These shaped then get placed in the world by a model-to-world matrix (TRS matrix = Scale, Rotate, Translate). Each point is then converted to a position relative to the camera using the camera's inverse TR matrix ( R^-1\*T^-1 or R^T\*T^T). Every 3D point is then projected and squashed to the screen from 3D to 2D using a projection matrix and perspective division. Everything visualized is a 2D point, not 3D. All rotations happen using 3x3 rotation matrices before being composed into 4x4 TRS.

All praise to Jehovah for creating the real 3D world which no man could do.
