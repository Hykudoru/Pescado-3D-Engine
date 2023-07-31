# Pescado 3D Engine
 
This is a 3D graphics and physics engine coded from scratch in C++ and OpenGL. All the libraries, including the vector and matrix math libraries were written from scratch. The physics and geometry processing happens in 3D but when you want to display the image to your 2D monitor you can only plot 2D points. Thus, every 3D point is projected and squashed to the screen using a perspective projection matrix and perspective division. Although it looks 3D, it is actually 2D. There is no such thing as 3D graphics.  

## Meshes
Points and lines are grouped into triangles. Triangles are grouped into meshes (shapes). These shapes then get placed (transformed) into the world by a model-to-world matrix (TRS). This TRS matrix scales, rotates, and translates every point visible (in that exact order) from local space to world space.

## Camera
The camera itself is just a position and a rotation transformation that gets transformed into the world the same way meshes are. To see the world from the cameras view, each point in the world is repositioned or converted to a position relative to the camera using the camera's Inverse TR matrix ( R^-1\*T^-1 or R^T\*T^T). Everything outside the camera's view is then algorithmically discarded.

All praise to Jehovah for creating the real 3D world which no man could do.
