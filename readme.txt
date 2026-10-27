C++ Ray Tracing Renderer
A CPU-based recursive ray tracer developed in C++ for USC's CSCI 420 Computer Graphics course and subsequently extended with additional rendering features, scene-driven materials, and performance optimizations. OpenGL/GLUT is used to display the completed image; ray tracing itself runs on the CPU.

View the project on my portfolio
Features
Rendering and materials
- Phong shading: Configurable diffuse color, specular color, and shininess.
- Recursive reflections: Mirror-like reflections traced through secondary rays.
- Refraction and Fresnel: Transparent materials with configurable transmission and index of refraction (IOR), using Snell's law and Schlick's Fresnel approximation.
- Soft shadows: Multiple shadow rays sampled across a disk-shaped area light to produce penumbrae.
- Stratified supersampling: Jittered subpixel rays averaged to reduce aliasing (configurable samples per pixel).
- Scene-driven materials: Shared material definitions used by both spheres and triangles, with material parameters read from .scene files.
Performance
- Bounding Volume Hierarchy (BVH): Hierarchical axis-aligned bounding boxes reduce unnecessary sphere and triangle intersection tests for primary, reflection, and shadow rays.
- Multithreaded CPU rendering: Image rows are distributed across worker threads, with per-thread random-number generators and a shared pixel buffer. OpenGL display stays on the main thread.
- Configurable parallelism: Select worker thread count on the command line.