CSCI-420 Computer Graphics
Assignment #3: Ray tracing
Anna Zhu

This program implements a recursive ray tracer in C++. It renders a static scene (triangles, spheres, multiple point lights) using backwards ray tracing from a fixed camera at the origin. The final output can be displayed on screen or saved as a JPEG.

Features:
- Phong illumination model
- Soft shadows (area light sampling)
- Recursive reflections