#ifdef WIN32
#include <windows.h>
#endif

#if defined(WIN32) || defined(linux)
#include <GL/gl.h>
#include <GL/glut.h>
#elif defined(__APPLE__)
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h> // for rand
#ifdef WIN32
#define strcasecmp _stricmp
#endif

#include <imageIO.h>

//multithreading
#include <thread>
#include <vector>
#include <random>
#include <chrono>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_TRIANGLES 20000
#define MAX_SPHERES 100
#define MAX_LIGHTS 100
#define MAX_DEPTH 3 // max bounces for reflections
#define SHADOW_SAMPLES 64 // samples per light for soft shadows
#define LIGHT_RADIUS 5.0 // area light radius

// anti-aliasing: camera rays per pixel
int SAMPLES_PER_PIXEL = 1;

// rendering configuration
int THREAD_COUNT = 1;

char* filename = NULL;

// The different display modes.
#define MODE_DISPLAY 1
#define MODE_JPEG 2

int mode = MODE_DISPLAY;

#define WIDTH 640
#define HEIGHT 480

#define fov 60.0

// buffer to store the image when saving it to a JPEG
unsigned char buffer[HEIGHT][WIDTH][3];

struct Vertex
{
    double position[3];
    double color_diffuse[3];
    double color_specular[3];
    double normal[3];
    double shininess;
};

// shared material properties for all primitive types. Geometry keeps its own
// legacy per-vertex Phong attributes for backward compatibility with .scenes.
struct Material
{
    double baseColor[3] = { 1.0, 1.0, 1.0 };
    double specular[3] = { 0.0, 0.0, 0.0 };
    double shininess = 0.0;
    double transmission = 0.0;
    double ior = 1.5;
    double metallic = 0.0;   // Stored for future PBR shading
    double roughness = 0.5;  // Stored for future PBR shading
};

struct Triangle
{
    Vertex v[3];
    int materialId = -1;
};

struct Sphere
{
    double position[3];
    double color_diffuse[3];
    double color_specular[3];
    double shininess;
    double radius;

    int materialId = -1;
};

struct Light
{
    double position[3];
    double color[3];
};

std::vector<Material> materials;

static bool sameMaterial(const Material& a, const Material& b)
{
    for (int c = 0; c < 3; ++c)
        if (a.baseColor[c] != b.baseColor[c] || a.specular[c] != b.specular[c])
            return false;
    return a.shininess == b.shininess &&
        a.transmission == b.transmission && a.ior == b.ior &&
        a.metallic == b.metallic && a.roughness == b.roughness;
}

static int registerMaterial(const Material& material)
{
    for (size_t i = 0; i < materials.size(); ++i)
        if (sameMaterial(materials[i], material)) return (int)i;
    materials.push_back(material);
    return (int)materials.size() - 1;
}

Triangle triangles[MAX_TRIANGLES];

Sphere spheres[MAX_SPHERES];
Light lights[MAX_LIGHTS];
double ambient_light[3];

int num_triangles = 0;
int num_spheres = 0;
int num_lights = 0;

void plot_pixel_display(int x, int y, unsigned char r, unsigned char g, unsigned char b);
void plot_pixel_jpeg(int x, int y, unsigned char r, unsigned char g, unsigned char b);
void plot_pixel(int x, int y, unsigned char r, unsigned char g, unsigned char b);

//various vec helpers
void vec_add(const double a[3], const double b[3], double r[3])
{
    r[0] = a[0] + b[0];
    r[1] = a[1] + b[1];
    r[2] = a[2] + b[2];
}

void vec_sub(const double a[3], const double b[3], double r[3])
{
    r[0] = a[0] - b[0];
    r[1] = a[1] - b[1];
    r[2] = a[2] - b[2];
}

void vec_scale(const double a[3], double s, double r[3])
{
    r[0] = a[0] * s;
    r[1] = a[1] * s;
    r[2] = a[2] * s;
}

void vec_set(double v[3], double x, double y, double z)
{
    v[0] = x; v[1] = y; v[2] = z;
}

void vec_copy(const double a[3], double r[3])
{
    r[0] = a[0]; r[1] = a[1]; r[2] = a[2];
}

double vec_dot(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vec_cross(const double a[3], const double b[3], double r[3])
{
    r[0] = a[1] * b[2] - a[2] * b[1];
    r[1] = a[2] * b[0] - a[0] * b[2];
    r[2] = a[0] * b[1] - a[1] * b[0];
}

double vec_length(const double a[3])
{
    return sqrt(vec_dot(a, a));
}

static const double EPSILON = 1e-6;

void vec_normalize(double a[3])
{
    double len = vec_length(a);
    if (len > EPSILON)
    {
        a[0] /= len;
        a[1] /= len;
        a[2] /= len;
    }
}

void vec_reflect(const double L[3], const double N[3], double R[3])
{
    double ndotl = vec_dot(N, L);
    double tmp[3];
    vec_scale(N, 2.0 * ndotl, tmp);
    vec_sub(tmp, L, R);
}

bool vec_refract(
    const double I[3],
    const double N[3],
    double eta,
    double T[3])
{
    // I points in the direction of ray travel.
    // N opposes I.
    // eta = incident IOR / transmitted IOR.

    double cosTheta = -vec_dot(I, N);

    double sin2ThetaT =
        eta * eta * (1.0 - cosTheta * cosTheta);

    // Total internal reflection:
    // no transmitted ray exists.
    if (sin2ThetaT > 1.0)
        return false;

    double cosThetaT =
        sqrt(fmax(0.0, 1.0 - sin2ThetaT));

    for (int c = 0; c < 3; ++c)
    {
        T[c] = eta * I[c] +
            (eta * cosTheta - cosThetaT) * N[c];
    }

    vec_normalize(T);
    return true;
}

struct HitObjectInfo
{
    bool hit;
    double t;
    double point[3];
    double normal[3];

    double kd[3];
    double ks[3];
    double shininess;

    int materialId = -1;
    double transmission = 0.0;
    double ior = 1.5;
    double metallic = 0.0;
    double roughness = 0.5;
};

// for soft shadow
// independent for each worker thread
thread_local std::mt19937 rng;

// seed based on pixel coordinates, makes sampling reproducible across thread counts.
void seed_pixel_rng(int x, int y)
{
    unsigned int seed =
        12345u ^
        ((unsigned int)x * 73856093u) ^
        ((unsigned int)y * 19349663u);

    rng.seed(seed);
}

double rand01()
{
    thread_local std::uniform_real_distribution<double>
        distribution(0.0, 1.0);

    return distribution(rng);
}

void sample(double radius, double offset[3])
{
    double r = radius * sqrt(rand01());
    double theta = 2.0 * M_PI * rand01();
    offset[0] = r * cos(theta);
    offset[1] = r * sin(theta);
    offset[2] = 0.0;
}

// sphere intersection
bool intersect_sphere(const double rayOrigin[3], const double rayDir[3],
    const Sphere& s, double& tHit)
{
    double oc[3];
    vec_sub(rayOrigin, s.position, oc);

    double a = vec_dot(rayDir, rayDir);
    double b = 2.0 * vec_dot(rayDir, oc);
    double c = vec_dot(oc, oc) - s.radius * s.radius;

    double disc = b * b - 4.0 * a * c;
    if (disc < 0.0)
        return false;

    double sqrtDisc = sqrt(disc);
    double t0 = (-b - sqrtDisc) / (2.0 * a);
    double t1 = (-b + sqrtDisc) / (2.0 * a);

    double t = t0;
    if (t < EPSILON)
        t = t1;
    if (t < EPSILON)
        return false;

    tHit = t;
    return true;
}

// triangle intersection
bool intersect_triangle(const double rayOrigin[3], const double rayDir[3],
    const Triangle& tri, double& tHit, double& u, double& v)
{
    const double* p0 = tri.v[0].position;
    const double* p1 = tri.v[1].position;
    const double* p2 = tri.v[2].position;

    double edge1[3], edge2[3];
    vec_sub(p1, p0, edge1);
    vec_sub(p2, p0, edge2);

    double pvec[3];
    vec_cross(rayDir, edge2, pvec);

    double det = vec_dot(edge1, pvec);
    if (fabs(det) < EPSILON)
        return false;

    double invDet = 1.0 / det;

    double tvec[3];
    vec_sub(rayOrigin, p0, tvec);

    u = vec_dot(tvec, pvec) * invDet;
    if (u < 0.0 || u > 1.0)
        return false;

    double qvec[3];
    vec_cross(tvec, edge1, qvec);

    v = vec_dot(rayDir, qvec) * invDet;
    if (v < 0.0 || u + v > 1.0)
        return false;

    double t = vec_dot(edge2, qvec) * invDet;
    if (t < EPSILON)
        return false;

    tHit = t;
    return true;
}
#include "bvh.h"
// find closest intersection

bool find_nearest_hit(
    const double rayOrigin[3],
    const double rayDir[3],
    HitObjectInfo& hit)
{
    // Find the closest primitive using BVH traversal.
    BVHResult result = bvhTrace(rayOrigin, rayDir);

    hit.hit = result.hit;
    hit.t = result.t;

    if (!result.hit)
        return false;

    // Compute the intersection point.
    double scaledDir[3];
    vec_scale(rayDir, result.t, scaledDir);
    vec_add(rayOrigin, scaledDir, hit.point);

    // Both geometry types resolve their material through the same table.
    const int materialId = (result.type == 0)
        ? spheres[result.index].materialId
        : triangles[result.index].materialId;
    if (materialId < 0 || materialId >= (int)materials.size())
        return false;
    hit.materialId = materialId;
    const Material& material = materials[materialId];
    hit.transmission = material.transmission;
    hit.ior = material.ior;
    hit.metallic = material.metallic;
    hit.roughness = material.roughness;

    if (result.type == 0)
    {
        // Sphere
        const Sphere& s = spheres[result.index];

        vec_sub(hit.point, s.position, hit.normal);
        vec_normalize(hit.normal);

        for (int c = 0; c < 3; ++c)
        {
            hit.kd[c] = material.baseColor[c];
            hit.ks[c] = material.specular[c];
        }

        hit.shininess = material.shininess;

    }
    else
    {
        // Triangle: interpolate vertex attributes using barycentric coordinates.
        const Triangle& tri = triangles[result.index];

        double u = result.u;
        double v = result.v;
        double w = 1.0 - u - v;

        for (int c = 0; c < 3; ++c)
        {
            hit.normal[c] =
                w * tri.v[0].normal[c] +
                u * tri.v[1].normal[c] +
                v * tri.v[2].normal[c];

            hit.kd[c] =
                w * tri.v[0].color_diffuse[c] +
                u * tri.v[1].color_diffuse[c] +
                v * tri.v[2].color_diffuse[c];

            hit.ks[c] =
                w * tri.v[0].color_specular[c] +
                u * tri.v[1].color_specular[c] +
                v * tri.v[2].color_specular[c];
        }

        vec_normalize(hit.normal);

        hit.shininess =
            w * tri.v[0].shininess +
            u * tri.v[1].shininess +
            v * tri.v[2].shininess;
    }

    return true;
}

// lower level occlusion check for soft shadows

bool occluded_to_point(
    const double point[3],
    const double normal[3],
    const double targetPos[3])
{
    // Offset ray origin to avoid self-intersections.
    double origin[3];
    double offset[3];

    vec_scale(normal, EPSILON * 10.0, offset);
    vec_add(point, offset, origin);

    // Construct the shadow ray.
    double dir[3];
    vec_sub(targetPos, origin, dir);

    double distToTarget = vec_length(dir);
    if (distToTarget <= EPSILON)
        return false;

    vec_normalize(dir);

    // Only intersections before the light can occlude it.
    double maxT = distToTarget - EPSILON;

    // Any-hit BVH traversal: return as soon as one blocker is found.
    BVHResult result = bvhTrace(origin, dir, maxT, true);

    return result.hit;
}

// shadow test
bool is_in_shadow(const double point[3], const double normal[3], const Light& light)
{
    return occluded_to_point(point, normal, light.position);
}

// 1 = fully lit, 0 = fully in shadow
double soft_shadow_visibility(const double point[3], const double normal[3], const Light& light)
{
    int visibleCount = 0;

    for (int s = 0; s < SHADOW_SAMPLES; ++s)
    {
        double offset[3];
        sample(LIGHT_RADIUS, offset);

        double samplePos[3];
        samplePos[0] = light.position[0] + offset[0];
        samplePos[1] = light.position[1] + offset[1];
        samplePos[2] = light.position[2] + offset[2];

        if (!occluded_to_point(point, normal, samplePos))
            visibleCount++;
    }

    return (double)visibleCount / (double)SHADOW_SAMPLES;
}

// local phong shading w/o reflection
void shade_local(const HitObjectInfo& hit, const double viewDir[3], double outColor[3])
{
    // ambient
    for (int c = 0; c < 3; ++c)
    {
        outColor[c] = ambient_light[c] * hit.kd[c];
    }

    // add diffuse + specular (if not in shadow) for each light
    for (int i = 0; i < num_lights; ++i)
    {
        // Calculate shadow visibility for every ray bounce
        double visibility = soft_shadow_visibility(hit.point, hit.normal, lights[i]);

        // Skip this light if fully occluded
        if (visibility <= 0.0)
            continue;

        // point to light
        double L[3];
        vec_sub(lights[i].position, hit.point, L);
        vec_normalize(L);

        double NdotL = vec_dot(hit.normal, L);
        if (NdotL < 0.0) NdotL = 0.0;

        // reflection of -L around N  
        double Lneg[3] = { -L[0], -L[1], -L[2] };
        double R[3];
        vec_reflect(Lneg, hit.normal, R);
        vec_normalize(R);

        double V[3];
        vec_copy(viewDir, V);
        vec_normalize(V);

        double RdotV = vec_dot(R, V);
        if (RdotV < 0.0)
            RdotV = 0.0;

        double specTerm = 0.0;
        if (RdotV > 0.0 && hit.shininess > 0.0)
            specTerm = pow(RdotV, hit.shininess);

        for (int c = 0; c < 3; ++c)
        {
            double diffuse = hit.kd[c] * NdotL;
            double specular = hit.ks[c] * specTerm;
            outColor[c] += visibility * lights[i].color[c] * (diffuse + specular); // scale w visibility
        }
    }

    // clamp
    for (int c = 0; c < 3; ++c)
    {
        if (outColor[c] < 0.0)
            outColor[c] = 0.0;
        if (outColor[c] > 1.0)
            outColor[c] = 1.0;
    }
}

double fresnel_schlick(
    double cosTheta,
    double n1,
    double n2)
{
    double f0 = (n1 - n2) / (n1 + n2);
    f0 *= f0;

    cosTheta = fmax(0.0, fmin(1.0, cosTheta));

    return f0 + (1.0 - f0) *
        pow(1.0 - cosTheta, 5.0);
}


void offset_ray_origin(
    const double point[3],
    const double normal[3],
    const double direction[3],
    double result[3])
{
    // Choose the side of the surface toward
    // which the outgoing ray travels.
    double sign =
        vec_dot(direction, normal) >= 0.0
        ? 1.0 : -1.0;

    const double bias = EPSILON * 10.0;

    for (int c = 0; c < 3; ++c)
    {
        result[c] =
            point[c] + normal[c] * bias * sign;
    }
}


// recursive raytracer
void trace_ray(
    const double origin[3],
    const double dir[3],
    int depth,
    unsigned char outColor[3])
{
    // Stop recursion at the configured maximum.
    if (depth > MAX_DEPTH)
    {
        outColor[0] = outColor[1] = outColor[2] = 0;
        return;
    }

    HitObjectInfo hit;

    if (!find_nearest_hit(origin, dir, hit))
    {
        // Existing background color
        for (int c = 0; c < 3; ++c)
            outColor[c] = 5;

        return;
    }

    double V[3] = { -dir[0], -dir[1], -dir[2] };

    double localColor[3];
    shade_local(hit, V, localColor);

    // Determine whether the ray enters or exits
    // the surface. A geometric outward normal
    // is assumed for refractive spheres.
    bool entering = vec_dot(dir, hit.normal) < 0.0;

    double N[3];
    vec_copy(hit.normal, N);

    if (!entering)
    {
        for (int c = 0; c < 3; ++c)
            N[c] = -N[c];
    }

    // Existing opaque reflection strength
    double reflectivity = fmax(
        hit.ks[0],
        fmax(hit.ks[1], hit.ks[2])
    );

    reflectivity = fmax(
        0.0, fmin(1.0, reflectivity)
    );

    bool isGlass = hit.transmission > 0.0;

    double n1 = entering ? 1.0 : hit.ior;
    double n2 = entering ? hit.ior : 1.0;

    // Reflection direction
    double reflectionDir[3];
    double dotDN = vec_dot(dir, N);

    for (int c = 0; c < 3; ++c)
    {
        reflectionDir[c] =
            dir[c] - 2.0 * dotDN * N[c];
    }

    vec_normalize(reflectionDir);

    // Calculate refraction and Fresnel
    double refractionDir[3] = {};
    bool canRefract = false;
    double fresnel = 0.0;

    if (isGlass)
    {
        double eta = n1 / n2;

        canRefract = vec_refract(
            dir, N, eta, refractionDir
        );

        double cosTheta = fmax(
            0.0, -vec_dot(dir, N)
        );

        fresnel = fresnel_schlick(
            cosTheta, n1, n2
        );

        // Total internal reflection
        if (!canRefract)
            fresnel = 1.0;
    }

    double reflected[3] = {};
    double refracted[3] = {};

    // Recursive rays
    if (depth < MAX_DEPTH)
    {
        // Reflection
        if (isGlass || reflectivity > 1e-3)
        {
            double newOrigin[3];
            offset_ray_origin(
                hit.point, hit.normal,
                reflectionDir, newOrigin
            );

            unsigned char rgb[3];
            trace_ray(
                newOrigin, reflectionDir,
                depth + 1, rgb
            );

            for (int c = 0; c < 3; ++c)
                reflected[c] = rgb[c] / 255.0;
        }

        // Transmission
        if (isGlass && canRefract)
        {
            double newOrigin[3];
            offset_ray_origin(
                hit.point, hit.normal,
                refractionDir, newOrigin
            );

            unsigned char rgb[3];
            trace_ray(
                newOrigin, refractionDir,
                depth + 1, rgb
            );

            for (int c = 0; c < 3; ++c)
                refracted[c] = rgb[c] / 255.0;
        }
    }

    // Combine local shading, reflection, and transmission.
    double finalColor[3];

    for (int c = 0; c < 3; ++c)
    {
        if (isGlass)
        {
            double transmission =
                fmax(0.0, fmin(
                    1.0, hit.transmission
                ));

            double reflectedWeight =
                transmission * fresnel;

            double transmittedWeight =
                transmission * (1.0 - fresnel);

            if (depth >= MAX_DEPTH)
            {
                // Avoid black edges caused by combining terminated child rays.
                reflectedWeight = 0.0;
                transmittedWeight = 0.0;
            }

            finalColor[c] =
                (1.0 - transmission) * localColor[c]
                + reflectedWeight * reflected[c]
                + transmittedWeight * refracted[c];
        }
        else
        {
            // Preserve original opaque behavior.
            if (depth < MAX_DEPTH)
            {
                finalColor[c] =
                    (1.0 - reflectivity) * localColor[c]
                    + reflectivity * reflected[c];
            }
            else
            {
                finalColor[c] = localColor[c];
            }
        }

        finalColor[c] =
            fmax(0.0, fmin(1.0, finalColor[c]));

        outColor[c] = (unsigned char)(
            finalColor[c] * 255.0 + 0.5
            );
    }
}

void render_rows(int startY, int endY)
{
    double aspect = (double)WIDTH / (double)HEIGHT;
    double fovRad = fov * M_PI / 180.0;
    double halfHeight = tan(fovRad * 0.5);
    double halfWidth = aspect * halfHeight;

    double camOrigin[3] = { 8.0, 5.0, -20.0 };
    double camTarget[3] = { 2.0, 0.0, -35.0 };
    double worldUp[3] = { 0.0, 1.0, 0.0 };

    // Camera forward vector
    double forward[3];
    vec_sub(camTarget, camOrigin, forward);
    vec_normalize(forward);

    // Camera right vector
    double right[3];
    vec_cross(forward, worldUp, right);
    vec_normalize(right);

    // Camera up vector
    double up[3];
    vec_cross(right, forward, up);
    vec_normalize(up);

    // Stratified sampling grid
    int gridSize = (int)sqrt(SAMPLES_PER_PIXEL);

    for (int y = startY; y < endY; ++y)
    {
        for (int x = 0; x < WIDTH; ++x)
        {
            // Make this pixel's sampling independent of the worker thread that renders it.
            seed_pixel_rng(x, y);

            double accumulatedColor[3] =
            { 0.0, 0.0, 0.0 };

            for (int sy = 0; sy < gridSize; ++sy)
            {
                for (int sx = 0; sx < gridSize; ++sx)
                {
                    double offsetX, offsetY;

                    if (SAMPLES_PER_PIXEL == 1)
                    {
                        offsetX = 0.5;
                        offsetY = 0.5;
                    }
                    else
                    {
                        offsetX =
                            (sx + rand01()) / gridSize;
                        offsetY =
                            (sy + rand01()) / gridSize;
                    }

                    double u =
                        (x + offsetX) / (double)WIDTH;
                    double v =
                        (y + offsetY) / (double)HEIGHT;

                    double px =
                        (2.0 * u - 1.0) * halfWidth;
                    double py =
                        (2.0 * v - 1.0) * halfHeight;

                    double rayDir[3];

                    for (int c = 0; c < 3; ++c)
                    {
                        rayDir[c] =
                            forward[c]
                            + px * right[c]
                            + py * up[c];
                    }

                    vec_normalize(rayDir);

                    unsigned char sampleColor[3];

                    // Uses existing BVH, shading, shadows, and recursive reflections.
                    trace_ray(camOrigin, rayDir,
                        0, sampleColor);

                    for (int c = 0; c < 3; ++c)
                    {
                        accumulatedColor[c] +=
                            sampleColor[c];
                    }
                }
            }

            // Average the subpixel samples, each worker writes to different rows so no mutex is needed.
            for (int c = 0; c < 3; ++c)
            {
                double average =
                    accumulatedColor[c] /
                    SAMPLES_PER_PIXEL;

                buffer[y][x][c] =
                    (unsigned char)(average + 0.5);
            }
        }
    }
}

void draw_scene()
{
    auto startTime =
        std::chrono::steady_clock::now();

    printf("Rendering: %d SPP, %d threads\n",
        SAMPLES_PER_PIXEL, THREAD_COUNT);

    std::vector<std::thread> workers;

    // Avoid creating more workers than image rows.
    int threadsToUse = THREAD_COUNT;
    if (threadsToUse > HEIGHT)
        threadsToUse = HEIGHT;

    // Distribute rows evenly across workers.
    int rowsPerThread =
        HEIGHT / threadsToUse;

    for (int i = 0; i < threadsToUse; ++i)
    {
        int startY = i * rowsPerThread;

        int endY = (i == threadsToUse - 1)
            ? HEIGHT
            : startY + rowsPerThread;

        workers.emplace_back(
            render_rows, startY, endY
        );
    }

    // Wait for every thread to finish.
    for (auto& worker : workers)
    {
        worker.join();
    }

    auto endTime =
        std::chrono::steady_clock::now();

    double elapsed =
        std::chrono::duration<double>(
            endTime - startTime
        ).count();

    printf("Render time: %.3f seconds\n", elapsed);

    // All OpenGL operations stay on the main thread.
    glPointSize(1.0f);
    glBegin(GL_POINTS);

    for (int y = 0; y < HEIGHT; ++y)
    {
        for (int x = 0; x < WIDTH; ++x)
        {
            plot_pixel_display(
                x, y,
                buffer[y][x][0],
                buffer[y][x][1],
                buffer[y][x][2]
            );
        }
    }

    glEnd();
    glFlush();

    printf("Ray tracing completed.\n");
    fflush(stdout);
}

void plot_pixel_display(int x, int y, unsigned char r, unsigned char g, unsigned char b)
{
    glColor3f(((float)r) / 255.0f, ((float)g) / 255.0f, ((float)b) / 255.0f);
    glVertex2i(x, y);
}

void plot_pixel_jpeg(int x, int y, unsigned char r, unsigned char g, unsigned char b)
{
    buffer[y][x][0] = r;
    buffer[y][x][1] = g;
    buffer[y][x][2] = b;
}

void plot_pixel(int x, int y, unsigned char r, unsigned char g, unsigned char b)
{
    plot_pixel_display(x, y, r, g, b);
    if (mode == MODE_JPEG)
        plot_pixel_jpeg(x, y, r, g, b);
}

void save_jpg()
{
    printf("Saving JPEG file: %s\n", filename);

    ImageIO img(WIDTH, HEIGHT, 3, &buffer[0][0][0]);
    if (img.save(filename, ImageIO::FORMAT_JPEG) != ImageIO::OK)
        printf("Error in saving\n");
    else
        printf("File saved successfully\n");
}

void parse_check(const char* expected, char* found)
{
    if (strcasecmp(expected, found))
    {
        printf("Expected '%s ' found '%s '\n", expected, found);
        printf("Parsing error; abnormal program abortion.\n");
        exit(0);
    }
}

void parse_doubles(FILE* file, const char* check, double p[3])
{
    char str[100];
    fscanf(file, "%s", str);
    parse_check(check, str);
    fscanf(file, "%lf %lf %lf", &p[0], &p[1], &p[2]);
    printf("%s %lf %lf %lf\n", check, p[0], p[1], p[2]);
}

void parse_rad(FILE* file, double* r)
{
    char str[100];
    fscanf(file, "%s", str);
    parse_check("rad:", str);
    fscanf(file, "%lf", r);
    printf("rad: %f\n", *r);
}

void parse_shi(FILE* file, double* shi)
{
    char s[100];
    fscanf(file, "%s", s);
    parse_check("shi:", s);
    fscanf(file, "%lf", shi);
    printf("shi: %f\n", *shi);
}

// Optional per-object material fields after the legacy geometry fields.
// This works for BOTH spheres and triangles, and preserves old .scene files.
// The next object keyword is rewound so loadScene can read it normally.
void parse_optional_material(FILE* file, Material& material, char nextObject[50])
{
    nextObject[0] = '\0';
    while (true)
    {
        char key[50];
        if (fscanf(file, "%49s", key) != 1) break;

        double* target = nullptr;
        if (strcmp(key, "trans:") == 0) target = &material.transmission;
        else if (strcmp(key, "ior:") == 0) target = &material.ior;
        else if (strcmp(key, "metallic:") == 0) target = &material.metallic;
        else if (strcmp(key, "roughness:") == 0) target = &material.roughness;
        else
        {
            // Preserve the next object keyword without seeking backwards.
            strncpy_s(nextObject, 50, key, _TRUNCATE);
            break;
        }

        if (fscanf(file, "%lf", target) != 1 || !std::isfinite(*target))
        {
            fprintf(stderr, "Invalid material value after %s\n", key);
            exit(1);
        }
    }
    if (material.transmission < 0 || material.transmission > 1 ||
        material.ior <= 0 || material.metallic < 0 || material.metallic > 1 ||
        material.roughness < 0 || material.roughness > 1)
    {
        fprintf(stderr, "Material values out of range\n");
        exit(1);
    }
}

int loadScene(char* argv)
{
    FILE* file = fopen(argv, "r");
    if (!file)
    {
        printf("Unable to open input file %s. Program exiting.\n", argv);
        exit(0);
    }

    int number_of_objects;
    char type[50];
    Light l;
    // Material table is rebuilt whenever a scene is loaded.
    materials.clear();
    fscanf(file, "%i", &number_of_objects);

    printf("number of objects: %i\n", number_of_objects);

    parse_doubles(file, "amb:", ambient_light);

    char nextObject[50] = {};
    for (int i = 0; i < number_of_objects; i++)
    {
        if (nextObject[0] != '\0') {
            strcpy_s(type, sizeof(type), nextObject);
            nextObject[0] = '\0';
        }
        else if (fscanf(file, "%49s", type) != 1) {
            fprintf(stderr, "Unexpected end of scene while reading object %d\n", i);
            exit(1);
        }
        printf("%s\n", type);
        if (strcasecmp(type, "triangle") == 0)
        {
            Triangle t{};
            Material material;
            printf("found triangle\n");
            for (int j = 0;j < 3;j++)
            {
                parse_doubles(file, "pos:", t.v[j].position);
                parse_doubles(file, "nor:", t.v[j].normal);
                parse_doubles(file, "dif:", t.v[j].color_diffuse);
                parse_doubles(file, "spe:", t.v[j].color_specular);
                parse_shi(file, &t.v[j].shininess);
            }

            if (num_triangles == MAX_TRIANGLES)
            {
                printf("too many triangles, you should increase MAX_TRIANGLES!\n");
                exit(0);
            }
            // Legacy triangle vertex attributes stay interpolated at intersection.
            for (int c = 0; c < 3; ++c) {
                material.baseColor[c] = (t.v[0].color_diffuse[c] + t.v[1].color_diffuse[c] + t.v[2].color_diffuse[c]) / 3.0;
                material.specular[c] = (t.v[0].color_specular[c] + t.v[1].color_specular[c] + t.v[2].color_specular[c]) / 3.0;
            }
            material.shininess = (t.v[0].shininess + t.v[1].shininess + t.v[2].shininess) / 3.0;
            parse_optional_material(file, material, nextObject);
            t.materialId = registerMaterial(material);
            triangles[num_triangles++] = t;
        }
        else if (strcasecmp(type, "sphere") == 0)
        {
            Sphere s{};
            Material material;
            printf("found sphere\n");

            parse_doubles(file, "pos:", s.position);
            parse_rad(file, &s.radius);
            parse_doubles(file, "dif:", s.color_diffuse);
            parse_doubles(file, "spe:", s.color_specular);
            parse_shi(file, &s.shininess);
            for (int c = 0; c < 3; ++c) {
                material.baseColor[c] = s.color_diffuse[c];
                material.specular[c] = s.color_specular[c];
            }
            material.shininess = s.shininess;
            parse_optional_material(file, material, nextObject);
            s.materialId = registerMaterial(material);

            if (num_spheres == MAX_SPHERES)
            {
                printf("too many spheres, you should increase MAX_SPHERES!\n");
                exit(0);
            }
            spheres[num_spheres++] = s;
        }
        else if (strcasecmp(type, "light") == 0)
        {
            printf("found light\n");
            parse_doubles(file, "pos:", l.position);
            parse_doubles(file, "col:", l.color);

            if (num_lights == MAX_LIGHTS)
            {
                printf("too many lights, you should increase MAX_LIGHTS!\n");
                exit(0);
            }
            lights[num_lights++] = l;
        }
        else
        {
            printf("unknown type in scene description:\n%s\n", type);
            exit(0);
        }
    }
    return 0;
}

void display()
{
}

void init()
{
    glMatrixMode(GL_PROJECTION);
    glOrtho(0, WIDTH, 0, HEIGHT, 1, -1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);
}

void idle()
{
    // Hack to make it only draw once.
    static int once = 0;
    if (!once)
    {
        draw_scene();
        if (mode == MODE_JPEG)
            save_jpg();
    }
    once = 1;
}


int main(int argc, char** argv)
{
    if (argc < 2)
    {
        printf("Usage: %s <scene> [output.jpg] "
            "[samples] [--threads N]\n", argv[0]);
        return 1;
    }

    // Defaults
    THREAD_COUNT = 1;
    SAMPLES_PER_PIXEL = 1;

    // Gather positional arguments while parsing flags.
    std::vector<const char*> positional;

    for (int i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], "--threads") == 0)
        {
            if (i + 1 >= argc)
            {
                printf("Missing thread count.\n");
                return 1;
            }

            char* end = nullptr;
            long count = strtol(argv[++i], &end, 10);

            if (*end != '\0' || count < 1 || count > 256)
            {
                printf("Thread count must be 1-256.\n");
                return 1;
            }

            THREAD_COUNT = (int)count;
        }
        else
        {
            positional.push_back(argv[i]);
        }
    }

    if (positional.size() < 1 ||
        positional.size() > 3)
    {
        printf("Usage: %s <scene> [output.jpg] "
            "[samples] [--threads N]\n", argv[0]);
        return 1;
    }

    // Optional output filename
    if (positional.size() >= 2)
    {
        mode = MODE_JPEG;
        filename = const_cast<char*>(positional[1]);
    }
    else
    {
        mode = MODE_DISPLAY;
    }

    // Optional samples per pixel
    if (positional.size() == 3)
    {
        char* end = nullptr;
        long samples = strtol(positional[2], &end, 10);

        int grid = (int)sqrt((double)samples);

        if (*end != '\0' ||
            samples < 1 || samples > 256 ||
            grid * grid != samples)
        {
            printf("Samples must be a perfect square "
                "between 1 and 256.\n");
            return 1;
        }

        SAMPLES_PER_PIXEL = (int)samples;
    }

    printf("Samples per pixel: %d\n",
        SAMPLES_PER_PIXEL);
    printf("CPU threads: %d\n", THREAD_COUNT);

    // Prevent GLUT from interpreting our custom flags.
    int glutArgc = 1;
    glutInit(&glutArgc, argv);

    // Keep the remainder of your main() below.

    loadScene(argv[1]);

    printf("Loaded %zu unique materials\n", materials.size());
    // Build the acceleration structure once, after loading.
    buildBVH();

    glutInitDisplayMode(GLUT_RGBA | GLUT_SINGLE);
    glutInitWindowPosition(0, 0);
    glutInitWindowSize(WIDTH, HEIGHT);
    int window = glutCreateWindow("Ray Tracer");
#ifdef __APPLE__
    // This is needed on recent Mac OS X versions to correctly display the window.
    glutReshapeWindow(WIDTH - 1, HEIGHT - 1);
#endif
    glutDisplayFunc(display);
    glutIdleFunc(idle);
    init();

    srand((unsigned int)time(NULL));

    glutMainLoop();
}

