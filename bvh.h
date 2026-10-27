#pragma once
// Include this AFTER intersect_sphere() and intersect_triangle() are defined.
// Requires existing Triangle, Sphere, triangles[], spheres[], num_triangles, num_spheres, EPSILON.
#include <vector>
#include <algorithm>
#include <numeric>
#include <limits>
#include <cmath>
#include <cstdio>

struct BVHPrimitive {
    int type; // 0 sphere, 1 triangle
    int index;
    double lo[3], hi[3], centroid[3];
};
struct BVHNode {
    double lo[3], hi[3];
    int left = -1, right = -1;
    int start = 0, count = 0;
};
static std::vector<BVHPrimitive> bvhPrims;
static std::vector<BVHNode> bvhNodes;

static bool bvhBoxHit(const BVHNode& node, const double o[3], const double d[3], double maxT) {
    double nearT = EPSILON, farT = maxT;
    for (int axis = 0; axis < 3; ++axis) {
        if (std::fabs(d[axis]) < 1e-15) {
            if (o[axis] < node.lo[axis] || o[axis] > node.hi[axis]) return false;
            continue;
        }
        double a = (node.lo[axis] - o[axis]) / d[axis];
        double b = (node.hi[axis] - o[axis]) / d[axis];
        if (a > b) std::swap(a, b);
        nearT = (std::max)(nearT, a);
        farT = (std::min)(farT, b);
        if (farT < nearT) return false;
    }
    return true;
}

static int bvhBuildNode(int start, int end) {
    BVHNode node;
    for (int k = 0; k < 3; ++k) {
        node.lo[k] = std::numeric_limits<double>::infinity();
        node.hi[k] = -std::numeric_limits<double>::infinity();
    }
    double cLo[3], cHi[3];
    for (int k = 0; k < 3; ++k) {
        cLo[k] = std::numeric_limits<double>::infinity();
        cHi[k] = -std::numeric_limits<double>::infinity();
    }
    for (int i = start; i < end; ++i) {
        for (int k = 0; k < 3; ++k) {
            node.lo[k] = (std::min)(node.lo[k], bvhPrims[i].lo[k]);
            node.hi[k] = (std::max)(node.hi[k], bvhPrims[i].hi[k]);
            cLo[k] = (std::min)(cLo[k], bvhPrims[i].centroid[k]);
            cHi[k] = (std::max)(cHi[k], bvhPrims[i].centroid[k]);
        }
    }
    const int id = (int)bvhNodes.size();
    bvhNodes.push_back(node);
    const int count = end - start;
    if (count <= 4) {
        bvhNodes[id].start = start;
        bvhNodes[id].count = count;
        return id;
    }
    int axis = 0;
    for (int k = 1; k < 3; ++k)
        if (cHi[k] - cLo[k] > cHi[axis] - cLo[axis]) axis = k;
    if (cHi[axis] - cLo[axis] <= 1e-15) {
        bvhNodes[id].start = start;
        bvhNodes[id].count = count;
        return id;
    }
    const int mid = start + count / 2;
    std::nth_element(bvhPrims.begin() + start, bvhPrims.begin() + mid,
        bvhPrims.begin() + end, [axis](const BVHPrimitive& a, const BVHPrimitive& b) {
            return a.centroid[axis] < b.centroid[axis];
        });
    const int left = bvhBuildNode(start, mid);
    const int right = bvhBuildNode(mid, end);
    bvhNodes[id].left = left;
    bvhNodes[id].right = right;
    return id;
}

static void buildBVH() {
    bvhPrims.clear(); bvhNodes.clear();
    bvhPrims.reserve(num_spheres + num_triangles);
    for (int i = 0; i < num_spheres; ++i) {
        BVHPrimitive p = {};
        p.type = 0; p.index = i;
        const Sphere& s = spheres[i];
        for (int k = 0; k < 3; ++k) {
            p.lo[k] = s.position[k] - s.radius;
            p.hi[k] = s.position[k] + s.radius;
            p.centroid[k] = s.position[k];
        }
        bvhPrims.push_back(p);
    }
    for (int i = 0; i < num_triangles; ++i) {
        BVHPrimitive p = {};
        p.type = 1; p.index = i;
        const Triangle& tri = triangles[i];
        for (int k = 0; k < 3; ++k) {
            p.lo[k] = (std::min)(tri.v[0].position[k], (std::min)(tri.v[1].position[k], tri.v[2].position[k]));
            p.hi[k] = (std::max)(tri.v[0].position[k], (std::max)(tri.v[1].position[k], tri.v[2].position[k]));
            p.centroid[k] = (tri.v[0].position[k] + tri.v[1].position[k] + tri.v[2].position[k]) / 3.0;
        }
        bvhPrims.push_back(p);
    }
    if (!bvhPrims.empty()) bvhBuildNode(0, (int)bvhPrims.size());
    std::printf("BVH built: %zu primitives, %zu nodes\n", bvhPrims.size(), bvhNodes.size());
}

struct BVHResult {
    bool hit = false;
    int type = -1, index = -1;
    double t = std::numeric_limits<double>::infinity();
    double u = 0.0, v = 0.0;
};

// maxT: finite for shadow rays; infinity for closest-hit rays.
// anyHit: exit immediately after first valid occluder.
static BVHResult bvhTrace(const double origin[3], const double dir[3],
    double maxT = std::numeric_limits<double>::infinity(), bool anyHit = false) {
    BVHResult result;
    result.t = maxT;
    if (bvhNodes.empty()) return result;
    // Balanced median splits keep depth below 64 for the project limits.
    int stack[64];
    int top = 0;
    stack[top++] = 0;
    while (top > 0) {
        const int id = stack[--top];
        const BVHNode& node = bvhNodes[id];
        if (!bvhBoxHit(node, origin, dir, result.t)) continue;
        if (node.left >= 0) {
            stack[top++] = node.left;
            stack[top++] = node.right;
            continue;
        }
        for (int i = node.start; i < node.start + node.count; ++i) {
            const BVHPrimitive& p = bvhPrims[i];
            double t = 0, u = 0, v = 0;
            bool hit = (p.type == 0)
                ? intersect_sphere(origin, dir, spheres[p.index], t)
                : intersect_triangle(origin, dir, triangles[p.index], t, u, v);
            if (hit && t > EPSILON && t < result.t) {
                result.hit = true; result.type = p.type; result.index = p.index;
                result.t = t; result.u = u; result.v = v;
                if (anyHit) return result;
            }
        }
    }
    return result;
}
