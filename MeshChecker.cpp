#include "MeshChecker.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <utility>

namespace {

using VertexKey = std::array<long long, 3>;
using EdgeKey = std::pair<int, int>;  // always stored as (smaller index, larger index)

struct EdgeInfo {
    int count = 0;         // how many triangles use this edge
    int directionSum = 0;  // +1 if traversed low->high, -1 if high->low
};

double distance(const Vec3& a, const Vec3& b) {
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double triangleArea(const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 u{b.x - a.x, b.y - a.y, b.z - a.z};
    const Vec3 v{c.x - a.x, c.y - a.y, c.z - a.z};
    const double cx = u.y * v.z - u.z * v.y;
    const double cy = u.z * v.x - u.x * v.z;
    const double cz = u.x * v.y - u.y * v.x;
    return 0.5 * std::sqrt(cx * cx + cy * cy + cz * cz);
}

}  // namespace

MeshChecker::MeshChecker(double weldTolerance, double maxAspectRatio)
    : weldTolerance_(weldTolerance), maxAspectRatio_(maxAspectRatio) {}

CheckReport MeshChecker::run(const Mesh& mesh) const {
    CheckReport report;

    // 1. Merge coincident points: snap each coordinate to a grid of size weldTolerance_.
    std::map<VertexKey, int> vertexIds;
    auto vertexId = [&](const Vec3& p) {
        const VertexKey key{std::llround(p.x / weldTolerance_), std::llround(p.y / weldTolerance_),
                            std::llround(p.z / weldTolerance_)};
        auto it = vertexIds.find(key);
        if (it != vertexIds.end()) return it->second;
        const int id = static_cast<int>(vertexIds.size());
        vertexIds.emplace(key, id);
        return id;
    };

    std::map<EdgeKey, EdgeInfo> edges;
    std::set<std::array<int, 3>> seenTriangles;

    for (const auto& t : mesh.triangles()) {
        const int ids[3] = {vertexId(t.v[0]), vertexId(t.v[1]), vertexId(t.v[2])};

        // 2. Degenerate: two corners are the same point, or the area is ~zero.
        const double area = triangleArea(t.v[0], t.v[1], t.v[2]);
        if (ids[0] == ids[1] || ids[1] == ids[2] || ids[0] == ids[2] ||
            area < weldTolerance_ * weldTolerance_) {
            ++report.degenerateTriangles;
            continue;
        }

        // 3. Sliver: longest edge compared with the height on that edge.
        //    Aspect ratio = Lmax^2 / (2 * area); an equilateral triangle gives ~1.15.
        const double lmax = std::max({distance(t.v[0], t.v[1]), distance(t.v[1], t.v[2]),
                                      distance(t.v[2], t.v[0])});
        if (lmax * lmax / (2.0 * area) > maxAspectRatio_) ++report.sliverTriangles;

        // 4. Duplicate: same three vertices in any order.
        std::array<int, 3> sorted{ids[0], ids[1], ids[2]};
        std::sort(sorted.begin(), sorted.end());
        if (!seenTriangles.insert(sorted).second) ++report.duplicateTriangles;

        // 5. Record the three directed edges a->b, b->c, c->a.
        for (int k = 0; k < 3; ++k) {
            const int a = ids[k];
            const int b = ids[(k + 1) % 3];
            EdgeInfo& e = edges[{std::min(a, b), std::max(a, b)}];
            ++e.count;
            e.directionSum += (a < b) ? 1 : -1;
        }
    }

    // 6. Classify edges. In a clean closed surface every edge is used exactly twice,
    //    once in each direction, so directionSum is 0.
    for (const auto& [key, e] : edges) {
        if (e.count == 1) {
            ++report.openEdges;
        } else if (e.count > 2) {
            ++report.nonManifoldEdges;
        } else if (e.directionSum != 0) {
            ++report.flippedEdges;
        }
    }

    report.uniqueVertices = vertexIds.size();
    report.edges = edges.size();
    return report;
}
