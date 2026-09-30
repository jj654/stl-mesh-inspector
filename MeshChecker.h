#pragma once

#include <cstddef>

#include "Mesh.h"

struct CheckReport {
    std::size_t uniqueVertices = 0;
    std::size_t edges = 0;
    std::size_t openEdges = 0;            // edge used by 1 triangle  -> hole in the surface
    std::size_t nonManifoldEdges = 0;     // edge used by 3+ triangles
    std::size_t flippedEdges = 0;         // neighbours disagree on orientation -> flipped triangle
    std::size_t degenerateTriangles = 0;  // zero area or repeated corner
    std::size_t sliverTriangles = 0;      // very thin triangles (poor aspect ratio)
    std::size_t duplicateTriangles = 0;

    bool isWatertight() const { return openEdges == 0 && nonManifoldEdges == 0; }
    bool isClean() const {
        return isWatertight() && flippedEdges == 0 && degenerateTriangles == 0 &&
               duplicateTriangles == 0;
    }
};

class MeshChecker {
public:
    // weldTolerance: points closer than this are treated as the same vertex (model units).
    // maxAspectRatio: triangles thinner than this are reported as slivers.
    explicit MeshChecker(double weldTolerance = 1e-6, double maxAspectRatio = 20.0);

    CheckReport run(const Mesh& mesh) const;

private:
    double weldTolerance_;
    double maxAspectRatio_;
};
