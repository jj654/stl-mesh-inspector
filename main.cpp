#include <iostream>

#include "Mesh.h"
#include "MeshChecker.h"

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: inspector <file.stl>\n";
        return 1;
    }

    Mesh mesh;
    if (!mesh.loadStl(argv[1])) {
        std::cerr << "Could not read " << argv[1] << "\n";
        return 1;
    }

    Vec3 mn, mx;
    mesh.boundingBox(mn, mx);
    const MassProperties mp = mesh.computeMassProperties();
    const CheckReport r = MeshChecker().run(mesh);

    std::cout << "File:                 " << argv[1] << "\n"
              << "Triangles:            " << mesh.triangleCount() << "\n"
              << "Unique vertices:      " << r.uniqueVertices << "\n"
              << "Edges:                " << r.edges << "\n"
              << "Size:                 " << mx.x - mn.x << " x " << mx.y - mn.y << " x "
              << mx.z - mn.z << "\n\n";

    std::cout << "--- Geometry checks ---\n"
              << "Open edges (holes):   " << r.openEdges << "\n"
              << "Non-manifold edges:   " << r.nonManifoldEdges << "\n"
              << "Flipped edges:        " << r.flippedEdges << "\n"
              << "Degenerate triangles: " << r.degenerateTriangles << "\n"
              << "Duplicate triangles:  " << r.duplicateTriangles << "\n"
              << "Sliver triangles:     " << r.sliverTriangles << "\n"
              << "Result:               " << (r.isClean() ? "CLEAN" : "DEFECTS FOUND") << "\n\n";

    std::cout << "--- Mass properties ---\n"
              << "Surface area:         " << mp.area << "\n";
    if (r.isWatertight() && r.flippedEdges == 0) {
        std::cout << "Volume:               " << mp.volume << "\n"
                  << "Centre of mass:       (" << mp.centroid.x << ", " << mp.centroid.y << ", "
                  << mp.centroid.z << ")\n";
    } else {
        std::cout << "Volume:               not reliable (mesh is not closed and consistent)\n";
    }
    return 0;
}
