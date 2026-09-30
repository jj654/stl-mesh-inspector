#include "Mesh.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>

bool Mesh::loadStl(const std::string& path) {
    triangles_.clear();

    std::ifstream in(path, std::ios::binary);
    if (!in) return false;

    in.seekg(0, std::ios::end);
    const auto fileSize = static_cast<std::uint64_t>(in.tellg());
    in.seekg(0, std::ios::beg);

    // Binary STL: 80-byte header, 4-byte triangle count, then 50 bytes per triangle.
    // Many CAD tools write "solid" at the start of binary files too, so the file
    // size is a more reliable test than the first word.
    if (fileSize >= 84) {
        char header[80];
        std::uint32_t count = 0;
        in.read(header, 80);
        in.read(reinterpret_cast<char*>(&count), sizeof(count));
        if (in && fileSize == 84ull + 50ull * count) {
            return readBinary(in, count);
        }
    }

    in.clear();
    in.seekg(0, std::ios::beg);
    return readAscii(in);
}

bool Mesh::readBinary(std::istream& in, std::uint32_t count) {
    triangles_.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        float f[12];
        std::uint16_t attribute = 0;
        in.read(reinterpret_cast<char*>(f), sizeof(f));
        in.read(reinterpret_cast<char*>(&attribute), sizeof(attribute));
        if (!in) return false;

        Triangle t;
        t.normal = {f[0], f[1], f[2]};
        for (int k = 0; k < 3; ++k) {
            t.v[k] = {f[3 + 3 * k], f[4 + 3 * k], f[5 + 3 * k]};
        }
        triangles_.push_back(t);
    }
    return true;
}

bool Mesh::readAscii(std::istream& in) {
    std::string word;
    Triangle t;
    int vertexIndex = 0;

    while (in >> word) {
        if (word == "normal") {
            in >> t.normal.x >> t.normal.y >> t.normal.z;
        } else if (word == "vertex") {
            if (vertexIndex < 3) {
                in >> t.v[vertexIndex].x >> t.v[vertexIndex].y >> t.v[vertexIndex].z;
            }
            ++vertexIndex;
        } else if (word == "endfacet") {
            if (vertexIndex == 3) triangles_.push_back(t);
            vertexIndex = 0;
        }
    }
    return !triangles_.empty();
}

void Mesh::boundingBox(Vec3& mn, Vec3& mx) const {
    const double big = std::numeric_limits<double>::max();
    mn = {big, big, big};
    mx = {-big, -big, -big};

    for (const auto& t : triangles_) {
        for (const auto& p : t.v) {
            mn.x = std::min(mn.x, p.x);
            mn.y = std::min(mn.y, p.y);
            mn.z = std::min(mn.z, p.z);
            mx.x = std::max(mx.x, p.x);
            mx.y = std::max(mx.y, p.y);
            mx.z = std::max(mx.z, p.z);
        }
    }
}

namespace {

Vec3 sub(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

double length(const Vec3& a) { return std::sqrt(dot(a, a)); }

}  // namespace

MassProperties Mesh::computeMassProperties() const {
    MassProperties mp;
    Vec3 weighted;  // sum of (tetrahedron centroid * signed volume)

    for (const auto& t : triangles_) {
        const Vec3& a = t.v[0];
        const Vec3& b = t.v[1];
        const Vec3& c = t.v[2];

        // Triangle area = half the length of the cross product of two edges.
        mp.area += 0.5 * length(cross(sub(b, a), sub(c, a)));

        // Signed volume of the tetrahedron (origin, a, b, c).
        const double vol = dot(a, cross(b, c)) / 6.0;
        mp.volume += vol;

        // Centroid of that tetrahedron is (0 + a + b + c) / 4.
        weighted.x += vol * (a.x + b.x + c.x) / 4.0;
        weighted.y += vol * (a.y + b.y + c.y) / 4.0;
        weighted.z += vol * (a.z + b.z + c.z) / 4.0;
    }

    if (mp.volume != 0.0) {
        mp.centroid = {weighted.x / mp.volume, weighted.y / mp.volume, weighted.z / mp.volume};
    }
    return mp;
}
