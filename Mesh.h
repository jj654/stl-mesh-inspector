#pragma once

#include <array>
#include <cstdint>
#include <istream>
#include <string>
#include <vector>

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;
};

struct Triangle {
    Vec3 normal;
    std::array<Vec3, 3> v;
};

struct MassProperties {
    double area = 0.0;     // total surface area
    double volume = 0.0;   // enclosed volume (valid for closed meshes)
    Vec3 centroid;         // centre of volume
};

class Mesh {
public:
    // Loads a binary or ASCII STL file. Returns false if the file can't be read.
    bool loadStl(const std::string& path);

    std::size_t triangleCount() const { return triangles_.size(); }
    const std::vector<Triangle>& triangles() const { return triangles_; }

    void boundingBox(Vec3& min, Vec3& max) const;
    MassProperties computeMassProperties() const;

private:
    bool readBinary(std::istream& in, std::uint32_t count);
    bool readAscii(std::istream& in);

    std::vector<Triangle> triangles_;
};
