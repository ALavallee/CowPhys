#ifndef COWPHYS_SIMPLEX_H
#define COWPHYS_SIMPLEX_H

#include <array>
#include "Vec3.h"

namespace cp {

class Simplex {

private:
    std::array<Vec3d, 4> m_points;
    int m_size;

public:
    Simplex() : m_size(0) {}

    Simplex &operator=(std::initializer_list<Vec3d> list) {
        m_size = 0;

        for (auto point: list)
            m_points[m_size++] = point;

        return *this;
    }

    void push_front(Vec3d point) {
        m_points = {point, m_points[0], m_points[1], m_points[2]};
        m_size = std::min(m_size + 1, 4);
    }

    Vec3d &operator[](int i) { return m_points[i]; }

    size_t size() const { return m_size; }

    auto begin() const { return m_points.begin(); }

    auto end() const { return m_points.end() - (4 - m_size); }
};


}

#endif //COWPHYS_SIMPLEX_H
