#ifndef COWPHYS_MAT3_H
#define COWPHYS_MAT3_H

#include "Vec3.h"
#include <array>

namespace cp {

class Mat3 {
public:
    Mat3() {
        setIdentity();
    }

    Mat3(float val) {
        for (int i = 0; i < 9; ++i) {
            mData[i] = val;
        }
    }

    Mat3(float m00, float m01, float m02,
         float m10, float m11, float m12,
         float m20, float m21, float m22) {
        mData[0] = m00;
        mData[1] = m01;
        mData[2] = m02;
        mData[3] = m10;
        mData[4] = m11;
        mData[5] = m12;
        mData[6] = m20;
        mData[7] = m21;
        mData[8] = m22;
    }

    static Mat3 diagonal(float a, float b, float c) {
        return Mat3(a, 0, 0,
                    0, b, 0,
                    0, 0, c);
    }

    void setIdentity() {
        mData = {1, 0, 0,
                 0, 1, 0,
                 0, 0, 1};
    }

    Mat3 inverse() const {
        float det = determinant();
        if (det == 0) {
            return Mat3(0); // Non-invertible matrix
        }
        float invDet = 1.0f / det;

        Mat3 inv;
        inv(0, 0) = (mData[4] * mData[8] - mData[5] * mData[7]) * invDet;
        inv(0, 1) = -(mData[1] * mData[8] - mData[2] * mData[7]) * invDet;
        inv(0, 2) = (mData[1] * mData[5] - mData[2] * mData[4]) * invDet;
        inv(1, 0) = -(mData[3] * mData[8] - mData[5] * mData[6]) * invDet;
        inv(1, 1) = (mData[0] * mData[8] - mData[2] * mData[6]) * invDet;
        inv(1, 2) = -(mData[0] * mData[5] - mData[2] * mData[3]) * invDet;
        inv(2, 0) = (mData[3] * mData[7] - mData[4] * mData[6]) * invDet;
        inv(2, 1) = -(mData[0] * mData[7] - mData[1] * mData[6]) * invDet;
        inv(2, 2) = (mData[0] * mData[4] - mData[1] * mData[3]) * invDet;

        return inv;
    }

    float determinant() const {
        return mData[0] * (mData[4] * mData[8] - mData[5] * mData[7])
               - mData[1] * (mData[3] * mData[8] - mData[5] * mData[6])
               + mData[2] * (mData[3] * mData[7] - mData[4] * mData[6]);
    }

    Vec3f operator*(const Vec3f &vec) const {
        return Vec3f(
                mData[0] * vec.x + mData[1] * vec.y + mData[2] * vec.z,
                mData[3] * vec.x + mData[4] * vec.y + mData[5] * vec.z,
                mData[6] * vec.x + mData[7] * vec.y + mData[8] * vec.z
        );
    }

    Mat3 operator*(const Mat3 &mat) const {
        Mat3 result;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                result(row, col) = 0;
                for (int k = 0; k < 3; ++k) {
                    result(row, col) += (*this)(row, k) * mat(k, col);
                }
            }
        }
        return result;
    }

    float &operator()(int row, int col) {
        return mData[row * 3 + col];
    }

    float operator()(int row, int col) const {
        return mData[row * 3 + col];
    }

private:
    std::array<float, 9> mData;
};

} // namespace cp

#endif // COWPHYS_MAT3_H