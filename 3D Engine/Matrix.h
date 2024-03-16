#pragma once
#ifndef MATRIX_H
#define MATRIX_H
#include <Vector.h>
#include <math.h>
/*
    This matrix library can be used for calculating rotation matrices used in 3-dimensional coordinate space.

    Euler conventions: Right-handed, Intrinsic.
    ===============[ Things to Know ]================
    Rotation matrices can be used to track the orientation of a mobile rigid-body relative to a globally fixed world axis frame of reference.
    Combining a sequence of elemental rotations in any order to get the newly rotated axis frame.
    Intrinsic rotations rotate about the axes of the fixed coordinate system frame attached to the rigid body in motion.
    ===============[ How To use this library ]================
    - Extracting colums and rows follow the pattern: Matrix[row][column].
    - Matrix multiplication A*B or Multiply(A, B) assumes A is the row matrix while B is the column matrix.
    - The *= operator can be used like so: A *= B, means A = A*B, where B is the column matrix.
    EXAMPLES:
    --------------------------------------------------------------------------------------------------
    Z AXIS ROTATION:
        rotation = Matrix3x3::RotZ(PI/2.0);
    --------------------------------------------------------------------------------------------------
    EULER ROTATION:
        Matrix3x3 rotation = (Matrix3x3::RotZ(yaw) * Matrix3x3::RotY(pitch)) * Matrix3x3::RotX(roll);
    --------------------------------------------------------------------------------------------------
    YAW-PITCH-ROLL:
        rotation = YPR(180*PI/180, 45*PI/180, 90*PI/180);
    --------------------------------------------------------------------------------------------------
    Undo Previous rotation:

        1st. rotation *= YPR(180*PI/180, 45*PI/180, 90*PI/180);
        2nd. rotation *= RPY(-180*PI/180, -45*PI/180, -90*PI/180);
    --------------------------------------------------------------------------------------------------
*/

class Matrix3x3
{
public:
    float m[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };

    static float identity[3][3];
    static float zero[3][3];

    void Set(const float matrix3x3[3][3])
    {
        for (size_t r = 0; r < 3; r++)
        {
            for (size_t c = 0; c < 3; c++)
            {
                this->m[r][c] = matrix3x3[r][c];
            }
        }
    }

    Matrix3x3() {}
    Matrix3x3(const float matrix3x3[3][3]) { Set(matrix3x3); }

    static Matrix3x3 Transpose(const Matrix3x3& matrix)
    {
        Matrix3x3 result;
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++)
            {
                result.m[c][r] = matrix.m[r][c];
            }
        }
        return result;
    }

    // A*B
    static Matrix3x3 Multiply(const Matrix3x3& matrixA, const Matrix3x3& matrixB)
    {
        Matrix3x3 result = Matrix3x3();
        for (size_t r = 0; r < 3; r++)
        {
            Vector3<float> rowVec = Vector3<float>(matrixA.m[r][0], matrixA.m[r][1], matrixA.m[r][2]);
            for (size_t c = 0; c < 3; c++)
            {
                Vector3<float> columnVec;
                columnVec.x = matrixB.m[0][c];
                columnVec.y = matrixB.m[1][c];
                columnVec.z = matrixB.m[2][c];
                result.m[r][c] = DotProduct(rowVec, columnVec);
            }
        }

        return result;
    }

    // Same as matrixA = matrixA * matrixB
    Matrix3x3& operator*=(const Matrix3x3& matrixB)
    {
        *this = Multiply(*this, matrixB);
        return *this;
    }

    Matrix3x3& operator=(const float matrix[3][3])
    {
        //*this = Matrix3x3(matrix);
        this->Set(matrix);
        return *this;
    }

    // Rotation Matrix about the X axis (in radians)
    // | 1,   0,      0     |
    // | 0, Cos(T), -Sin(T) |
    // | 0, Sin(T),  Cos(T) |
    static Matrix3x3 RotX(float theta)
    {
        float Cos = cos(theta);
        float Sin = sin(theta);

        float standardRotX[3][3] =
        {
            {1, 0, 0},
            {0, Cos, -Sin},
            {0, Sin, Cos}
        };

        Matrix3x3 matrix(standardRotX);
        return matrix;
    }

    // Rotation Matrix about the Y axis (in radians)
    // | Cos(T),  0,   Sin(T)  |
    // |   0,     1,     0     |
    // |-Sin(T),  0,   Cos(T)  |
    static Matrix3x3 RotY(float theta)
    {
        float Cos = cos(theta);
        float Sin = sin(theta);

        float standardRotY[3][3] =
        {
            {Cos, 0, Sin},
            {0,   1,  0 },
            {-Sin,0, Cos}
        };

        Matrix3x3 matrix(standardRotY);
        return matrix;
    }

    // Rotation Matrix about the Z axis (in radians)
    // | Cos(T), -Sin(T), 0 |
    // | Sin(T),  Cos(T), 0 |
    // |   0,      0,     1 |
    static Matrix3x3 RotZ(float theta)
    {
        float Cos = cos(theta);
        float Sin = sin(theta);

        float standardRotZ[3][3] =
        {
            {Cos, -Sin,  0},
            {Sin,  Cos,  0},
            {0,     0,   1}
        };

        Matrix3x3 matrix(standardRotZ);
        return matrix;
    }
};
float Matrix3x3::identity[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};
float Matrix3x3::zero[3][3] = {
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0}
};
// A * B
Matrix3x3 operator*(const Matrix3x3& matrixA, const Matrix3x3& matrixB)
{
   return Matrix3x3::Multiply(matrixA, matrixB);
}
//Matrix * floatMatrix
Matrix3x3 operator*(const Matrix3x3& matrixA, float matrixB[3][3])
{
    return Matrix3x3::Multiply(matrixA, matrixB);
}
//floatMatrix * Matrix
Matrix3x3 operator*(float matrixA[3][3], const Matrix3x3& matrixB)
{
    return Matrix3x3::Multiply(matrixA, matrixB);
}
// M * p
Vector3<float> operator*(const Matrix3x3& matrix, const Vector3<float>& colVec)
{
    Vector3<float> result = Vector3<float>();
    Vector3<float> row1 = Vector3<float>(matrix.m[0][0], matrix.m[0][1], matrix.m[0][2]);
    Vector3<float> row2 = Vector3<float>(matrix.m[1][0], matrix.m[1][1], matrix.m[1][2]);
    Vector3<float> row3 = Vector3<float>(matrix.m[2][0], matrix.m[2][1], matrix.m[2][2]);
    result.x = DotProduct(row1, colVec);
    result.y = DotProduct(row2, colVec);
    result.z = DotProduct(row3, colVec);

    return result;
}

// Roll-Pitch-Yaw x-y'-z''(intrinsic rotation) or z-y-x (extrinsic rotation)
Matrix3x3 RPY(const float& roll, const float& pitch, const float& yaw)
{
    //Matrix3x3 rotation = Matrix3x3::Multiply(Matrix3x3::Multiply(Matrix3x3::RotX(roll).m, Matrix3x3::RotY(pitch).m).m, Matrix3x3::RotZ(yaw).m);//Multiply(rotation.m, RotZ(PI/2.0).m);
    Matrix3x3 rotation = (Matrix3x3::RotX(roll) * Matrix3x3::RotY(pitch)) * Matrix3x3::RotZ(yaw);//Multiply(rotation.m, RotZ(PI/2.0).m);
    return rotation;
}

// Yaw-Pitch-Roll z-y'-x''(intrinsic rotation) or x-y-z (extrinsic rotation)
Matrix3x3 YPR(const float& roll, const float& pitch, const float& yaw)
{
    //Matrix3x3 rotation = Matrix3x3::Multiply(Matrix3x3::Multiply(Matrix3x3::RotZ(yaw).m, Matrix3x3::RotY(pitch).m).m, Matrix3x3::RotX(roll).m);//Multiply(rotation.m, RotZ(PI/2.0).m);
    Matrix3x3 rotation = (Matrix3x3::RotZ(yaw) * Matrix3x3::RotY(pitch)) * Matrix3x3::RotX(roll);//Multiply(rotation.m, RotZ(PI/2.0).m);

    return rotation;
}

//-------------------4x4---------------------

class Matrix4x4
{
public:
    float m[4][4] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };

    static float identity[4][4];
    static float zero[4][4];

    Matrix4x4() {}
    Matrix4x4(float matrix[4][4]) { Set(matrix); }

    void Set(const float matrix[4][4])
    {
        for (int r = 0; r < 4; r++)
        {
            for (int c = 0; c < 4; c++)
            {
                this->m[r][c] = matrix[r][c];
            }
        }
    }

    static Matrix4x4 Transpose(const Matrix4x4& matrix)
    {
        Matrix4x4 result;
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) {
                result.m[c][r] = matrix.m[r][c];
            }
        }
        return result;
    }

    // A*B
    static Matrix4x4 Multiply(const Matrix4x4& matrixA, const Matrix4x4& matrixB)
    {
        Matrix4x4 result;
        for (size_t r = 0; r < 4; r++)
        {
            Vector4<float> rowVec = Vector4<float>(matrixA.m[r][0], matrixA.m[r][1], matrixA.m[r][2], matrixA.m[r][3]);
            for (size_t c = 0; c < 4; c++)
            {
                Vector4<float> columnVec = Vector4<float>(matrixB.m[0][c], matrixB.m[1][c], matrixB.m[2][c], matrixB.m[3][c]);
                result.m[r][c] = DotProduct(rowVec, columnVec);
            }
        }

        return result;
    }

    // Same as matrixA = matrixA * matrixB
    Matrix4x4& operator*=(const Matrix4x4& matrixB)
    {
        *this = Multiply(this->m, matrixB);
        return *this;
    }

    // Same as matrixA = matrixA * matrixB
    Matrix4x4& operator*=(float matrixB[4][4])
    {
        *this = Multiply(*this, matrixB);
        return *this;
    }

    Matrix4x4& operator=(float matrix[4][4])
    {
        this->Set(matrix);
        return *this;
    }


    // Homogeneous Rotation Matrix about the X axis (in radians)
    static Matrix4x4 RotX(const float& theta)
    {
        float Cos = cos(theta);
        float Sin = sin(theta);

        float standardRotX[4][4] =
        {
            {1, 0, 0, 0},
            {0, Cos, -Sin, 0},
            {0, Sin, Cos, 0},
            {0, 0, 0, 1}
        };

        return standardRotX;
    }

    // Homogeneous Rotation Matrix about the Y axis (in radians)
    static Matrix4x4 RotY(const float& theta)
    {
        float Cos = cos(theta);
        float Sin = sin(theta);

        float standardRotY[4][4] =
        {
            {Cos, 0, Sin, 0},
            {0,   1,  0, 0 },
            {-Sin,0, Cos, 0},
            {0, 0, 0, 1}
        };

        return standardRotY;
    }

    // Homogeneous Rotation Matrix about the Z axis (in radians)
    static Matrix4x4 RotZ(const float& theta)
    {
        float Cos = cos(theta);
        float Sin = sin(theta);

        float standardRotZ[4][4] = {
            {Cos, -Sin,  0, 0},
            {Sin,  Cos,  0, 0},
            {0,     0,   1, 0},
            {0,     0,   0, 1}
        };

        return standardRotZ;
    }

};
float Matrix4x4::identity[4][4] = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
};
float Matrix4x4::zero[4][4] = {
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
};

// A * B
Matrix4x4 operator*(const Matrix4x4& matrixA, const Matrix4x4& matrixB)
{
    return Matrix4x4::Multiply(matrixA, matrixB);
}
//Matrix * floatMatrix
Matrix4x4 operator*(const Matrix4x4& matrixA, float matrixB[4][4])
{
    return Matrix4x4::Multiply(matrixA, matrixB);
}
//floatMatrix * Matrix
Matrix4x4 operator*(float matrixA[4][4], const Matrix4x4& matrixB)
{
    return Matrix4x4::Multiply(matrixA, matrixB);
}
// M * p
Vector4<float> operator*(const Matrix4x4& matrix, const Vector4<float>& colVec)
{
    Vector4<float> result = Vector4<float>();
    Vector4<float> row1 = Vector4<float>(matrix.m[0][0], matrix.m[0][1], matrix.m[0][2], matrix.m[0][3]);
    Vector4<float> row2 = Vector4<float>(matrix.m[1][0], matrix.m[1][1], matrix.m[1][2], matrix.m[1][3]);
    Vector4<float> row3 = Vector4<float>(matrix.m[2][0], matrix.m[2][1], matrix.m[2][2], matrix.m[2][3]);
    Vector4<float> row4 = Vector4<float>(matrix.m[3][0], matrix.m[3][1], matrix.m[3][2], matrix.m[3][3]);
    result.x = DotProduct(row1, colVec);
    result.y = DotProduct(row2, colVec);
    result.z = DotProduct(row3, colVec);
    result.w = DotProduct(row4, colVec);

    if (result.w != 0.0000)
    {
        result.x /= result.w;
        result.y /= result.w;
        result.z /= result.w;
    }

    return result;
}
#endif