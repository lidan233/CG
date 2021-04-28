//
// Created by lidan on 2021/4/28.
//

#ifndef CG_GEOMETRY_H
#define CG_GEOMETRY_H

#include "EigenLidan.h"
#include <vector>

inline void generateSphere(std::vector<Vec3>& vertices, std::vector<Vec3>& normals, float radius, int stackCount, int sectorCount )
{
    std::vector<Vec3> vertices, normals ;
    std::vector<Vec3>().swap(vertices);
    std::vector<Vec3>().swap(normals);

    float x, y, z, xy;                              // vertex position
    float nx, ny, nz, lengthInv = 1.0f / radius;    // vertex normal
    float s, t;                                     // vertex texCoord

    float sectorStep = 2 * PI / sectorCount;
    float stackStep = PI / stackCount;
    float sectorAngle, stackAngle;

    for(int i = 0; i <= stackCount; ++i)
    {
        stackAngle = PI / 2 - i * stackStep;        // starting from pi/2 to -pi/2
        xy = radius * cosf(stackAngle);             // r * cos(u)
        z = radius * sinf(stackAngle);              // r * sin(u)

        for(int j = 0; j <= sectorCount; ++j)
        {
            sectorAngle = j * sectorStep;           // starting from 0 to 2pi

            x = xy * cosf(sectorAngle);             // r * cos(u) * cos(v)
            y = xy * sinf(sectorAngle);             // r * cos(u) * sin(v)
            vertices.push_back(Vec3(x,y,z)) ;

            nx = x * lengthInv;
            ny = y * lengthInv;
            nz = z * lengthInv;
            normals.push_back(Vec3(nx,ny,nz)) ;
        }
    }


}
#endif //CG_GEOMETRY_H
