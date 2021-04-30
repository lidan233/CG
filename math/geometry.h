//
// Created by lidan on 2021/4/28.
//

#ifndef CG_GEOMETRY_H
#define CG_GEOMETRY_H

#include "EigenLidan.h"
#include "quaternion.h"
#include <vector>

inline void drawSphere(std::vector<float>& vertices, std::vector<int>& indices, Vec3 pos , float radius, int subAxis, int subHeight) {
    if (subHeight < 2 || subAxis < 2)
        return;

    Vec3 top(0, radius, 0);
    Vec3 bottom(0, -radius, 0);
    int count = 0 ;
    top += pos ;
    bottom += pos ;
    //top
    for (int a = 0; a < subAxis; a++) {
        float ty0 = Math::PI / subHeight;
        float r0 = sin(ty0);

        float th0 = Math::PI * 2 * a     / subAxis;
        float th1 = Math::PI * 2 * (a+1) / subAxis;
        Vec3 p0(cos(th0), 0, sin(th0));
        Vec3 p1(cos(th1), 0, sin(th1));

        Vec3 v0 = p0 * radius * r0;
        Vec3 v1 = p1 * radius * r0;
        v0[1] = v1[1] = cos(ty0);

        v0 += pos ;
        v1 += pos ;

        vertices.push_back(top[0]) ;
        vertices.push_back(top[1]) ;
        vertices.push_back(top[2]) ;
        indices.push_back(vertices.size()/3-1) ;
        vertices.push_back(v1[0]) ;
        vertices.push_back(v1[1]) ;
        vertices.push_back(v1[2]) ;
        indices.push_back(vertices.size()/3-1) ;
        vertices.push_back(v0[0]) ;
        vertices.push_back(v0[1]) ;
        vertices.push_back(v0[2]) ;
        indices.push_back(vertices.size()/3-1) ;

    }

    //body
    for (int y = 1; y < subHeight - 1; y++) {
        float ty0 = Math::PI *  y    / subHeight;
        float ty1 = Math::PI * (y+1) / subHeight;

        float r0 = sin(ty0);
        float r1 = sin(ty1);

        for (int a = 0; a < subAxis; a++) {
            float th0 = Math::PI * 2 * a     / subAxis;
            float th1 = Math::PI * 2 * (a+1) / subAxis;

            Vec3 p0(cos(th0), 0, sin(th0));
            Vec3 p1(cos(th1), 0, sin(th1));

            Vec3 v0 = p0 * radius * r0;
            Vec3 v1 = p1 * radius * r0;

            Vec3 v2 = p0 * radius * r1;
            Vec3 v3 = p1 * radius * r1;

            v0[1] = v1[1] = cos(ty0);
            v2[1] = v3[1] = cos(ty1);

            v0 += pos ;
            v1 += pos ;
            v2 += pos ;
            v3 += pos ;


            vertices.push_back(v0[0]) ;
            vertices.push_back(v0[1]) ;
            vertices.push_back(v0[2]) ;
            indices.push_back(vertices.size()/3-1) ;
            vertices.push_back(v1[0]) ;
            vertices.push_back(v1[1]) ;
            vertices.push_back(v1[2]) ;
            indices.push_back(vertices.size()/3-1) ;
            vertices.push_back(v3[0]) ;
            vertices.push_back(v3[1]) ;
            vertices.push_back(v3[2]) ;
            indices.push_back(vertices.size()/3-1) ;

            vertices.push_back(v0[0]) ;
            vertices.push_back(v0[1]) ;
            vertices.push_back(v0[2]) ;
            indices.push_back(vertices.size()/3-1) ;
            vertices.push_back(v3[0]) ;
            vertices.push_back(v3[1]) ;
            vertices.push_back(v3[2]) ;
            indices.push_back(vertices.size()/3-1) ;
            vertices.push_back(v2[0]) ;
            vertices.push_back(v2[1]) ;
            vertices.push_back(v2[2]) ;
            indices.push_back(vertices.size()/3-1) ;
        }
    }

    //bottom
    for (int a = 0; a < subAxis; a++) {
        float ty0 = Math::PI * (subHeight - 1) / subHeight;
        float r0 = sin(ty0);

        float th0 = Math::PI * 2 * a     / subAxis;
        float th1 = Math::PI * 2 * (a+1) / subAxis;
        Vec3 p0(cos(th0), 0, sin(th0));
        Vec3 p1(cos(th1), 0, sin(th1));

        Vec3 v0 = p0 * radius * r0;
        Vec3 v1 = p1 * radius * r0;
        v0[1] = v1[1] = cos(ty0);

        v0 += pos ;
        v1 += pos ;


        vertices.push_back(bottom[0]) ;
        vertices.push_back(bottom[1]) ;
        vertices.push_back(bottom[2]) ;
        indices.push_back(vertices.size()/3-1) ;
        vertices.push_back(v0[0]) ;
        vertices.push_back(v0[1]) ;
        vertices.push_back(v0[2]) ;
        indices.push_back(vertices.size()/3-1) ;
        vertices.push_back(v1[0]) ;
        vertices.push_back(v1[1]) ;
        vertices.push_back(v1[2]) ;
        indices.push_back(vertices.size()/3-1) ;
    }

}

inline std::vector<Vec3> createPointPanel(float width, float height,
                                        float spacing, int numLayers,
                                        Vec3 w, Vec3 h, bool isStaggered)
{
    int spaces = floor(width/spacing);
    float xpad = width/spaces;
    int nx = spaces+1;

    spaces = floor(height/spacing);
    float ypad = height/spaces;
    int ny = spaces + 1;

    float zpad = spacing;
    int nz = numLayers;

    w = glm::normalize(w);
    h = glm::normalize(h);
    Vec3 normal = cross(w, h);

    std::vector<Vec3> points;
    Vec3 start = (float)(-0.5*width)*w -
                      (float)(0.5*height)*h -
                      (float)(0.5*(nz-1)*zpad)*normal;

    for (int k=0; k<nz; k++) {
        for (int j=0; j<ny; j++) {
            for (int i=0; i<nx; i++) {
                Vec3 p = start + i*xpad*w + j*ypad*h + k*zpad*normal;
                points.push_back(p);

                if (isStaggered && i != nx-1 && j != ny-1 && (k != nz-1 || nz == 1)) {
                    p = p + (float)(0.5*xpad)*w +
                        (float)(0.5*ypad)*h +
                        (float)(0.5*zpad)*normal;
                    points.push_back(p);
                }
            }
        }
    }

    return points;
}

std::vector<Vec3> translatePoints(std::vector<Vec3> points, Vec3 trans)
{
    std::vector<Vec3> newPoints;
    for (uint i=0; i<points.size(); i++) {
        newPoints.push_back(points[i] + trans);
    }

    return newPoints;
}

std::vector<Vec3> rotatePoints(std::vector<Vec3> points, Quaternion q)
{

}

std::vector<Vec3> mergePoints(std::vector<Vec3> points1,
                                   std::vector<Vec3> points2)
{
    for (uint i=0; i<points2.size(); i++) {
        points1.push_back(points2[i]);
    }

    return points1;
}

//void drawPoints(std::vector<Vec3> points)
//{
//
//}

float lerp(float x1, float x2, float t) {
    return x1 + t*(x2 - x1);
}

float smoothstep(float t) {
    return t*t*(3 - 2*t);
}





#endif //CG_GEOMETRY_H
