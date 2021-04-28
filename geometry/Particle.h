//
// Created by 李源 on 2021-04-23.
//

#ifndef CG_PARTICLE_H
#define CG_PARTICLE_H
#include "util.h"
#include <vector>

class Point{

    Vec3 pos;
    int id ;
    double tx, ty, tz;
    int x,y,z;
    bool isInBox = false;
    bool isMarkedForRemoval = false;
};


class Particle {
public:

    Vec3 pos ;
    Vec3 ppox ;
    Vec3 npos ;

    Vec3 velocity ;
    Vec3 velocityAtMiddle ;
    Vec3 SPHVelocity ;
    Vec3 acceleration ;

    double soundSpeed ;
    double mass ;
    double density ;
    double pressure ;

    std::vector<Particle*> neighbors ;
    int gridId ;
    bool isHalfStep ;
    bool isObstacle ;
    bool isRemoved  = false ;
    bool isVisible = true ;
    double adistance = 0.0 ;

    Vec3 color ;
    double colorDensity ;
    double colorVelocity = 0.0  ;
    bool isStuckInBoundary = false ;
    double boundaryAlphaValue = 1.0 ;
    double alpha = 1.0 ;


};


class Obstacle{
    Vec3 pos ;
    std::vector<Particle*> particles ;
    bool isVisiable = true ;
    int id ;
};


class Box{
private:
    std::vector<Point*> points ;
public:
    int x, y, z ;
    std::vector<Box*> neighbors ;

    Box():x(0),y(0),z(0) {};
    void reset()
    {
        for(int i = 0; i < points.size(); i++)
        {
            points[i]->isInBox = false ;
        }
        points.clear() ;
        x = 0 ;
        y = 0 ;
        z = 0 ;
    }
    void intial(int i , int j , int k):x(i),y(j),z(k) {};
    void addPoint(Point* a){
        a->x = x ;
        a->y = y ;
        a->z = z ;

        a->isInBox = true ;
        points.push_back(a) ;
    } ;
    void rmPoint(Point* a)
    {
        for(auto t = points.begin(); t!=points.end(); t++)
        {
            if((*t)->id == a.id)
            {
                (*t)->isInBox = false ;
                points.erase(t) ;
            }
        }

    }
    bool isEmpty()
    {
        return points.size() == 0;
    }
    std::vector<Point*> getPoints()
    {
        return points ;
    }

};

#endif //CG_PARTICLE_H
