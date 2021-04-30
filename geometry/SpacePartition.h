//
// Created by lidan on 2021/4/27.
//

#ifndef CG_SPACEPARTITION_H
#define CG_SPACEPARTITION_H
#include "util.h"
#include "Particle.h"
#include <vector>
#include <unordered_map>

class SpacePartition {
private:
    double size ;
    int id ;
    std::vector<Point*> points ;
    std::unordered_map<int, Point*> id2point ;
//    std::vector<
public:
    SpacePartition() ;
    SpacePartition(double size) ;
    int addPoint(Vec3 pos) ;
    int mPoint(int id, Vec3 Pos) ;
    void rmPoint(int id) ;
    std::vector<Vec3> DistanceQueryPos(int ref, double radius) ;
    std::vector<int> DistanceQueryID(int ref, double radius);
    void update();
    void draw();


};


#endif //CG_SPACEPARTITION_H
