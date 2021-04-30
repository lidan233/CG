#ifndef SPATIALGRID_H
#define SPATIALGRID_H

#include <vector>
#include <unordered_map>
#include <GL/glu.h>
#include <cmath>
#include <time.h>
#include "EigenLidan.h"
//#include "glm/glm.hpp"
#include "cellhash.h"
#include "Particle.h"

class SpatialGrid
{
public:
    SpatialGrid();
    SpatialGrid(double cell_size);
    int insertPoint(Vec3 point);
    void movePoint(int id, Vec3 position);
    void removePoint(int id);
    std::vector<Vec3> getObjectsInRadiusOfPoint(int ref, double radius);
    std::vector<int> getIDsInRadiusOfPoint(int ref, double radius);
    void update();
//    void draw();

private:
    int generateUniqueGridPointID();
    void initializeFreeCells();
    void insertGridPointIntoGrid(Point *p);
    void positionToIJK(Vec3 p, int *i, int *j, int *k);
    Vec3 IJKToPosition(int i, int j, int k);
    Box* getNewGridCell(int i, int j, int k);
    void updateGridPointCellOffset(Point *gp, int i, int j, int k);
    std::vector<int> fastIDNeighbourSearch(int ref, double r, Point *gp);
    void removeGridPointsMarkedForRemoval();

    double size;
    int currentGridPointID;
    std::vector<Point*> points;
    std::unordered_map<int,Point*> gridPointsByID;
    std::vector<Box*> freeCells;
    int numInitialFreeCells;
    BoxHash cellHashTable;
    bool isCellRemoved = false;
};

#endif // SPATIALGRID_H













