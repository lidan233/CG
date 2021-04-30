#include "spatialgrid.h"

SpatialGrid::SpatialGrid() {

}

SpatialGrid::SpatialGrid(double cell_size)
{
    size = cell_size;
    currentGridPointID = 0;
    numInitialFreeCells = 10000;

    initializeFreeCells();
}

void SpatialGrid::initializeFreeCells() {
    for (int i=0; i<numInitialFreeCells; i++) {
        Box *cell = new Box();
        freeCells.push_back(cell);
    }
}

int SpatialGrid::insertPoint(Vec3 p) {
    Point *point = new Point();
    point->pos = p;
    point->id = generateUniqueGridPointID();
    point->isMarkedForRemoval = false;
    points.push_back(point);

    std::pair<int,Point*> pair(point->id, point);
    gridPointsByID.insert(pair);

    insertGridPointIntoGrid(point);
    return point->id;
}

void SpatialGrid::positionToIJK(Vec3 p, int *i, int *j, int *k) {
    double inv = 1 / size;
    *i = ceil(p[0]*inv)-1;
    *j = ceil(p[1]*inv)-1;
    *k = ceil(p[2]*inv)-1;

    double eps = 0.00000000001;
    if (fabs(fmod(p[0], size)) < eps) {
        *i = *i + 1;
    }
    if (fabs(fmod(p[1], size)) < eps) {
        *j = *j + 1;
    }
    if (fabs(fmod(p[2], size)) < eps) {
        *k = *k + 1;
    }
}

Vec3 SpatialGrid::IJKToPosition(int i, int j, int k) {
    return Vec3(i*size, j*size, k*size);
}

Box* SpatialGrid::getNewGridCell(int i, int j, int k) {
    if (freeCells.size() == 0) {
        int n = 200;
        for (int i=0; i<n; i++) {
            freeCells.push_back(new Box());
        }
    }

    Box *cell = freeCells.back();
    freeCells.pop_back();

    cell->intial(i, j, k);
    return cell;
}

void SpatialGrid::insertGridPointIntoGrid(Point *p) {
    int i, j, k;
    positionToIJK(p->pos, &i, &j, &k);

    bool isCellInTable = false;
    Box *cell = cellHashTable.findBox(i, j, k, &isCellInTable);
    if (isCellInTable) {
        cell->addPoint(p);
    } else {
        cell = getNewGridCell(i, j ,k);
        cell->addPoint(p);
        cellHashTable.insertBox(cell);
    }

    // offset used for updating position
    updateGridPointCellOffset(p, i, j, k);
}

int SpatialGrid::generateUniqueGridPointID() {
    int id = currentGridPointID;
    currentGridPointID++;
    return id;
}

void SpatialGrid::updateGridPointCellOffset(Point *gp, int i, int j, int k) {
    Vec3 cp = IJKToPosition(i, j, k);
    gp->tx = gp->pos[0] - cp[0];
    gp->ty = gp->pos[1] - cp[1];
    gp->tz = gp->pos[2] - cp[2];
}

void SpatialGrid::movePoint(int id, Vec3 newPos) {
    if (gridPointsByID.find(id) == gridPointsByID.end()) {
        return;
    }

    Point *point = gridPointsByID[id];
    int i = point->x;
    int j = point->y;
    int k = point->z;

    Vec3 trans = newPos - point->pos;
    point->tx += trans[0];
    point->ty += trans[1];
    point->tz += trans[2];
    point->pos = newPos;

    // point has moved to new cell
    if (point->tx >= size || point->ty >= size || point->tz >= size ||
              point->tx < 0 || point->ty < 0 || point->tz < 0) {
        int nexti, nextj, nextk;
        positionToIJK(point->pos, &nexti, &nextj, &nextk);

        // remove grid point from old cell
        Box *oldCell = cellHashTable.getBox(i, j, k);
        oldCell->rmPoint(point);

        // remove cell from hash if empty
        if (oldCell->isEmpty()) {
            cellHashTable.removeBox(oldCell);
            oldCell->reset();
            freeCells.push_back(oldCell);
        }

        // insert into new cell
        bool isCellInTable = false;
        Box *cell = cellHashTable.findBox(nexti, nextj, nextk, &isCellInTable);
        if (isCellInTable) {
            cell->addPoint(point);
        } else {
            Box *cell = getNewGridCell(nexti, nextj, nextk);
            cell->addPoint(point);
            cellHashTable.insertBox(cell);
        }

        updateGridPointCellOffset(point, nexti, nextj, nextk);
    }
}

void SpatialGrid::removePoint(int id) {
    if (gridPointsByID.find(id) == gridPointsByID.end()) {
        return;
    }
    isCellRemoved = true;

    Point *point = gridPointsByID[id];
    gridPointsByID.erase(id);

    int i = point->x;
    int j = point->y;
    int k = point->z;

    point->isMarkedForRemoval = true;
    bool isCellInTable = false;
    Box *cell = cellHashTable.findBox(i, j, k, &isCellInTable);

    if (!isCellInTable) {
        return;
    }

    cell->rmPoint(point);
    if (cell->isEmpty()) {
        cellHashTable.removeBox(cell);
        cell->reset();
        freeCells.push_back(cell);
    }
}

std::vector<Vec3> SpatialGrid::getObjectsInRadiusOfPoint(int ref, double r) {
    std::vector<Vec3> objects;

    if (gridPointsByID.find(ref) == gridPointsByID.end()) {
        return objects;
    }

    GridPoint *p = gridPointsByID[ref];
    double tx = p->tx;
    double ty = p->ty;
    double tz = p->tz;
    int i, j, k;
    positionToIJK(p->position, &i, &j, &k);
    double inv = 1/size;
    double rsq = r*r;

    int imin = i - fmax(0, ceil((r-tx)*inv));
    int jmin = j - fmax(0, ceil((r-ty)*inv));
    int kmin = k - fmax(0, ceil((r-tz)*inv));
    int imax = i + fmax(0, ceil((r-size+tx)*inv));
    int jmax = j + fmax(0, ceil((r-size+ty)*inv));
    int kmax = k + fmax(0, ceil((r-size+tz)*inv));

    Box *cell;
    GridPoint *gp;
    Vec3 v;
    std::vector<GridPoint*> points;
    for (int ii=imin; ii<=imax; ii++) {
      for (int jj=jmin; jj<=jmax; jj++) {
        for (int kk=kmin; kk<=kmax; kk++) {

            bool isInHash = false;
            cell = cellHashTable.findBox(ii, jj, kk, &isInHash);
            if (isInHash) {
                points = cell->getPoints();
                for (int idx=0; idx<(int)points.size(); idx++) {
                    gp = points[idx];
                    if (gp->id != ref) {
                        v = p->position - gp->position;
                        if (dot(v, v) < rsq) {
                            objects.push_back(gp->position);
                        }
                    }
                }
            }

        }
      }
    }

    return objects;
}

std::vector<int> SpatialGrid::fastIDNeighbourSearch(int ref, double r, Point *p) {
    std::vector<int> objects;

    bool isInHash = false;
    Box *cell = cellHashTable.findBox(p->x, p->y, p->z, &isInHash);
    if (!isInHash) {
        return objects;
    }

    std::vector<Point*> points = cell->getPoints();
    Point *gp;
    Vec3 v;
    double rsq = r*r;
    for (uint i=0; i<points.size(); i++) {
        gp = points[i];
        if (gp->id != ref) {
            v = p->pos - gp->pos;
            if (dot(v, v) < rsq) {
                objects.push_back(gp->id);
            }
        }
    }

    std::vector<Box*> neighbours = cell->neighbors;
    for (uint i=0; i<neighbours.size(); i++) {
        points = neighbours[i]->getPoints();
        for (uint j=0; j<points.size(); j++) {
            gp = points[j];
            v = p->pos - gp->pos;
            if (dot(v, v) < rsq) {
                objects.push_back(gp->id);
            }
        }
    }

    return objects;
}

std::vector<int> SpatialGrid::getIDsInRadiusOfPoint(int ref, double r) {

    if (gridPointsByID.find(ref) == gridPointsByID.end()) {
        std::vector<int> objects;
        return objects;
    }

    GridPoint *p = gridPointsByID[ref];
    double tx = p->tx;
    double ty = p->ty;
    double tz = p->tz;
    int i, j, k;
    positionToIJK(p->position, &i, &j, &k);
    double inv = 1/size;
    double rsq = r*r;

    int imin = i - fmax(0, ceil((r-tx)*inv));
    int jmin = j - fmax(0, ceil((r-ty)*inv));
    int kmin = k - fmax(0, ceil((r-tz)*inv));
    int imax = i + fmax(0, ceil((r-size+tx)*inv));
    int jmax = j + fmax(0, ceil((r-size+ty)*inv));
    int kmax = k + fmax(0, ceil((r-size+tz)*inv));

    if (imax - imin <= 3 and imax - imin >= 1) {
        return fastIDNeighbourSearch(ref, r, p);
    }

    std::vector<int> objects;
    Point *gp;
    Box *cell;
    Vec3 v;
    std::vector<Point*> points;
    for (int ii=imin; ii<=imax; ii++) {
      for (int jj=jmin; jj<=jmax; jj++) {
        for (int kk=kmin; kk<=kmax; kk++) {

            bool isInHash = false;
            cell = cellHashTable.findBox(ii, jj, kk, &isInHash);
            if (isInHash) {
                points = cell->getPoints();
                for (int idx=0; idx<(int)points.size(); idx++) {
                    gp = points[idx];
                    if (gp->id != ref) {
                        v = p->position - gp->pos;
                        if (dot(v, v) < rsq) {
                            objects.push_back(gp->id);
                        }
                    }
                }
            }

        }
      }
    }

    return objects;
}

void SpatialGrid::removeGridPointsMarkedForRemoval() {
    if (points.size() == 0 || !isCellRemoved) {
        return;
    }

    GridPoint *p;
    for (int i=(int)points.size()-1; i>=0; i--) {
        if (points[i]->isMarkedForRemoval) {
            p = points[i];
            points.erase(points.begin() + i);

            delete p;
        }
    }

    isCellRemoved = false;
}

void SpatialGrid::update() {
    removeGridPointsMarkedForRemoval();

    // update each cell's cell neighbours
    std::vector<Box*> cells;
    cellHashTable.getBoxs(&cells);

    Box* cell;
    Box* gc;
    for (uint idx=0; idx<cells.size(); idx++) {
        cell = cells[idx];
        cell->neighbors.clear();

        int ii = cell->x;
        int jj = cell->y;
        int kk = cell->z;

        for (int k=kk-1; k<=kk+1; k++) {
            for (int j=jj-1; j<=jj+1; j++) {
                for (int i=ii-1; i<=ii+1; i++) {
                    if (!(i==ii && j==jj && k==kk)) {
                        bool isInTable = false;
                        gc = cellHashTable.findBox(i, j, k, &isInTable);
                        if (isInTable) {
                            cell->neighbors.push_back(gc);
                        }
                    }
                }
            }
        }

    }
}

//void SpatialGrid::draw() {
//    if (points.size() == 0) { return; }
//
//    glColor3f(1.0, 0.4, 0.0);
//    glPointSize(6.0);
//    glBegin(GL_POINTS);
//    for (int i=0; i<(int)points.size(); i++) {
//      glm::vec3 p = points[i]->pos;
//      glVertex3f(p.x, p.y, p.z);
//    }
//    glEnd();
//
//    std::vector<Box*> cells;
//    cellHashTable.getBoxs(&cells);
//
//    glLineWidth(1.0);
//    glColor4f(0.0, 0.0, 1.0, 0.4);
//    for (int i=0; i<(int)cells.size(); i++) {
//        Box *c = cells[i];
//        glm::vec3 pos = IJKToPosition(c->i, c->j, c->k);
//        pos = pos + glm::vec3(0.5*size, 0.5*size, 0.5*size);
//        utils::drawWireframeCube(pos, size);
//    }
//}










