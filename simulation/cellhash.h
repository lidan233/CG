#ifndef CELLHASH_H
#define CELLHASH_H

#include <unordered_map>
#include <vector>
#include "Particle.h"

class BoxHash
{
public:
    BoxHash();
    bool isBoxInHash(int i, int j, int k);
    void insertBox(Box *cell);
    void removeBox(Box *cell);
    Box* getBox(int i, int j, int k);
    Box* findBox(int i, int j, int k, bool *isGridCellFound);
    void getBoxs(std::vector<Box*> *boxs);

private:
    inline long computeHash(int i, int j, int k);

    long maxNumHashValues;
    std::unordered_map<long, std::vector<Box*>> cellMap;
};

#endif // CELLHASH_H
