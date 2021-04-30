#include "cellhash.h"

BoxHash::BoxHash()
{
    maxNumHashValues = 10000;
}

inline long BoxHash::computeHash(int i, int j, int k) {
    return (abs(541*(long)i + 79*(long)j + 31*(long)k) % maxNumHashValues);
}

void BoxHash::insertBox(Box *cell) {
    long h = computeHash(cell->x, cell->y, cell->z);

    if (cellMap.find(h) == cellMap.end()) {
        std::vector<Box*> newChain;
        std::pair<long,std::vector<Box*>> pair(h, newChain);
        cellMap.insert(pair);
    }

    cellMap[h].push_back(cell);
}

void BoxHash::removeBox(Box *cell) {
    int i = cell->x;
    int j = cell->y;
    int k = cell->z;
    long h = computeHash(i, j, k);

    if (cellMap.find(h) == cellMap.end()) {
        spdlog::info( "cant find cell %d, %d, %d, %d", i , j , k , h) ;
        return;
    }

    // remove from hash chain
    bool isRemoved = false;
    std::vector<Box*> chain = cellMap[h];
    for (int idx=0; idx<(int)chain.size(); idx++) {
        Box *c = (cellMap[h])[idx];
        if (c->x == i && c->y == j && c->z == k) {
            cellMap[h].erase(cellMap[h].begin() + idx);
            isRemoved = true;
            break;
        }
    }

    if (!isRemoved) {
        spdlog::info( "Could not find/remove Box %d, %d, %d", i , j , k ) ;
    }

    // remove chain from map if empty
    if (chain.size() == 0) {
        cellMap.erase(h);
    }

}

Box* BoxHash::getBox(int i, int j, int k) {
    long h = computeHash(i, j, k);

    Box *c;
    std::vector<Box*> chain = cellMap[h];
    for (int idx=0; idx<(int)chain.size(); idx++) {
        c = chain[idx];
        if (c->x == i && c->y == j and c->z == k) {
            return c;
        }
    }

    return c;
}

Box* BoxHash::findBox(int i, int j, int k, bool *isBoxFound) {
    long h = computeHash(i, j, k);

    Box *c;
    std::vector<Box*> chain = cellMap[h];
    for (int idx=0; idx<(int)chain.size(); idx++) {
        c = chain[idx];
        if (c->x == i && c->y == j and c->z == k) {
            *isBoxFound = true;
            return c;
        }
    }

    *isBoxFound = false;
    return c;
}

bool BoxHash::isBoxInHash(int i, int j, int k) {
    long h = computeHash(i, j, k);

    if (cellMap.find(h) == cellMap.end()) {
        return false;
    }

    Box *c;
    std::vector<Box*> chain = cellMap[h];
    for (int idx=0; idx<(int)chain.size(); idx++) {
        c = chain[idx];
        if (c->x == i && c->y == j and c->z == k) {
            return true;
        }
    }

    return false;
}



void BoxHash::getBoxs(std::vector<Box*> *cells) {
    for (std::pair<int, std::vector<Box*>> pair: cellMap) {
        for (int i=0; i < (int)pair.second.size(); i++) {
            cells->push_back(pair.second[i]);
        }
    }

}










