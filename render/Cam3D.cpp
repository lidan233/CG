//
// Created by lidan on 2021/4/28.
//

#include "Cam3D.h"
float Cam3D::getFieldOfView() {
    return fov * (3.141592653/180.0);
}

float Cam3D::getAspectRatio() {
    return width/height;
}