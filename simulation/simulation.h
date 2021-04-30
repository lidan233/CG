//
// Created by lidan on 2021/4/30.
//

#ifndef CG_SIMULATION_H
#define CG_SIMULATION_H
#include <vector>
#include <time.h>
#include <string>
#include <cmath>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include "EigenLidan.h"
#include "Particle.h"
#include "Watch.h"
#include "quaternion.h"
#include "gradients.h"
#include "Cam3D.h"
#include "spatialgrid.h"
#include "geometry.h"

class SPHFluidSimulation {
public:
    SPHFluidSimulation();
    SPHFluidSimulation(double smoothingRadius);
    void update(float dt);
    void draw();
    void drawBounds();

    // user functions
    void addFluidParticles(std::vector<Vec3> points);
    void addFluidParticle(Vec3 pos, Vec3 velocity);
    int addObstacleParticles(std::vector<Vec3> points);
    void removeObstacle(int id);
    void setBounds(double xmin, double xmax,
                   double ymin, double ymax,
                   double zmin, double zmax);
    void setDampingConstant(double c);
    void setTexture(GLuint *tex);
    void setCamera(Cam3D *cam);
    std::string getTimingData();
    std::vector<Particle*> getFluidParticles();
    std::vector<Particle*> getObstacleParticles();
    std::vector<Particle*> getAllParticles();
    float getParticleSize();
    float getInitialDensity();
    void setObstaclePosition(int id, Vec3 pos);
    void translateObstacle(int id, Vec3 trans);
    void rotateObstacle(int id, Quaternion q);

private:

    // init
    void initSimulationConstants();
    void initKernelConstants();

    Particle* createParticle(Vec3 pos, Vec3 velocity);
    Particle* createObstacleParticle(Vec3 pos);
    Particle* addObstacleParticle(Vec3 pos);
    int getUniqueObstacleID();
    int currentObstacleID = 0;

    // simulation
    inline double evaluateSpeedOfSound(Particle *sp);
    inline double evaluateSpeedOfSoundSquared(Particle *sp);
    void removeParticlesMarkedForRemoval();
    void initializeBoundaryParticles();
    void updateFluidConstants();
    void updateObstacleVelocity(double dt);
    void updateGrid();
    void updateNearestNeighbours();
    void updateFluidDensityAndPressure();
    Vec3 calculateBoundaryAcceleration(Particle *sp);
    void updateFluidAcceleration();
    void updateFluidPosition(double dt);
    void enforceFluidParticlePositionBounds(Particle *p);
    bool isEnforcingFluidParticlePositionBoundsThisTimeStep = false;
    double calculateTimeStep();

    // graphics
    void updateGraphics(double dt);
    void updateZSortingDistance();
    void updateFluidColor(double dt);
    void updateFluidParticleColorDensity(double dt, Particle *sp);
    void updateFluidParticleAlpha(double dt, Particle *sp);
    Vec3 calculateFluidParticleColor(Particle *sp);
    bool isFluidParticleStuckToBoundary(Particle *sp);
    std::vector<std::array<double, 3>> fluidGradient;
    double maxColorVelocity = 1.0;
    double maxColorAcceleration = 1.0;
    double minColorDensity = 0.0;
    double maxColorDensity = 100.0;
    double colorArrivalRadius = 0.5;
    double stuckToBoundaryRadius = 0.01;
    double stuckToBoundaryAlphaVelocity = 1.0;
    bool isTextureInitialized = false;
    GLuint *texture;
    bool isCameraInitialized = false;
    Cam3D *camera;

    // simulation constants
    double h;                              // smoothing radius
    double hsq;                            // radius squared
    Vec3 gravityForce;
    double courantSafetyFactor = 1.0;
    double minTimeStep = 1.0/240.0;
    double gravityMagnitude;
    double initialDensity;
    double pressureCoefficient;
    double particleMass;
    bool isMotionDampingEnabled = false;
    bool displayConsoleOutput = false;
    double motionDampingCoefficient;
    double boundaryDampingCoefficient;
    double ratioOfSpecificHeats;
    double viscosityCoefficient;
    double maximumVelocity;
    double maximumAcceleration;


    // kernel constants
    double poly6Coefficient;
    double spikeyGradCoefficient;
    double viscosityLaplacianCoefficient;

    // boundary constraints
    double boundaryForceRadius = 0.1;
    double minBoundaryForce = 0.0;
    double maxBoundaryForce = 0.0;
    double xmin = 0.0;
    double xmax = 1.0;
    double ymin = 0.0;
    double ymax = 1.0;
    double zmin = 0.0;
    double zmax = 1.0;
    int boundaryObstacleID;
    bool isBoundaryObstacleInitialized = false;
    bool isHiddenBoundaryParticlesEnabled = true;
    bool isBoundaryParticlesEnabled = false;

    SpatialGrid grid;
    std::vector<Particle*> fluidParticles;
    std::vector<Particle*> obstacleParticles;
    std::vector<Particle*> allParticles;
    std::vector<Obstacle*> obstacles;
    std::unordered_map<int,Particle*> particlesByGridID;
    std::unordered_map<int,Obstacle*> obstaclesByID;
    bool isParticleRemoved = false;
    Vec3 cameraPosition;

    // timing metrics
    double neighbourSearchTime = 0.0;
    double simulationTime = 0.0;
    double graphicsUpdateTime = 0.0;
    double graphicsDrawTime = 0.0;

};


#endif //CG_SIMULATION_H
