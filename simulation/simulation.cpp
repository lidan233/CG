//
// Created by lidan on 2021/4/30.
//

#include "simulation.h"

SPHFluidSimulation::SPHFluidSimulation()
{
    h = 1.0;
}

SPHFluidSimulation::SPHFluidSimulation(double smoothingRadius)
{
    h = smoothingRadius;
    grid = SpatialGrid(h);

    initSimulationConstants();
    initKernelConstants();
    initializeBoundaryParticles();

    cameraPosition = Vec3(0.0, 0.0, 0.0);
    fluidGradient = Gradients::getSkyblueGradient();
}

void SPHFluidSimulation::initSimulationConstants() {
    hsq = h*h;
    ratioOfSpecificHeats             = 1.0,
    pressureCoefficient              = 20.0,
    initialDensity                   = 20.0,
    viscosityCoefficient             = 0.018,
    particleMass                     = 1.0,
    maximumVelocity                  = 75.0,
    maximumAcceleration              = 75.0,
    motionDampingCoefficient         = 0.0,
    boundaryDampingCoefficient       = 0.6,
    gravityMagnitude                 = 9.8,
    isMotionDampingEnabled           = true,
    isBoundaryParticlesEnabled       = false,
    displayConsoleOutput             = true,


    minColorDensity                = 0.0,
    maxColorDensity                = 100,
    maxColorVelocity               = 100.0,
    maxColorAcceleration           = 100.0,
    colorArrivalRadius             = 0.1,
    stuckToBoundaryRadius          = 0.01,
    stuckToBoundaryAlphaVelocity   = 3.0,


    gravityForce = Vec3(0.0, -gravityMagnitude, 0.0);
}

void SPHFluidSimulation::updateFluidConstants() {
    initSimulationConstants();
}

void SPHFluidSimulation::initKernelConstants() {
    double pi = 3.1415926535897;

    poly6Coefficient = 315.0/(64.0*pi*powf(h, 9.0));
    spikeyGradCoefficient = -45.0/(pi*powf(h, 6.0));
    viscosityLaplacianCoefficient = 45.0/(pi*powf(h, 6.0f));
}

void SPHFluidSimulation::setBounds(double _xmin, double _xmax,
                                   double _ymin, double _ymax,
                                   double _zmin, double _zmax) {
    xmin = _xmin; xmax = _xmax;
    ymin = _ymin; ymax = _ymax;
    zmin = _zmin; zmax = _zmax;
    initializeBoundaryParticles();

    isEnforcingFluidParticlePositionBoundsThisTimeStep = false;
}

void SPHFluidSimulation::setDampingConstant(double c) {
    motionDampingCoefficient = c;
}

void SPHFluidSimulation::setTexture(GLuint *tex) {
    texture = tex;
    isTextureInitialized = true;
}

void SPHFluidSimulation::setCamera(Cam3D *cam) {
    camera = cam;
    isCameraInitialized = true;
}

std::string  SPHFluidSimulation::getTimingData() {
    double total = neighbourSearchTime + simulationTime +
                   graphicsUpdateTime + graphicsDrawTime;


    std::string str = std::to_string(total) + " " +
                    std::to_string(neighbourSearchTime) + " " +
                    std::to_string(simulationTime) + " " +
                    std::to_string(graphicsUpdateTime) + " " +
                    std::to_string(graphicsDrawTime);

    return str;
}

std::vector<Particle*> SPHFluidSimulation::getFluidParticles() {
    return fluidParticles;
}

std::vector<Particle*> SPHFluidSimulation::getObstacleParticles() {
    return obstacleParticles;
}

std::vector<Particle*> SPHFluidSimulation::getAllParticles() {
    return allParticles;
}

float SPHFluidSimulation::getParticleSize() {
    return h;
}

float SPHFluidSimulation::getInitialDensity() {
    return initialDensity;
}

void SPHFluidSimulation::addFluidParticles(std::vector<Vec3> points) {
    for (uint i=0; i<points.size(); i++) {
        addFluidParticle(points[i], Vec3(0.0, 0.0, 0.0));
    }
}

int SPHFluidSimulation::addObstacleParticles(std::vector<Vec3> points) {
    Obstacle *obs = new Obstacle();
    obs->id = getUniqueObstacleID();

    Particle *p;
    for (uint i=0; i<points.size(); i++) {
        p = addObstacleParticle(points[i]);
        obs->particles.push_back(p);
    }
    obstacles.push_back(obs);

    std::pair<int,Obstacle*> pair(obs->id, obs);
    obstaclesByID.insert(pair);

    return obs->id;
}

void SPHFluidSimulation::removeObstacle(int id) {
    if (obstaclesByID.find(id) == obstaclesByID.end()) {
        return;
    }
    isParticleRemoved = true;

    Obstacle *o = obstaclesByID[id];
    obstaclesByID.erase(id);

    Particle *p;
    for (uint i=0; i<o->particles.size(); i++) {
        p = o->particles[i];
        grid.removePoint(p->gridId);
        particlesByGridID.erase(p->gridId);
        p->isRemoved = true;
    }

    for (uint i=0; i<obstacles.size(); i++) {
        Obstacle *op = obstacles[i];
        if (op->id == o->id) {
            obstacles.erase(obstacles.begin() + i);
            break;
        }
    }

    delete o;
}

void SPHFluidSimulation::setObstaclePosition(int id, Vec3 pos) {
    if (obstaclesByID.find(id) == obstaclesByID.end()) {
        return;
    }

    Obstacle *o = obstaclesByID[id];
    translateObstacle(id, pos - o->pos);
}

void SPHFluidSimulation::translateObstacle(int id, Vec3 trans) {
    if (obstaclesByID.find(id) == obstaclesByID.end()) {
        return;
    }
    Obstacle *o = obstaclesByID[id];

    Particle *sp;
    for (uint i=0; i<o->particles.size(); i++) {
        sp = o->particles[i];
        sp->pos += trans;
    }

    o->pos += trans;
}

void SPHFluidSimulation::rotateObstacle(int id, Quaternion q) {
    if (obstaclesByID.find(id) == obstaclesByID.end()) {
        return;
    }

    Obstacle *o = obstaclesByID[id];
    glm::mat4 rot = q.getRotationMatrix();

    Particle *sp;
    Vec3 op = o->pos;
    glm::vec4 p;
    for (uint i=0; i<o->particles.size(); i++) {
        sp = o->particles[i];
        p = glm::vec4(sp->pos[0] - op[0],
                      sp->pos[1] - op[1],
                      sp->pos[2] - op[2], 1.0);
        p = rot*p;

        sp->pos = Vec3(p[0] + op[0], p[1] + op[1], p[2] + op[2]);
    }

}

int SPHFluidSimulation::getUniqueObstacleID() {
    int id = currentObstacleID;
    currentObstacleID++;
    return id;
}

void SPHFluidSimulation::initializeBoundaryParticles() {
    if (!isBoundaryParticlesEnabled) {
        return;
    }

    if (isBoundaryObstacleInitialized) {
        removeObstacle(boundaryObstacleID);
    }

    std::vector<Vec3> obsPoints;
    Vec3 xdir = Vec3(1.0, 0.0, 0.0);
    Vec3 ydir = Vec3(0.0, 1.0, 0.0);
    Vec3 zdir = Vec3(0.0, 0.0, 1.0);
    float xwidth = xmax - xmin;
    float ywidth = ymax - ymin;
    float zwidth = zmax - zmin;
    float pad = h;
    int layers = 1;
    bool isStaggered = true;

    // x-z (ymin) plane
    Vec3 o = Vec3(xmin + 0.5*xwidth, 0.0, zmin + 0.5*zwidth);
    std::vector<Vec3> points = createPointPanel(xwidth, zwidth, pad,
                                                            layers, xdir, zdir, isStaggered);
    points = translatePoints(points, o);
    obsPoints = mergePoints(obsPoints, points);

    // x-z (y-max) plane
    o = Vec3(xmin + 0.5*xwidth, ymin + ywidth, zmin + 0.5*zwidth);
    points = createPointPanel(xwidth, zwidth, pad, layers, xdir, zdir, isStaggered);
    points = translatePoints(points, o);
    obsPoints = mergePoints(obsPoints, points);

    // x-y (z-min) plane
    o = Vec3(xmin + 0.5*xwidth, ymin + 0.5*xwidth, 0.0);
    points = createPointPanel(xwidth, ywidth, pad, layers, xdir, ydir, isStaggered);
    points = translatePoints(points, o);
    obsPoints = mergePoints(obsPoints, points);

    // x-y (z-max) plane
    o = Vec3(xmin + 0.5*xwidth, ymin + 0.5*xwidth, zmin + zwidth);
    points = createPointPanel(xwidth, ywidth, pad, layers, xdir, ydir, isStaggered);
    points = translatePoints(points, o);
    obsPoints = mergePoints(obsPoints, points);

    // y-z (x-min) plane
    o = Vec3(0.0 ,ymin + 0.5*xwidth, zmin + 0.5*zwidth);
    points = createPointPanel(ywidth, zwidth, pad, layers, ydir, zdir, isStaggered);
    points = translatePoints(points, o);
    obsPoints = mergePoints(obsPoints, points);

    // y-z (x-max) plane
    o = Vec3(xmin + xwidth, ymin + 0.5*xwidth, zmin + 0.5*zwidth);
    points = createPointPanel(ywidth, zwidth, pad, layers, ydir, zdir, isStaggered);
    points = translatePoints(points, o);
    obsPoints = mergePoints(obsPoints, points);

    boundaryObstacleID = addObstacleParticles(obsPoints);

    Obstacle *obs = obstaclesByID[boundaryObstacleID];
    obs->isVisiable = false;

    for (uint i=0; i<obs->particles.size(); i++) {
        obs->particles[i]->isVisible = false;
    }

    isBoundaryObstacleInitialized = true;
}

Particle* SPHFluidSimulation::createParticle(Vec3 pos, Vec3 velocity) {
    Particle *s = new Particle();

    s->pos = pos;
    s->ppos = pos;
    s->velocity = velocity;
    s->velocityAtMiddle = Vec3(0.0, 0.0, 0.0);
    s->isHalfStep = false;
    s->SPHVelocity = Vec3(0.0, 0.0, 0.0);
    s->soundSpeed = 0;
    s->acceleration = Vec3(0.0, 0.0, 0.0);

    // Create pressure offset from initial pressure
    // uniform densities will cause uniform pressure of 0, meaning no acceleration
    // of system
    s->density = initialDensity;
    s->densityVelocity = 0.0;

    // mass of sphere
    s->mass = particleMass;

    // initial pressure will be calculated once all particles are in place
    s->pressure = 0.0;

    // graphics
    s->color = Vec3(1.0, 1.0, 1.0);
    s->colorDensity = s->density;

    return s;
}

Particle* SPHFluidSimulation::createObstacleParticle(Vec3 pos) {
    Particle *p = createParticle(pos, Vec3(0.0, 0.0, 0.0));
    return p;
}

void SPHFluidSimulation::addFluidParticle(Vec3 pos, Vec3 velocity) {
    Particle *sp = createParticle(pos, velocity);
    sp->isObstacle = false;

    sp->gridId = grid.insertPoint(sp->pos);
    std::pair<int,Particle*> pair(sp->gridId, sp);
    particlesByGridID.insert(pair);

    fluidParticles.push_back(sp);
    allParticles.push_back(sp);
}

Particle* SPHFluidSimulation::addObstacleParticle(Vec3 pos) {
    Particle *sp = createObstacleParticle(pos);
    sp->isObstacle = true;

    sp->gridID = grid.insertPoint(sp->pos);
    std::pair<int,Particle*> pair(sp->gridId, sp);
    particlesByGridID.insert(pair);

    obstacleParticles.push_back(sp);
    allParticles.push_back(sp);

    return sp;
}

inline double SPHFluidSimulation::evaluateSpeedOfSound(Particle *sp) {
    double sqr = ratioOfSpecificHeats*(sp->pressure)/sp->density;
    if (sqr < 0) {
        sqr = -sqr;
    }
    return sqrt(sqr);
}

inline double SPHFluidSimulation::evaluateSpeedOfSoundSquared(Particle *sp) {
    if (sp->density < 0.00001) {
        return 0.0;
    }
    return ratioOfSpecificHeats*(sp->pressure)/sp->density;
}
void SPHFluidSimulation::removeParticlesMarkedForRemoval() {
    if (allParticles.size() == 0 || !isParticleRemoved) {
        return;
    }

    Particle *p;
    for (int i=(int)fluidParticles.size() - 1; i>=0; i--) {
        if (fluidParticles[i]->isRemoved) {
            p = fluidParticles[i];
            fluidParticles.erase(fluidParticles.begin() + i);
        }
    }

    for (int i=(int)obstacleParticles.size() - 1; i>=0; i--) {
        if (obstacleParticles[i]->isRemoved) {
            p = obstacleParticles[i];
            obstacleParticles.erase(obstacleParticles.begin() + i);
        }
    }

    for (int i=(int)allParticles.size() - 1; i>=0; i--) {
        if (allParticles[i]->isRemoved) {
            p = allParticles[i];
            allParticles.erase(allParticles.begin() + i);
            delete p;
        }
    }

    isParticleRemoved = false;
}


void SPHFluidSimulation::updateGrid() {
    Particle *sp;
    for (uint i=0; i<allParticles.size(); i++) {
        sp = allParticles[i];
        grid.movePoint(sp->gridId, sp->pos);
    }

    grid.update();
}

double SPHFluidSimulation::calculateTimeStep() {
    double maxvsq = 0.0;         // max velocity squared
    double maxcsq = 0.0;         // max speed of sound squared
    double maxasq = 0.0;         // max accelleration squared
    Particle *sp;
    for (uint i=0; i<fluidParticles.size(); i++) {
        sp = fluidParticles[i];
        double vsq = dot(sp->velocity, sp->velocity);
        double asq = dot(sp->acceleration, sp->acceleration);
        double csq = evaluateSpeedOfSoundSquared(sp);
        if (vsq > maxvsq) { maxvsq = vsq; }
        if (csq > maxcsq) { maxcsq = csq; }
        if (asq > maxasq) { maxasq = asq; }
    }

    double maxv = sqrt(maxvsq);
    double maxc = sqrt(maxcsq);
    double maxa = sqrt(maxasq);

    double vStep = courantSafetyFactor*h / fmax(1.0, maxv);
    double cStep = courantSafetyFactor*h / maxc;
    double aStep = sqrt(h/maxa);
    double tempMin = fmin(vStep, cStep);

    //qDebug() << maxv << maxa << maxc;

    return fmax(minTimeStep, fmin(tempMin, aStep));
}

void SPHFluidSimulation::updateNearestNeighbours() {
    Particle *sp;
    for (uint i=0; i<allParticles.size(); i++) {
        sp = allParticles[i];
        sp->neighbors.clear();
        std::vector<int> refs = grid.getIDsInRadiusOfPoint(sp->gridId, h);
        for (uint j=0; j<refs.size(); j++) {
            sp->neighbors.push_back(particlesByGridID[refs[j]]);
        }
    }
}

void SPHFluidSimulation::updateObstacleVelocity(double dt) {
    Obstacle *obs;
    Particle *sp;
    Vec3 trans;
    for (uint i=0; i<obstacles.size(); i++) {
        obs = obstacles[i];

        for (uint j=0; j<obs->particles.size(); j++) {
            sp = obs->particles[j];

            if (sp->pos == sp->ppos) {
                sp->velocity = Vec3(0.0, 0.0, 0.0);
            }

            trans = sp->pos - sp->ppos;

            double dist = trans.length() ;
            double eps = 0.00000001;
            if (dist > eps) {
                float speed = fmin((dist/dt), maximumVelocity);
                sp->velocity = (trans / (float)dist) * speed;
            } else {
                sp->velocity = Vec3(0.0, 0.0, 0.0);
            }

            sp->ppos = sp->pos;
        }
    }
}

void SPHFluidSimulation::updateFluidDensityAndPressure() {
    // once we find a particle's density, we can find it's pressure
    Particle *pi, *pj;
    Vec3 r;
    for (uint i=0; i<allParticles.size(); i++) {
        pi = allParticles[i];
        double density = 0.0;

        for (uint j=0; j<pi->neighbors.size(); j++) {
            pj = pi->neighbors[j];
            r = pi->pos - pj->pos;
            double distsq = dot(r, r);
            double diff = hsq - distsq;
            density += pj->mass*poly6Coefficient*diff*diff*diff;
        }

        pi->density = fmax(density, initialDensity);  // less than initial density
        // produces negative pressures
        pi->pressure = pressureCoefficient*(pi->density - initialDensity);
    }
}

void SPHFluidSimulation::updateFluidAcceleration() {
    // we know particle density and pressure, so acceleration can be found
    Particle *pi, *pj;
    Vec3 acc;
    Vec3 r;
    Vec3 vdiff;
    for (uint i=0; i<fluidParticles.size(); i++) {
        pi = fluidParticles[i];
        acc = Vec3(0.0, 0.0, 0.0);

        for (uint j=0; j<pi->neighbors.size(); j++) {
            pj = pi->neighbors[j];
            r = pi->pos - pj->pos;
            double dist = r.length();

            if (dist == 0.0) { continue; }
            float inv = 1/dist;
            r = inv*r;

            // acceleration due to pressure
            float diff = h - dist;
            float spikey = spikeyGradCoefficient*diff*diff;
            float massRatio = pj->mass/pi->mass;
            float pterm = (pi->pressure + pj->pressure) / (2*pi->density*pj->density);
            acc = acc - Vec3((float)(massRatio*pterm*spikey)*r);

            // acceleration due to viscosity
            if (!pj->isObstacle) {
                float lap = viscosityLaplacianCoefficient*diff;
                vdiff = pj->velocity - pi->velocity;
                acc += (float)(viscosityCoefficient*massRatio*(1/pj->density)*lap)*vdiff;
            }
        }

        // acceleration due to gravity
        acc += gravityForce;

        // acceleration due to simulation bounds
        acc += calculateBoundaryAcceleration(pi);

        // motion damping;
        double mag = acc.length();
        if (isMotionDampingEnabled) {
            Vec3 damp = pi->velocity * (float) motionDampingCoefficient;
            if (damp.length() > mag) {
                acc = Vec3(0.0, 0.0, 0.0);
            } else {
                acc = acc - damp;
            }
        }


        if (mag > maximumAcceleration) {
            acc = (acc / (float)mag) * (float)maximumAcceleration;
        }

        pi->acceleration = acc;

    }
}

void SPHFluidSimulation::enforceFluidParticlePositionBounds(Particle *p) {
    if (!isEnforcingFluidParticlePositionBoundsThisTimeStep) {
        isEnforcingFluidParticlePositionBoundsThisTimeStep = true;
        return;
    }

    double eps = 0.001;
    float d = boundaryDampingCoefficient;
    if (p->pos[0] < xmin) {
        p->pos = Vec3(xmin + eps, p->pos[1], p->pos[2]);
        p->velocity = Vec3(-d*p->velocity[0], p->velocity[1], p->velocity[2]);
    } else if (p->pos[0] > xmax) {
        p->pos = Vec3(xmax - eps, p->pos[1], p->pos[2]);
        p->velocity = Vec3(-d*p->velocity[0], p->velocity[1], p->velocity[2]);
    }

    if (p->pos[1] < ymin) {
        p->pos = Vec3(p->pos[0], ymin + eps, p->pos[2]);
        p->velocity = Vec3(p->velocity[0], -d*p->velocity[1], p->velocity[2]);
    } else if (p->pos[1] > ymax) {
        p->pos = Vec3(p->pos[0], ymax - eps, p->pos[2]);
        p->velocity = Vec3(p->velocity[0], -d*p->velocity[1], p->velocity[2]);
    }

    if (p->pos[2] < zmin) {
        p->pos = Vec3(p->pos[0], p->pos[1], zmin + eps);
        p->velocity = Vec3(p->velocity[0], p->velocity[1], -d*p->velocity[2]);
    } else if (p->pos[2] > zmax) {
        p->pos = Vec3(p->pos[0], p->pos[1], zmax - eps);
        p->velocity = Vec3(p->velocity[0], p->velocity[1], -d*p->velocity[2]);
    }

}

void SPHFluidSimulation::updateFluidPosition(double dt) {
    Particle *p;
    for (uint i=0; i<fluidParticles.size(); i++) {
        p = fluidParticles[i];

        // calculate velocity at half timestep interval for leapfrog integration
        if (p->isHalfStep) {
            p->velocityAtMiddle += (float)dt * p->acceleration;
        } else {
            p->velocityAtMiddle = p->velocity + (float)(0.5*dt)*p->acceleration;
            p->isHalfStep = true;
        }

        // new position calculated with half time step for leap frog integration
        p->pos += (float)dt * p->velocityAtMiddle;

        // update sph velocity by advancing half time step velocty by 1/2 interval
        p->velocity = p->isHalfStep + (float)(0.5*dt) * p->acceleration;
        if (p->velocity.length() > maximumVelocity) {
            Vec3 unit = glm::normalize(p->velocity);
            p->velocity = (float)maximumVelocity*unit;
        }

        enforceFluidParticlePositionBounds(p);
    }

}

Vec3 SPHFluidSimulation::calculateBoundaryAcceleration(Particle *sp) {
    double r = boundaryForceRadius;
    double minf = minBoundaryForce;
    double maxf = maxBoundaryForce;

    Vec3 p = sp->pos;
    Vec3 acceleration = Vec3(0.0, 0.0, 0.0);

    if (p[0] < xmin + r) {
        double dist = fmax(0.0, p[0] - xmin);
        double force = lerp(maxf, minf, dist/r);
        acceleration += Vec3(force/sp->mass, 0.0, 0.0);
    } else if (p[0] > xmax - r) {
        double dist = fmax(0.0, xmax - p[0]);
        double force = lerp(maxf, minf, dist/r);
        acceleration += Vec3(-force/sp->mass, 0.0, 0.0);
    }

    if (p[1] < ymin + r) {
        double dist = fmax(0.0, p[1] - ymin);
        double force = lerp(maxf, minf, dist/r);
        acceleration += Vec3(0.0, force/sp->mass, 0.0);
    } else if (p[1] > ymax - r) {
        double dist = fmax(0.0, ymax - p[1]);
        double force = lerp(maxf, minf, dist/r);
        acceleration += Vec3(0.0, -force/sp->mass, 0.0);
    }

    if (p[2] < zmin + r) {
        double dist = fmax(0.0, p[2] - zmin);
        double force = lerp(maxf, minf, dist/r);
        acceleration += Vec3(0.0, 0.0, force/sp->mass);
    } else if (p[2] > zmax - r) {
        double dist = fmax(0.0, zmax - p[2]);
        double force = lerp(maxf, minf, dist/r);
        acceleration += Vec3(0.0, 0.0, -force/sp->mass);
    }

    return acceleration;
}

void SPHFluidSimulation::updateZSortingDistance() {
    if (!isCameraInitialized || !isTextureInitialized) {
        return;
    }

    Vec3 r;
    Vec3 cpos = camera->getPosition();
    for (uint i=0; i<allParticles.size(); i++) {
        r = cpos - allParticles[i]->pos;
        allParticles[i]->zdistance = dot(r, r);
    }
}

bool SPHFluidSimulation::isFluidParticleStuckToBoundary(Particle *sp) {
    double r = stuckToBoundaryRadius;
    bool isStuck = false;
    Vec3 p = sp->pos;

    if (p[0] < xmin + r || p[0] > xmax - r ||
        p[1] < ymin + r || p[1] > ymax - r ||
        p[2] < zmin + r || p[2] > zmax - r) {
        isStuck = true;
    }

    return isStuck;
}

void SPHFluidSimulation::updateFluidParticleColorDensity(double dt, Particle *sp) {
    // update color density
    // find color density target
    double targetDensity = sp->density;
    double currentDensity = sp->colorDensity;
    double desired = targetDensity - currentDensity;
    if (fabs(desired) < colorArrivalRadius) {
        double r = fabs(desired) / colorArrivalRadius;
        double newMag = lerp(0, maxColorVelocity, r);
        if (desired > 0) {
            desired = newMag;
        } else {
            desired = -newMag;
        }
    } else {
        if (desired > 0) {
            desired = maxColorVelocity;
        } else {
            desired = -maxColorVelocity;
        }
    }

    // color density acceleration
    double acc = desired - sp->colorVelocity;
    if (fabs(acc) > maxColorAcceleration) {
        if (acc > 0) {
            acc = maxColorAcceleration;
        } else {
            acc = -maxColorAcceleration;
        }
    }

    // update color density velocity and position from acceleration
    sp->colorVelocity += acc*dt;
    sp->colorDensity += sp->colorVelocity*dt;
    sp->colorDensity = fmax(0.0, sp->colorDensity);
}

Vec3 SPHFluidSimulation::calculateFluidParticleColor(Particle *sp) {
    int idxOffset = 50;
    int maxIdx = fluidGradient.size() - 1;
    int minIdx = fmin(0 + idxOffset, maxIdx);
    double minD = minColorDensity;
    double maxD = maxColorDensity;
    double invDiff = 1/(maxD - minD);
    Vec3 targetColor;
    std::array<double, 3> cmin;
    std::array<double, 3> cmax;

    // update color based upon color density
    double d = sp->colorDensity;
    double r = 1 - (d - minD) * invDiff;
    r = fmax(0.0, r);
    r = fmin(1.0, r);

    // find target color
    double lerpVal = lerp(minIdx, maxIdx, r);
    int minCIdx = floor(lerpVal);
    int maxCIdx = ceil(lerpVal);
    if (minCIdx == maxCIdx) {
        cmin = fluidGradient[minCIdx];
        targetColor = Vec3(cmin[0], cmin[1], cmin[2]);
    } else {
        cmin = fluidGradient[minCIdx];
        cmax = fluidGradient[maxCIdx];
        double f = lerpVal - minCIdx;

        targetColor = Vec3(lerp(cmin[0], cmax[0], f),
                                lerp(cmin[1], cmax[1], f),
                                lerp(cmin[2], cmax[2], f));
    }

    return targetColor;
}

void SPHFluidSimulation::updateFluidParticleAlpha(double dt, Particle *sp) {
    sp->isStuckInBoundary = isFluidParticleStuckToBoundary(sp);

    if (sp->isStuckInBoundary && sp->boundaryAlphaValue > 0.0) {
        sp->boundaryAlphaValue -= stuckToBoundaryAlphaVelocity*dt;
        sp->boundaryAlphaValue = fmax(0.0, sp->boundaryAlphaValue);
        sp->alpha = smoothstep(sp->boundaryAlphaValue);
    } else if (!sp->isStuckInBoundary && sp->boundaryAlphaValue < 1.0) {
        sp->boundaryAlphaValue += stuckToBoundaryAlphaVelocity*dt;
        sp->boundaryAlphaValue = fmin(1.0, sp->boundaryAlphaValue);
        sp->alpha = smoothstep(sp->boundaryAlphaValue);
    }
}

void SPHFluidSimulation::updateFluidColor(double dt) {
    if (dt == 0) { return; }

    Particle *sp;

    // find target color index in gradient. Lerp between color indices.
    // Move color to target based upon smoothed value of particle density.
    // Particle density is smoothed by keeping track of acceleration and velocity
    // of smoothed value changes
    for (uint i=0; i<fluidParticles.size(); i++) {
        sp = fluidParticles[i];

        updateFluidParticleColorDensity(dt, sp);
        updateFluidParticleAlpha(dt, sp);
        sp->color = calculateFluidParticleColor(sp);

        // update fluid alpha.

    }
}

bool compareByZDistance(const Particle *p1, const Particle *p2) {
    return p1->zdistance > p2->zdistance;
}

void SPHFluidSimulation::updateGraphics(double dt) {
    updateZSortingDistance();
    updateFluidColor(dt);

    std::sort(allParticles.begin(), allParticles.end(), compareByZDistance);
}

void SPHFluidSimulation::update(float dt) {
    updateFluidConstants();
    removeParticlesMarkedForRemoval();

    Watch neighbourTimer = Watch();
    Watch simulationTimer = Watch();
    Watch graphicsTimer = Watch();

    int numSteps = 0;
    double timeLeft = dt;
    while (timeLeft > 0.0) {

        neighbourTimer.start();
        updateGrid();
        updateNearestNeighbours();
        neighbourTimer.stop();

        simulationTimer.start();
        updateFluidDensityAndPressure();
        updateFluidAcceleration();

        // calculate next time step
        double timeStep = calculateTimeStep();
        timeLeft -= timeStep;
        if (timeLeft < 0.0) {
            timeStep = timeStep + timeLeft;
            timeLeft = 0.0;
        }
        numSteps += 1;

        updateFluidPosition((double)timeStep);
        updateObstacleVelocity((double)timeStep);
        simulationTimer.stop();
    }
    graphicsTimer.start();
    updateGraphics(dt);
    graphicsTimer.stop();

    neighbourSearchTime = neighbourTimer.getTime();
    simulationTime = simulationTimer.getTime();
    graphicsUpdateTime = graphicsTimer.getTime();

    double total = neighbourSearchTime + simulationTime;
    if (displayConsoleOutput) {
        std::cout<< "total: " << std::to_string(total)
                                 + " pct neighbour: " +
                                 std::to_string((100*(neighbourSearchTime/total))) +
                                 " #particles: " + std::to_string(allParticles.size());

    }

}

//void SPHFluidSimulation::drawBounds() {
//    double w = xmax - xmin;
//    double h = ymax - ymin;
//    double d = zmax - zmin;
//    glColor3f(0.0, 0.0, 0.0);
//    utils::drawWireframeCube(Vec3(xmin + w/2, ymin + h/2, zmin + d/2),
//                             w, h, d);
//}
//
//void SPHFluidSimulation::draw() {
//    Watch drawTimer = Watch();
//    drawTimer.start();
//
//    //grid.draw();
//
//    Particle *sp;
//    float size = 0.5*h;
//    for (uint i=0; i<allParticles.size(); i++) {
//        sp = allParticles[i];
//        Vec3 p = sp->pos;
//
//        if (sp->isObstacle) {
//            glColor3f(0.5, 0.5, 0.5);
//        } else {
//            if (isHiddenBoundaryParticlesEnabled) {
//                glColor4f(sp->color[0], sp->color[1], sp->color[2], sp->alpha);
//            } else {
//                glColor3f(sp->color[0], sp->color[1], sp->color[2]);
//            }
//        }
//
//        if (sp->isVisible) {
//            utils::drawBillboard(camera, texture, p, size);
//        }
//    }
//
//    drawBounds();
//
//    drawTimer.stop();
//    graphicsDrawTime = drawTimer.getTime();
//
//}
