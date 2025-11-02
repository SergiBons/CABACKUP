#include "scenecloth.h"
#include "glutils.h"
#include "model.h"
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLBuffer>


SceneCloth::SceneCloth() {
    widget = new WidgetCloth();
    connect(widget, SIGNAL(updatedParameters()), this, SLOT(updateSimParams()));
    connect(widget, SIGNAL(freeAnchors()), this, SLOT(freeAnchors()));
}

SceneCloth::~SceneCloth() {
    if (widget)      delete widget;
    if (shaderPhong) delete shaderPhong;
    if (vaoSphereS)  delete vaoSphereS;
    if (vaoSphereL)  delete vaoSphereL;
    if (vaoCube)     delete vaoCube;
    if (vaoMesh)     delete vaoMesh;
    if (vboMesh)     delete vboMesh;
    if (iboMesh)     delete iboMesh;

    system.deleteParticles();
    if (fGravity)  delete fGravity;
    for (ForceSpring* f : springsStretch) delete f;
    for (ForceSpring* f : springsShear) delete f;
    for (ForceSpring* f : springsBend) delete f;
}

void SceneCloth::initialize() {

    // load shaders
    shaderPhong = glutils::loadShaderProgram(":/shaders/phong.vert", ":/shaders/phong.frag");
    shaderCloth = glutils::loadShaderProgram(":/shaders/cloth.vert", ":/shaders/cloth.geom", ":/shaders/cloth.frag");

    // create sphere VAOs
    Model sphere = Model::createIcosphere(3);
    vaoSphereL = glutils::createVAO(shaderPhong, &sphere, buffers);
    numFacesSphereL = sphere.numFaces();
    glutils::checkGLError();

    sphere = Model::createIcosphere(1);
    vaoSphereS = glutils::createVAO(shaderPhong, &sphere, buffers);
    numFacesSphereS = sphere.numFaces();
    glutils::checkGLError();

    // create cube VAO
    Model cube = Model::createCube();
    vaoCube = glutils::createVAO(shaderPhong, &cube, buffers);
    glutils::checkGLError();


    // create cloth mesh VAO
    vaoMesh = new QOpenGLVertexArrayObject();
    vaoMesh->create();
    vaoMesh->bind();
    vboMesh = new QOpenGLBuffer(QOpenGLBuffer::Type::VertexBuffer);
    vboMesh->create();
    vboMesh->bind();
    vboMesh->setUsagePattern(QOpenGLBuffer::UsagePattern::DynamicDraw);
    vboMesh->allocate(1000*1000*3*3*sizeof(float)); // sync with widget max particles
    shaderCloth->setAttributeBuffer("vertex", GL_FLOAT, 0, 3, 0);
    shaderCloth->enableAttributeArray("vertex");
    iboMesh = new QOpenGLBuffer(QOpenGLBuffer::Type::IndexBuffer);
    iboMesh->create();
    iboMesh->bind();
    iboMesh->setUsagePattern(QOpenGLBuffer::UsagePattern::StaticDraw);
    iboMesh->allocate(1000*1000*2*3*sizeof(unsigned int));
    vaoMesh->release();

    // create gravity force
    fGravity = new ForceConstAcceleration();
    system.addForce(fGravity);

    colliderBall.setCenter(Vec3(20,-20,-5));
    colliderBall.setRadius(30);
    //colliderCube.setFromCenterSize(Vec3(-60,30,0), Vec3(60, 40, 60));
    //colliderWalls.setFromCenterSize(Vec3(0, 0, 0), Vec3(200, 200, 200));
}

void SceneCloth::reset()
{
    // we only update numParticles on resets
    updateSimParams();

    // reset particles
    system.deleteParticles();

    // reset forces
    system.clearForces();
    fGravity->clearInfluencedParticles();
    for (ForceSpring* f : springsStretch) delete f;
    springsStretch.clear();
    for (ForceSpring* f : springsShear) delete f;
    springsShear.clear();
    for (ForceSpring* f : springsBend) delete f;
    springsBend.clear();

    // cloth props
    Vec2 dims = widget->getDimensions();
    Vec2i dimParticles = widget->getNumParticles();
    numParticlesX = dimParticles.x();
    numParticlesY = dimParticles.y();
    clothWidth  = dims[0];
    clothHeight = dims[1];
    double edgeX = dims[0]/numParticlesX;
    double edgeY = dims[1]/numParticlesY;
    particleRadius = widget->getParticleRadius();

    // create particles
    numParticles = numParticlesX * numParticlesY;
    fixedParticle = std::vector<bool>(numParticles, false);

    for (int i = 0; i < numParticlesX; i++) {
        for (int j = 0; j < numParticlesY; j++) {

            int idx = i*numParticlesY + j;

            // TODO: you can play here with different start positions and/or fixed particles
            fixedParticle[idx] = false;
            double angle = M_PI / 6.0; //30 deg tilt
            double tx = i*edgeX - 0.5*clothWidth;
            double ty = j*edgeY - 0.5*clothHeight;
            Vec3 pos = Vec3(
                ty + edgeY,
                70 - tx - edgeX * cos(angle),
                (70 - tx - edgeX) * sin(angle) -30
                );

            Particle* p = new Particle();
            p->id = idx;
            p->pos = pos;
            p->prevPos = pos;
            p->vel = Vec3(0,0,0);
            p->mass = 1;
            p->radius = particleRadius;
            p->color = Vec3(235/255.0, 51/255.0, 36/255.0);

            system.addParticle(p);
            fGravity->addInfluencedParticle(p);
        }
    }

    // forces: gravity
    system.addForce(fGravity);

    // TODO: create spring forces
    // Code for PROVOT layout
    int width = numParticlesX;
    int height = numParticlesY;

    // Stretch springs
    for (int i = 0; i < numParticlesX; ++i) {
        for (int j = 0; j < numParticlesY; ++j) {
            int idx = i * numParticlesY + j;
            Particle* p = system.getParticle(idx);

            // Horizontal neighbor (j -> j+1)
            if (j < numParticlesY - 1) {
                int rightIdx = i * numParticlesY + (j + 1);
                Particle* pRight = system.getParticle(rightIdx);
                double L = (p->pos - pRight->pos).norm();
                ForceSpring* spring = new ForceSpring(p, pRight, L, 0, 0);
                springsStretch.push_back(spring);
                system.addForce(spring);
            }

            // Vertical neighbor (i -> i+1)
            if (i < numParticlesX - 1) {
                int downIdx = (i + 1) * numParticlesY + j;
                Particle* pDown = system.getParticle(downIdx);
                double L = (p->pos - pDown->pos).norm();
                ForceSpring* spring = new ForceSpring(p, pDown, L, 0, 0);
                springsStretch.push_back(spring);
                system.addForce(spring);
            }
        }
    }

    // Shear springs
    for (int i = 0; i < numParticlesX - 1; ++i) {
        for (int j = 0; j < numParticlesY - 1; ++j) {
            Particle* p00 = system.getParticle(i * numParticlesY + j);
            Particle* p11 = system.getParticle((i + 1) * numParticlesY + (j + 1));
            Particle* p10 = system.getParticle((i + 1) * numParticlesY + j);
            Particle* p01 = system.getParticle(i * numParticlesY + (j + 1));

            double L1 = (p00->pos - p11->pos).norm();
            ForceSpring* s1 = new ForceSpring(p00, p11, L1, 0, 0);
            springsShear.push_back(s1);
            system.addForce(s1);

            double L2 = (p01->pos - p10->pos).norm();
            ForceSpring* s2 = new ForceSpring(p01, p10, L2, 0, 0);
            springsShear.push_back(s2);
            system.addForce(s2);
        }
    }

    // Bend springs
    for (int i = 0; i < numParticlesX; ++i) {
        for (int j = 0; j < numParticlesY; ++j) {
            Particle* p = system.getParticle(i * numParticlesY + j);

            // Horizontal bend (j -> j+2)
            if (j < numParticlesY - 2) {
                Particle* p2 = system.getParticle(i * numParticlesY + (j + 2));
                double L = (p->pos - p2->pos).norm();
                ForceSpring* spring = new ForceSpring(p, p2, L, 0, 0);
                springsBend.push_back(spring);
                system.addForce(spring);
            }

            // Vertical bend (i -> i+2)
            if (i < numParticlesX - 2) {
                Particle* p2 = system.getParticle((i + 2) * numParticlesY + j);
                double L = (p->pos - p2->pos).norm();
                ForceSpring* spring = new ForceSpring(p, p2, L, 0, 0);
                springsBend.push_back(spring);
                system.addForce(spring);
            }
        }
    }

    updateSprings();

    // update index buffer
    iboMesh->bind();
    numMeshIndices = (numParticlesX - 1)*(numParticlesY - 1)*2*3;
    int* indices = new int[numMeshIndices];
    int idx = 0;
    for (int i = 0; i < numParticlesX-1; i++) {
        for (int j = 0; j < numParticlesY-1; j++) {
            indices[idx  ] = i*numParticlesY + j;
            indices[idx+1] = (i+1)*numParticlesY + j;
            indices[idx+2] = i*numParticlesY + j + 1;
            indices[idx+3] = i*numParticlesY + j + 1;
            indices[idx+4] = (i+1)*numParticlesY + j;
            indices[idx+5] = (i+1)*numParticlesY + j + 1;
            idx += 6;
        }
    }
    void* bufptr = iboMesh->mapRange(0, numMeshIndices*sizeof(int),
                                     QOpenGLBuffer::RangeInvalidateBuffer | QOpenGLBuffer::RangeWrite);
    memcpy(bufptr, (void*)(indices), numMeshIndices*sizeof(int));
    iboMesh->unmap();
    iboMesh->release();
    delete[] indices;
    glutils::checkGLError();
}


void SceneCloth::updateSprings()
{
    double ks = widget->getStiffness();
    double kd = widget->getDamping();

    // here I update all ks and kd parameters.
    // idea: if you want to enable/disable a spring type, you can set ks to 0 for these
    for (ForceSpring* f : springsStretch) {
        f->setSpringConstant(ks);
        f->setDampingCoeff(kd);
    }
    for (ForceSpring* f : springsShear) {
        f->setSpringConstant(ks);
        f->setDampingCoeff(kd);

    }
    for (ForceSpring* f : springsBend) {
        f->setSpringConstant(ks);
        f->setDampingCoeff(kd);
    }
}

void SceneCloth::updateSimParams()
{
    double g = widget->getGravity();
    fGravity->setAcceleration(Vec3(0, -g, 0));

    updateSprings();

    for (Particle* p : system.getParticles()) {
        p->radius = widget->getParticleRadius();
    }

    showParticles = widget->showParticles();
}

void SceneCloth::freeAnchors()
{
    fixedParticle = std::vector<bool>(numParticles, false);
}

void SceneCloth::paint(const Camera& camera)
{
    QOpenGLFunctions* glFuncs = nullptr;
    glFuncs = QOpenGLContext::currentContext()->functions();

    shaderPhong->bind();
    shaderPhong->setUniformValue("normalSign", 1.0f);

    // camera matrices
    QMatrix4x4 camProj = camera.getPerspectiveMatrix();
    QMatrix4x4 camView = camera.getViewMatrix();
    shaderPhong->setUniformValue("ProjMatrix", camProj);
    shaderPhong->setUniformValue("ViewMatrix", camView);

    // lighting
    static const int numLights = 1;
    static const QVector3D lightPosWorld[numLights] = {QVector3D(80,80,80)};
    static const QVector3D lightColor[numLights] = {QVector3D(1,1,1)};
    QVector3D lightPosCam[numLights];
    for (int i = 0; i < numLights; i++) {
        lightPosCam[i] = camView.map(lightPosWorld[i]);
    }
    shaderPhong->setUniformValue("numLights", numLights);
    shaderPhong->setUniformValueArray("lightPos", lightPosCam, numLights);
    shaderPhong->setUniformValueArray("lightColor", lightColor, numLights);

    // draw the particle spheres
    QMatrix4x4 modelMat;
    if (showParticles) {
        vaoSphereS->bind();
        shaderPhong->setUniformValue("matspec", 1.0f, 1.0f, 1.0f);
        shaderPhong->setUniformValue("matshin", 100.f);
        //modified for recued particle calls.
        const auto& particles = system.getParticles();
        for (int i = 0; i < numParticles; ++i) {
            const Particle* particle = particles[i];
            Vec3   p = particle->pos;
            Vec3   c = particle->color;


            if (fixedParticle[i])      c = Vec3(63/255.0, 72/255.0, 204/255.0);
            if (i == selectedParticle) c = Vec3(1.0,0.9,0);

            //small change to avoid reinstatiation
            modelMat.setToIdentity();
            modelMat.translate(p[0], p[1], p[2]);
            modelMat.scale(particle->radius);
            shaderPhong->setUniformValue("ModelMatrix", modelMat);
            shaderPhong->setUniformValue("matdiff", GLfloat(c[0]), GLfloat(c[1]), GLfloat(c[2]));
            glFuncs->glDrawElements(GL_TRIANGLES, 3*numFacesSphereS, GL_UNSIGNED_INT, 0);
        }
    }

    // TODO: draw colliders and walls
    //Sphere collider
    {
        Vec3 c = colliderBall.getCenter();
        double r = colliderBall.getRadius();

        QMatrix4x4 modelMat;
        modelMat.translate(c.x(), c.y(), c.z());
        modelMat.scale(r);

        shaderPhong->setUniformValue("ModelMatrix", modelMat);
        shaderPhong->setUniformValue("matdiff", 0.1f, 0.8f, 0.1f); // green

        vaoSphereL->bind();
        glFuncs->glDrawElements(GL_TRIANGLES, 3 * numFacesSphereL, GL_UNSIGNED_INT, 0);
        vaoSphereL->release();
    }
    shaderPhong->release();


    // update cloth mesh VBO coords
    vboMesh->bind();
    float* pos = new float[3*numParticles];
    for (int i = 0; i < numParticles; i++) {
        pos[3*i  ] = system.getParticle(i)->pos.x();
        pos[3*i+1] = system.getParticle(i)->pos.y();
        pos[3*i+2] = system.getParticle(i)->pos.z();
    }
    void* bufptr = vboMesh->mapRange(0, 3*numParticles*sizeof(float),
                       QOpenGLBuffer::RangeInvalidateBuffer | QOpenGLBuffer::RangeWrite);
    memcpy(bufptr, (void*)(pos), 3*numParticles*sizeof(float));
    vboMesh->unmap();
    vboMesh->release();
    delete[] pos;

    if (drawDebugRay) {
        shaderPhong->bind();

        shaderPhong->setUniformValue("ProjMatrix", camProj);
        shaderPhong->setUniformValue("ViewMatrix", camView);
        shaderPhong->setUniformValue("ModelMatrix", QMatrix4x4());
        shaderPhong->setUniformValue("matdiff", 1.0f, 0.0f, 0.0f);
        shaderPhong->setUniformValue("matspec", 0.0f, 0.0f, 0.0f);
        shaderPhong->setUniformValue("matshin", 0.0f);
        shaderPhong->setUniformValue("normalSign", 1.0f);

        // Convert Eigen::Vec3 → float array
        GLfloat lineVerts[] = {
            (GLfloat)debugRayStart.x(), (GLfloat)debugRayStart.y(), (GLfloat)debugRayStart.z(),
            (GLfloat)debugRayEnd.x(),   (GLfloat)debugRayEnd.y(),   (GLfloat)debugRayEnd.z()
        };

        GLuint debugVBO = 0;
        glFuncs->glGenBuffers(1, &debugVBO);
        glFuncs->glBindBuffer(GL_ARRAY_BUFFER, debugVBO);
        glFuncs->glBufferData(GL_ARRAY_BUFFER, sizeof(lineVerts), lineVerts, GL_DYNAMIC_DRAW);

        // Use attribute 0 (vertex position)
        glFuncs->glEnableVertexAttribArray(0);
        glFuncs->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        // Draw the line
        glFuncs->glLineWidth(2.0f);
        glFuncs->glDrawArrays(GL_LINES, 0, 2);

        // Cleanup
        glFuncs->glDisableVertexAttribArray(0);
        glFuncs->glBindBuffer(GL_ARRAY_BUFFER, 0);
        glFuncs->glDeleteBuffers(1, &debugVBO);

        shaderPhong->release();
    }

    // draw mesh
    shaderCloth->bind();
    shaderCloth->setUniformValue("ProjMatrix", camProj);
    shaderCloth->setUniformValue("ViewMatrix", camView);
    shaderCloth->setUniformValue("NormalMatrix", camView.normalMatrix());
    shaderCloth->setUniformValue("matdiffFront", 0.7f, 0.0f, 0.0f);
    shaderCloth->setUniformValue("matspecFront", 1.0f, 1.0f, 1.0f);
    shaderCloth->setUniformValue("matshinFront", 100.0f);
    shaderCloth->setUniformValue("matdiffBack", 0.7f, 0.3f, 0.0f);
    shaderCloth->setUniformValue("matspecBack", 0.0f, 0.0f, 0.0f);
    shaderCloth->setUniformValue("matshinBack", 0.0f);
    shaderCloth->setUniformValue("numLights", numLights);
    shaderCloth->setUniformValueArray("lightPos", lightPosCam, numLights);
    shaderCloth->setUniformValueArray("lightColor", lightColor, numLights);
    vaoMesh->bind();
    glFuncs->glDrawElements(GL_TRIANGLES, numMeshIndices, GL_UNSIGNED_INT, 0);
    vaoMesh->release();
    shaderCloth->release();

    glutils::checkGLError();
}

//relaxation func:
void SceneCloth::relaxSpring(ForceSpring* s)
{
    Particle* p1 = s->getParticle1();
    Particle* p2 = s->getParticle2();

    if (!p1 || !p2) return;

    Vec3 delta = p2->pos - p1->pos;
    double dist = delta.norm();
    if (dist == 0.0) return;

    double rest = s->getRestLength();
    double diff = (dist - rest) / dist;

    // Determine how much to move each particle (half each if both movable)
    Vec3 correction = 0.5 * diff * delta;

    if (!fixedParticle[p1->id])
        p1->pos += correction;

    if (!fixedParticle[p2->id])
        p2->pos -= correction;
}

void SceneCloth::update(double dt)
{
    // fixed particles: no velocity, no force acting
    for (int i = 0; i < numParticles; i++) {
        if (fixedParticle[i]) {
            Particle* p = system.getParticle(i);
            p->vel = Vec3(0,0,0);
            p->force = Vec3(0,0,0);
        }
    }

    // integration step
    Vecd ppos = system.getPositions();
    integrator.step(system, dt);
    system.setPreviousPositions(ppos);

    // collisions
    Collision colInfo;
    double kBounce = 0.2;
    double kFriction = 0.2;

    // user interaction
    if (selectedParticle >= 0) {
        Particle* p = system.getParticle(selectedParticle);
        // p->pos = ?; TODO: assign cursor world position (see code, it's already computed)
        p->pos = cursorWorldPos;
        p->prevPos = cursorWorldPos; // prevent spring overshoot

        p->vel = Vec3(0,0,0);

        // TODO: test and resolve for collisions during user movement
        for (Particle* p : system.getParticles()) {
            if (colliderBall.testCollision(p, colInfo)) {
                colliderBall.resolveCollision(p, colInfo, kBounce, kFriction);
            }
        }

    }

    // TODO: relaxation
    int numRelaxIters = 5;
    for (int iter = 0; iter < numRelaxIters; ++iter) {
        for (ForceSpring* s : springsStretch) relaxSpring(s);
        for (ForceSpring* s : springsShear)   relaxSpring(s);
        for (ForceSpring* s : springsBend)    relaxSpring(s);
    }

    for (Particle* p : system.getParticles()) {
        if (colliderBall.testCollision(p, colInfo)) {
            colliderBall.resolveCollision(p, colInfo, kBounce, kFriction);
        }
    }


    // needed after we have done collisions and relaxation, since spring forces depend on p and v
    system.updateForces();
}


double minDist = 1e9;
double pickRadius = 3.0; // world-space radius threshold
void SceneCloth::mousePressed(const QMouseEvent* e, const Camera& cam)
{
    grabX = e->pos().x();
    grabY = e->pos().y();

    if (!(e->modifiers() & Qt::ControlModifier)) {

        Vec3 rayDir = cam.getRayDir(grabX, grabY);
        // make sure rayDir is normalized
        double len = rayDir.norm();
        if (len > 0.0) rayDir /= len;

        Vec3 origin = cam.getPos();

        // extend debug ray to scene scale
        debugRayStart = origin;
        debugRayEnd = origin + rayDir * 1000.0; // long enough to cross scene
        drawDebugRay = true;

        selectedParticle = -1;

        // local best distance per click
        double bestDist = 1e9;
        const double pickRadiusWorld = pickRadius; // world-space threshold (you can tune)

        for (int i = 0; i < numParticles; ++i) {
            Particle* p = system.getParticle(i);
            Vec3 toParticle = p->pos - origin;

            // project particle onto ray (t - distance along ray)
            double t = toParticle.dot(rayDir);
            if (t < 0.0) continue; // behind camera

            Vec3 closestPoint = origin + rayDir * t;
            // use squared distances to avoid sqrt
            double dist2 = (p->pos - closestPoint).squaredNorm();

            if (dist2 < pickRadiusWorld * pickRadiusWorld && dist2 < bestDist) {
                bestDist = dist2;
                selectedParticle = i;
            }
        }

        if (selectedParticle >= 0) {
            cursorWorldPos = system.getParticle(selectedParticle)->pos;
        }
    }
}
void SceneCloth::mouseMoved(const QMouseEvent* e, const Camera& cam)
{
    int dx = e->pos().x() - grabX;
    int dy = e->pos().y() - grabY;
    grabX = e->pos().x();
    grabY = e->pos().y();

    //if we're dragging a particle, only move it, not the camera
    if (selectedParticle >= 0) {
        // Depth from camera to selected particle
        double d = -(system.getParticle(selectedParticle)->pos - cam.getPos()).dot(cam.zAxis());
        Vec3 disp = cam.worldSpaceDisplacement(dx, -dy, d);
        cursorWorldPos += disp;
        return;
    }

    if (e->modifiers() & Qt::ControlModifier) {

    }
    else if (e->modifiers() & Qt::ShiftModifier) {

    }
    else {

    }
}

void SceneCloth::mouseReleased(const QMouseEvent*, const Camera&)
{
    selectedParticle = -1;
}

void SceneCloth::keyPressed(const QKeyEvent* e, const Camera&)
{
    if (selectedParticle >= 0 && e->key() == Qt::Key_F) {
        fixedParticle[selectedParticle] = true;
        Particle* p = system.getParticle(selectedParticle);
        p->prevPos = p->pos;
        p->vel = Vec3(0,0,0);
        p->force = Vec3(0,0,0);
    }
}
