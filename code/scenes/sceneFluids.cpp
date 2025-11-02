#include "scenefluids.h"
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QDebug>
#include "glutils.h"
#include "forces.h"
#include "colliders.h"

//sph helper functions
static inline qint64 hashCell(int ix, int iy, int iz) {
    qint64 key = ((qint64)ix & 0x1FFFFF) | (((qint64)iy & 0x1FFFFF) << 21) | (((qint64)iz & 0x1FFFFF) << 42);
    return key;
}

inline float W_poly6(float r, float h) {
    float h2 = h*h;
    if (r >= 0.0f && r <= h) {
        float r2 = r*r;
        float term = (h2 - r2);
        return 315.0f / (64.0f * M_PI * pow(h, 9)) * (term * term * term);
    }
    return 0.0f;
}

// Spiky gradient kernel (for pressure forces)
inline Vec3 gradW_spiky(const Vec3& r, float rlen, float h) {
    if (rlen >= 0.0f && rlen <= h) {
        float coef = 45.0f / (M_PI * pow(h, 6)*rlen)*(h-rlen)*(h-rlen);
        return coef * -r;
    }
    return Vec3(0,0,0);
}

// Viscosity kernel (scalar)
inline float laplacianW_viscosity(float rlen, float h) {
    if (rlen >= 0.0f && rlen <= h) {
        return 45.0f / (M_PI * pow(h, 5)) * (1- rlen/h);
    }
    return 0.0f;
}

// SPH end

SceneFluids::SceneFluids()
{
    widget = new WidgetFluids();
}

SceneFluids::~SceneFluids()
{
    vaoHourglass.destroy();
    vboHourglass.destroy();
    if (vaoSphereL)
        delete vaoSphereL;
    delete hourglassShader;
    delete widget;
}

void SceneFluids::buildGrid()
{
    grid.clear();
    const auto& systemParticles = system.getParticles();
    int n = systemParticles.size();

    for (int i = 0; i < n; ++i) {
        const Vec3 &p = systemParticles[i]->pos;
        int ix = floor(p.x() / cellSize);
        int iy = floor(p.y() / cellSize);
        int iz = floor(p.z() / cellSize);
        qint64 key = hashCell(ix, iy, iz);
        grid[key].push_back(i);
    }
}

void SceneFluids::createParticles()
{
    system.deleteParticles();

    int cols = 20;
    int rows = 12;
    int layers = 8;
    Vec3 start(0.0f, 0.5f, 0.0f);

    int count = 0;
    for (int k = 0; k < layers; ++k) {
        for (int j = 0; j < rows; ++j) {
            for (int i = 0; i < cols; ++i) {
                if (count >= maxParticles) break;
                Particle* p = new Particle();
                p->pos = start + Vec3(i*spacing, j*spacing, k*spacing);
                p->vel = Vec3(0,0,0);
                p->force = Vec3(0,0,0);
                p->density = restDensity;
                p->pressure = 0.0f;
                p->radius = h * 0.5;
                p->color = Vec3(0.9f, 0.6f, 0.2f);
                p->mass = mass;
                system.addParticle(p);
                count++;
            }
        }
    }
    qDebug() << "Created SPH particles in system:" << system.getParticles().size();
}

void SceneFluids::initialize()
{
    QOpenGLContext* ctx = QOpenGLContext::currentContext();
    if (!ctx) return;
    auto* gl = ctx->functions();

    gl->glEnable(GL_DEPTH_TEST);
    gl->glEnable(GL_BLEND);
    gl->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    gl->glClearColor(0.05f, 0.05f, 0.15f, 1.0f);

    // Parameters
    const int slices = 32;
    const float height = 2.0f;
    const float radius = 0.6f;
    const float neckRadius = 0.05f;

    struct Vertex { QVector3D pos; QVector3D normal; };
    QVector<Vertex> vertices;

    // --- Top cone (y from 0 to +height/2)
    for (int i = 0; i <= slices; ++i) {
        float theta = 2.0f * M_PI * i / slices;
        float x = cos(theta);
        float z = sin(theta);

        QVector3D p1 = QVector3D(x * neckRadius, 0.0f, z * neckRadius);
        QVector3D p2 = QVector3D(x * radius, height / 2.0f, z * radius);
        QVector3D n = QVector3D(x, 0.5f, z).normalized();

        vertices.push_back({p1, n});
        vertices.push_back({p2, n});
    }

    int topConeCount = vertices.size();

    // --- Bottom cone (y from 0 to -height/2)
    for (int i = 0; i <= slices; ++i) {
        float theta = 2.0f * M_PI * i / slices;
        float x = cos(theta);
        float z = sin(theta);

        QVector3D p1 = QVector3D(x * neckRadius, 0.0f, z * neckRadius);
        QVector3D p2 = QVector3D(x * radius, -height / 2.0f, z * radius);
        QVector3D n = QVector3D(x, -0.5f, z).normalized();

        vertices.push_back({p1, n});
        vertices.push_back({p2, n});
    }

    int bottomConeCount = vertices.size() - topConeCount;
    int bottomConeStart = topConeCount;

    // --- Top cap
    int topCapStart = vertices.size();
    for (int i = 0; i <= slices; ++i) {
        float theta = 2.0f * M_PI * i / slices;
        float x = cos(theta);
        float z = sin(theta);
        QVector3D n(0.0f, 1.0f, 0.0f);
        vertices.push_back({ QVector3D(0.0f, height / 2.0f, 0.0f), n });
        vertices.push_back({ QVector3D(x * radius, height / 2.0f, z * radius), n });
    }
    int topCapCount = vertices.size() - topCapStart;

    // --- Bottom cap
    int bottomCapStart = vertices.size();
    for (int i = 0; i <= slices; ++i) {
        float theta = 2.0f * M_PI * i / slices;
        float x = cos(theta);
        float z = sin(theta);
        QVector3D n(0.0f, -1.0f, 0.0f);
        vertices.push_back({ QVector3D(0.0f, -height / 2.0f, 0.0f), n });
        vertices.push_back({ QVector3D(x * radius, -height / 2.0f, z * radius), n });
    }
    int bottomCapCount = vertices.size() - bottomCapStart;

    // Save counts for rendering
    this->topConeCount = topConeCount;
    this->bottomConeStart = bottomConeStart;
    this->bottomConeCount = bottomConeCount;
    this->topCapStart = topCapStart;
    this->topCapCount = topCapCount;
    this->bottomCapStart = bottomCapStart;
    this->bottomCapCount = bottomCapCount;
    this->hourglassVertexCount = vertices.size();

    // --- Create VAO/VBO
    vaoHourglass.create();
    vaoHourglass.bind();

    vboHourglass.create();
    vboHourglass.bind();
    vboHourglass.allocate(vertices.constData(), vertices.size() * sizeof(Vertex));

    // --- Shader
    hourglassShader = glutils::loadShaderProgram(":/shaders/fluids.vert", ":/shaders/fluids.frag");
    hourglassShader->link();
    hourglassShader->bind();

    gl->glEnableVertexAttribArray(0);
    gl->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(0));
    gl->glEnableVertexAttribArray(1);
    gl->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(sizeof(QVector3D)));

    vaoHourglass.release();
    vboHourglass.release();


    Model sphereLowres = Model::createIcosphere(1);
    vaoSphereL = glutils::createVAO(hourglassShader, &sphereLowres, buffers);
    numFacesSphereL = sphereLowres.numFaces();
    glutils::checkGLError();

    //Initialize SPH particles
    cellSize = h;
    particles.clear();

    //create particles in the top chamber region (simple box sampling)
    createParticles();

    hourglassShader->release();
    glutils::checkGLError();


    //outside box
    const float xmax = 1.0f;
    const float ymax = 1.5f;
    const float zmax = 1.0f;

    // Clear any old colliders
    colliders.clear();

    colliders.push_back(new ColliderHourglass());

}


void SceneFluids::reset()
{

    createParticles(); //we delete particles before creating inside.
    qDebug() << "SceneFluids reset() called";
}

void SceneFluids::update(double dt)
{
    /*
    // Class properties:
    // spatial hash / uniform grid for neighbor search
    float h = 0.02f;               // smoothing radius
    float spacing = h*0.9; //grid initial spacing

    float h2 = h*h;
    float restDensity = 1000.0f;   // rest density
    float k = 100.0f;          // stiffness
    float viscosity = 0.02f;        // viscosity coefficient
    float mass = 0.0002f;            // per-particle mass (tune)
     */
    const auto& systemParticles = system.getParticles();
    int n = systemParticles.size();
    if (n == 0) return;



    //1 Find neighbors to each particle and store them in a list
    buildGrid();
    std::vector<int> neighCount(n, 0);
    for (int i = 0; i < n; ++i) {
        const Vec3& xi = systemParticles[i]->pos;
        int ix = floor(xi.x() / cellSize);
        int iy = floor(xi.y() / cellSize);
        int iz = floor(xi.z() / cellSize);
        int count = 0;
        for (int dx=-1; dx<=1; ++dx) for (int dy=-1; dy<=1; ++dy) for (int dz=-1; dz<=1; ++dz) {
                    qint64 key = hashCell(ix+dx, iy+dy, iz+dz);
                    auto it = grid.find(key);
                    if (it==grid.end()) continue;
                    for (int j : it.value()) {
                        if (j==i) continue;
                        float r2 = (xi - systemParticles[j]->pos).squaredNorm();
                        if (r2 <= h*h) count++;
                    }
                }
        neighCount[i] = count;
    }
    int minN = INT_MAX, maxN = 0;
    double avgN = 0;
    for (int c : neighCount) { minN = std::min(minN, c); maxN = std::max(maxN, c); avgN += c; }
    avgN /= n;
    //qDebug() << "neighbor counts: min" << minN << "avg" << avgN << "max" << maxN;


    //2 Calculate density for each particle
    for (int i = 0; i < n; ++i) {
        Particle* pi = systemParticles[i];
        Vec3 xi = pi->pos;
        int ix = floor(xi.x() / cellSize);
        int iy = floor(xi.y() / cellSize);
        int iz = floor(xi.z() / cellSize);

        float density = 0.0f;
        for (int dx=-1; dx<=1; ++dx) for (int dy=-1; dy<=1; ++dy) for (int dz=-1; dz<=1; ++dz) {
                    qint64 key = hashCell(ix+dx, iy+dy, iz+dz);
                    auto it = grid.find(key);
                    if (it == grid.end()) continue;
                    for (int j : it.value()) {
                        const Particle* pj = systemParticles[j];
                        Vec3 rij = xi - pj->pos;
                        float r = rij.norm();
                        if (r*r <= h*h) {
                            density += mass * W_poly6(r, h);
                        }
                    }
                }

        if (density < 1e-6f) density = 1e-6f; // small epsilon only, do NOT set to restDensity
        pi->density = density;
    }

    // compute pressures
    for (int i = 0; i < n; ++i) {
        Particle* p = systemParticles[i];
        p->pressure = k * (p->density - restDensity);
        //if (p->pressure < 0.0f) p->pressure = 0.0f; // tensile clamp
    }

    float avgRho=0, minRho=1e9, maxRho=0;
    for (auto p : system.getParticles()) {
        avgRho += p->density;
        minRho = std::min(minRho, p->density);
        maxRho = std::max(maxRho, p->density);
    }
    avgRho /= system.getParticles().size();
    qDebug() << "rho avg" << avgRho << "min" << minRho << "max" << maxRho;

    // --- Compute accelerations
    for (int i = 0; i < n; ++i) {
        Particle* pi = systemParticles[i];
        Vec3 xi = pi->pos;
        Vec3 vi = pi->vel;
        float rhoi = pi->density;
        float pi_pressure = pi->pressure;

        Vec3 f_pressure(0,0,0);
        Vec3 f_visc(0,0,0);

        int ix = floor(xi.x() / cellSize);
        int iy = floor(xi.y() / cellSize);
        int iz = floor(xi.z() / cellSize);

        for (int dx=-1; dx<=1; ++dx) for (int dy=-1; dy<=1; ++dy) for (int dz=-1; dz<=1; ++dz) {
                    qint64 key = hashCell(ix+dx, iy+dy, iz+dz);
                    auto it = grid.find(key);
                    if (it == grid.end()) continue;
                    for (int j : it.value()) {
                        if (j == i) continue;
                        Particle* pj = systemParticles[j];

                        Vec3 rij = xi - pj->pos;
                        float rlen = rij.norm();
                        if (rlen > h || rlen <= 1e-8f) continue;

                        Vec3 gradW = gradW_spiky(rij, rlen, h);
                        // stable pressure term:
                        float term = (pi_pressure/(rhoi*rhoi)+pj->pressure/(pj->density*pj->density));

                        f_pressure += - mass * term * gradW;
                        if (j == 0) qDebug()<<"pressure"<< f_pressure.x()<<f_pressure.y()<<f_pressure.z() <<"mass" << mass <<" term " << term <<"gradW" <<gradW.x() << gradW.y() << gradW.z();

                        float lap = laplacianW_viscosity(rlen, h);
                        f_visc += viscosity * mass * (pj->vel - vi) / (pi->density*pj->density) * lap;
                    }
                }

        Vec3 f_gravity = mass * Vec3(0.0f, -9.8f, 0.0f);
        Vec3 f_interactive = (1/mass)*Vec3(0,0,0);
        Vec3 f_total = f_pressure + f_visc + f_gravity + f_interactive;
        //qDebug() << "f_pressure" << f_pressure.x() << f_pressure.y() << f_pressure.z();
        Vec3 accel = f_total;
        pi->force = accel;
    }

    //5 Integration Step
    Vecd ppos = system.getPositions();
    integrator.step(system, dt);

    //Boundary Handling (Simple Bounding Box)
    const double kBounce = 0.2;
    const double kFriction = 0.3;

    // collisions
    Collision colInfo;
    for (Particle* p : system.getParticles()) {

        // Loop through all defined colliders
        for (Collider* collider : colliders) {

            // Note: ColliderPlane testCollision automatically handles penetration based on position
            if (collider->testCollision(p, colInfo)) {

                // If a collision is found, resolve it.
                collider->resolveCollision(p, colInfo, kBounce, kFriction);
            }
        }
    }
}

void SceneFluids::paint(const Camera& cam)
{
    QOpenGLFunctions *gl = QOpenGLContext::currentContext()->functions();

    gl->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    gl->glEnable(GL_DEPTH_TEST);
    gl->glEnable(GL_BLEND);
    gl->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    hourglassShader->bind();

    // --- Common uniforms ---
    hourglassShader->setUniformValue("ProjMatrix", cam.getPerspectiveMatrix());
    hourglassShader->setUniformValue("ViewMatrix", cam.getViewMatrix());

    // simple light setup
    QVector3D lightPos(1.0f, 2.0f, 1.0f);
    QVector3D lightColor(1.0f, 1.0f, 1.0f);
    hourglassShader->setUniformValue("numLights", 1);
    hourglassShader->setUniformValue("lightPos[0]", lightPos);
    hourglassShader->setUniformValue("lightColor[0]", lightColor);

    //  Draw SPH particles
    hourglassShader->setUniformValue("simpleShading", false);
    hourglassShader->setUniformValue("roundParticles", false);

    hourglassShader->setUniformValue("matspec", QVector3D(0.0f, 0.0f, 0.0f));
    hourglassShader->setUniformValue("matshin", 0.0f);
    hourglassShader->setUniformValue("alpha", 1.0f); // Opaque

    vaoSphereL->bind();
    QMatrix4x4 modelMat;
    for (const Particle* p : system.getParticles())
    {
        // Use the particle pointer p directly
        Vec3 p_pos = p->pos;
        Vec3 p_color = p->color;
        float p_radius = p->radius;

        // Create a unique ModelMatrix for this particle
        modelMat.setToIdentity();
        modelMat.translate(p_pos.x(), p_pos.y(), p_pos.z());
        modelMat.scale(p_radius);

        // Send matrix to shader
        hourglassShader->setUniformValue("ModelMatrix", modelMat);

        // Send this particle's color to the shader
        hourglassShader->setUniformValue("matdiff",
                                         GLfloat(p_color[0]),
                                         GLfloat(p_color[1]),
                                         GLfloat(p_color[2]));

        // Draw one (1) sphere
        gl->glDrawElements(GL_TRIANGLES, 3 * numFacesSphereL, GL_UNSIGNED_INT, 0);
    }

    vaoSphereL->release();


    //  Draw the hourglass
    QMatrix4x4 modelHourglass;
    modelHourglass.setToIdentity();
    modelHourglass.scale(1.0f);

    hourglassShader->setUniformValue("ModelMatrix", modelHourglass);
    hourglassShader->setUniformValue("simpleShading", false);
    hourglassShader->setUniformValue("roundParticles", false);

    hourglassShader->setUniformValue("matdiff", QVector3D(0.7f, 0.8f, 0.9f));
    hourglassShader->setUniformValue("matspec", QVector3D(1.0f, 1.0f, 1.0f));
    hourglassShader->setUniformValue("matshin", 64.0f);
    hourglassShader->setUniformValue("alpha", 0.25f);

    vaoHourglass.bind();
    gl->glDrawArrays(GL_TRIANGLE_STRIP, 0, topConeCount);
    gl->glDrawArrays(GL_TRIANGLE_STRIP, bottomConeStart, bottomConeCount);
    gl->glDrawArrays(GL_TRIANGLE_STRIP, topCapStart, topCapCount);
    gl->glDrawArrays(GL_TRIANGLE_STRIP, bottomCapStart, bottomCapCount);
    vaoHourglass.release();

    hourglassShader->release();
}



void SceneFluids::getSceneBounds(Vec3& bmin, Vec3& bmax)
{
    bmin = Vec3(-1, -1, -1);
    bmax = Vec3( 1,  1,  1);
}

QWidget* SceneFluids::sceneUI()
{
    return widget;
}

void SceneFluids::mousePressed(const QMouseEvent*, const Camera&) {}
void SceneFluids::mouseMoved(const QMouseEvent*, const Camera&) {}
void SceneFluids::mouseReleased(const QMouseEvent*, const Camera&) {}
void SceneFluids::keyPressed(const QKeyEvent*, const Camera&) {}
