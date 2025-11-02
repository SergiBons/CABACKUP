#pragma once
#include "scene.h"
#include "widgetfluids.h"
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QHash>
#include "particlesystem.h"
#include "defines.h"
#include "integrators.h"
#include "model.h"
#include "colliders.h"


class SceneFluids : public Scene
{
    Q_OBJECT
public:
    SceneFluids();
    ~SceneFluids() ;


    void initialize() ;
    void reset() ;
    void update(double dt) ;
    void paint(const Camera& cam) ;
    void getSceneBounds(Vec3& bmin, Vec3& bmax) ;
    QWidget* sceneUI() ;



    void mousePressed (const QMouseEvent*, const Camera&) ;
    void mouseMoved   (const QMouseEvent*, const Camera&) ;
    void mouseReleased(const QMouseEvent*, const Camera&) ;
    void keyPressed   (const QKeyEvent*,   const Camera&) ;
    void createParticles();
    void buildGrid();


protected:
    WidgetFluids* widget;

    IntegratorSymplecticEuler integrator;
    QOpenGLVertexArrayObject vaoHourglass;
    QOpenGLBuffer vboHourglass{ QOpenGLBuffer::VertexBuffer };
    QOpenGLShaderProgram* hourglassShader = nullptr;
    int hourglassVertexCount = 0;

    // Geometry section indices and counts
    int topConeCount = 0;
    int bottomConeStart = 0;
    int bottomConeCount = 0;

    int topCapStart = 0;
    int topCapCount = 0;
    int bottomCapStart = 0;
    int bottomCapCount = 0;
    ParticleSystem system;
    QVector<Particle> particles;
    int maxParticles = 1000;

    // spatial hash / uniform grid for neighbor search
    float h = 0.02f;               // smoothing radius
    float spacing = h*0.5; //grid initial spacing

    float h2 = h*h;
    float restDensity = 1000.0f;   // rest density
    float k = 1.0f;          // stiffness
    float viscosity = 0.001f;        // viscosity coefficient
    float mass = 0.01;            // per-particle mass (tune)

    // uniform grid
    float cellSize = spacing;        //
    QHash<qint64, QVector<int>> grid; // maps hashed cell -> list of particle indices

    QOpenGLVertexArrayObject* vaoSphereL = nullptr;
    int numFacesSphereL = 0;
    int particleCountToRender = 0;

    ForceConstAcceleration* fGravity;
    QVector<Collider*> colliders;
};
