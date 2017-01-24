/* Description: simulatorWindow.cpp
 * This program is responsible for creating a quadrotor using its vertex information and
 * then drawing the same on GPU window. The program is also reponsible for calling the
 * motor software and quadrotor dynamics calculation module and providing them with user
 * inputs (key strokes).
 * Vertex Shader: Takes vertex information) and processes this information to find final
 * positions that the GPU must draw and interpolate between.
 * Fragment Shader: Take information from vertex shader and provide output to back buffer.
*/

#include <QDebug>
#include <QString>
#include <QOpenGLShaderProgram>
#include <QKeyEvent>
#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include "simulatorWindow.h"
#include "vertex.h"
#include "input.h"
using namespace std;

#define debugMode true // preprocessor directives for Debug Mode

// Front Verticies
#define VERTEX_FTR Vertex( QVector3D( 0.5f,  0.05f,  0.5f), QVector3D( 0.5f, 0.5f, 1.0f ) )
#define VERTEX_FTL Vertex( QVector3D(-0.5f,  0.05f,  0.5f), QVector3D( 0.5f, 0.5f, 1.0f ) )
#define VERTEX_FBL Vertex( QVector3D(-0.5f, -0.05f,  0.5f), QVector3D( 0.5f, 0.5f, 0.5f ) )
#define VERTEX_FBR Vertex( QVector3D( 0.5f, -0.05f,  0.5f), QVector3D( 0.5f, 0.5f, 0.5f ) )

// Back Verticies
#define VERTEX_BTR Vertex( QVector3D( 0.5f,  0.05f, -0.5f), QVector3D( 0.5f, 0.5f, 1.0f ) )
#define VERTEX_BTL Vertex( QVector3D(-0.5f,  0.05f, -0.5f), QVector3D( 0.5f, 0.5f, 1.0f ) )
#define VERTEX_BBL Vertex( QVector3D(-0.5f, -0.05f, -0.5f), QVector3D( 0.5f, 0.5f, 0.5f ) )
#define VERTEX_BBR Vertex( QVector3D( 0.5f, -0.05f, -0.5f), QVector3D( 0.5f, 0.5f, 0.5f ) )

// Mid Vertices
#define VERTEX_FTM Vertex( QVector3D( 0.0f,  0.05f,  0.1f), QVector3D( 0.5f, 0.5f, 1.0f ) )
#define VERTEX_FBM Vertex( QVector3D( 0.0f, -0.05f,  0.1f), QVector3D( 0.5f, 0.5f, 0.5f ) )
#define VERTEX_BTM Vertex( QVector3D( 0.0f,  0.05f, -0.1f), QVector3D( 0.5f, 0.5f, 1.0f ) )
#define VERTEX_BBM Vertex( QVector3D( 0.0f, -0.05f, -0.1f), QVector3D( 0.5f, 0.5f, 0.5f ) )
#define VERTEX_MTL Vertex( QVector3D(-0.1f,  0.05f,  0.0f), QVector3D( 0.5f, 0.5f, 1.0f ) )
#define VERTEX_MTR Vertex( QVector3D( 0.1f,  0.05f,  0.0f), QVector3D( 0.5f, 0.5f, 1.0f ) )
#define VERTEX_MBL Vertex( QVector3D(-0.1f, -0.05f,  0.0f), QVector3D( 0.5f, 0.5f, 0.5f ) )
#define VERTEX_MBR Vertex( QVector3D( 0.1f, -0.05f,  0.0f), QVector3D( 0.5f, 0.5f, 0.5f ) )

// Create a colored quadrotor
static const Vertex sg_vertexes[] = {
    // Face 1 (Front)
    VERTEX_FTR, VERTEX_FTM, VERTEX_FBM,
    VERTEX_FBM, VERTEX_FBR, VERTEX_FTR,
    VERTEX_FTM, VERTEX_FTL, VERTEX_FBL,
    VERTEX_FBL, VERTEX_FBM, VERTEX_FTM,
    // Face 2 (Back)
    VERTEX_BBR, VERTEX_BTM, VERTEX_BTR,
    VERTEX_BTM, VERTEX_BBR, VERTEX_BBM,
    VERTEX_BBM, VERTEX_BTL, VERTEX_BTM,
    VERTEX_BTL, VERTEX_BBM, VERTEX_BBL,
    // Face 3 (Top)
    VERTEX_FTL, VERTEX_FTM, VERTEX_MTL,
    VERTEX_MTL, VERTEX_BTM, VERTEX_BTL,
    VERTEX_BTM, VERTEX_MTR, VERTEX_BTR,
    VERTEX_MTR, VERTEX_FTM, VERTEX_FTR,
    VERTEX_FTM, VERTEX_MTR, VERTEX_MTL,
    VERTEX_BTM, VERTEX_MTL, VERTEX_MTR,
    // Face 4 (Bottom)
    VERTEX_FBL, VERTEX_MBL, VERTEX_FBM,
    VERTEX_MBL, VERTEX_BBL, VERTEX_BBM,
    VERTEX_BBM, VERTEX_BBR, VERTEX_MBR,
    VERTEX_MBR, VERTEX_FBR, VERTEX_FBM,
    VERTEX_FBM, VERTEX_MBL, VERTEX_MBR,
    VERTEX_BBM, VERTEX_MBR, VERTEX_MBL,
    // Face 5 (Left)
    VERTEX_FBL, VERTEX_FTL, VERTEX_MTL,
    VERTEX_MTL, VERTEX_MBL, VERTEX_FBL,
    VERTEX_MTL, VERTEX_BBL, VERTEX_MBL,
    VERTEX_BBL, VERTEX_MTL, VERTEX_BTL,
    // Face 6 (Right)
    VERTEX_FBR, VERTEX_MTR, VERTEX_FTR,
    VERTEX_MTR, VERTEX_FBR, VERTEX_MBR,
    VERTEX_MTR, VERTEX_MBR, VERTEX_BBR,
    VERTEX_BBR, VERTEX_BTR, VERTEX_MTR
};

#undef VERTEX_BBR
#undef VERTEX_BBL
#undef VERTEX_BTL
#undef VERTEX_BTR

#undef VERTEX_FBR
#undef VERTEX_FBL
#undef VERTEX_FTL
#undef VERTEX_FTR

#undef VERTEX_BBM
#undef VERTEX_BTM
#undef VERTEX_FBM
#undef VERTEX_FTM
#undef VERTEX_MTL
#undef VERTEX_MTR
#undef VERTEX_MBL
#undef VERTEX_MBR

simulatorWindow::simulatorWindow(double dMass, double dIx, double dArmLength, state_type in_state)
{
    m_transform.translate(0.0f, -2.0f, -10.0f); // Negative z to keep quadrotor in screen @ t=0
    m_transform.rotate(5.0f, QVector3D(1.0f, 0.0f, 0.0f)); // To rotate the quadrotor

    // Initialize protected variables
    dFront         = 0.0;
    dBack          = 0.0;
    dLeft          = 0.0;
    dRight         = 0.0;
    dThrottle      = 0.0;

    // Play quadrotor sound
    ptrQuadrotorSound = new QMediaPlayer();
    ptrQuadrotorSound->setMedia(QUrl("qrc:/Music/HelicoptorMusic.mp3"));
    ptrQuadrotorSound->setVolume(50);
    ptrQuadrotorSound->play();

    // Create quadcoptor class instance
    q = new quadcoptor(dMass, dIx, dArmLength, in_state);
}

simulatorWindow::~simulatorWindow()
{
    makeCurrent();
    m_object.destroy();
    m_vertex.destroy();
    delete m_program;
}

/*******************************************************************************
 * OpenGL Events
 ******************************************************************************/

void simulatorWindow::initializeGL()
{
    // Initialize OpenGL Backend
    initializeOpenGLFunctions();
    connect(this, SIGNAL(frameSwapped()), this, SLOT(update()));

    // Set global information
    glEnable(GL_CULL_FACE);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    // Application-specific initialization
    {
        // Create Shader (Do not release until QOpenGLVertexArrayObjects object is created)
        m_program = new QOpenGLShaderProgram();
        m_program->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/simple.vert");
        m_program->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/simple.frag");
        m_program->link();
        m_program->bind();

        // Cache Uniform Locations
        u_modelToWorld = m_program->uniformLocation("modelToWorld");
        u_worldToCamera = m_program->uniformLocation("worldToCamera");
        u_cameraToView = m_program->uniformLocation("cameraToView");

        // Create Buffer (Do not release until QOpenGLVertexArrayObjects object is created)
        m_vertex.create();
        m_vertex.bind();
        m_vertex.setUsagePattern(QOpenGLBuffer::StaticDraw);
        m_vertex.allocate(sg_vertexes, sizeof(sg_vertexes));

        // Create Vertex Array Object
        m_object.create();
        m_object.bind();
        m_program->enableAttributeArray(0);
        m_program->enableAttributeArray(1);
        m_program->setAttributeBuffer(0, GL_FLOAT, Vertex::positionOffset(), Vertex::PositionTupleSize, Vertex::stride());
        m_program->setAttributeBuffer(1, GL_FLOAT, Vertex::colorOffset(), Vertex::ColorTupleSize, Vertex::stride());

        // Release (unbind) all
        m_object.release();
        m_vertex.release();
        m_program->release();
    }
}

void simulatorWindow::resizeGL(int width, int height)
{
    m_projection.setToIdentity();
    m_projection.perspective(45.0f, width / float(height), 0.0f, 1000.0f);
}

void simulatorWindow::paintGL()
{
    // Clear
    glClear(GL_COLOR_BUFFER_BIT);

    // Render using our shader
    m_program->bind();
    m_program->setUniformValue(u_worldToCamera, m_camera.toMatrix());
    m_program->setUniformValue(u_cameraToView, m_projection);
    {
        m_object.bind();
        m_program->setUniformValue(u_modelToWorld, m_transform.toMatrix());
        glDrawArrays(GL_TRIANGLES, 0, sizeof(sg_vertexes) / sizeof(sg_vertexes[0]));
        m_object.release();
    }
    m_program->release();
}

void simulatorWindow::update()
{
    if (t==0.0)
    {
        tStart.start();
    }

    static dvar* ptrSpeeds;
    static const float scale = 0.01f;

    // Update input
    Input::update();

    if (debugMode==true)
    {
        cout << "dFront = " << dFront << endl;
        cout << "dBack = " << dBack << endl;
        cout << "dLeft = " << dLeft << endl;
        cout << "dRight = " << dRight << endl;
        cout << "dThrottle = " << dThrottle << endl;
    }

    // Process key press event
    if (Input::keyPressed(Qt::Key_Right))
    {
        dFront = 1.0;

        // Backgroung music
        if (ptrQuadrotorSound->state() == QMediaPlayer::StoppedState) {
            ptrQuadrotorSound->play();
        }

        if (debugMode==true)
            cout << "Up key pressed" << endl;
    }
    if (Input::keyReleased(Qt::Key_Right))
    {
        dFront = 0.0;

        if (debugMode==true)
            cout << "Up key released" << endl;
    }
    if (Input::keyPressed(Qt::Key_Left))
    {
        dBack  = 1.0;

        // Backgroung music
        if (ptrQuadrotorSound->state() == QMediaPlayer::StoppedState) {
            ptrQuadrotorSound->play();
        }

        if (debugMode==true)
            cout << "Down key pressed" << endl;
    }
    if (Input::keyReleased(Qt::Key_Left))
    {
        dBack  = 0.0;

        if (debugMode==true)
            cout << "Down key released" << endl;
    }
    if (Input::keyPressed(Qt::Key_Up))
    {
        dLeft  = 1.0;

        // Backgroung music
        if (ptrQuadrotorSound->state() == QMediaPlayer::StoppedState) {
            ptrQuadrotorSound->play();
        }

        if (debugMode==true)
            cout << "Left key pressed" << endl;
    }
    if (Input::keyReleased(Qt::Key_Up))
    {
        dLeft  = 0.0;

        if (debugMode==true)
            cout << "Left key released" << endl;
    }
    if (Input::keyPressed(Qt::Key_Down))
    {
        dRight = 1.0;

        // Backgroung music
        if (ptrQuadrotorSound->state() == QMediaPlayer::StoppedState) {
            ptrQuadrotorSound->play();
        }

        if (debugMode==true)
            cout << "Right key pressed" << endl;
    }
    if (Input::keyReleased(Qt::Key_Down))
    {
        dRight = 0.0;

        if (debugMode==true)
            cout << "Right key released" << endl;
    }
    if (Input::keyPressed(Qt::Key_1))
    {
        dThrottle = 0.2;
        ptrQuadrotorSound->setVolume(60);

        // Backgroung music
        if (ptrQuadrotorSound->state() == QMediaPlayer::StoppedState) {
            ptrQuadrotorSound->play();
        }

        if (debugMode==true)
            cout << "1 pressed" << endl;
    }
    if (Input::keyPressed(Qt::Key_2))
    {
        dThrottle = 0.4;
        ptrQuadrotorSound->setVolume(70);

        // Backgroung music
        if (ptrQuadrotorSound->state() == QMediaPlayer::StoppedState) {
            ptrQuadrotorSound->play();
        }

        if (debugMode==true)
            cout << "2 pressed" << endl;
    }
    if (Input::keyPressed(Qt::Key_3))
    {
        dThrottle = 0.6;
        ptrQuadrotorSound->setVolume(80);

        // Backgroung music
        if (ptrQuadrotorSound->state() == QMediaPlayer::StoppedState) {
            ptrQuadrotorSound->play();
        }

        if (debugMode==true)
            cout << "3 pressed" << endl;
    }
    if (Input::keyPressed(Qt::Key_4))
    {
        dThrottle = 0.8;
        ptrQuadrotorSound->setVolume(90);

        // Backgroung music
        if (ptrQuadrotorSound->state() == QMediaPlayer::StoppedState) {
            ptrQuadrotorSound->play();
        }

        if (debugMode==true)
            cout << "4 pressed" << endl;
    }
    if (Input::keyPressed(Qt::Key_5))
    {
        dThrottle = 1.0;
        ptrQuadrotorSound->setVolume(100);

        // Backgroung music
        if (ptrQuadrotorSound->state() == QMediaPlayer::StoppedState) {
            ptrQuadrotorSound->play();
        }

        if (debugMode==true)
            cout << "5 pressed" << endl;
    }
    if (Input::keyPressed(Qt::Key_Escape))
    {
        exit (0);
    }

    // Set motor voltage and get updated motor speeds based on the instantaneous inputs
    quadSoftware->setVoltage(dThrottle,dFront,dBack,dLeft,dRight);
    ptrSpeeds = quadSoftware->getAllSpeed();
    if (debugMode==true)
    {
        cout << "Speed of motor 1 : " << *(ptrSpeeds) << endl;
        cout << "Speed of motor 2 : " << *(ptrSpeeds+1) << endl;
        cout << "Speed of motor 3 : " << *(ptrSpeeds+2) << endl;
        cout << "Speed of motor 4 : " << *(ptrSpeeds+3) << endl;
    }

    // Quadcoptor dynamics update:
    double dFactor = 9906/40; // Double to convert % motor speed to actual motor speed
    inp1 = {{dFactor* *(ptrSpeeds),dFactor* *(ptrSpeeds+1),dFactor* *(ptrSpeeds+2),dFactor* *(ptrSpeeds+3)}};
    q->setInp(inp1);
    q->updateSys();

    // Update t and dt from the system clock
    dt = -t; // Save previous time
    t  = tStart.elapsed()/1000.0; // Update time
    dt = t + dt; // Calculate actual delta t
    if (debugMode==true)
    {
        cout << "Time = " << t << "; dt = " << dt << endl;
    }

    // Integrate for time period t to t+dt
    if (t>0.0 && dThrottle>0)
    {
    integrate_const( rk4, *q , q->x , t , t+dt, dt );
    }
    if (debugMode==true)
    {
        cout << t << " " << q->x[0] << " " << q->x[1] << " " << q->x[2] << " " << q->x[3] << " " << q->x[4] << " " << q->x[5] << " " << q->x[6] << " " << q->x[7] << " " << q->x[8]<< " " << q->x[9] << " " << q->x[10] << " " << q->x[11] << endl;
    }

    // Update: Position and orientation of quadrotor
    if (dThrottle!=0.0) // Check if quadcoptor motors are turned on
    {
        // Update: Position of quadrotor
        m_transform.translate(float(scale*q->x[6]*dt), float(scale*q->x[8]*dt), -float(scale*q->x[7]*dt)); // (dx,dz,dy)

        // Update: Orientation of quadrotor
        if (dFront==1.0)
        {
            if (dLeft==1.0)
                m_transform.setRotation(-5.0f, QVector3D(1.0f, 0.0f, 1.0f));
            else if (dRight==1.0)
                m_transform.setRotation(-5.0f, QVector3D(-1.0f, 0.0f, 1.0f));
            else
                m_transform.setRotation(-5.0f, QVector3D(0.0f, 0.0f, 1.0f));
        }
        else if (dBack==1.0)
        {
            if (dLeft==1.0)
                m_transform.setRotation(5.0f, QVector3D(-1.0f, 0.0f, 1.0f));
            else if (dRight==1.0)
                m_transform.setRotation(5.0f, QVector3D(1.0f, 0.0f, 1.0f));
            else
                m_transform.setRotation(5.0f, QVector3D(0.0f, 0.0f, 1.0f));
        }
        else if (dLeft==1.0)
        {
            m_transform.setRotation(-5.0f, QVector3D(1.0f, 0.0f, 0.0f));
        }
        else if (dRight==1.0)
        {
            m_transform.setRotation(5.0f, QVector3D(1.0f, 0.0f, 0.0f));
        }
        else
        {
            m_transform.setRotation(0.0f, QVector3D(0.0f, 0.0f, 1.0f));
        }
    }

    // Schedule a redraw
    QOpenGLWindow::update();

}

void simulatorWindow::keyPressEvent(QKeyEvent *event)
{
    Input::registerKeyPress(event->key());
}

void simulatorWindow::keyReleaseEvent(QKeyEvent *event)
{

    Input::registerKeyRelease(event->key());
}
