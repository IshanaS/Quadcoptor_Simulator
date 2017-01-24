#ifndef SIMULATORWINDOW_H
#define SIMULATORWINDOW_H

#include <QOpenGLWindow>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>
#include <QMediaPlayer>
#include <boost/numeric/odeint.hpp>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <QTime>
#include "transform3d.h"
#include "camera3d.h"
#include "MotorSoftware.h"
#include "quadcoptor.h"
using namespace std;

class QOpenGLShaderProgram;

class simulatorWindow : public QOpenGLWindow, protected QOpenGLFunctions, public MotorSoftware
{
  Q_OBJECT

// OpenGL Events
public:
  simulatorWindow(double dMass, double dIx, double dArmLength, state_type in_state);
  ~simulatorWindow();
  void initializeGL();
  void resizeGL(int width, int height);
  void paintGL();
protected slots:
  void update();

protected:
  void keyPressEvent(QKeyEvent *event);
  void keyReleaseEvent(QKeyEvent *event);
  double dFront, dBack, dLeft, dRight, dThrottle;

private:
  // OpenGL State Information
  QOpenGLBuffer m_vertex;
  QOpenGLVertexArrayObject m_object;
  QOpenGLShaderProgram *m_program;

  // Shader Information
  int u_modelToWorld;
  int u_worldToCamera;
  int u_cameraToView;
  QMatrix4x4 m_projection;
  Camera3D m_camera;
  Transform3D m_transform;

  // An instance of QMediaPlayer
  QMediaPlayer * ptrQuadrotorSound;

  // An instance of motor software
  MotorSoftware * quadSoftware = new MotorSoftware;

  // Variables required for quadcoptor dynamics
  odeint::runge_kutta4<state_type> rk4;
  QTime tStart;
  double t = 0.0;
  double dt = 0.01;
  quadcoptor* q;
  inp_type inp1;

};

#endif // SIMULATORWINDOW_H
