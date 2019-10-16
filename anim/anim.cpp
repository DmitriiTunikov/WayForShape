/*
 * Motion planning problem project.
 *
 * FILE: anim.cpp
 * AUTHORS:
 *   Denisov Pavel
 * LAST UPDATE: 19.05.2018
 * NOTE: animation class implementation file
 */

#include <glut.h>

#include "anim.h"
#include "render\render.h"
#include "../motion_planning/input/motion_planning_input.h"
#include "../motion_planning/motion_planning.h"

using namespace anim;

/* Class constructor */
void Anim::init(int Argc, char **Argv, const int W, const int H)
{
  Render &rnd = Render::getInstance();
  rnd.init(Argc, Argv, W, H);

  glutDisplayFunc(glutRender);
  glutKeyboardFunc(glutKeyboard);
  glutReshapeFunc(glutResize);
  glutMouseFunc(glutMouse);
} /* End of constructor */

/* Looping function */
void Anim::run(void)
{
  glutMainLoop();
} /* End of 'Anim::run' function */

/* Getting instance function */
Anim & Anim::getInstance(void)
{
  static Anim anim;
  return anim;
} /* End of 'Anim::getInstance' function */

/* GLUT render function */
void anim::glutRender(void)
{
  Anim &anim = Anim::getInstance();
  anim.render();
} /* End of 'anim::glutRender' function */

/* GLUT reshape function */
void anim::glutResize(int W, int H)
{
  Anim &anim = Anim::getInstance();
  anim.resize(W, H);
} /* End of 'anim::glutResize' function */

/* GLUT keyboard function */
void anim::glutKeyboard(unsigned char Key, int X, int Y)
{
  Anim &anim = Anim::getInstance();
  anim.keyboard(Key, X, Y);
} /* End of 'anim::glutKeyboard' function */

/* GLUT mouse function */
void anim::glutMouse(int Button, int State, int X, int Y)
{
  Anim &anim = Anim::getInstance();
  anim.mouseFunc(Button, State, X, Y);
} /* End of 'glutMouse' function */

/* Render function */
void Anim::render(void)
{
  glClearColor(0.3, 0.5, 0.7, 1.0);
  glClear(GL_COLOR_BUFFER_BIT);

  MotionPlanning &plan = MotionPlanning::getInstance();
  plan.draw();

  glutSwapBuffers();
  glutPostRedisplay();
  glFlush();
} /* End of 'Anim::render' function */

/* Resize function */
void Anim::resize(const int W, const int H)
{

} /* End of 'Anim::resize' function */

/* Keyboard function */
void Anim::keyboard(const unsigned char Key, const int X, const int Y)
{
  MotionPlanningInput &motionPlanInput = MotionPlanningInput::getInstance();
  motionPlanInput.onKeyboard(Key, X, Y);
} /* End of 'Anim::keyboard' function */

/* Mouse function */
void Anim::mouseFunc(const int Button, const int State, const int X, const int Y)
{
  MotionPlanningInput &motionPlanInput = MotionPlanningInput::getInstance();
  motionPlanInput.onMouse(Button, State, X, Y);
} /* End of 'Anim::mouseFunc' function */

/* END OF 'anim.cpp' FILE */
