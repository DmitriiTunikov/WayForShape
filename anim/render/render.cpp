/*
 * Motion planning problem project.
 *
 * FILE: render.cpp
 * AUTHORS:
 *   Denisov Pavel
 * LAST UPDATE: 19.05.2018
 * NOTE: render class implementation file
 */

#include "render.h"

using namespace anim;

/* Getting instance function */
Render & Render::getInstance(void)
{
  static Render rnd;
  return rnd;
} /* End of 'Render::getInstance' function */

/* Init function */
void Render::init(int Argc, char **Argv, const int W, const int H)
{
  _w = W;
  _h = H;

  glutInit(&Argc, Argv);
  glutInitWindowPosition(50, 50);
  glutInitWindowSize(W, H);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutCreateWindow("Motion Planning");

  glOrtho(0, _w, 0, _h, -5.0f, 5.0f);
} /* End of 'Render::init' function */

/* Getting screen width function */
int Render::getWidth(void) const
{
  return _w;
} /* End of 'Render::getWidth' function */

/* Getting screen height function */
int Render::getHeight(void) const
{
  return _h;
} /* End of 'Render::getHeight' function */

/* Drawing line segment function */
void Render::drawSegment(const sup::Vecf &Begin, const sup::Vecf &End) const
{
  glLineWidth(3.0f);
  glBegin(GL_LINES);
  glVertex2f(Begin._coords[0], Begin._coords[1]);
  glVertex2f(End._coords[0], End._coords[1]);
  glEnd();
} /* End of 'Render::drawSegment' function */

/* Drawing circle function */
void Render::drawCircle(const sup::Vecf &pos, float rad) const
{
  //glColor3f(0.0f, 1.0f, 0.0f);
  glBegin(GL_TRIANGLE_FAN);
  glVertex2f(pos._coords[0], pos._coords[1]);
  for (int i = 0; i <= 50; i++)
  {
    float a = (float)i * 3.1415f * 2.0f / 50.0f;
    glVertex2f(pos._coords[0] + cos(a) * rad, pos._coords[1] + sin(a) * rad);
  }
  glEnd();
} /* End of 'Render::drawCircle' function */

/* Drawing polygon function */
void Render::drawPolygon(const Polygon &poly, const bool isSolid) const
{
  //glColor3f(0.0f, 0.0f, 0.0f);
  std::vector<sup::Vecf> vertices = poly._vertices;

  if (vertices.size() == 1)
    glBegin(GL_POINTS);
  else if (vertices.size() == 2)
    glBegin(GL_LINES);
  else
  {
    if (!isSolid)
      glBegin(GL_LINE_LOOP);
    else
      glBegin(GL_POLYGON);
  }

  for (auto vert : vertices)
    glVertex2f(vert._coords[0], vert._coords[1]);
  glEnd();
} /* End of 'Render::drawPolygon' function */

/* Drawing point function */
void Render::drawPoint(const sup::Vecf &Pos, const float R, const float G, const float B)
{
  glPointSize(5.0f);
  glBegin(GL_POINTS);
  glColor3f(R, G, B);
  glVertex2f(Pos._coords[0], Pos._coords[1]);
  glEnd();
  glPointSize(1.0f);
} /* End of 'Render::drawPoint' function */

/* END OF 'render.cpp' FILE */
