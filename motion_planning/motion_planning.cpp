/*
 * Motion planning problem project.
 *
 * FILE: motion_planning.cpp
 * AUTHORS:
 *   Denisov Pavel
 * LAST UPDATE: 19.05.2018
 * NOTE: motion planning class implementation file
 */

#include <glut.h>
#include <ostream>
#include <fstream>
#include "motion_planning.h"
#include "math_types\distances\distances.h"
#include "../anim/render/render.h"
#include "input\motion_planning_input.h"

/* Default class constructor */
MotionPlanning::MotionPlanning(void) : _rad(25.0f), _startP({-2, -2}), _finishP({-2, -2}), _isBuilded(false), _isReady(false)
{
} /* End of constructor */

 /* Getting instance function */
MotionPlanning & MotionPlanning::getInstance(void)
{
  static MotionPlanning mp;
  return mp;
} /* End of 'MotionPlanning::getInstance' function */

/* Setting start point function */
void MotionPlanning::setStartPoint(const sup::Vecf &SP)
{
  _startP = SP;
  // TODO: rebuild path
} /* End of 'MotionPlanning::setStartPoint' function */

/* Setting finish point function */
void MotionPlanning::setFinishPoint(const sup::Vecf &FP)
{
  _finishP = FP;
} /* End of 'MotionPlanning::setFinishPoint' function */

/* Adding new polygon function */
void MotionPlanning::addPolygon(const Polygon &Poly)
{
  _polygons.push_back(Poly);
  _polygons[_polygons.size() - 1].setId(_polygons.size() - 1);
} /* End of 'MotionPlanning::addPolygon' function */

/* Adding vertex to last polygon function */
void MotionPlanning::addVertexToLastPolygon(const sup::Vecf &Vert, const int Index)
{
  if (_polygons.size() == 0 || _polygons.size() == Index)
    addPolygon(Polygon());
  _polygons[Index].addVertex(Vert);
} /* End of 'MotionPlanning::addVertexToLastPolygon' function */

/* Setting _isReady variable function */
void MotionPlanning::setReady(const bool Flag)
{
  _isReady = Flag;
  if (_isReady)
  {
    build();
    return;
  }

  _isBuilded = false;
} /* End of 'MotionPlanning::setReady' function */

/* Drawing function */
void MotionPlanning::draw(void) const
{
  anim::Render &rnd = anim::Render::getInstance();
  glColor3f(0.0f, 0.0f, 0.0f);
  glLineWidth(4);

  if (_polygons.size() != 0)
  {
    for (int i = 0; i < _polygons.size() - 1; i++)
      _polygons[i].draw(true);
    _polygons[_polygons.size() - 1].draw();
  }

  glLineWidth(1);
  rnd.drawPoint(_startP, 1.0f, 0.0f, 0.0f);
  rnd.drawPoint(_finishP, 1.0f, 1.0f, 0.0f);

  if (_isBuilded)
  {
    // draw all
    glLineWidth(6);
    glColor3f(1.0f, 1.0f, 1.0f);
    for (auto elem : _adjGraph.getElements())
      elem.draw();
    glLineWidth(1);

    glColor3f(1.0f, 0.0f, 0.0f);
    for (auto triangle : _triang._triangles)
      triangle.draw();

    glColor3f(0.0f, 1.0f, 0.0f);
    std::vector<Vertex> verts = _adjVertGraph.getElements();
    for (size_t i = 0; i < verts.size(); i++)
    {      
      std::vector<int> neigbs = verts[i].getNeighbors();
      for (size_t j = 0; j < neigbs.size(); j++)
        rnd.drawSegment(verts[i].getPos(), verts[neigbs[j]].getPos());
    }

    glColor3f(0.0f, 1.0f, 1.0f);
    for (size_t i = 0; i < _path.size() - 1; i++)
      rnd.drawSegment(_path[i], _path[i + 1]);

    static sup::Vecf curPos = _path[0];
    static float T = 0.0f;
    static int Ind = 1;

    if (fabs((curPos - _finishP).length()) > sup::eps)
    {
      float dist = (_path[Ind] - _path[Ind - 1]).length();
      curPos = _path[Ind - 1] + (_path[Ind] - _path[Ind - 1]).getNormalized() * T * dist;
      T += 0.0005f;

      if (fabs(1.0f - T) < sup::eps || (curPos - _path[Ind]).length() < sup::eps)
      {
        curPos = _path[Ind];

        Ind++;
        Ind %= _path.size();
        T = 0.0f;
      }
    }
    else
    {
      curPos = _startP;
      Ind = 1;
    }

    glColor3f(1.0f, 0.0f, 0.0f);
    rnd.drawCircle(curPos, _rad);
  }
} /* End of 'MotionPlanning::draw' function */

/* Clear function */
void MotionPlanning::clear(void)
{
  _adjGraph.clear();
  _adjVertGraph.clear();
  _cameFrom.clear();
  _rad = 25.0f;
  _path.clear();
  _triang.clear();
  _polygons.clear();
  _startP = _finishP = {-1.0f, -1.0f};
} /* End of 'MotionPlanning::clear' function */

/* Class destructor */
MotionPlanning::~MotionPlanning(void)
{
  clear();
} /* End of destructor */

/* Saving current scene function */
void MotionPlanning::save(const std::string &FileName) const
{
  std::ofstream out(FileName);

  out << _polygons.size() - 1 << std::endl;
  for (size_t i = 0; i < _polygons.size() - 1; i++)
  {
    out << _polygons[i]._vertices.size() << " ";

    for (size_t j = 0; j < _polygons[i]._vertices.size(); j++)
    {
      out << _polygons[i]._vertices[j]._coords[0] << " ";
      out << _polygons[i]._vertices[j]._coords[1] << " ";
    }
    out << std::endl;
  }
  out << _startP._coords[0] << " " << _startP._coords[1] << std::endl;
  out << _finishP._coords[0] << " " << _finishP._coords[1] << std::endl;

  out << std::endl;
} /* End of 'MotionPlanning::save' function */

/* Loading current scene function */
void MotionPlanning::load(const std::string &FileName)
{
  clear();
  std::ifstream in(FileName);

  int n;
  in >> n;

  for (int i = 0; i < n; i++)
  {
    int curN;

    in >> curN;

    Polygon poly;
    for (int j = 0; j < curN; j++)
    {
      sup::Vecf vert;
      in >> vert._coords[0];
      in >> vert._coords[1];

      poly.addVertex(vert);
    }

    poly.setId(i);
    _polygons.push_back(poly);
  }

  in >> _startP._coords[0];
  in >> _startP._coords[1];
  in >> _finishP._coords[0];
  in >> _finishP._coords[1];

  MotionPlanningInput &minput = MotionPlanningInput::getInstance();
  minput.setType(MotionPlanningInput::MODE_TYPE::BUILDING);

  setReady(true);
} /* End of 'MotionPlanning::load' function */

/* END OF 'motion_planning.cpp' FILE */
