/*
 * Motion planning problem project.
 *
 * FILE: triangulate.cpp
 * AUTHORS:
 *   Denisov Pavel,
 *   Federov Dmitrii
 * LAST UPDATE: 22.05.2018
 * NOTE: triangulating space class declaration file
 */

#include <numeric>
#include <algorithm>

#include "triangulate.h"
#include "../math_types/collisions/collisions.h"
#include "../math_types/distances/distances.h"
#include "../motion_planning.h"

/* Finding T parametr between to segments */
static float s_findTParametr(const Segment &AB_Per, const Segment &AB, const Segment &BP_Per)
{
  collision::CollideRes res = collision::collideSegmentSegmentAsLines(AB_Per, BP_Per);
  float sin = (AB._sPoint - AB._fPoint).getNormalized().getSin((res._intersectPoints[0] - AB._fPoint).getNormalized());
  int sign = sin > 0 ? 1 : -1;

  return -sign * distances::distBetweenPointAndSegment(res._intersectPoints[0], AB)._dist;
} /* End of 's_findTParametr' function */

/* IsValid */
bool Triangulate::isValid(const Segment &Seg)
{
  MotionPlanning &m_plan = MotionPlanning::getInstance();

  if (m_plan.isSegmentIntersectPolygons(Seg))
    return false;

  return true;
} /* End of 'Triangulate::isValid' function */

/* Finding new vertex to add to triangle */
bool Triangulate::mate(const Segment &e, std::vector<sup::Vecf> &points, sup::Vecf &conj)
{
  sup::Vecf bestp(-1, -1);

  float t, bestt = std::numeric_limits<float>::max();

  Segment f = e.rotate(sup::pi / 2);
  for (size_t i = 0; i < points.size(); i++)
  {
    if (e.classify(points[i]) == Segment::ClassifyRes::RIGHT)
    {
      Segment g = Segment(e._sPoint, points[i]).rotate(sup::pi / 2);
      t = s_findTParametr(f, e, g);
      if (t < bestt)// && (isValid(Segment(e._fPoint, points[i])) && isValid(Segment(e._sPoint, points[i]))))
      {
        bestp = points[i];
        bestt = t;
      }
    }
  }

  if (bestp != sup::Vecf(-1, -1))
  {
    conj = bestp;
    return true;
  }

  return false;
}  /* End of 'Triangulate::mate' function */

/* Find first edge function */
Segment Triangulate::hullEdge(std::vector<sup::Vecf> &points)
{
  size_t m = 0, i;
  for (i = 1; i < points.size(); i++)
    if (points[i] < points[m])
      m = i;

  std::swap(points[0], points[m]);

  for (m = 1, i = 2; i < points.size(); i++)
  {
    Segment::ClassifyRes c = Segment(points[0], points[m]).classify(points[i]);
    if ((c == Segment::ClassifyRes::LEFT) || (c == Segment::ClassifyRes::BETWEEN))
      m = i;
  }

  return Segment(points[0], points[m]);
} /* End of 'Triangulate::hullEdge' function */

/* Updating the frontier array */
void Triangulate::updateFrontier(std::vector<Segment> &frontier, const sup::Vecf &a, const sup::Vecf &b)
{
  Segment e(a, b);
  auto it = std::find(frontier.begin(), frontier.end(), e);

  if (it  != frontier.end())
    frontier.erase(it);
  else
  {
    e.flip();
    frontier.push_back(e);
  }
} /* End of 'Triangulate::updateFrontier' function*/

/* Returning and removing min from Segment vector by using edgeCmp func function */
static Segment s_removeMin(std::vector<Segment> &s)
{
  auto res = s.begin();
  for (auto it = s.begin() + 1; it != s.end(); it++)
    if (*it < *res)
      res = it;

  Segment resSeg = *res;
  s.erase(res);
  return resSeg;
} /* End of 'removeMin' function */

/* Deleting inner triangles function */
void Triangulate::deleteInnerTriangles(void)
{
  MotionPlanning &m_plan = MotionPlanning::getInstance();

  for (auto it = _triangles.begin(); it != _triangles.end(); )
  {
    bool isBad = false;
    if (m_plan.isTrianglePolygon(it->_vertices[0], it->_vertices[1], it->_vertices[2]))
      isBad = true;

    if (isBad)
      it = _triangles.erase(it);
    else
      it++;
  }

  for (auto it = _triangles.begin(); it != _triangles.end(); )
  {
    bool isBad = false;
    if (m_plan.isSegmentConnectNotNeighbours(Segment(it->_vertices[0], it->_vertices[1])))
      isBad = true;
    else if (m_plan.isSegmentConnectNotNeighbours(Segment(it->_vertices[1], it->_vertices[2])))
      isBad = true;
    else if (m_plan.isSegmentConnectNotNeighbours(Segment(it->_vertices[2], it->_vertices[0])))
      isBad = true;

    if (isBad)
      it = _triangles.erase(it);
    else
      it++;
  }
} /* End of 'Triangulate::deleteInnerTriangles' function*/

/* Building trinagulating space function */
void Triangulate::build(const std::vector<sup::Vecf> &Verts, const std::vector<Polygon> &polygons, bool main)
{
  std::vector<sup::Vecf> points(Verts);
  sup::Vecf p;

  std::vector<Segment> frontier;

  Segment e = hullEdge(points);

  frontier.push_back(e);
  while (!frontier.empty())
  {
    e = s_removeMin(frontier);
    if (mate(e, points, p))
    {
      Polygon triangle;
      updateFrontier(frontier, p, e._fPoint);
      updateFrontier(frontier, e._sPoint, p);
      triangle.addVertex(e._fPoint);
      triangle.addVertex(e._sPoint);
      triangle.addVertex(p);
      triangle.setId(_triangles.size());
      _triangles.push_back(triangle);
    }
  }

  addPolygons(polygons);

  if (main && Verts.size() != 4)
    deleteInnerTriangles();

  for (size_t i = 0; i < _triangles.size(); i++)
    _triangles[i].setId(i);
} /* End of 'Triangulate::build' function */

/* Clear function */
void Triangulate::clear(void)
{
  _triangles.clear();
} /* End of 'Triangulate::clear' function */

/* Adding new obstacle segment in triangulation array function */
void Triangulate::addSegmet(const Segment &seg)
{
  bool needMege = false;
  Polygon SegPol;
  SegPol.addVertex(seg._sPoint);
  SegPol.addVertex(seg._fPoint);

  Triangulate tmpR, tmpL;
  std::vector<sup::Vecf> polyToAddR, polyToAddL;

  for (std::vector<Polygon>::iterator tri = _triangles.begin(); tri != _triangles.end(); )
  {
    collision::CollideRes res;
    if (collision::isCollideSegmentPolygon(seg, *tri, res))
    {
      for (int i = 0; i < 3; i++)
        if ((seg._sPoint - seg._fPoint).getSin(seg._sPoint - tri->_vertices[i]) < -sup::eps)
        {
          if (std::find(polyToAddR.begin(), polyToAddR.end(), tri->_vertices[i]) == polyToAddR.end())
            polyToAddR.push_back(tri->_vertices[i]);
        }
        else if ((seg._sPoint - seg._fPoint).getSin(seg._sPoint - tri->_vertices[i]) > sup::eps)
        {
          if (std::find(polyToAddL.begin(), polyToAddL.end(), tri->_vertices[i]) == polyToAddL.end())
             polyToAddL.push_back(tri->_vertices[i]);
        }
        else
        {
          if (std::find(polyToAddL.begin(), polyToAddL.end(), tri->_vertices[i]) == polyToAddL.end())
            polyToAddL.push_back(tri->_vertices[i]);
          if (std::find(polyToAddR.begin(), polyToAddR.end(), tri->_vertices[i]) == polyToAddR.end())
            polyToAddR.push_back(tri->_vertices[i]);
        }
      tri = _triangles.erase(tri);
      needMege = true;
    }
    else
      tri++;
  }

  if (needMege)
  {
    std::vector<Polygon> empty; 
    tmpR.build(polyToAddR, empty, false);
    tmpL.build(polyToAddL, empty, false);
    
    for (auto tr : tmpR._triangles)
      _triangles.push_back(tr);
    
    for (auto tr : tmpL._triangles)
      _triangles.push_back(tr);
  }
} /* End of 'Triangulate::addSegmet' function */

/* Adding new obstacle in triangulation array function */
void Triangulate::addPolygon(const Polygon &poly)
{
  std::vector<Segment> sidePoly;
  int size = poly._vertices.size();
  for (int i = 0; i < size - 1; i++)
    sidePoly.push_back(Segment(poly._vertices[i], poly._vertices[i + 1]));
  sidePoly.push_back(Segment(poly._vertices[size - 1], poly._vertices[0]));

  for (auto side : sidePoly)
    addSegmet(side);
} /* End of 'Triangulate::addPolygon' function */

/* Adding new obstacles in triangulation array function */
void Triangulate::addPolygons(const std::vector<Polygon> &polygons)
{
  for (auto poly : polygons)
    addPolygon(poly);
} /* End of 'Triangulate::addPolygons' function */