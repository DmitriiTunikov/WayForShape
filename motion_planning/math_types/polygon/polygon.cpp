/*
 * Motion planning problem project.
 *
 * FILE: polygon.cpp
 * AUTHORS:
 *   Denisov Pavel
 * LAST UPDATE: 19.05.2018
 * NOTE: polygon class implementation file
 */

#include "polygon.h"
#include "../../../anim/render/render.h"
#include "../../motion_planning.h"

/* Adding new vertex function */
void Polygon::addVertex(const sup::Vecf &Vert)
{
  _vertices.push_back(Vert);
} /* End of 'Polygon::addVertex' function */

/* Adding new vertex operator */
void Polygon::operator<<(const sup::Vecf &Vert)
{
  addVertex(Vert);
} /* End of 'Polygon::operator<<' operator */

/* Getting id function */
int Polygon::getId(void) const
{
  return _id;
} /* End of 'Polygon::getId' function */

/* Virtual draw function */
void Polygon::draw(const bool isSolid) const
{
  anim::Render &rnd = anim::Render::getInstance();
  rnd.drawPolygon(*this, isSolid);
} /* End of 'Polygon::draw' function */

/* Equal operator */
bool Polygon::operator==(const Polygon &Poly) const
{
  return _id == Poly._id;
} /* End of 'Polygon::operator==' operator */

/* Non-equal operator */
bool Polygon::operator!=(const Polygon &Poly) const
{
  return _id != Poly._id;
} /* End of 'Polygon::operator!=' function */

/* Setting id function */
void Polygon::setId(const int ID)
{
  _id = ID;
} /* End of 'Polygon::setId' function */

/* Contains the vertex function */
int Polygon::contains(const sup::Vecf &Vertex) const
{
  for (auto vert : _vertices)
    if (fabs((vert - Vertex).length()) < sup::eps)
      return true;
  return false;
} /* End of 'Polygon::contains' function */

/* Contains the segment as edge function */
bool Polygon::containsSegmentAsEdge(const Segment &Seg) const
{
  if (!contains(Seg._fPoint) || !contains(Seg._sPoint))
    return false;

  if (_vertices[0] == Seg._sPoint)
  {
    if (_vertices[1] == Seg._fPoint || _vertices[_vertices.size() - 1] == Seg._fPoint)
      return true;
  }
  else if (_vertices[0] == Seg._fPoint)
  {
    if (_vertices[1] == Seg._sPoint || _vertices[_vertices.size() - 1] == Seg._sPoint)
      return true;
  }

  for (size_t i = 1; i < _vertices.size() - 1; i++)
  {
    if (_vertices[i] == Seg._fPoint)
    {
      if (_vertices[i + 1] == Seg._sPoint || _vertices[i - 1] == Seg._sPoint)
        return true;
    }
    else if (_vertices[i] == Seg._sPoint)
    {
      if (_vertices[i + 1] == Seg._fPoint || _vertices[i - 1] == Seg._fPoint)
        return true;
    }
  }

  return false;
} /* End of 'Polygon::containsSegmentAsEdge' function */

/* Is adjacency with another polygon */
int Polygon::isAdjacencyToTriangle(const Polygon &Tri) const
{
  if (containsSegmentAsEdge(Segment(Tri._vertices[0], Tri._vertices[1])) ||
    containsSegmentAsEdge(Segment(Tri._vertices[1], Tri._vertices[2])) ||
    containsSegmentAsEdge(Segment(Tri._vertices[2], Tri._vertices[0])))
    return true;
  return false;
} /* End of 'Polygon::isAdjacency' function */

/* Getting neighbors function */
std::vector<int> Polygon::getNeighbors(void) const
{
  return _neighbors;
} /* End of 'Polygon::getNeighbors' function */

/* Adding neighbor function */
void Polygon::addNeighbor(const int ID)
{
  _neighbors.push_back(ID);
} /* End of 'Polygon::addNeighbor' function */

/* Deleting neighbor function */
void Polygon::deleteNeighbor(const int Id)
{
  _neighbors.erase(std::find(_neighbors.begin(), _neighbors.end(), Id));
} /* End of 'Polygon::deleteNeighbor' function */

/* Getting weight to another vertex function */
float Polygon::getWeight(const Polygon &Poly) const
{
  return 1.0f;
} /* End of 'Polygon::getWeight' function */

/* Checking valid of merging with triangle */
bool Polygon::isValidToMerge(const Polygon &Tri)
{
  Polygon newPol(*this);
  sup::Vecf newVert, ver1, ver2;
  //чекнуть что триуголник
  if (Tri._vertices.size() != 3)
    return false;

  //чекнуть что при слиянии не теряет выпуклость

  if (containsSegmentAsEdge(Segment(Tri._vertices[0], Tri._vertices[1])))
    newVert = Tri._vertices[2], ver1 = Tri._vertices[0], ver2 = Tri._vertices[1];
  else if (containsSegmentAsEdge(Segment(Tri._vertices[1], Tri._vertices[2])))
    newVert = Tri._vertices[0], ver1 = Tri._vertices[1], ver2 = Tri._vertices[2];
  else if (containsSegmentAsEdge(Segment(Tri._vertices[2], Tri._vertices[0])))
    newVert = Tri._vertices[1], ver1 = Tri._vertices[2], ver2 = Tri._vertices[0];

  auto it1 = std::find(newPol._vertices.begin(), newPol._vertices.end(), ver1);
  auto it2 = std::find(newPol._vertices.begin(), newPol._vertices.end(), ver2);

  auto it = it1 > it2 ? it1 : it2;
  if (it == newPol._vertices.end() - 1)
    it = newPol._vertices.begin();

  newPol._vertices.insert(it, newVert);

  //My convex
  return newPol.isConvex();
}  /* End of 'Polygon::isValidToMerge' function */

/* Check convex */
bool Polygon::isConvex(void)
{
  sup::Vecf first = _vertices[0];
  int sign = (_vertices[1] - _vertices[0]).getSin(_vertices[2] - _vertices[1]) > 0 ? 1 : -1;
  int prev_sign = sign;

  for (size_t i = 1; i < _vertices.size() - 1; i++)
  {
    prev_sign = sign;
    sup::Vecf second = _vertices[i];
    sup::Vecf third = _vertices[i + 1];
    sign = (second - first).getSin(third - second) > 0 ? 1 : -1;
    if (sign != prev_sign)
      return false;
    first = second;
  }

  prev_sign = sign;
  sign = (_vertices[_vertices.size() - 1] - _vertices[_vertices.size() - 2]).getSin(_vertices[0] - _vertices[_vertices.size() - 1]) > 0 ? 1 : -1;

  if (sign != prev_sign)
    return false;

  if ((_vertices[0] - _vertices[_vertices.size() - 1]).getSin(_vertices[1] - _vertices[0]) * sign < 0)
    return false;
  return true;
} /* End of 'isConvex' function */

/* Merging polygon with triangle */
void Polygon::mergeWithTriangle(const Polygon &Tri)
{
  //слить по каефу
  sup::Vecf newVert, ver1, ver2;
  
  if (containsSegmentAsEdge(Segment(Tri._vertices[0], Tri._vertices[1])))
    newVert = Tri._vertices[2], ver1 = Tri._vertices[0], ver2 = Tri._vertices[1];
  else if (containsSegmentAsEdge(Segment(Tri._vertices[1], Tri._vertices[2])))
    newVert = Tri._vertices[0], ver1 = Tri._vertices[1], ver2 = Tri._vertices[2];
  else if (containsSegmentAsEdge(Segment(Tri._vertices[2], Tri._vertices[0])))
    newVert = Tri._vertices[1], ver1 = Tri._vertices[2], ver2 = Tri._vertices[0];

  auto it1 = std::find(_vertices.begin(), _vertices.end(), ver1);
  auto it2 = std::find(_vertices.begin(), _vertices.end(), ver2);

  auto it = it1 > it2 ? it1 : it2;
  if (it == _vertices.end() - 1)
    it = _vertices.begin();

  _vertices.insert(it, newVert);
} /* End if 'Polygon::mergeWithTriangle' function */

/* Merge function for graph */
void Polygon::merge(const Polygon &Poly)
{
  mergeWithTriangle(Poly);
} /* End of 'Polygon::merge' function */

/* Does the point inside polygon */
bool Polygon::hasInside(const sup::Vecf &Point)
{
  std::vector<sup::Vecf> curVerts = _vertices;
  sup::Vecf vert1 = curVerts[curVerts.size() - 1];
  sup::Vecf start = vert1;
  curVerts.pop_back();

  sup::Vecf vert2 = curVerts[curVerts.size() - 1];
  curVerts.pop_back();

  sup::Vecf toSq = vert1 - Point;
  sup::Vecf side = vert2 - vert1;

  int sign;
  sign = side.getSin(toSq) > 0 ? 1 : -1;

  vert1 = vert2;
  //check that squad is into polygon
  bool is_lay_in = true;
  while (curVerts.size() != 0)
  {
    vert2 = curVerts[curVerts.size() - 1];
    curVerts.pop_back();

    toSq = vert1 - Point;
    side = vert2 - vert1;

    if (side.getSin(toSq) * sign < 0)
      return false;
    vert1 = vert2;
  }

  toSq = start - Point;
  side = start - vert1;

  if (side.getSin(toSq) * sign < 0)
    return false;
  return true;
} /* End of 'Polygon::hasInside' function */

/* END OF 'polygon.cpp' FILE */
