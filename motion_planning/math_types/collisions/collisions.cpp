/*
 * Motion planning problem project.
 *
 * FILE: collisions.cpp
 * AUTHORS:
 *   Denisov Pavel
 * LAST UPDATE: 22.05.2018
 * NOTE: collisions namespace implementation file
 */

#include "collisions.h"

using namespace collision;

/* Collide segment with segment function */
bool collision::isCollideSegmentSegment(const Segment &Seg1, const Segment &Seg2, CollideRes &Res)
{
  sup::Vecf dir1 = Seg1._sPoint - Seg1._fPoint;
  sup::Vecf dir2 = Seg2._sPoint - Seg2._fPoint;

  float a1 = -dir1._coords[1];
  float b1 = dir1._coords[0];
  float d1 = -(a1 * Seg1._fPoint._coords[0] + b1 * Seg1._fPoint._coords[1]);

  float a2 = -dir2._coords[1];
  float b2 = dir2._coords[0];
  float d2 = -(a2 * Seg2._fPoint._coords[0] + b2 * Seg2._fPoint._coords[1]);

  float seg1_line2_start = a2 * Seg1._fPoint._coords[0] + b2 * Seg1._fPoint._coords[1] + d2;
  float seg1_line2_end = a2 * Seg1._sPoint._coords[0] + b2 * Seg1._sPoint._coords[1] + d2;

  float seg2_line1_start = a1 * Seg2._fPoint._coords[0] + b1 * Seg2._fPoint._coords[1] + d1;
  float seg2_line1_end = a1 * Seg2._sPoint._coords[0] + b1 * Seg2._sPoint._coords[1] + d1;

  if (seg1_line2_start * seg1_line2_end >= 0 || seg2_line1_start * seg2_line1_end >= 0)
    return false;

  float u = seg1_line2_start / (seg1_line2_start - seg1_line2_end);

  Res._intersectPoints.clear();
  Res._intersectPoints.push_back(Seg1._fPoint + dir1 * u);

  return true;
} /* End of 'collision::isCollideSegmentSegment' function */

static void s_moveEndPoints(sup::Vecf &FP, sup::Vecf &SP)
{
  float dist = (FP - SP).length();
  sup::Vecf norm = (FP - SP).getNormalized();

  FP += norm * (-0.01f * dist);
  SP += norm * 0.01f * dist;
} /* End of 's_moveEndPoints' function */

/* Collide segment with polygon function */
bool collision::isCollideSegmentPolygon(const Segment &Seg, const Polygon &Poly, CollideRes &Res)
{
  Segment seg = Seg;
  s_moveEndPoints(seg._sPoint, seg._fPoint);

  //get sides of polygon
  std::vector<Segment> sidePoly;
  int size = Poly._vertices.size();
  for (int i = 0; i < size - 1; i++)
    sidePoly.push_back(Segment(Poly._vertices[i], Poly._vertices[i + 1]));
  sidePoly.push_back(Segment(Poly._vertices[size - 1], Poly._vertices[0]));

  Res._intersectPoints.clear();
  CollideRes tmp;
  bool isIntersect = false;
  for (auto curPolySide : sidePoly)
    if (isCollideSegmentSegment(seg, curPolySide, tmp))
    {
      isIntersect = true;
      Res._intersectPoints.push_back(tmp._intersectPoints[0]);
    }

  return isIntersect;
} /* End of 'collision::isCollideSegmentPolygon' function */

/* Find the collision point between two lines */
CollideRes collision::collideSegmentSegmentAsLines(const Segment &Seg1, const Segment &Seg2)
{
  sup::Line Line1 = sup::getLineEq(Seg1._fPoint, Seg1._sPoint);
  sup::Line Line2 = sup::getLineEq(Seg2._fPoint, Seg2._sPoint);

  float a1, b1 = -1.0f, c1;
  float a2, b2 = -1.0f, c2;

  if (Line1._k == INFINITY || Line1._k == -INFINITY)
  {
    a1 = 1.0f;
    b1 = 0.0f;
    c1 = -Seg1._fPoint._coords[0];
  }
  else
  {
    a1 = Line1._k;
    c1 = Line1._b;
  }

  if (Line2._k == INFINITY || Line2._k == -INFINITY)
  {
    a2 = 1.0f;
    b2 = 0.0f;
    c2 = -Seg2._fPoint._coords[0];
  }
  else
  {
    a2 = Line2._k;
    c2 = Line2._b;
  }

  float x = -(c1 * b2 - c2 * b1) / (a1 * b2 - a2 * b1);
  float y = -(a1 * c2 - a2 * c1) / (a1 * b2 - a2 * b1);

  CollideRes res;
  res._intersectPoints.push_back({x, y});

  return res;
} /* End of 'collision::collideSegmentSegmentAsLines' function */

/* END OF 'collisions.cpp' FILE */
