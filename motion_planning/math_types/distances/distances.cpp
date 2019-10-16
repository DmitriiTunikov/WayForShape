/*
 * Motion planning problem project.
 *
 * FILE: distances.cpp
 * AUTHORS:
 *   Denisov Pavel
 *   Tunikov Dmitrii
 * LAST UPDATE: 22.05.2018
 * NOTE: distances namespace implementation file
 */

#include <numeric>
#include "distances.h"

using namespace distances;

// get shortes dist between polygons and point
DistRes distances::distBetweenPointAndPolygons(const sup::Vecf &Point, const std::vector<Polygon> &Polys)
{
  float minDist = std::numeric_limits<float>::max();
  DistRes res;
  
  for (size_t i = 0; i < Polys.size(); i++)
  {
    if (Polys[i].contains(Point))
      continue;

    DistRes cur_dist = distBetweenPointAndPolygon(Point, Polys[i]);
    if (cur_dist._dist < minDist)
    {
      res = cur_dist;
      minDist = res._dist;
    }
  }

  return res;
}

/* Finding the distance between point and segment */
DistRes distances::distBetweenPointAndSegment(const sup::Vecf &Point, const Segment &Seg)
{
  return distBetweenPointAndSegment(Point, Seg._fPoint, Seg._sPoint);
} /* End of 'distances::distBetweenPointAndSegment' function */

/* Finding the distance between point and segment */
DistRes distances::distBetweenPointAndSegment(const sup::Vecf &Point, const sup::Vecf &FPoint, const sup::Vecf &SPoint)
{
  float dist1 = (Point - FPoint).length();
  float dist2 = (Point - SPoint).length();
  float dist3 = INFINITY;
  float segLen = (FPoint - SPoint).length();

  if (dist1 * dist1 - dist2 * dist2 + segLen * segLen > 0 && -dist1 * dist1 + dist2 * dist2 + segLen * segLen > 0)
  {
    sup::Line line = getLineEq(FPoint, SPoint);
    float a, b = -1.0f, c;

    if (line._k == INFINITY || line._k == -INFINITY)
    {
      a = 1.0f;
      b = 0.0f;
      c = -FPoint._coords[0];
    }
    else
    {
      a = line._k;
      c = line._b;
    }

    dist3 = fabs(Point._coords[0] * a + Point._coords[1] * b + c) / sqrt(a * a + b * b);
    float x = 0, y = 0;

    try {
      x = (b * (b * Point._coords[0] - a * Point._coords[1]) - a * c) / (a * a + b * b);
      y = (a * (-b * Point._coords[0] + a * Point._coords[1]) - b * c) / (a * a + b * b);
    }
    catch (std::exception e) {
      std::cout << e.what();
    }

    return DistRes(dist3, { x, y });
  }

  if (dist1 < dist2)
    return DistRes(dist1, FPoint);
  return DistRes(dist2, SPoint);
} /* End of 'distances::distBetweenPointAndSegment' function */

/* Find shortest distance from point to certain polygon */
DistRes distances::distBetweenPointAndPolygon(const sup::Vecf &Point, const Polygon &Poly)
{
  std::vector<DistRes> dists;
  std::vector<sup::Vecf> verts = Poly._vertices;

  for (size_t i = 0; i < verts.size() - 1; i++)
    dists.push_back(distBetweenPointAndSegment(Point, verts[i], verts[i + 1]));
  dists.push_back(distBetweenPointAndSegment(Point, verts[verts.size() - 1], verts[0]));

  DistRes res = dists[0];
  for (size_t i = 1; i < dists.size(); i++)
    if (res._dist > dists[i]._dist)
      res = dists[i];

  return res;
} /* End of 'distances::distBetweenPointAndPolygon' function */

/* END OF 'distances.cpp' FILE */
