/*
 * Motion planning problem project.
 *
 * FILE: distances.h
 * AUTHORS:
 *   Denisov Pavel
 * LAST UPDATE: 22.05.2018
 * NOTE: distances namespace declaration file
 */

#include "../../../support/support.h"
#include "../segment/segment.h"
#include "../vertex/vertex.h"
#include "../polygon/polygon.h"

/* Distances namespace */
namespace distances
{
  /* Distance result type */
  struct DistRes
  {
    float _dist;
    sup::Vecf _pos;

    /* Default constructor */
    DistRes(void) = default;

    /* Constructor */
    DistRes(const float Dist, const sup::Vecf &Pos) : _dist(Dist), _pos(Pos)
    {
    } /* End of constructor */
  }; /* End of 'DistRes' structure */

  /* Finding the distance between point and segment */
  DistRes distBetweenPointAndSegment(const sup::Vecf &Point, const Segment &Seg);

  /* Finding the distance between point and segment */
  DistRes distBetweenPointAndSegment(const sup::Vecf &Point, const sup::Vecf &FPoint, const sup::Vecf &SPoint);

  /* Find shortest distance from point to certain polygon */
  DistRes distBetweenPointAndPolygon(const sup::Vecf &Point, const Polygon &Poly);

  //get shortes dist between polygons and point
  DistRes distBetweenPointAndPolygons(const sup::Vecf &Point, const std::vector<Polygon> &Poly);
} /* end of 'distances' namespace */

/* END OF 'distances.h' FILE */
