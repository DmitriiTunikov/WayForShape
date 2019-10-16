/*
 * Motion planning problem project.
 *
 * FILE: collisions.h
 * AUTHORS:
 *   Denisov Pavel
 * LAST UPDATE: 22.05.2018
 * NOTE: collisions namespace declaration file
 */

#include "../segment/segment.h"
#include "../vertex/vertex.h"
#include "../polygon/polygon.h"
#include "../../../support/support.h"

/* Collision system namespace */
namespace collision
{
  /* Collision result type */
  struct CollideRes
  {
    std::vector<sup::Vecf> _intersectPoints;
  }; /* End of 'CollideRes' structure */

  /* Collide segment with segment function */
  bool isCollideSegmentSegment(const Segment &Seg1, const Segment &Seg2, CollideRes &Res);

  /* Collide segment with polygon function */
  bool isCollideSegmentPolygon(const Segment &Seg, const Polygon &Poly, CollideRes &Res);

  /* Find the collision point between two lines */
  CollideRes collideSegmentSegmentAsLines(const Segment &Seg1, const Segment &Seg2);
} /* end of 'collision' namespace */

/* END OF 'collisions.h' FILE */
