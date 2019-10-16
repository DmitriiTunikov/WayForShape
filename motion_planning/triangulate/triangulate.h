/*
 * Motion planning problem project.
 *
 * FILE: triangulate.h
 * AUTHORS:
 *   Denisov Pavel,
 *   Federov Dmitrii
 * LAST UPDATE: 22.05.2018
 * NOTE: triangulating space class declaration file
 */

#pragma once
#include <vector>
#include "../../support/support.h"
#include "../math_types/polygon/polygon.h"
#include "../math_types/segment/segment.h"

/* Triangulating class */
class Triangulate
{
public:
  /* Default class constructor */
  Triangulate(void) {};

  /* Building trinagulating space function */
  void build(const std::vector<sup::Vecf> &Verts, const std::vector<Polygon> &polygons, bool main);

  /* Clear function */
  void clear(void);

private:
  /* Finding new vertex to add to triangle */
  bool mate(const Segment &e, std::vector<sup::Vecf> &points, sup::Vecf &conj);

  /* Updating the frontier array */
  void updateFrontier(std::vector<Segment> &frontier, const sup::Vecf &a, const sup::Vecf &b);

  /* Finding new edge to add in frontier */
  Segment hullEdge(std::vector<sup::Vecf> &points);

  /* IsValid */
  bool isValid(const Segment &Seg);

  /* Deleting triangles, which located in barricades */
  void deleteInnerTriangles(void);

  /* Adding new obstacle segment in triangulation array function */
  void addSegmet(const Segment &seg);

  /* Adding new obstacle in triangulation array function */
  void addPolygon(const Polygon &seg);

  /* Adding new obstacles in triangulation array function */
  void addPolygons(const std::vector<Polygon> &poly);

public:
  /* Triangles */
  std::vector<Polygon> _triangles;
}; /* End of 'Triangulate' class */

/* END OF 'triangulate.h' FILE */
