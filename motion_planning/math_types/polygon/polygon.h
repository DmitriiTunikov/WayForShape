/*
 * Motion planning problem project.
 *
 * FILE: polygon.h
 * AUTHORS:
 *   Denisov Pavel
 * LAST UPDATE: 19.05.2018
 * NOTE: polygon class declaration file
 */

#pragma once
#include <vector>
#include "../../../support/support.h"
#include "../../../anim/render/drawable/drawable.h"
#include "../segment/segment.h"

/* Polygon class */
class Polygon : public Drawable
{
public:
  /* Array of vertices */
  std::vector<sup::Vecf> _vertices;

  /* Adding new vertex function */
  void addVertex(const sup::Vecf &Vert);

  /* Adding new vertex operator */
  void operator<<(const sup::Vecf &Vert);

  /* Getting id function */
  int getId(void) const;

  /* Virtual draw function */
  virtual void draw(const bool isSolid = false) const;

  /* Equal operator */
  bool operator==(const Polygon &Poly) const;

  /* Non-equal operator */
  bool operator!=(const Polygon &Poly) const;

  /* Setting id function */
  void setId(const int ID);

  /* Contains the vertex function */
  int contains(const sup::Vecf &Vertex) const;

  /* Contains the segment as edge function */
  bool containsSegmentAsEdge(const Segment &Seg) const;

  /* Is adjacency with another polygon */
  int isAdjacencyToTriangle(const Polygon &Tri) const;

  /* Getting neighbors function */
  std::vector<int> getNeighbors(void) const;

  /* Adding neighbor function */
  void addNeighbor(const int ID);

  /* Deleting neighbors function */
  void deleteNeighbor(const int Id);

  /* Getting weight to another vertex function */
  float getWeight(const Polygon &Poly) const;

  /* Check convex */
  bool isConvex(void);

  /* Checking valid of merging with triangle */
  bool isValidToMerge(const Polygon &Tri);

  /* Merging polygon with triangle */
  void mergeWithTriangle(const Polygon &Tri);

  /* Merge function for graph */
  void merge(const Polygon &Poly);

  /* Does the point inside polygon */
  bool hasInside(const sup::Vecf &Point);

private:
  /* Polygon id */
  int _id;
  /* Array of neighbours */
  std::vector<int> _neighbors;
}; /* End of 'Polygon' class */

/* END OF 'polygon.h' FILE */
