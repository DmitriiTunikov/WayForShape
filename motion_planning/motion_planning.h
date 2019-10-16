/*
 * Motion planning problem project.
 *
 * FILE: motion_planning.h
 * AUTHORS:
 *   Denisov Pavel
 * LAST UPDATE: 19.05.2018
 * NOTE: motion planning class declaration file
 */

#pragma once
#include "../graph/graph.h"
#include "math_types\vertex\vertex.h"
#include "math_types\polygon\polygon.h"
#include "triangulate\triangulate.h"

/* Motion planning algorithm class */
class MotionPlanning
{
public:
  /* Type definition */
  using vert_vec = std::vector<sup::Vecf>;

public:
  /* Getting instance function */
  static MotionPlanning & getInstance(void);

  /* Setting start point function */
  void setStartPoint(const sup::Vecf &SP);

  /* Setting finish point function */
  void setFinishPoint(const sup::Vecf &FP);

  /* Adding new polygon function */
  void addPolygon(const Polygon &Poly);

  /* Adding vertex to last polygon function */
  void addVertexToLastPolygon(const sup::Vecf &Vert, const int Index);

  /* Drawing function */
  void draw(void) const;

  /* Setting _isReady variable function */
  void setReady(const bool Flag);

  /* Build path function */
  void build(void);

  /* Does the segment intesrect polygons */
  bool isSegmentIntersectPolygons(const Segment &Seg);

  /* Are two vertices neighbours */
  bool isSegmentConnectNotNeighbours(const Segment &Seg);

  /* Is this a triangle polygon */
  bool isTrianglePolygon(const sup::Vecf &Point1, const sup::Vecf &Point2, const sup::Vecf &Point3);

  /* Update adjacency graph */
  void updateAdjacencyGraph(void);

  /* Clear function */
  void clear(void);

  /* Class destructor */
  ~MotionPlanning(void);

  /* Saving current scene function */
  void save(const std::string &FileName) const;

  /* Loading current scene function */
  void load(const std::string &FileName);

private:
  /* Default class constructor */
  MotionPlanning(void);

  /* Initing _adjGraph */
  void initAdjGraph(void);

  /* Get adjective verts graph */
  void initAdjVertGraph(Vertex start, Vertex goal);

  /* Check containing segment in polys */
  std::vector<int> getPolysIdBySegment(const Segment &Seg, const std::vector<Polygon>& polys);

  /* Checking if the segment is edge of polygon */
  bool containsSegmentAsEdgeOfPolys(const Segment &Seg, const std::vector<Polygon>& polys);

private:
  /* Adjacency graph */
  Graph<Polygon> _adjGraph;
  /* Adjacency vertex graph */
  Graph<Vertex> _adjVertGraph;

  /* Came_from vector of vertices for path */
  std::vector<Vertex> _cameFrom;
  /* Current radius of the "agent" */
  float _rad;
  /* Current path */
  std::vector<sup::Vecf> _path;

  /* Triangulate class */
  Triangulate _triang;

  /* Polygon array */
  std::vector<Polygon> _polygons;
  /* Start and finish points */
  sup::Vecf _startP, _finishP;

  /* Is builded */
  bool _isReady;
  bool _isBuilded;
}; /* End of 'MotionPlanning' class */

/* END OF 'motion_planning.h' FILE */
