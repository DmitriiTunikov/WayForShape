/*
 * Motion planning problem project.
 *
 * FILE: build.cpp
 * AUTHORS:
 *   Denisov Pavel,
 *   Federov Dmitrii
 * LAST UPDATE: 20.05.2018
 * NOTE: build path implementation file
 */

#include <algorithm>
#include <queue>

#include "../anim/render/render.h"
#include "../graph/dijkstra/dijkstra.h"
#include "math_types\collisions\collisions.h"
#include "math_types\distances\distances.h"
#include "motion_planning.h"

/* Getting possible radius for certain segment */
static float getPossibleRad(Vertex v1, Vertex v2, const std::vector<Polygon> polys)
{
  float possible_rad_v1 = distances::distBetweenPointAndPolygons(v1.getPos(), polys)._dist;
  float possible_rad_v2 = distances::distBetweenPointAndPolygons(v2.getPos(), polys)._dist;

  float possible_rad = possible_rad_v1 < possible_rad_v2 ? possible_rad_v1 : possible_rad_v2;
  return possible_rad;
} /* End of 'getPossibleRad' function */

/* Checking if the segment is edge of polygon */
bool MotionPlanning::containsSegmentAsEdgeOfPolys(const Segment &Seg, const std::vector<Polygon>& polys)
{
  for (auto pol : polys)
    if (pol.containsSegmentAsEdge(Seg))
      return true;
  return false;
} /* End of 'MotionPlanning::containsSegmentAsEdgeOfPolys' function */

/* Check containing segment in polys */
std::vector<int> MotionPlanning::getPolysIdBySegment(const Segment &Seg, const std::vector<Polygon>& polys)
{
  std::vector<int> res;
  for (auto pol : polys)
    if (pol.containsSegmentAsEdge(Seg))
      res.push_back(pol.getId());
  return res;
} /* End of 'MotionPlanning::getPolysIdBySegment' function */

/* Initing graph of adjacency vertices */
void MotionPlanning::initAdjVertGraph(Vertex start, Vertex goal)
{
  int startId = 0;
  for (auto pol : _adjGraph.getElements())
    if (pol.hasInside(start.getPos()))
    {
      startId = pol.getId();
      break;
    }

  std::vector<std::vector<int>> res_pols;
  std::vector<int> start_pol_id(0);
  start_pol_id.push_back(startId);
  start_pol_id.push_back(startId);
  res_pols.push_back(start_pol_id);

  //add start vert
  start.setRad(distances::distBetweenPointAndPolygons(start.getPos(), _polygons)._dist);
  _adjVertGraph.addElem(start);

  //add polygons segments centers verts
  for (auto pol : _adjGraph.getElements())
  {
    for (size_t i = 0; i < pol._vertices.size() - 1; i++)
    {
      Vertex v1 = pol._vertices[i];
      Vertex v2 = pol._vertices[i + 1];

      if (containsSegmentAsEdgeOfPolys(Segment(v1.getPos(), v2.getPos()), _polygons))
        continue;

      res_pols.push_back(getPolysIdBySegment(Segment(v1.getPos(), v2.getPos()), _adjGraph.getElements()));
      sup::Vecf pos = Segment(v1.getPos(), v2.getPos()).getCenter();
      //get possible rad for cur vert
      float possible_rad = getPossibleRad(pol._vertices[i], pol._vertices[i + 1], _polygons);
      _adjVertGraph.addElem(Vertex(pos, possible_rad));
    }

    //add vert on last segment
    Vertex v1 = pol._vertices[pol._vertices.size() - 1];
    Vertex v2 = pol._vertices[0];
    if (containsSegmentAsEdgeOfPolys(Segment(v1.getPos(), v2.getPos()), _polygons))
      continue;
    sup::Vecf pos = Segment(v1.getPos(), v2.getPos()).getCenter();
    res_pols.push_back(getPolysIdBySegment(Segment(v1.getPos(), v2.getPos()), _adjGraph.getElements()));
    float possible_rad = getPossibleRad(v1, v2, _polygons);
    _adjVertGraph.addElem(Vertex(pos, possible_rad));
  }

  //add last vert
  int goalId = 0;
  for (auto pol : _adjGraph.getElements())
    if (pol.hasInside(goal.getPos()))
    {
      goalId = pol.getId();
      break;
    }

  std::vector<int> goal_pol_id(0);
  goal_pol_id.push_back(goalId);
  goal_pol_id.push_back(goalId);
  res_pols.push_back(goal_pol_id);

  goal.setRad(distances::distBetweenPointAndPolygons(goal.getPos(), _polygons)._dist);
  _adjVertGraph.addElem(goal);

  std::vector<Vertex> verts = _adjVertGraph.getElements();

  //add neighbours for first vertex
  for (size_t i = 1; i < verts.size(); i++)
  {
    std::vector<int> curNeigbs = res_pols[i];
    if (curNeigbs[0] == startId || (curNeigbs.size() == 2 && curNeigbs[1] == startId))
      _adjVertGraph.addLink(0, i);
  }

  // Do for all
  for (size_t i = 1; i < verts.size(); i++)
  {
    std::vector<int> i_neibgs = res_pols[i];
    for (auto pol_id : i_neibgs)
      for (size_t j = i + 1; j < verts.size(); j++)
      {
        std::vector<int> j_neibgs = res_pols[j];
        if (j_neibgs[0] == pol_id || (j_neibgs.size() == 2 && j_neibgs[1] == pol_id))
          _adjVertGraph.addLink(i, j);
      }
  }

  //add neighbours for last vertex
  for (size_t i = 1; i < verts.size(); i++)
  {
    std::vector<int> curNeigbs = res_pols[i];
    if (curNeigbs[0] == goalId || (curNeigbs.size() == 2 && curNeigbs[1] == goalId))
      _adjVertGraph.addLink(_adjVertGraph.getElements().size() - 1, i);
  }
} /* End of 'MotionPlanning::initAdjVertGraph' function */

/* Does the segment intesrect polygons */
bool MotionPlanning::isSegmentIntersectPolygons(const Segment &Seg)
{
  collision::CollideRes res;
  for (auto poly : _polygons)
    if (collision::isCollideSegmentPolygon(Seg, poly, res))
      return true;
  return false;
} /* End of 'MotionPlanning::isSegmentIntersectPolygons' function */

/* Are two vertices neighbours */
bool MotionPlanning::isSegmentConnectNotNeighbours(const Segment &Seg)
{
  for (auto poly : _polygons)
    if (poly.contains(Seg._fPoint) && poly.contains(Seg._sPoint) && !poly.containsSegmentAsEdge(Seg))
      return true;
  return false;
} /* End of 'MotionPlanning::isSegmentConnectNotNeighbours' function */

/* Is this a triangle polygon */
bool MotionPlanning::isTrianglePolygon(const sup::Vecf &Point1, const sup::Vecf &Point2, const sup::Vecf &Point3)
{
  for (auto poly : _polygons)
    if (poly._vertices.size() == 3 && poly.contains(Point1) && poly.contains(Point2) && poly.contains(Point3))
      return true;
  return false;
} /* End of 'MotionPlanning::isTrianglePolygon' function */

/* Initing _adjGraph */
void MotionPlanning::initAdjGraph(void)
{
  for (auto iTr : _triang._triangles)
    _adjGraph.addElem(iTr);

  for (auto iTr = _triang._triangles.begin(); iTr != _triang._triangles.end() - 1; iTr++)
    for (auto jTr = iTr; jTr != _triang._triangles.end(); jTr++)
    {
      if (jTr == iTr)
        continue;

      if (iTr->isAdjacencyToTriangle(*jTr))
        _adjGraph.addLink(iTr->getId(), jTr->getId());
    }
} /* End of 'MotionPlanning::initAdjGraph' function */

/* Make adjacency graph */
void MotionPlanning::updateAdjacencyGraph(void)
{
  for (size_t i = 0; i < _adjGraph.getElements().size(); i++)
  {
    std::vector<int> neigh = _adjGraph.getElements()[i].getNeighbors();

    for (size_t j = 0; j < neigh.size(); j++)
    {
      int curId = _adjGraph.getElements()[i].getId();
      Polygon tmp = sup::findElemInArrayById(_adjGraph.getElements(), neigh[j]);
      if (_adjGraph.getElements()[i].isValidToMerge(tmp))
      {
        _adjGraph.pullEdge(_adjGraph.getElements()[i].getId(), neigh[j]);
        j = -1;
      }

      i = sup::findElemIndexInArrayById(_adjGraph.getElements(), curId);
      neigh = _adjGraph.getElements()[i].getNeighbors();
    }
  }
} /* End of 'MotionPlanning::updateAdjacencyGraph' function*/

/* Build path function */
void MotionPlanning::build(void)
{
  anim::Render &rnd = anim::Render::getInstance();

  // Init wall polygons
  Polygon walls;
  walls.addVertex({ 10, 10 });
  walls.addVertex({ 10, rnd.getHeight() - 10 });
  walls.addVertex({ rnd.getWidth() - 10, rnd.getHeight() - 10 });
  walls.addVertex({ rnd.getWidth() - 10, 10 });

  addPolygon(walls);

  // Saving - for debug
  save("output_new.mp");

  // Triangulate
  std::vector<sup::Vecf> verts;
  for (auto poly : _polygons)
    verts.insert(verts.end(), poly._vertices.begin(), poly._vertices.end());
  _triang.build(verts, _polygons, true);

  // Build graph
  initAdjGraph();
  updateAdjacencyGraph();

  // Build path
  initAdjVertGraph(_startP, _finishP);
  bool res = Dijkstra::GetAllWays(_adjVertGraph.getElements()[0], _adjVertGraph.getElements()[_adjVertGraph.getElements().size() - 1], 
    _adjVertGraph, _rad, _polygons, _cameFrom);
  if (res)
    _path = Dijkstra::GetGoalWay(_adjVertGraph.getElements()[0], _adjVertGraph.getElements()[_adjVertGraph.getElements().size() - 1], _cameFrom);

  _isBuilded = true;
} /* End of 'MotionPlanning::build' function */

/* END OF 'build.cpp' FILE */
