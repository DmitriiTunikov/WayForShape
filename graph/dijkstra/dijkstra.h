/*
* Motion planning problem project.
*
* FILE: vertex.h
* AUTHORS:
*   Tunikov Dmitrii
* LAST UPDATE: 19.05.2018
* NOTE: vectex class declaration file
*/
#pragma once
#include "../../support/support.h"
#include <vector>
#include "../../motion_planning/motion_planning.h"
#include "../../graph/graph.h"

class Dijkstra
{
public:
  static std::vector<sup::Vecf> GetGoalWay(const Vertex& start, const Vertex& goal, const std::vector<Vertex> &came_from);
  static bool GetAllWays(const Vertex& start, const Vertex& goal, Graph<Vertex>& graph, float r,
    const std::vector<Polygon>& polys, std::vector<Vertex> &came_from);
};