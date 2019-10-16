/*
* Motion planning problem project.
*
* FILE: vertex.h
* AUTHORS:
*   Tunikov Dmitrii
* LAST UPDATE: 19.05.2018
* NOTE: vectex class declaration file
*/
#include <queue>
#include <cmath>
#include "dijkstra.h"
#include "../../support/support.h"
#include "../../motion_planning/motion_planning.h"
#include "../../queue/priority_queue.h"

float heuristic(const sup::Vecf &a, const sup::Vecf &b)
{
  //Manhattan distance on a square grid
  return fabs(a._coords[0] - b._coords[0]) + fabs(a._coords[1]- b._coords[1]);
}

std::vector<sup::Vecf> Dijkstra::GetGoalWay(const Vertex& start, const Vertex& goal, const std::vector<Vertex> &came_from)
{
  Vertex current = goal;
  std::vector<sup::Vecf> path;
  path.push_back(current.getPos());

  while (current.getPos() != start.getPos())
  {
    current = came_from[current.getId()];
    path.push_back(current.getPos());
  }
  //path.push_back(start.getPos());
  std::reverse(path.begin(), path.end());
  return path;
} /* End of 'GetGoalWay' function */

bool Dijkstra::GetAllWays(const Vertex& start, const Vertex& goal, Graph<Vertex>& graph, float r,
  const std::vector<Polygon>& polys, std::vector<Vertex> &came_from)
{
  //init vectors
  PriorityQueue<Vertex, float> frontier;
  frontier.push(start, 0);

  const std::vector<Vertex> graph_vert = graph.getElements();
  came_from = std::vector<Vertex>(graph_vert.size());
  std::vector<float> cost_so_far(graph_vert.size());
  
  came_from[start.getId()] = Vertex(sup::Vecf(-1, -1));
  cost_so_far[start.getId()] = 0;
  
  while (!frontier.empty()) 
  {
    Vertex current = frontier.pop();

    if (current.getPos() == goal.getPos())
      return true;
    //add neighbors to frontier 
    std::vector<int> neighbors = current.getNeighbors();
    for (auto next : neighbors)
    {
      //check Intersection circle with polygons
      if (graph_vert[next].getRad() < r)
        continue;
      //check cost of this move
      float new_cost = cost_so_far[current.getId()] + graph.getWeight(current.getId(), next);
      if (!cost_so_far[next] || new_cost < cost_so_far[next])
      {
        cost_so_far[next] = new_cost;
        float priority = new_cost + heuristic(current.getPos(), graph_vert[next].getPos());
        frontier.push(graph.getElements()[next], 1.0f / priority);
        came_from[next] = current;
      }
    }
  }
  return false;
} /* End of 'GetAllWays' function */
