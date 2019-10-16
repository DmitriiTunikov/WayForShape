/*
 * Motion planning problem project.
 *
 * FILE: main.cpp
 * AUTHORS:
 *   Denisov Pavel,
 *   Tunikov Dmitrii,
 *   Federov Dmitrii
 * LAST UPDATE: 04.05.2018
 * NOTE: main project file
 */

#include <iostream>
#include "graph\graph.h"
#include "motion_planning\math_types\vertex\vertex.h"
#include "support\support.h"
#include "support\vec2d.h"
#include "graph\dijkstra\dijkstra.h"
#include "anim\anim.h"

int main(int argc, char **argv)
{
  //Graph<Vertex> gr;
  //gr.addElem(Vertex(sup::Vecf(0.1f, 0.1f)));
  //gr.addElem(Vertex(sup::Vecf(0.2f, 0.3f)));
  //gr.addElem(Vertex(sup::Vecf(0.1f, 0.7f)));
  //gr.addElem(Vertex(sup::Vecf(0.6f, 0.8f)));
  //gr.addElem(Vertex(sup::Vecf(0.9f, 0.2f)));
  //
  //gr.addLink(0, 1);
  //gr.addLink(0, 2);
  //gr.addLink(0, 3);
  //gr.addLink(1, 2);
  //gr.addLink(3, 4);
  //gr.addLink(2, 3);
  //gr.addLink(1, 4);
  //
  //std::vector<Vertex> came_from = Dijkstra::GetAllWays(gr.getElements()[0], gr, 0.1f);

  anim::Anim &anim = anim::Anim::getInstance();
  anim.init(argc, argv, 1200, 700);
  anim.run();

  return 0;
} /* End of 'main' function */

/* END OF 'main.cpp' FILE */
