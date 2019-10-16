/*
 * Motion planning problem project.
 *
 * FILE: vertex.h
 * AUTHORS:
 *   Denisov Pavel
 *   Tunikov Dmitrii
 * LAST UPDATE: 19.05.2018
 * NOTE: vectex class declaration file
 */

#pragma once
#include <vector>
#include "../../../support/support.h"

/* Vertex class */
class Vertex
{
public:
  /* Class consturctor */
  Vertex(const sup::Vecf &Pos, float rad = 0, const int ID = 0) : _id(ID), _pos(Pos), _rad(rad)
  {
  } /* End of constructor */

  Vertex() {}
  /* Getting id function */
  int getId(void) const
  {
    return _id;
  } /* End of 'getId' function */

  /* Setting id function */
  void setId(const int ID)
  {
    _id = ID;
  } /* End of 'setId' function */

  /* Getting position function */
  sup::Vecf getPos(void) const
  {
    return _pos;
  } /* End of 'getPos' function */

  /* Setting position function */
  void setPos(const sup::Vecf &Pos)
  {
    _pos = Pos;
  } /* End of 'setPos' function */

  /* Getting weight to another vertex function */
  float getWeight(const Vertex &v) const
  {
    return (_pos - v._pos).length();
  } /* End of 'getWeight' function */

  /* Getting neighbors function */
  std::vector<int> getNeighbors(void) const
  {
    return _neighbors;
  } /* End of 'getNeighbors' function */

  /* Adding neighbor function */
  void addNeighbor(const int ID)
  {
    _neighbors.push_back(ID);
  } /* End of 'addNeighbor' function */

  float getRad() const
  {
    return _rad;
  }

  void setRad(float r) 
  {
    _rad = r;
  }

private:
  sup::Vecf _pos;
  std::vector<int> _neighbors;
  float _rad;
  int _id;
}; /* End of 'Vertex' class */

/* END OF 'vertex.h' FILE */
