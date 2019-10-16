/*
 * Motion planning problem project.
 *
 * FILE: support.h
 * AUTHORS:
 *   Denisov Pavel
 *   Tunikov Dmitrii
 * LAST UPDATE: 19.05.2018
 * NOTE: support class declaration file
 */

#pragma once
#include <cmath>
#include <iostream>
#include <vector>
#include "vec2d.h"

/* Support namespace */
namespace sup
{
  // Contants
  const float eps = 1e-4f;
  const float pi = 3.14159265359f;

  // Types
  using Vecf = Vec2D<float>;
  using Veci = Vec2D<int>;
  using Vecd = Vec2D<double>;

  /* Line structure */
  struct Line
  {
    float _k, _b;

    /* Constructor */
    Line(float k = 0, float b = 0) : _k(k), _b(b)
    {
    } /* End of constructor */
  }; /* End of 'Line' structure */

  /* Getting line structre by two points */
  inline Line getLineEq(const sup::Vecf &P1, const sup::Vecf &P2)
  {
    Line line;
    line._k = (P2._coords[1] - P1._coords[1]) / (P2._coords[0] - P1._coords[0]);
    line._b = P1._coords[1] - line._k * P1._coords[0];
    return line;
  } /* End of 'getLineEq' function */

  /* Getting f(x) by line structure */
  inline float getFX(const Line &Line, const float X)
  {
    return Line._k * X + Line._b;
  } /* End of 'getFX' function */

  /* Finding element in array by id */
  template <class T>
  inline T & findElemInArrayById(std::vector<T> &Arr, const int Id)
  {
    for (size_t i = 0; i < Arr.size(); i++)
      if (Arr[i].getId() == Id)
        return Arr[i];
  } /* End of 'findElemInArrayById' function */

  /* Finding element in array by id */
  template <class T>
  inline int findElemIndexInArrayById(std::vector<T> &Arr, const int Id)
  {
    for (size_t i = 0; i < Arr.size(); i++)
      if (Arr[i].getId() == Id)
        return i;
  } /* End of 'findElemIndexInArrayById' function */
} /* end of 'sup' namespace */

/* END OF 'support.h' FILE */
