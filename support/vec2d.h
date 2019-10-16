/*
 * Motion planning problem project.
 *
 * FILE: vec2d.h
 * AUTHORS:
 *   Denisov Pavel
 *   Tunikov Dmitrii
 * LAST UPDATE: 19.05.2018
 * NOTE: two-dimenshional vector class declaration file
 */

#pragma once
#include <initializer_list>
#include <fstream>

#pragma once

/* Support namespace */
namespace sup
{
  /* Two-dimenshional class */
  template<typename Type>
  class Vec2D
  {
  public:
    /* Vector components */
    Type _coords[2];

    Vec2D() {}

    /* Constructor with all set params */
    template <typename DType>
    Vec2D(const std::initializer_list<DType> &Init)
    {
      size_t i = 0;
      for (auto it = Init.begin(); it != Init.end(); it++, i++)
        _coords[i] = (Type)*it;
    } /* End of 'Vector' function */

    Vec2D(Type x, Type y)
    {
      _coords[0] = x;
      _coords[1] = y;
    }

    bool operator==(Vec2D<Type> p) const
    {
      if (fabs(_coords[0] - p._coords[0]) > eps || fabs(_coords[1] - p._coords[1]) > eps)
        return false;
      return true;
    }

    bool operator!=(Vec2D<Type> p) const
    {
      return !(*this == p);
    }

    float distToVec(Vec2D<Type> p) const
    {
      return sqrt((_coords[0] - p._coords[0]) * (_coords[0] - p._coords[0]) + (_coords[1] - p._coords[1]) * (_coords[1] - p._coords[1]));
    }

    /* Additive operator */
    Vec2D<Type> operator+(const Vec2D<Type> &Vec) const
    {
      return Vec2D<Type>(_coords[0] + Vec._coords[0], _coords[1] + Vec._coords[1]);
    } /* End of 'operator+' operator */

    /* Additive operator with assignment */
    Vec2D<Type> & operator+=(const Vec2D<Type> &Vec)
    {
      _coords[0] += Vec._coords[0];
      _coords[1] += Vec._coords[1];

      return *this;
    } /* End of 'operator+=' operator */

    /* Subtraction operator */
    Vec2D<Type> operator-(const Vec2D<Type> &Vec) const
    {
      return Vec2D<Type>(_coords[0] - Vec._coords[0], _coords[1] - Vec._coords[1]);
    } /* End of 'operator-' operator */

    /* Subtraction operator with assignment */
    Vec2D<Type> & operator-=(const Vec2D<Type> &Vec)
    {
      _coords[0] -= Vec._coords[0];
      _coords[1] -= Vec._coords[1];

      return *this;
    } /* End of 'operator-=' operator */

    /* Divide by number operator */
    Vec2D<Type> operator/(const Type Num) const
    {
      return Vec2D<Type>(_coords[0] / Num, _coords[1] / Num);
    } /* End of 'operator/' operator */

    /* Divide by number operator with assignment */
    Vec2D<Type> operator/=(const Type Num)
    {
      _coords[0] /= Num;
      _coords[1] /= Num;

      return *this;
    } /* End of 'operator/=' operator */

    /* Multiply by number operator */
    Vec2D<Type> operator*(const Type Num) const
    {
      return Vec2D<Type>(_coords[0] * Num, _coords[1] * Num);
    } /* End of 'operator*' operator */

    /* Multiply by number operator with assignment */
    Vec2D<Type> operator*=(const Type Num)
    {
      _coords[0] *= Num;
      _coords[1] *= Num;

      return *this;
    } /* End of 'operator*=' operator */

    /* Getting length of the vertor function */
    Type length(void) const
    {
      return sqrt(_coords[0] * _coords[0] + _coords[1] * _coords[1]);
    } /* End of 'length' function */


    Type vecProd(Vec2D b)
    {
      return _coords[0] * b._coords[1] - _coords[1] * b._coords[0];
    }

    float getSin(Vec2D b)
    {
      Type z = _coords[0] * b._coords[1] - _coords[1] * b._coords[0];

      return z / fabs(length()) * fabs(b.length());
    }

    /* Normalize vector */
    Vec2D<Type> getNormalized(void)
    {
      Type len = length();
      return Vec2D<Type>(_coords[0] / len, _coords[1] / len);
    } /* End of 'normalize' function */

    /* Compare two vectors operator */
    bool operator<(const Vec2D<Type> &Vec) const
    {
      if (_coords[0] >= Vec._coords[0])
        return false;

      if (_coords[1] <= Vec._coords[1])
        return false;

      return true;
    } /* End of 'operator<' operator */

    /* Compare two vectors operator */
    bool operator>(const Vec2D<Type> &Vec) const
    {
      return !(*this < Vec);
    } /* End of 'operator>' operator */

    /* Rotate vector */
    Vec2D<Type> rotate(const float Angle) const
    {
      Vec2D<Type> res;

      res._coords[0] = _coords[0] * cos(Angle) - _coords[1] * sin(Angle);
      res._coords[1] = _coords[0] * sin(Angle) + _coords[1] * cos(Angle);

      if (fabs(res._coords[0]) < sup::eps)
        res._coords[0] = 0.0f;
      if (fabs(res._coords[1]) < sup::eps)
        res._coords[1] = 0.0f;

      return res;
    } /* End of 'rotate' function */
  }; /* End of 'Vec2D' class */
} /* end of 'sup' namespace */

/* END OF 'vec2d.h' FILE */
