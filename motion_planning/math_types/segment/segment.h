/*
 * Motion planning problem project.
 *
 * FILE: segment.h
 * AUTHORS:
 *   Denisov Pavel
 * LAST UPDATE: 22.05.2018
 * NOTE: segment class declaration file
 */

#pragma once

#include "../../../support/support.h"
#include "../../../anim/render/drawable/drawable.h"

/* Segment class */
class Segment : public Drawable
{
public:
  /* Classify result */
  enum struct ClassifyRes
  {
    LEFT,
    RIGHT,
    BETWEEN
  }; /* End of 'ClassifyRes' enumeration */

public:
  /* Two points */
  sup::Vecf _fPoint, _sPoint;

  /* Default class constructor */
  Segment(void);

  /* Class constructor */
  explicit Segment(const sup::Vecf &FPoint, const sup::Vecf &SPoint);

  /* Virtual draw function */
  virtual void draw(const bool isSolid) const;

  /* Flip function */
  void flip(void);

  /* Classify the point function */
  ClassifyRes classify(const sup::Vecf &Vec) const;

  sup::Vecf getCenter() const
  {
    return sup::Vecf((_fPoint._coords[0] + _sPoint._coords[0]) / 2, (_fPoint._coords[1] + _sPoint._coords[1]) / 2);
  }
  float len() const
  {
    return (_fPoint - _sPoint).length();
  }

  /* Returning the rotated segment on pi / 2 angle */
  Segment rotate(const float Angle) const;

  /* Equal operator */
  bool operator==(const Segment &Seg) const;

  /* Compare operator */
  bool operator<(const Segment &Seg) const;
}; /* End of 'Segment' class */

/* END OF 'segment.h' FILE */
