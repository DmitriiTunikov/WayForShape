/*
 * Motion planning problem project.
 *
 * FILE: segment.cpp
 * AUTHORS:
 *   Denisov Pavel
 * LAST UPDATE: 22.05.2018
 * NOTE: segment class implementation file
 */

#include <cmath>
#include "segment.h"
#include "../../../anim/render/render.h"

/* Default class constructor */
Segment::Segment(void) : _fPoint({ 0, 0, 0 }), _sPoint({ 0, 0, 0 })
{
} /* End of constructor */

/* Class constructor */
Segment::Segment(const sup::Vecf &FPoint, const sup::Vecf &SPoint) : _fPoint(FPoint), _sPoint(SPoint)
{
} /* End of constructor */

/* Virtual draw function */
void Segment::draw(const bool isSolid) const
{
  anim::Render &rnd = anim::Render::getInstance();
  rnd.drawSegment(_fPoint, _sPoint);
} /* End of 'Segment::draw' function */

/* Flip function */
void Segment::flip(void)
{
  std::swap(_fPoint, _sPoint);
} /* End of 'Segment::flip' function */

/* Classify the point function */
Segment::ClassifyRes Segment::classify(const sup::Vecf &Vec) const
{
  sup::Vecf dir = (_sPoint - _fPoint).getNormalized();
  sup::Vecf fromStartToVec = (Vec - _fPoint).getNormalized();

  if (dir.vecProd(fromStartToVec) < -sup::eps)
    return ClassifyRes::RIGHT;
  else if (dir.vecProd(fromStartToVec) > sup::eps)
    return ClassifyRes::LEFT;

  return ClassifyRes::BETWEEN;
} /* End of 'Segment::classify' function */

/* Returning the rotated segment on pi / 2 angle */
Segment Segment::rotate(const float Angle) const
{
  sup::Vecf middle = _fPoint + (_sPoint - _fPoint) / 2;
  sup::Vecf dirFromMiddleToFinish = (_sPoint - middle).getNormalized();

  dirFromMiddleToFinish = dirFromMiddleToFinish.rotate(Angle);

  //Segment res;
  //res._sPoint = middle + dirFromMiddleToFinish * middle.length();
  //res._fPoint = middle - dirFromMiddleToFinish * middle.length();

  return Segment(middle + dirFromMiddleToFinish * middle.length(), middle - dirFromMiddleToFinish * middle.length());
} /* End of 'Segment::rotate' function */

/* Equal operator */
bool Segment::operator==(const Segment &Seg) const
{
  return _sPoint == Seg._sPoint && _fPoint == Seg._fPoint;
} /* End of 'operator==' operator */

/* Compare operator */
bool Segment::operator<(const Segment &Seg) const
{
  if (_fPoint < Seg._fPoint)
    return true;
  if (_fPoint > Seg._fPoint)
    return false;
  if (_sPoint < Seg._sPoint)
    return false;
  if (_sPoint > Seg._sPoint)
    return false;
  return true;
} /* End of 'Segment::operator<' operator */

/* END OF 'segment.cpp' FILE */
