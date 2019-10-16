/*
 * Motion planning problem project.
 *
 * FILE: drawable.h
 * AUTHORS:
 *   Denisov Pavel
 * LAST UPDATE: 19.05.2018
 * NOTE: drawable object class declaration file
 */

#pragma once

/* Drawable object class */
class Drawable
{
public:
  /* Virtual draw function */
  virtual void draw(const bool isSolid = false) const
  {
  } /* End of 'draw' function */

  /* Virtual constructor */
  virtual ~Drawable(void)
  {
  } /* End of destructor */
}; /* End of 'Drawable' class */

/* END OF 'drawable.h' FILE */
