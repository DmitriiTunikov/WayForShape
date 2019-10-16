/*
 * Motion planning problem project.
 *
 * FILE: anim.h
 * AUTHORS:
 *   Denisov Pavel
 * LAST UPDATE: 19.05.2018
 * NOTE: animation class declaration file
 */

#pragma once

/* Animation namespace */
namespace anim
{
  /* Animation class */
  class Anim
  {
  public:
    /* Getting instance function */
    static Anim & getInstance(void);

    /* Init function */
    void init(int Argc, char **Argv, const int W, const int H);

    /* Looping function */
    void run(void);

    /* Render function */
    void render(void);

    /* Resize function */
    void resize(const int W, const int H);

    /* Keyboard function */
    void keyboard(const unsigned char Key, const int X, const int Y);

    /* Mouse function */
    void mouseFunc(const int Button, const int State, const int X, const int Y);

  private:
    /* Class default constructor */
    Anim(void) = default;
  }; /* End of 'Anim' class */

  /*
   * Static GLUT functions
   */
  static void glutRender(void);
  static void glutResize(int W, int H);
  static void glutKeyboard(unsigned char Key, int X, int Y);
  static void glutMouse(int Button, int State, int X, int Y);
} /* end of 'anim' namespace */

/* END OF 'anim.h' FILE */
