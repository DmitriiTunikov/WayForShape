#pragma once

#include <cstdlib>
#include <iostream>
#include <vector>

/* Priority queue class */
template<class DataT, typename PriorityT>
class PriorityQueue
{
private:
  /* Queue element class */
  class Elem
  {
  public:
    DataT _data;
    PriorityT _priority;

    /* Class constructor */
    Elem(const DataT &D, const PriorityT Pr) : _data(D), _priority(Pr)
    {
    } /* End of constructor */
  }; /* End of 'Elem' class */

  /* Array of elements */
  std::vector<Elem> _elements;

private:
  /* Moving elemnt up function */
  void up(int Ind) 
  {
    while (Ind != 0 && _elements[Ind]._priority > _elements[(Ind - 1) / 2]._priority) 
    {
      std::swap(_elements[Ind], _elements[(Ind - 1) / 2]);
      Ind = (Ind - 1) / 2;
    }
  } /* End of 'up' function */

  /* Moving element down function */
  void down(int Ind)
  {
    int size = _elements.size();
    while (Ind < size / 2) 
    {
      int maxI = 2 * Ind + 1;
      if (2 * Ind + 2 < size && _elements[2 * Ind + 2]._priority > _elements[2 * Ind + 1]._priority)
        maxI = 2 * Ind + 2;
      if (_elements[Ind]._priority >= _elements[maxI]._priority)
        return;
      std::swap(_elements[Ind], _elements[maxI]);
      Ind = maxI;
    }
  } /* End of 'down' function */

public:
  /* empty function*/
  bool empty()
  {
    return _elements.size() == 0;
  }/*end of 'empty' function*/

  /* Class constructor */
  PriorityQueue(void) = default; 

  /* Adding new element */
  void push(const DataT &Value, const PriorityT Priority) 
  {
    _elements.push_back(Elem(Value, Priority));
    up(_elements.size() - 1);
  } /* End of 'push' function */

  /* Pop function */
  DataT pop(void)
  {
    std::swap(_elements[0], _elements[_elements.size() - 1]);
    down(0);
    DataT res = _elements[_elements.size() - 1]._data;
    _elements.pop_back();
    return res;
  } /* End of 'pop' function */
}; /* End of 'PriorityQueue' class */

/* END OF 'priority_queue.h' FILE */