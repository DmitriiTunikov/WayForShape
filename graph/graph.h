/*
 * Motion planning problem project.
 *
 * FILE: graph.h
 * AUTHORS:
 *   Denisov Pavel
 * LAST UPDATE: 19.05.2018
 * NOTE: graph class declaration file
 */

#pragma once
#include <vector>

/* Graph class */
template<class GraphElem, typename WeightT = float>
class Graph
{
public:
  /* Add new elem function */
  void addElem(GraphElem &NewElem)
  {
    NewElem.setId(_elements.size());
    _elements.push_back(GraphElem(NewElem));
  } /* End of 'addElem' function */

  /* Add new link function */
  void addLink(const int Id1, const int Id2)
  {
    GraphElem &it1 = sup::findElemInArrayById<GraphElem>(_elements, Id1);
    GraphElem &it2 = sup::findElemInArrayById<GraphElem>(_elements, Id2);

    it1.addNeighbor(Id2);
    it2.addNeighbor(Id1);
  } /* End of 'addLink' function */

  /* Add new link function */
  void deleteLink(const int Id1, const int Id2)
  {
    GraphElem &it1 = sup::findElemInArrayById<GraphElem>(_elements, Id1);
    GraphElem &it2 = sup::findElemInArrayById<GraphElem>(_elements, Id2);

    it1.deleteNeighbor(Id2);
    it2.deleteNeighbor(Id1);
  } /* End of 'deleteLink' function */

  /* Getting weight between two vertices function */
  WeightT getWeight(int start, int end)
  {
    GraphElem &it1 = sup::findElemInArrayById<GraphElem>(_elements, start);
    GraphElem &it2 = sup::findElemInArrayById<GraphElem>(_elements, end);

    return it1.getWeight(it2);
  } /* End of 'getWeight' function */

  /* Getting array of vertices function */
  std::vector<GraphElem> getElements(void) const
  {
    return _elements;
  } /* End of 'getElements' function */

  /* Pulling edge function */
  void pullEdge(const int Id1, const int Id2)
  {
    deleteLink(Id1, Id2);

    GraphElem &it1 = sup::findElemInArrayById<GraphElem>(_elements, Id1);
    GraphElem &it2 = sup::findElemInArrayById<GraphElem>(_elements, Id2);

    std::vector<int> id2Neigh = it2.getNeighbors();
    for (size_t i = 0; i < id2Neigh.size(); i++)
      addLink(Id1, id2Neigh[i]);

    it1.merge(it2);
    for (auto ind : it2.getNeighbors())
    {
      GraphElem &cur = sup::findElemInArrayById<GraphElem>(_elements, ind);
      cur.deleteNeighbor(Id2);
    }

    _elements.erase(std::find(_elements.begin(), _elements.end(), it2));
  } /* End of 'pullEdge' function */

  /* Clear function */
  void clear(void)
  {
    _elements.clear();
  } /* End of 'clear' function */

  /* Class destructor */
  ~Graph(void)
  {
    clear();
  } /* End of destructor */
private:
  /* Array of elements */
  std::vector<GraphElem> _elements;
}; /* End of 'Graph' class */

/* END OF 'graph.h' FILE */
