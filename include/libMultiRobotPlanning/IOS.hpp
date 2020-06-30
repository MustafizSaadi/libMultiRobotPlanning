#pragma once

#ifdef USE_FIBONACCI_HEAP
#include <boost/heap/fibonacci_heap.hpp>
#endif

#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include <unordered_set>

#include "neighbor.hpp"
#include "planresult.hpp"

// #define REBUILT_FOCAL_LIST
// #define CHECK_FOCAL_LIST

namespace libMultiRobotPlanning {

/*!
  \example a_star_epsilon.cpp Simple example using a 2D grid world and
  up/down/left/right
  actions
*/

/*! \brief A*_epsilon Algorithm to find the shortest path with a given
suboptimality bound (also known as focal search)
This class implements the A*_epsilon algorithm, an informed search
algorithm
that finds the shortest path for a given map up to a suboptimality factor.
It uses an admissible heuristic (to keep track of the optimum) and an
inadmissible heuristic (
to guide the search within a suboptimal bound w.)
Details of the algorithm can be found in the following paper:\n
Judea Pearl, Jin H. Kim:\n
"Studies in Semi-Admissible Heuristics."" IEEE Trans. Pattern Anal. Mach.
Intell. 4(4): 392-399 (1982)\n
https://doi.org/10.1109/TPAMI.1982.4767270
This class can either use a fibonacci heap, or a d-ary heap. The latter is the
default. Define "USE_FIBONACCI_HEAP" to use the fibonacci heap instead.
\tparam State Custom state for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Cost Custom Cost type (integer or floating point types)
\tparam Environment This class needs to provide the custom logic. In
    particular, it needs to support the following functions:
  - `Cost admissibleHeuristic(const State& s)`\n
    This function can return 0 if no suitable heuristic is available.
  - `Cost focalStateHeuristic(const State& s, Cost gScore)`\n
    This function computes a (potentially inadmissible) heuristic for the given
state.
  - `Cost focalTransitionHeuristic(const State& s1, const State& s2, Cost
gScoreS1, Cost gScoreS2)`\n
    This function computes a (potentially inadmissible) heuristic for the given
state transition.
  - `bool isSolution(const State& s)`\n
    Return true if the given state is a goal state.
  - `void getNeighbors(const State& s, std::vector<Neighbor<State, Action,
   int> >& neighbors)`\n
    Fill the list of neighboring state for the given state s.
  - `void onExpandNode(const State& s, int fScore, int gScore)`\n
    This function is called on every expansion and can be used for statistical
purposes.
  - `void onDiscover(const State& s, int fScore, int gScore)`\n
    This function is called on every node discovery and can be used for
   statistical purposes.
    \tparam StateHasher A class to convert a state to a hash value. Default:
   std::hash<State>
*/
template <typename State, typename Action, typename Cost, typename Environment,
          typename StateHasher = std::hash<State> >
class IOS {
 public:
  IOS(Environment& environment, float w)
      : m_env(environment), m_w(w) {}

  bool search(const State& startState,
              PlanResult<State, Action, Cost>& solution) {
    solution.states.clear();
    solution.states.push_back(std::make_pair<>(startState, 0));
    solution.actions.clear();
    solution.cost = INT32_MAX;

    openSet_t openSet;
    focalSet_t
        focalSet;  // subset of open nodes that are within suboptimality bound
    anopenset_t anopen;
    std::unordered_map<State, fibHeapHandle_t, StateHasher> stateToHeap;
    std::unordered_map<State, fibHeapHandle_t, StateHasher> stateToFocal;
    std::unordered_set<State, StateHasher> closedSet;
    std::unordered_map<State, std::tuple<State, Action, Cost, Cost>,
                       StateHasher>
        cameFrom;

    auto handle = openSet.push(
        Node(startState, m_env.admissibleHeuristic(startState), 0, 0,0));
    stateToHeap.insert(std::make_pair<>(startState, handle));
    (*handle).handle = handle;

    anopen.push(handle);
    focalSet.push(handle);

    stateToFocal.insert(std::make_pair<>(startState, handle));

    std::vector<Neighbor<State, Action, Cost> > neighbors;
    neighbors.reserve(10);

    Cost bestFScore = (*handle).fScore;

    std::cout << "new search" << std::endl;

//my work
while(!openSet.empty()){

  //printf("In Open\n");

  const auto& nopen = openSet.top();
  //std:: cout << m_w <<std::endl;
  if(solution.cost <= m_w * nopen.fScore ){
        //printf("Found\n");
        return true;
  }
  //Node nopen = *nopen_handle;
  if(focalSet.empty()){
    printf("Empty\n");
    return false;
  }
  auto currentHandle = focalSet.top();
  Node current = *currentHandle;

  auto currentHandle_anop = anopen.top();
  Node current_anop = *currentHandle_anop;


  if(current.focalHeuristic < solution.cost){
    // Do greedy step

    //printf("greedy\n");

    focalSet.pop();
    // openSet.erase(currentHandle);
    stateToFocal.erase(current.state);
    closedSet.insert(current.state);

    m_env.onExpandNode(current.state, current.fScore, current.gScore);

    if (m_env.isSolution(current.state)) {
        printf("Solution\n");
        solution.states.clear();
        solution.actions.clear();
        auto iter = cameFrom.find(current.state);
        while (iter != cameFrom.end()) {
          solution.states.push_back(
              std::make_pair<>(iter->first, std::get<3>(iter->second)));
          solution.actions.push_back(std::make_pair<>(
              std::get<1>(iter->second), std::get<2>(iter->second)));
          iter = cameFrom.find(std::get<0>(iter->second));
        }
        solution.states.push_back(std::make_pair<>(startState, 0));
        std::reverse(solution.states.begin(), solution.states.end());
        std::reverse(solution.actions.begin(), solution.actions.end());
        solution.cost = current.gScore;
        if(openSet.empty())
          solution.fmin = current.fScore;
        else
          solution.fmin = openSet.top().fScore;
        if(current.focalHeuristic <= (m_w*current_anop.fopenmax)){
          return true;
        }
        

      }
    else{
      neighbors.clear();
      m_env.getNeighbors(current.state, neighbors);
      for (const Neighbor<State, Action, Cost>& neighbor : neighbors) {
        //printf("Neighbour\n");
        if (closedSet.find(neighbor.state) == closedSet.end()) {
          Cost tentative_gScore = current.gScore + neighbor.cost;
          auto iter = stateToHeap.find(neighbor.state);
          auto iterFocal = stateToFocal.find(neighbor.state);
          if (iter == stateToHeap.end() && iterFocal == stateToFocal.end()) {  // Discover a new node
            //std::cout << "  this is a new node" << std::endl;

            // I didn't add Improved termination condition

            Cost fScore =
                tentative_gScore + m_env.admissibleHeuristic(neighbor.state);

            // std::cout << tentative_gScore << " " << current.focalHeuristic << " " << m_env.focalStateHeuristic(neighbor.state, tentative_gScore) << " " << m_env.focalTransitionHeuristic(current.state, neighbor.state,
            //                                    current.gScore,
            //                                    tentative_gScore) << std::endl;

            Cost focalHeuristic = tentative_gScore + ((2*m_w-1) * m_env.admissibleHeuristic(neighbor.state)) ;

            Cost fopenmax = (tentative_gScore/m_w) +  m_env.admissibleHeuristic(neighbor.state);


            /*
            ((2*m_w - 1)* 
                (m_env.focalStateHeuristic(neighbor.state, tentative_gScore) +
                 m_env.focalTransitionHeuristic(current.state, neighbor.state,
                                               current.gScore,
                                               tentative_gScore)));
                                               */

            auto handle = openSet.push(
                Node(neighbor.state, fScore, tentative_gScore, focalHeuristic, fopenmax));
            (*handle).handle = handle;
            focalSet.push(handle);
            anopen.push(handle);
            // if (fScore <= bestFScore * m_w) {
            //   // std::cout << "focalAdd: " << *handle << std::endl;
            //   focalSet.push(handle);
            // }
            stateToHeap.insert(std::make_pair<>(neighbor.state, handle));
            stateToFocal.insert(std::make_pair<>(neighbor.state, handle));
            m_env.onDiscover(neighbor.state, fScore, tentative_gScore);
            // std::cout << "  this is a new node " << fScore << "," <<
            // tentative_gScore << std::endl;
            cameFrom.erase(neighbor.state);
            cameFrom.insert(std::make_pair<>(
              neighbor.state,
              std::make_tuple<>(current.state, neighbor.action, neighbor.cost,
                                tentative_gScore)));
          }
        }
      }
    }

  }
  else{

    //Do optimal step

    std::cout<< "In Optimal" <<std::endl;

    const auto& nopen2 = openSet.top();
    std:: cout<< anopen.size() << std:: endl;
    openSet.pop();
    std:: cout<< anopen.size() << std:: endl;
    auto iter_for_open = stateToHeap.find(nopen2.state);
    currentHandle = iter_for_open->second;
    current = *currentHandle;
    // openSet.pop();
    // openSet.erase(currentHandle);
    //anopen.erase(current);
    stateToHeap.erase(current.state);
    if(closedSet.find(current.state)==closedSet.end()){
      std::cout<< "New Low level node" << std::endl;
      //m_env.onExpandNode(current.state, current.fScore, current.gScore);
      closedSet.insert(current.state);
    }

    //Do i need to add this node to closed set

    m_env.onExpandNode(current.state, current.fScore, current.gScore);

    if (m_env.isSolution(current.state)) {
        solution.states.clear();
        solution.actions.clear();
        auto iter = cameFrom.find(current.state);
        while (iter != cameFrom.end()) {
          solution.states.push_back(
              std::make_pair<>(iter->first, std::get<3>(iter->second)));
          solution.actions.push_back(std::make_pair<>(
              std::get<1>(iter->second), std::get<2>(iter->second)));
          iter = cameFrom.find(std::get<0>(iter->second));
        }
        solution.states.push_back(std::make_pair<>(startState, 0));
        std::reverse(solution.states.begin(), solution.states.end());
        std::reverse(solution.actions.begin(), solution.actions.end());
        solution.cost = current.gScore;
        solution.fmin = current.fScore;
        return true;
      }
    else{
      neighbors.clear();
      m_env.getNeighbors(current.state, neighbors);
      for (const Neighbor<State, Action, Cost>& neighbor : neighbors) {

        Cost tentative_gScore = current.gScore + neighbor.cost;
        auto iter = stateToHeap.find(neighbor.state);
        auto iterFocal = stateToFocal.find(neighbor.state);

        if (iter != stateToHeap.end()) {
          std::cout<< "In open" <<std::endl;
          auto handle = iter->second;
            // We found this node with a better path than previous path
            if (tentative_gScore < (*handle).gScore) {
              std:: cout << "Update Open" <<std::endl;
              (*handle).gScore = tentative_gScore;
              (*handle).fScore = tentative_gScore + m_env.admissibleHeuristic(neighbor.state);
              (*handle).focalHeuristic = tentative_gScore + ((2*m_w-1) * m_env.admissibleHeuristic(neighbor.state)) ;
              (*handle).fopenmax = (tentative_gScore/m_w) +  m_env.admissibleHeuristic(neighbor.state);
              // (*handle).focalHeuristic = tentative_gScore + ((2*m_w - 1)* 
              //   (m_env.focalStateHeuristic(neighbor.state, tentative_gScore) +
              //    m_env.focalTransitionHeuristic(current.state, neighbor.state,
              //                                  current.gScore,
              //                                  tentative_gScore)));
                openSet.update(handle);
                // m_env.onDiscover(neighbor.state, (*handle).fScore,
                //              (*handle).gScore);

                //Do i need to include this node to focal
                if(iterFocal != stateToFocal.end()){
                  std:: cout << "Update Focal" <<std::endl;
                  // handle = iterFocal->second;
                  // focalSet.update(handle);
                
                }
              cameFrom.erase(neighbor.state);
                  cameFrom.insert(std::make_pair<>(
                  neighbor.state,
                  std::make_tuple<>(current.state, neighbor.action, neighbor.cost,
                                    tentative_gScore)));
            }
        }
        else if(closedSet.find(neighbor.state) == closedSet.end()){

          if (iter == stateToHeap.end()) {  // Discover a new node
            // std::cout << "  this is a new node" << std::endl;
            Cost fScore =
                tentative_gScore + m_env.admissibleHeuristic(neighbor.state);

            Cost focalHeuristic = tentative_gScore + ((2*m_w-1) * m_env.admissibleHeuristic(neighbor.state)) ;

            Cost fopenmax = (tentative_gScore/m_w) +  m_env.admissibleHeuristic(neighbor.state);

            // Cost focalHeuristic = tentative_gScore + ((2*m_w - 1)* 
            //     (m_env.focalStateHeuristic(neighbor.state, tentative_gScore) +
            //      m_env.focalTransitionHeuristic(current.state, neighbor.state,
            //                                    current.gScore,
            //                                    tentative_gScore)));
            auto handle = openSet.push(
                Node(neighbor.state, fScore, tentative_gScore, focalHeuristic,fopenmax));
            (*handle).handle = handle;
            
            focalSet.push(handle);
            anopen.push(handle);
            
            stateToHeap.insert(std::make_pair<>(neighbor.state, handle));
            stateToFocal.insert(std::make_pair<>(neighbor.state, handle));
            m_env.onDiscover(neighbor.state, fScore, tentative_gScore);
            // std::cout << "  this is a new node " << fScore << "," <<
            // tentative_gScore << std::endl;

            cameFrom.erase(neighbor.state);
            cameFrom.insert(std::make_pair<>(
              neighbor.state,
              std::make_tuple<>(current.state, neighbor.action, neighbor.cost,
                                tentative_gScore)));
        }
        }
      }

    }


  }
}

    return false;
  }

 private:
  struct Node;

#ifdef USE_FIBONACCI_HEAP
  typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
  typedef typename openSet_t::handle_type fibHeapHandle_t;
// typedef typename boost::heap::fibonacci_heap<fibHeapHandle_t,
// boost::heap::compare<compareFocalHeuristic> > focalSet_t;
#else
  typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                           boost::heap::mutable_<true> >
      openSet_t;
  typedef typename openSet_t::handle_type fibHeapHandle_t;
// typedef typename boost::heap::d_ary_heap<fibHeapHandle_t,
// boost::heap::arity<2>, boost::heap::mutable_<true>,
// boost::heap::compare<compareFocalHeuristic> > focalSet_t;
#endif

  struct Node {
    Node(const State& state, Cost fScore, Cost gScore, Cost focalHeuristic, Cost fopenmax)
        : state(state),
          fScore(fScore),
          gScore(gScore),
          focalHeuristic(focalHeuristic),
          fopenmax(fopenmax) {}

    bool operator<(const Node& other) const {
      //printf("InOpenCompare\n");
      // Sort order
      // 1. lowest fScore
      // 2. highest gScore

      // Our heap is a maximum heap, so we invert the comperator function here
      if (fScore != other.fScore) {
        return fScore > other.fScore;
      } else {
        return gScore < other.gScore;
      }
    }

    friend std::ostream& operator<<(std::ostream& os, const Node& node) {
      os << "state: " << node.state << " fScore: " << node.fScore
         << " gScore: " << node.gScore << " focal: " << node.focalHeuristic;
      return os;
    }

    State state;

    Cost fScore;
    Cost gScore;
    Cost focalHeuristic;
    Cost fopenmax;

    fibHeapHandle_t handle;
    // #ifdef USE_FIBONACCI_HEAP
    //   typename boost::heap::fibonacci_heap<Node>::handle_type handle;
    // #else
    //   typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
    //   boost::heap::mutable_<true> >::handle_type handle;
    // #endif
  };

  struct compareFocalHeuristic {
    bool operator()(const fibHeapHandle_t& h1,
                    const fibHeapHandle_t& h2) const {
      // Sort order (see "Improved Solvers for Bounded-Suboptimal Multi-Agent
      // Path Finding" by Cohen et. al.)
      // 1. lowest focalHeuristic
      // 2. lowest fScore
      // 3. highest gScore

      //printf("InfocalCompare\n");

      // Our heap is a maximum heap, so we invert the comperator function here

      //compare fun() for A*eps
      if ((*h1).focalHeuristic != (*h2).focalHeuristic) {
        return (*h1).focalHeuristic > (*h2).focalHeuristic;
      } 
      else if ((*h1).fScore != (*h2).fScore) {
          return (*h1).fScore > (*h2).fScore;
      } 
      else {
        return (*h1).gScore < (*h2).gScore;
      }

      
      //compare fun() for IOS
      // if ((*h1).focalHeuristic != (*h2).focalHeuristic) {
      //   return (*h1).focalHeuristic > (*h2).focalHeuristic;
      //   // } else if ((*h1).fScore != (*h2).fScore) {
      //   //   return (*h1).fScore > (*h2).fScore;
      // } 
      // // else if ((*h1).fScore != (*h2).fScore) {
      // //   return (*h1).fScore > (*h2).fScore;
      // //}
      // else {
      //   return (*h1).gScore < (*h2).gScore;
      // }



      
    }
  };

#ifdef USE_FIBONACCI_HEAP
  // typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
  // typedef typename openSet_t::handle_type fibHeapHandle_t;
  typedef typename boost::heap::fibonacci_heap<
      fibHeapHandle_t, boost::heap::compare<compareFocalHeuristic> >
      focalSet_t;
#else
  // typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
  // boost::heap::mutable_<true> > openSet_t;
  // typedef typename openSet_t::handle_type fibHeapHandle_t;
  typedef typename boost::heap::d_ary_heap<
      fibHeapHandle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
      boost::heap::compare<compareFocalHeuristic> >
      focalSet_t;
#endif


struct compareOpenMax {
    bool operator()(const fibHeapHandle_t& h1,
                    const fibHeapHandle_t& h2) const {
      // Sort order (see "Improved termination condition")

      // Our heap is a maximum heap, so we invert the comperator function here

      //compare fun() for A*eps
      return (*h1).fopenmax < (*h2).fopenmax;


      
    }
  };

  typedef typename boost::heap::d_ary_heap<
      fibHeapHandle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
      boost::heap::compare<compareOpenMax> >
      anopenset_t;

 private:
  Environment& m_env;
  float m_w;
};

}  // namespace libMultiRobotPlanning

  
  

