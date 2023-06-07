#ifndef __PROGTEST__
#include <cassert>
#include <cstdint>
#include <iostream>
#include <memory>
#include <limits>
#include <optional>
#include <algorithm>
#include <bitset>
#include <list>
#include <array>
#include <vector>
#include <deque>
#include <set>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <stack>
#include <queue>
#include <random>

template < typename F, typename S >
struct std::hash<std::pair<F, S>> {
  std::size_t operator () (const std::pair<F, S> &p) const noexcept {
    // something like boost::combine would be much better
    return std::hash<F>()(p.first) ^ (std::hash<S>()(p.second) << 1);
  }
};

// For exposition only. In the tests Place will not
// be a string but some other type. This type will always
// be comparable, hashable and it will have default and
// copy constructors.
using Place = std::string;
using Capacity = unsigned;
using Connection = std::tuple<Place, Place, Capacity>;

struct Map {
  std::vector<Place> places;
  std::vector<Connection> connections;
};

#endif

/**
 * TODO
 * map names
 * Dinitz
 * Gomory–Hu tree
 */

// --- Preprocessing ----------------------------------------------------------
using Point = uint32_t;
using Profit = uint32_t;
using Level = uint32_t;
// translates places to int
using PlaceToPoint = std::unordered_map<Place, Point>;

using Graph = std::vector<std::vector<Point>>;

/*
 * Holds graph capacities and flow in n x n matrixes
 */
class Flow {
    size_t cnt;
    std::vector<Capacity> capacities;
    std::vector<Capacity> flow;

    size_t toIndex(Point u, Point v) const {
      return u * cnt + v;
    }

  public:
    Flow(size_t cnt) : cnt(cnt), capacities(cnt * cnt), flow(cnt * cnt) {}

    // this task expects undirected graph, can be updated to direct one here
    void addCapacity(Point u, Point v, Capacity c) {
      capacities[toIndex(u, v)] += c;
      capacities[toIndex(v, u)] += c;
    }

    void sendFlow(Point u, Point v, Capacity f) {
      const size_t forward  = toIndex(u, v);
      const size_t backward = toIndex(v, u);
      // printf("Sending for %2u %2u of %2u ", u, v, f);

      // we prefer lowering the flow in the opposite direction
      // and than increasing flow in our direction
      if (flow[backward] <= f) {
        f -= flow[backward];
        flow[backward] = 0;
      } else {
        flow[backward] -= f;
        f = 0;
      }

      flow[forward] += f;

      // printf("(%2u, %2u)\n", flow[forward], flow[backward]);
    }

    Capacity getCapacity(Point u, Point v) const {
      const size_t forward  = toIndex(u, v);
      return capacities[forward];
    }

    Capacity getReserve(Point u, Point v) const {
      const size_t forward  = toIndex(u, v);
      const size_t backward = toIndex(v, u);
      const size_t reserve  = capacities[forward] - flow[forward] + flow[backward];
      // printf("Reserve for %u %u is %2lu (%2u - %2u + %2u)\n", u, v, reserve, capacities[forward], flow[forward], flow[backward]);
      return reserve;
    }

    // yeah, that should be separated, I know
    void resetFlow() {
      std::fill(flow.begin(), flow.end(), 0);
    }

    /*
     * Computes the total flow in the network for a source vertex
     */
    Capacity amount(const Graph& graph, const Point u) const {
      Capacity sum = 0;
      for (const Point v : graph[u]) {
        sum += flow[toIndex(u, v)];
        sum -= flow[toIndex(v, u)];
      }
      return sum;
    }
};

struct Network {
  size_t size;
  Graph graph;
  Flow  flow;
};

// Prevents an edge being added twice
void addItem(std::vector<Point>& v, const Point item) {
  auto itr = std::lower_bound(v.begin(), v.end(), item);
  if (itr != v.end() && *itr == item) return;
  v.insert(itr, item);
}

// Removed edge
void eraseItem(std::vector<Point>& v, const Point item) {
  auto itr = std::lower_bound(v.begin(), v.end(), item);
  if (itr != v.end() && *itr == item) {
    v.erase(itr);
  }
}

void printGraph(const Graph& graph) {
  for (size_t i = 0; i < graph.size(); ++i) {
    printf("%2lu ->", i);
    for (const Point p : graph[i]) {
      printf(" %u", p);
    }
    printf("\n");
  }
}

// Graphviz dump
void dumpGraph(const Graph& graph, char name) {
  for (size_t i = 0; i < graph.size(); ++i) {
    for (const auto& children : graph[i]) {
      std::cout << name << i << " -> " << name << children << ";" << std::endl;
    }
  }
}

// Renames places to ints, clears loops and multi-edges
Network preprocess(const Map& map) {
  const size_t nodeCnt = map.places.size();

  PlaceToPoint toPoint;
  toPoint.reserve(nodeCnt);

  for (Point id = 0; id < nodeCnt; ++id) {
    const auto& place = map.places[id];
    toPoint.insert({place, id});
  }

  Graph graph(nodeCnt);
  Flow  capacities(nodeCnt);

  for (const auto& [fromPlace, toPlace, cap]: map.connections) {
    const Point from = toPoint[fromPlace];
    const Point to   = toPoint[toPlace];

    if (from == to) continue;
    
    addItem(graph[from], to);
    addItem(graph[to], from);
    capacities.addCapacity(from, to, cap);
  }

  return {nodeCnt, std::move(graph), std::move(capacities)};
}


// --- Dinitz -----------------------------------------------------------------
struct DinitzDfs {
  Graph& dinitzGraph;
  Flow&  flow;
  std::vector<Point>& toClean;

  DinitzDfs(Graph& dinitzGraph, Flow& flow,std::vector<Point>& toClean) : dinitzGraph(dinitzGraph), flow(flow), toClean(toClean) {}

  /**
   * Finds a route for saturation
   * @return new flow added, 0 if path was not found
   */
  Capacity saturateShortesPath(const Point end, const Point u, Capacity currReserve) {
    // printf("DFS Processing %u\n", u);

    if (end == u) {
      return currReserve;
    }

    auto& edges = dinitzGraph[u];
    // tries all the edges - should be useless (should stop only if u == from and graph is empty)
    while(!edges.empty()) {
      const Point v = edges.back();
      const Capacity localReserve = flow.getReserve(u, v);

      if (localReserve == 0) {
        edges.pop_back();
        continue;
      }

      // printf("DFS From %u to %u\n", u, v);
      const Capacity minReserve = saturateShortesPath(end, v, std::min(localReserve, currReserve));

      // no path found - should not happen
      if (minReserve == 0) {
        edges.pop_back();
        continue;
      }

      // updates flow
      flow.sendFlow(u, v, minReserve);

      // if the edge is fully saturated, clear it
      if (flow.getReserve(u, v) == 0) {
        edges.pop_back();

        if (edges.size() == 0) {
          toClean.push_back(u);
        }
      }

      // edge not fully saturated
      return minReserve;
    }
    return 0;
  };
};

/*
 * Cleans unreachable vertexes in the current Dinitz graph
 * Cleaning is scheduled before each shortest non-saturated path lookup
 */
void dinitzCleanGraph(std::vector<Level>& levels, std::vector<Point>& toClean, const Graph& graph, Graph& dinitzGraph) {
    while (!toClean.empty()) {
      const Point v = toClean.back();
      toClean.pop_back();

      const Level level = levels[v];
      levels[v] = 0;

      for (const auto& u : graph[v]) {
        if (levels[u] != level - 1) continue;

        auto& edges = dinitzGraph[u];
        eraseItem(edges, v);

        // vertex is useless now
        if (edges.size() == 0) {
          toClean.push_back(u);
        }
      }
    }
}

void dinitz(Network& network, const Point from, const Point to) {

  const Graph& graph = network.graph;
  Flow& flow = network.flow;
  flow.resetFlow();

  // printf("Dinitz\n");

  // Runs until non-saturated path is frond from from to to
  while(true) {
    // printf("Iteration\n");
    std::vector<Level> levels(network.size);
    std::vector<Point> toClean; toClean.reserve(network.size / 4);
    Graph dinitzGraph(network.size);
    bool targetFound = false;

    // BFS
    {
      std::queue<Point> queue;
      queue.emplace(from);
      levels[from] = 1;

      while (!queue.empty()) {
        Point u = queue.front();
        queue.pop();

        // printf("Poping\n");

        if (u == to) {
          targetFound = true;
          break;
        }

        const Level nextLevel = levels[u] + 1;

        const auto& edges = graph[u];

        for (size_t i = 0; i < edges.size(); ++i) {
          const Point v = edges[i];

          if (levels[v] != 0) {
            continue;
          }
          const Capacity reserve = flow.getReserve(u, v);
          if (reserve == 0) {
            continue;
          }

          levels[v] = nextLevel;
          queue.emplace(v);
          addItem(dinitzGraph[u], v);
        }

        // no child added - useless, let's clean it
        if (dinitzGraph[u].size() == 0) {
          toClean.push_back(u);
        }
      }

      // no path found, we got the max flow
      if (targetFound == false) {
        return;
      }
    }

    // printf("Got Dinitz graph:\n");
    // printGraph(dinitzGraph);

    // Finding shortest path
    DinitzDfs dfs(dinitzGraph, flow, toClean);
    while(true) {

      // clears useless edges/vetexes
      dinitzCleanGraph(levels, toClean, graph, dinitzGraph);

      // Tries to find the shortest unsaturated route
      if (dfs.saturateShortesPath(to, from, (Capacity) -1) == 0) {
        break;
      }
    }
  }
}





// --- Dinitz 2 ---------------------------------------------------------------
struct Dinitz2Dfs {
  const Graph& graph;
  Flow&  flow;
  std::vector<std::vector<bool>>& marked;
  std::vector<Point>& toClean;
  std::vector<uint32_t>& outCnt;

  Dinitz2Dfs(
      const Graph& graph,
      Flow& flow,
      std::vector<std::vector<bool>>& marked,
      std::vector<Point>& toClean,
      std::vector<uint32_t>& outCnt
      ) : graph(graph), flow(flow), marked(marked), toClean(toClean), outCnt(outCnt) {}

  /**
   * Finds a route for saturation
   * @return new flow added, 0 if path was not found
   */
  Capacity saturateShortesPath(const Point end, const Point u, Capacity currReserve) {
    // printf("DFS Processing %u\n", u);

    if (end == u) {
      return currReserve;
    }

    // tries all the edges - should be useless (should stop only if u == from and graph is empty)
    for (const Point v : graph[u]) {
      if (marked[u][v]) { continue; }

      const Capacity localReserve = flow.getReserve(u, v);
      // if (localReserve == 0) { continue; }
      assert(localReserve != 0);

      const Capacity minReserve = saturateShortesPath(end, v, std::min(localReserve, currReserve));

      // no path found - should not happen
      // if (minReserve == 0) { continue; }
      assert(minReserve != 0);

      // updates flow
      flow.sendFlow(u, v, minReserve);

      // if the edge is fully saturated, clear it
      if (minReserve == localReserve) {
        marked[u][v] = true;

        if (--outCnt[u] == 0) { toClean.push_back(u); }
      }

      // edge not fully saturated
      return minReserve;
    }

    return 0;
  };
};

/*
 * Cleans unreachable vertexes in the current Dinitz graph
 * Cleaning is scheduled before each shortest non-saturated path lookup
 */
void dinitz2CleanGraph(
        std::vector<std::vector<bool>>& marked,
        std::vector<Point>& toClean,
        const Graph& graph,
        std::vector<uint32_t>& outCnt
        ) {
    while (!toClean.empty()) {
      const Point v = toClean.back();
      toClean.pop_back();

      for (const auto& u : graph[v]) {

        if (marked[u][v]) { continue; }
        marked[u][v] = true;

        if (--outCnt[u] == 0) {
          // vertex is useless now
          toClean.push_back(u);
        }
      }
    }
}

void dinitz2(Network& network, const Point from, const Point to) {

  const Graph& graph = network.graph;
  Flow& flow = network.flow;
  flow.resetFlow();

  // printf("Dinitz (%u -> %u)\n", from, to);
  // printGraph(graph);

  // Runs until non-saturated path is found from from to to
  while(true) {
    // printf("Iteration (%u -> %u)\n", from, to);
    std::vector<uint32_t> outCnt(network.size);
    std::vector<Point> toClean; toClean.reserve(network.size / 4);
    std::vector<std::vector<bool>> marked(network.size, std::vector<bool>(network.size, false));
    bool targetFound = false;

    // BFS
    {
      std::vector<Level> levels(network.size);
      std::queue<Point> queue;
      queue.emplace(from);
      levels[from] = 2;

      while (!queue.empty()) {
        Point u = queue.front();
        queue.pop();

        if (u == to) {
          targetFound = true;
          // break; // full graph has to be marked
        }

        const Level nextLevel = levels[u] + 1;

        const auto& edges = graph[u];

        for (size_t i = 0; i < edges.size(); ++i) {
          const Point v = edges[i];

          const Capacity reserve = flow.getReserve(u, v);
          if (reserve == 0) {
            marked[u][v] = true;
            continue;
          }

          if (levels[v] == nextLevel) {
              ++outCnt[u];
              continue;
          }
          if (levels[v] != 0) {
            marked[u][v] = true;
            continue;
          }

          levels[v] = nextLevel;
          queue.emplace(v);
          ++outCnt[u];
        }
      }

      // no path found, we got the max flow
      if (targetFound == false) {
        return;
      }

      outCnt[to] = (Point) -1; // prevent to being removed
      for (Point u = 0; u < network.size; ++u) {
        // no child added - useless, let's clean it
        if (outCnt[u] == 0 && levels[u] != 0) {
          toClean.push_back(u);
        }
      }
    }

    // Finding shortest path
    Dinitz2Dfs dfs(graph, flow, marked, toClean, outCnt);
    while(true) {

      // clears useless edges/vetexes
      dinitz2CleanGraph(marked, toClean, graph, outCnt);

      // Tries to find the shortest unsaturated route
      if (dfs.saturateShortesPath(to, from, (Capacity) -1) == 0) {
        break;
      }
    }
  }
}




// --- Ford Fulkerson ---------------------------------------------------------
void fordFulkersonUpdateRoute(Network& network, const std::vector<Point>& parents, const Point from, const Point to) {
  Capacity minReserve = (Capacity) -1;
  {
    // Find min reserve on the path
    Point v = to;
    while(v != from) {
      const Point u = parents[v] - 1;
      minReserve = std::min(minReserve, network.flow.getReserve(u, v));
      v = u;
    }
  }
  {
    // Send flow trough the path
    Point v = to;
    while(v != from) {
      const Point u = parents[v] - 1;
      network.flow.sendFlow(u, v, minReserve);
      v = u;
    }
  }
}

void fordFulkerson(Network& network, const Point from, const Point to) {

  network.flow.resetFlow();

  // just BFS in a cycle
  while(true) {
    std::vector<Point> parents(network.size);
    std::queue<Point> queue;
    bool targetFound = false;

    queue.push(from);
    parents[from] = (Point) (-1 - 1);

    while(!queue.empty()) {
      const Point u = queue.front();
      queue.pop();

      for (const Point v : network.graph[u]) {
        if (parents[v] != 0) {
          continue;
        }

        const Capacity reserve = network.flow.getReserve(u, v);
        if (reserve == 0) {
          continue;
        }

        queue.push(v);
        parents[v] = u + 1;

        if (v == to) {
          fordFulkersonUpdateRoute(network, parents, from, to);
          targetFound = true;
          break;
        }

        if (targetFound == true) {
          break;
        }
      }
    }

    if (targetFound == false) {
      break;
    }
  }
}





// --- Goldberg ---------------------------------------------------------------
/**
 * Heuristic to improve rising time
 * Set default levels to the distance from the target
 */
void goldbergInitLevels(Network& network, const Point from, const Point to, std::vector<Level>& levels, std::vector<uint32_t>& inLevel) {
  const size_t nsize = network.size;
  inLevel[0] = nsize;

  std::queue<Point> queue;
  queue.push(to);
  while(!queue.empty()) {
    const Point u = queue.front();
    queue.pop();

    const Level level = levels[u];
    --inLevel[0];
    ++inLevel[level];

    for (const Point v : network.graph[u]) {
      if (levels[v] != 0) { continue; }
      levels[v] = level + 1;
      queue.push(v);
    }
  }

  // this will the true if the is at least 1 vertex incident with to
  // the algorithm above will got back to to (just one)
  if (levels[to] != 0) {
    --inLevel[levels[to]];
    ++inLevel[0];
    levels[to] = 0;
  }

  // According to the Goldberg rule
  --inLevel[levels[from]];
  ++inLevel[nsize];
  levels[from] = nsize;
}

// Also known as push-relabel
void goldberg(Network& network, const Point from, const Point to) {
  network.flow.resetFlow();

  const size_t nsize = network.size;
  std::vector<Profit> profits(nsize);
  std::vector<Level> levels(nsize);
  std::vector<uint32_t> inLevel(nsize * 2);
  std::queue<Point> queue;

  goldbergInitLevels(network, from, to, levels, inLevel);

  queue.push(from);

  while(!queue.empty()) {
    const Point u = queue.front();
    queue.pop();

    // for (size_t i = 0; i < levels.size(); ++i) {
    //   printf("[%2lu] DEBUG p: %2u, l: %2u - ", i, profits[i], levels[i]);
    //   for (const Point v : network.graph[i]) {
    //     printf(" |[%2u] %2u|,", v, network.flow.getReserve(i, v));
    //   }
    //   printf("\n");
    // }

    // printf("[%2u] Processing\n", u);

    // target cannot be changed in any way
    if (u == to) { continue; }

    // we can get as much as we want only from the source
    Profit canSend = from == u ? std::numeric_limits<Profit>::max() : profits[u];
    // The vertex was twice in the queue and was already resolved
    if (canSend == 0) { continue; }

    const Level level = levels[u];
    // printf("[%2u] Can send: %u\n", u, canSend);

    // if the vertex succeed to send some of it's profit to a neighbour
    bool updatedChilds = false;

    for (const Point v : network.graph[u]) {
      // can send only to the lower nodes
      if (level <= levels[v]) continue;
      if (canSend == 0) { break; }

      const Capacity reserve = network.flow.getReserve(u, v);
      // const Capacity reserveInverse = network.flow.getReserve(u, v);
      if (reserve == 0) { continue; }

      // calculate and send flow
      const Profit willSend = std::min(reserve, canSend);
      canSend -= willSend;
      if (v != from) {
        profits[v] += willSend;
      }
      network.flow.sendFlow(u, v, willSend);
      // printf("[%2u] Sedning to [%2u] flow %u\n", u, v, willSend);
      // printf("[%2u] Reserves: (%u - %u) - (%u - %u)\n", u, reserve, reserveInverse, network.flow.getReserve(u, v), network.flow.getReserve(v, u));

      // schedule the node if needed
      if (v != from && profits[v] > 0) {
        queue.push(v);
      }

      updatedChilds = true;
    }

    profits[u] = canSend;

    // failed to send any profit - inc level
    if (updatedChilds == false && u != from) {
      levels[u]++;
      --inLevel[level];
      ++inLevel[level + 1];

      // empty level found
      // all the nodes 0 < l[p] < |V| can be lifted to |V+1|
      if (inLevel[level] == 0 && level < nsize - 1) {
        const Level newLevel = nsize + 1;

        for (Point p = 0; p < nsize; ++p) {
          if (p == from || p == to) { continue; }

          const Level pLevel = levels[p];

          if (pLevel > level && pLevel < nsize) {
            --inLevel[pLevel];
            ++inLevel[newLevel];
            levels[p] = newLevel;
          }
        }
      }

      // printf("[%2u] Lifting to level %u\n", u, levels[u]);
    }

    // still can send something? Lets run again later
    if (canSend > 0 && u != from) {
      // printf("[%2u] Rescheduling\n", u);
      queue.push(u);
    }
  }
}


// --- Common code ------------------------------------------------------------
void algorithm(Network& network, const Point from, const Point to) {
  // dinitz(network, from, to);
  dinitz2(network, from, to);
  // fordFulkerson(network, from, to);
  // goldberg(network, from, to);
}

/**
 * Runs a simple BFS on the flow found by any of the prev algorithms
 */
std::set<Place> findReachable(const Map& map, const Network& network, Point from) {
  std::set<Place> places;
  const Graph& graph = network.graph;
  const Flow&  flow  = network.flow;

  std::queue<Point> queue;
  std::vector<bool> visited(network.size);
  queue.push(from);
  visited[from] = true;

  while(!queue.empty()) {
    const Point point = queue.front();
    queue.pop();

    places.insert(map.places[point]);

    for(const auto& target : graph[point]) {
      if (visited[target]) {
        continue;
      }
      if (flow.getReserve(point, target) == 0) {
        continue;
      }
      visited[target] = true;
      queue.push(target);
    }
  }

  return places;
}

std::pair<Capacity, std::set<Place>> critical_streets(const Map& map) {
  const size_t nodeCnt = map.places.size();
  if (nodeCnt <= 1) {
    return {0, std::set<Place>(map.places.begin(), map.places.end())};
  }

  Network network = preprocess(map);

  // printf("Parsed graph:\n");
  // printGraph(network.graph);
  // dumpGraph(network.graph, 'a');

  Point minFrom = 0;
  Point minTo   = 0;
  Point minPeople = (Point) -1;

  Point from = 0;
  // for (Point from = 0; from < nodeCnt; ++from) {
    for (Point to = from + 1; to < nodeCnt; ++to) {
      // printf("Running for %d - %d\n", from, to);
      algorithm(network, from, to);
      Capacity cnt = network.flow.amount(network.graph, from);
      if (cnt < minPeople) {
        minPeople = cnt;
        minFrom = from;
        minTo = to;
      }
      // printf("Result for %d - %d is %d\n\n\n", from, to, cnt);
    }
  // }

  algorithm(network, minFrom, minTo);
  return {network.flow.amount(network.graph, minFrom), findReachable(map, network, minFrom)};
}

#ifndef __PROGTEST__


using Test = std::pair<unsigned, Map>;

std::array TESTS = {
  Test{2, {{
    "Dejvicka", "Hradcanska", "Malostranska", "Staromestska", "Mustek", "Muzeum"
  }, {
    { "Dejvicka", "Hradcanska", 3 }, { "Hradcanska", "Malostranska", 2 },
    { "Malostranska", "Staromestska", 4 }, { "Staromestska", "Mustek", 5 },
    { "Mustek", "Muzeum", 3 }
  }}},
  Test{13, {{ 
    "Na Sklonku", "Poliklinika Čumpelíkova", "Šumavská", "Nové Podolí", "Vozovna Střešovice (Muzeum MHD)",
    "Florenc", "Franty Kocourka", "Cukrovar Čakovice", "Praha-Dejvice", "Pod Říhákem",
    "Sukovská", "Novoborská", "U Průhonu", "Nádraží Modřany",
  }, {
    { "Pod Říhákem", "Novoborská", 3 }, { "Nové Podolí", "Franty Kocourka", 4 },
    { "Cukrovar Čakovice", "Florenc", 10 }, { "Vozovna Střešovice (Muzeum MHD)", "Cukrovar Čakovice", 5 },
    { "U Průhonu", "Praha-Dejvice", 8 }, { "Sukovská", "Nové Podolí", 10 },
    { "Poliklinika Čumpelíkova", "Nové Podolí", 5 }, { "Florenc", "Na Sklonku", 5 },
    { "Praha-Dejvice", "Cukrovar Čakovice", 6 }, { "Franty Kocourka", "Šumavská", 10 },
    { "Florenc", "Vozovna Střešovice (Muzeum MHD)", 6 }, { "Pod Říhákem", "Florenc", 3 },
    { "Novoborská", "Šumavská", 8 }, { "Sukovská", "Šumavská", 8 },
    { "Nové Podolí", "Pod Říhákem", 10 }, { "Vozovna Střešovice (Muzeum MHD)", "Na Sklonku", 4 },
    { "Cukrovar Čakovice", "Nádraží Modřany", 9 }, { "Vozovna Střešovice (Muzeum MHD)", "Šumavská", 5 },
    { "Novoborská", "Cukrovar Čakovice", 3 }, { "Šumavská", "Na Sklonku", 5 },
    { "Poliklinika Čumpelíkova", "Vozovna Střešovice (Muzeum MHD)", 5 }, { "Nové Podolí", "Na Sklonku", 7 },
    { "Pod Říhákem", "U Průhonu", 5 }, { "Šumavská", "Sukovská", 7 },
    { "Šumavská", "Nádraží Modřany", 4 }, { "Nové Podolí", "Šumavská", 10 },
    { "Poliklinika Čumpelíkova", "Šumavská", 9 }, { "Šumavská", "Florenc", 3 },
    { "Franty Kocourka", "Nové Podolí", 3 }, { "U Průhonu", "Vozovna Střešovice (Muzeum MHD)", 5 },
    { "Praha-Dejvice", "Florenc", 3 }, { "Nové Podolí", "Praha-Dejvice", 1 },
    { "Pod Říhákem", "Florenc", 4 },
  }}},
  Test{5, {{ 
    "U Vojenské nemocnice", "Kuchyňka", "V Korytech", "Kelerka", "Vozovna Strašnice",
    "Geologická", "U Studánky", "U Jahodnice", "Hadovka", "Barrandovská",
    "K Netlukám", "Obchodní centrum Sárská", "Praha-Smíchov", "Sušická", "Moráň",
    "Praha-Bubny", "Rajská zahrada", "Strossmayerovo náměstí", "Průmstav",
  }, {
    { "U Vojenské nemocnice", "Vozovna Strašnice", 10 }, { "K Netlukám", "Obchodní centrum Sárská", 6 },
    { "Praha-Smíchov", "U Jahodnice", 8 }, { "Praha-Smíchov", "K Netlukám", 7 },
    { "Vozovna Strašnice", "Kelerka", 5 }, { "Obchodní centrum Sárská", "Geologická", 1 },
    { "K Netlukám", "Praha-Smíchov", 3 }, { "V Korytech", "Geologická", 9 },
    { "V Korytech", "Vozovna Strašnice", 1 }, { "Vozovna Strašnice", "V Korytech", 8 },
    { "U Vojenské nemocnice", "Kuchyňka", 6 }, { "Kelerka", "Geologická", 2 },
    { "Praha-Bubny", "Strossmayerovo náměstí", 4 }, { "Kuchyňka", "V Korytech", 3 },
    { "Praha-Smíchov", "Praha-Bubny", 9 }, { "Obchodní centrum Sárská", "Moráň", 1 },
    { "Kelerka", "V Korytech", 10 }, { "Kelerka", "V Korytech", 5 },
    { "Hadovka", "Rajská zahrada", 6 }, { "V Korytech", "Geologická", 7 },
    { "Sušická", "Praha-Smíchov", 2 }, { "Barrandovská", "K Netlukám", 6 },
    { "V Korytech", "Kelerka", 8 }, { "K Netlukám", "V Korytech", 6 },
    { "U Studánky", "Kuchyňka", 5 }, { "Hadovka", "Barrandovská", 6 },
    { "Praha-Bubny", "U Studánky", 4 }, { "Moráň", "K Netlukám", 4 },
    { "Strossmayerovo náměstí", "Kelerka", 7 }, { "Barrandovská", "U Jahodnice", 4 },
    { "V Korytech", "Kuchyňka", 5 }, { "Průmstav", "Praha-Smíchov", 5 },
    { "Geologická", "V Korytech", 7 }, { "Rajská zahrada", "Kuchyňka", 7 },
    { "U Jahodnice", "Kuchyňka", 1 }, { "Praha-Smíchov", "Sušická", 8 },
    { "K Netlukám", "Obchodní centrum Sárská", 3 }, { "Geologická", "Kelerka", 4 },
    { "Obchodní centrum Sárská", "K Netlukám", 2 }, { "Obchodní centrum Sárská", "K Netlukám", 6 },
    { "Hadovka", "U Studánky", 10 }, { "K Netlukám", "Sušická", 4 },
    { "Moráň", "U Vojenské nemocnice", 2 }, { "Obchodní centrum Sárská", "Praha-Smíchov", 3 },
    { "V Korytech", "U Studánky", 10 }, { "Kuchyňka", "Geologická", 6 },
    { "K Netlukám", "Moráň", 4 }, { "Sušická", "U Vojenské nemocnice", 5 },
    { "Kuchyňka", "U Vojenské nemocnice", 8 },
  }}},
  Test{0, {{ 
    "Na Lukách", "Plánická", "U Mezníku", "Bílá Hora", "Psohlavců",
    "Koupaliště Čakovice", "Volha", "Dolnopočernický hřbitov", "Studentský dům", "U Rozcestí",
    "Koleje Jižní Město", "Kusá", "Vozovna Pankrác", "Ke Koulce", "Opatov",
    "Nádraží Žvahov - V Násypu", "Na Blanici", "Nádraží Libeň", "Ořechovka", "Ke Kateřinkám",
    "Divadlo pod Palmovkou", "Přístav Radotín", "Žákovská", "Pankrác",
  }, {
    { "Koleje Jižní Město", "U Rozcestí", 7 }, { "Na Lukách", "Psohlavců", 7 },
    { "U Rozcestí", "U Mezníku", 5 }, { "Ke Koulce", "Kusá", 1 },
    { "Nádraží Žvahov - V Násypu", "Nádraží Libeň", 1 }, { "Koleje Jižní Město", "U Rozcestí", 1 },
    { "Koupaliště Čakovice", "Psohlavců", 1 }, { "Žákovská", "Pankrác", 7 },
    { "Kusá", "U Rozcestí", 10 }, { "Psohlavců", "Bílá Hora", 8 },
    { "Volha", "Na Lukách", 4 }, { "Na Lukách", "Plánická", 1 },
    { "Vozovna Pankrác", "Opatov", 7 }, { "Koupaliště Čakovice", "Bílá Hora", 1 },
    { "Ořechovka", "Na Blanici", 10 }, { "Psohlavců", "U Mezníku", 1 },
    { "U Mezníku", "Bílá Hora", 8 }, { "Divadlo pod Palmovkou", "U Rozcestí", 10 },
    { "Vozovna Pankrác", "U Mezníku", 7 }, { "Psohlavců", "Koupaliště Čakovice", 2 },
    { "Na Blanici", "Nádraží Žvahov - V Násypu", 2 }, { "Na Lukách", "Bílá Hora", 10 },
    { "Kusá", "Koleje Jižní Město", 10 }, { "Nádraží Libeň", "Ke Koulce", 2 },
    { "Opatov", "Ke Koulce", 10 }, { "Přístav Radotín", "Koupaliště Čakovice", 8 },
    { "Kusá", "Koleje Jižní Město", 5 }, { "Pankrác", "Žákovská", 2 },
    { "Vozovna Pankrác", "Koleje Jižní Město", 6 }, { "Plánická", "Na Lukách", 10 },
    { "Bílá Hora", "Koupaliště Čakovice", 10 }, { "Kusá", "Vozovna Pankrác", 7 },
    { "Koupaliště Čakovice", "Bílá Hora", 8 }, { "Psohlavců", "Koleje Jižní Město", 5 },
    { "Bílá Hora", "U Mezníku", 6 }, { "Psohlavců", "Bílá Hora", 4 },
    { "Koleje Jižní Město", "Kusá", 1 }, { "Přístav Radotín", "Opatov", 5 },
    { "Plánická", "U Mezníku", 3 }, { "Vozovna Pankrác", "Nádraží Žvahov - V Násypu", 1 },
    { "Koupaliště Čakovice", "Bílá Hora", 9 }, { "Nádraží Žvahov - V Násypu", "Opatov", 10 },
    { "Vozovna Pankrác", "Opatov", 9 }, { "Studentský dům", "Volha", 1 },
    { "Žákovská", "Pankrác", 1 }, { "U Mezníku", "Na Lukách", 9 },
    { "Ke Kateřinkám", "Koleje Jižní Město", 6 }, { "U Mezníku", "Plánická", 9 },
    { "Opatov", "Na Blanici", 9 }, { "U Mezníku", "Volha", 8 },
    { "Kusá", "Psohlavců", 2 }, { "Kusá", "Ke Koulce", 2 },
    { "Dolnopočernický hřbitov", "Bílá Hora", 4 },
  }}},
  Test{0, {{ "Na Lukách", }, { { "Na Lukách", "Na Lukách", 7 }, }}},
  Test{0, {{}, {}}},
};

void printAnswer(Capacity c, std::set<Place> places) {
  std::cout << "Res is: " << c << std::endl;
  for (const auto& place : places) {
    std::cout << place << " ";
  }
  std::cout << "\n\n\n" << std::endl;
}

template < typename C >
void test(C&& tests) {
  int fail = 0, ok = 0, cnt = 1;

  for (auto&& [ ref_s, map ] : tests) {
    // if (cnt != 2) {
    //   cnt++;
    //   continue;
    // }
    auto [ s, places ] = critical_streets(map);
    printAnswer(s, places);
    if (s == ref_s) {
      ok++;
      std::cout << "Test[" << cnt++ << "] PASSED" << std::endl;
    } else {
      fail++;
      std::cout << "Test[" << cnt++ << "] FAILED Got " << s << " but expected " << ref_s << std::endl;
    }
  }

  if (fail)
    std::cout << fail << " of " << fail + ok << " tests failed!" << std::endl;
  else
    std::cout << "All " << ok << " tests succeded!" << std::endl;
}

int main() {
  for (int i = 0; i < 100; ++i) {
    test(TESTS);
  }
}

#endif


