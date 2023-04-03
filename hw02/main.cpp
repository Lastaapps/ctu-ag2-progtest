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
// translates places to int
using PlaceToPoint = std::unordered_map<Place, Point>;

using Graph = std::vector<std::vector<Point>>;

class Flow {
    size_t cnt;
    std::vector<Capacity> capacities;
    std::vector<Capacity> flow;

    size_t toIndex(Point u, Point v) const {
      return u * cnt + v;
    }

  public:
    Flow(size_t cnt) : cnt(cnt), capacities(cnt * cnt), flow(cnt * cnt) {}

    void addCapacity(Point u, Point v, Capacity c) {
      capacities[toIndex(u, v)] += c;
      capacities[toIndex(v, u)] += c;
    }

    void sendFlow(Point u, Point v, Capacity f) {
      const size_t forward  = toIndex(u, v);
      const size_t backward = toIndex(v, u);
      // printf("Sending for %2u %2u of %2u ", u, v, f);

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

    Capacity getReserve(Point u, Point v) const {
      const size_t forward  = toIndex(u, v);
      const size_t backward = toIndex(v, u);
      const size_t reserve  = capacities[forward] - flow[forward] + flow[backward];
      // printf("Reserve for %u %u is %2lu (%2u - %2u + %2u)\n", u, v, reserve, capacities[forward], flow[forward], flow[backward]);
      return reserve;
    }

    void resetFlow() {
      std::fill(flow.begin(), flow.end(), 0);
    }

    Capacity amount(const Graph& graph, const Point u) const {
      Capacity sum = 0;
      for (const Point v : graph[u]) {
        const size_t index = toIndex(u, v);
        sum += flow[index];
      }
      return sum;
    }
};

struct Network {
  Graph graph;
  Flow  flow;
};

void addItem(std::vector<Point>& v, const Point item) {
  auto itr = std::lower_bound(v.begin(), v.end(), item);
  if (itr != v.end() && *itr == item) return;
  v.insert(itr, item);
}

void eraseItem(std::vector<Point>& v, const Point item) {
  auto itr = std::find(v.begin(), v.end(), item);
  if (itr != v.end()) {
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

Network preprocess(const Map& map) {
  const size_t nodeCnt = map.places.size();

  PlaceToPoint toPoint;
  toPoint.reserve(nodeCnt);

  for (Point id = 0; id < map.places.size(); ++id) {
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

  return {std::move(graph), std::move(capacities)};
}


struct Dfs {
  Graph& dinitzGraph;
  Flow&  flow;
  std::vector<Point>& toClean;

  Dfs(Graph& dinitzGraph, Flow& flow,std::vector<Point>& toClean) : dinitzGraph(dinitzGraph), flow(flow), toClean(toClean) {}

  /**
   * @return new flow added, 0 if path was not found
   */
  Capacity saturateShortesPath(const Point end, const Point u, Capacity currReserve) {
    // printf("DFS Processing %u\n", u);

    if (end == u) {
      return currReserve;
    }

    auto& edges = dinitzGraph[u];
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
        eraseItem(dinitzGraph[u], v);

        if (dinitzGraph[u].size() == 0) {
          toClean.push_back(u);
        }
      }

      // edge not fully saturated
      return minReserve;
    }
    return 0;
  };
};

void cleanGraph(const std::vector<uint32_t>& levels, std::vector<Point>& toClean, const Graph& graph, Graph& dinitzGraph) {
    // Dinitz cleaning
    while (!toClean.empty()) {
      const Point v = toClean.back();
      toClean.pop_back();

      const uint32_t level = levels[v];

      for (const auto& u : graph[v]) {
        if (levels[u] != level - 1) continue;

        auto& edges = dinitzGraph[u];
        eraseItem(edges, v);

        if (edges.size() == 0) {
          toClean.push_back(u);
        }
      }
    }
}

void dinitz(Network& network, const Point from, const Point to) {

  const Graph& graph = network.graph;
  Flow& flow = network.flow;
  const size_t nodeCnt = graph.size();
  flow.resetFlow();

  // printf("Dinitz\n");
  while(true) {
    // printf("Iteration\n");
    std::vector<uint32_t> levels(nodeCnt);
    std::vector<Point> toClean;
    Graph dinitzGraph(nodeCnt);
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

        uint32_t nextLevel = levels[u] + 1;

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

          queue.emplace(v);
          levels[v] = nextLevel;
          dinitzGraph[u].emplace_back(v);
        }

        if (dinitzGraph[u].size() == 0) {
          toClean.push_back(u);
        }
      }

      if (targetFound == false) {
        return;
        break;
      }
    }

    // printf("Got Dinitz graph:\n");
    // printGraph(dinitzGraph);

    // Finding shortest path
    Dfs dfs(dinitzGraph, flow, toClean);
    while(true) {

        // printf("Filling\n");
        cleanGraph(levels, toClean, graph, dinitzGraph);

      if (dfs.saturateShortesPath(to, from, (Capacity) -1) == 0) {
        break;
      }
    }
  }
}

std::set<Place> findReachable(const Map& map, const Network& network, Point from) {
  const size_t nodeCnt = map.places.size();

  std::set<Place> places;
  const Graph& graph = network.graph;
  const Flow&  flow  = network.flow;

  std::queue<Point> queue;
  std::vector<bool> visited(nodeCnt);
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

void fordFulkersonUpdateRoute(Network& network, const std::vector<Point>& parents, const Point from, const Point to) {
  Capacity minReserve = (Capacity) -1;
  {
    Point v = to;
    while(v != from) {
      const Point u = parents[v] - 1;
      minReserve = std::min(minReserve, network.flow.getReserve(u, v));
      v = u;
    }
  }
  {
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

  while(true) {
    std::vector<Point> parents(network.graph.size());
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

std::pair<Capacity, std::set<Place>> critical_streets(const Map& map) {
  const size_t nodeCnt = map.places.size();
  if (nodeCnt <= 1) {
    return {0, std::set<Place>(map.places.begin(), map.places.end())};
  }

  Network network = preprocess(map);

  // printf("Parsed graph:\n");
  // printGraph(network.graph);

  Point minFrom = 0;
  Point minTo   = 0;
  Point minPeople = (Point) -1;

  Point from = 0;
  // for (Point from = 0; from < nodeCnt; ++from) {
    for (Point to = from + 1; to < nodeCnt; ++to) {
      // printf("Running dinitz %d - %d\n", from, to);
      fordFulkerson(network, from, to);
      Capacity cnt = network.flow.amount(network.graph, from);
      if (cnt < minPeople) {
        minPeople = cnt;
        minFrom = from;
        minTo = to;
      }
      // printf("Result for %d - %d is %d\n\n\n", from, to, cnt);
    }
  // }

  fordFulkerson(network, minFrom, minTo);
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
  test(TESTS);
}

#endif


