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

using Edges = std::vector<std::vector<Point>>;
struct Flow {
  private:
    size_t cnt;
    std::vector<Capacity> flow;
  public:
    Flow(size_t cnt) : cnt(cnt), flow(cnt * cnt) {}

    Capacity& at(Point from, Point to) {
      if (from > to) std::swap(to, from);
      return flow[from * cnt + to];
    }

    const Capacity& at(Point from, Point to) const {
      if (from > to) std::swap(to, from);
      return flow[from * cnt + to];
    }
};

struct Graph {
  Edges edges;
  Flow capacities;
};

void addToVector(std::vector<Point>& v, const Point to) {
  auto itr = std::lower_bound(v.begin(), v.end(), to);
  if (*itr == to) return;
  v.insert(itr, to);
}

Graph preprocess(const Map& map) {
  const size_t nodeCnt = map.places.size();

  PlaceToPoint toPoint;
  toPoint.reserve(nodeCnt);

  for (Point id = 0; id < map.places.size(); ++id) {
    const auto& place = map.places[id];
    toPoint.insert({place, id});
  }

  Edges edges(nodeCnt);
  Flow  capacities(nodeCnt);

  for (const auto& [fromPlace, toPlace, cap]: map.connections) {
    const Point from = toPoint[fromPlace];
    const Point to   = toPoint[toPlace];

    if (from == to) continue;
    
    addToVector(edges[from], to);
    addToVector(edges[to], from);
    capacities.at(from, to) += cap;
  }

  return {std::move(edges), std::move(capacities)};
}

/**
 * @return new flow added, 0 if path was not found
 */
Capacity saturateShortesPath(Edges& graph, Flow& reserves, const Point end, Point point, Capacity minReserve) {
  if (end == point) {
    return minReserve;
  }

  auto& edges = graph[point];
  while(!edges.empty()) {
    const Point target = edges.back();
    const Capacity mReserve = reserves.at(point, target);
    if (mReserve == 0) {
      edges.pop_back();
      continue;
    }

    const Capacity res = saturateShortesPath(graph, reserves, end, target, std::min(mReserve, minReserve));

    // no path found
    if (res == 0) {
      edges.pop_back();
      continue;
    }

    reserves.at(point, target) -= res;

    // edge not fully saturated
    return res;
  }
  return 0;
};


Flow dinitz(const Graph& graph, const Point from, const Point to) {

  const auto& [graphEdges, capacities] = graph;
  const size_t nodeCnt = graphEdges.size();

  // build flow
  Flow reserves = capacities;

  while(true) {
    std::vector<uint32_t> levels(nodeCnt);
    std::vector<Point> toClean;
    Edges dinitzGraph(nodeCnt);
    bool targetFound = false;

    // BFS
    {
      std::queue<Point> queue;
      queue.emplace(from);
      levels[from] = 1;

      while (!queue.empty()) {
        Point point = queue.front();
        queue.pop();

        if (point == to) {
          targetFound = true;
          break;
        }

        uint32_t nextLevel = levels[point] + 1;

        const auto& edges = graphEdges[point];

        for (size_t i = 0; i < edges.size(); ++i) {
          const Point target = edges[i];

          if (levels[target] != 0) {
            continue;
          }
          const Capacity reserve = reserves.at(point, target);
          if (reserve == 0) {
            continue;
          }

          queue.emplace(target);
          levels[target] = nextLevel;
          dinitzGraph[point].emplace_back(target);
        }

        if (dinitzGraph[point].size() == 0) {
          toClean.push_back(point);
        }
      }

      if (targetFound == false) {
        return reserves;
        break;
      }
    }

    // Dinitz cleaning
    while (!toClean.empty()) {
      const Point point = toClean.back();
      toClean.pop_back();

      const uint32_t level = levels[point];
      for (const auto& target : graphEdges[point]) {
        if (levels[target] == level - 1) {

          auto& tEdges = dinitzGraph[target];
          auto itr = std::find(tEdges.begin(), tEdges.end(), point);
          tEdges.erase(itr);

          if (tEdges.size() == 0) {
            toClean.push_back(target);
          }
        }
      }
    }

    // Finding shortest path
    while(true) {
      if (saturateShortesPath(dinitzGraph, reserves, from, to, (Capacity) -1) == 0) {
        break;
      }
    }
  }
}

template<bool buildSet>
std::pair<Capacity, std::set<Place>> countReachable(const Map& map, const Graph& graph, const Flow& reserves, Point from) {
  const size_t nodeCnt = map.places.size();
  Capacity counter = 0;
  std::set<Place> places;

  std::queue<Point> queue;
  std::vector<bool> visited(nodeCnt);
  queue.push(from);
  visited[from] = true;

  while(!queue.empty()) {
    const Point point = queue.front();
    queue.pop();

    if constexpr (buildSet) {
      places.insert(map.places[point]);
    }

    for(const auto& target : graph.edges[point]) {
      if (visited[target]) {
        continue;
      }
      if (reserves.at(point, target) == 0) {
        counter += graph.capacities.at(point, target);
        continue;
      }
      visited[target] = true;
      queue.push(target);
    }
  }

  return {counter, std::move(places)};
}

std::pair<Capacity, std::set<Place>> critical_streets(const Map& map) {
  const size_t nodeCnt = map.places.size();
  if (nodeCnt <= 1) {
    return {0, std::set<Place>(map.places.begin(), map.places.end())};
  }

  const Graph graph = preprocess(map);

  Point minFrom = 0;
  Point minTo   = 0;
  Point minPeople = (Point) -1;

  for (Point from = 0; from < nodeCnt; ++from) {
    for (Point to = from + 1; to < nodeCnt; ++to) {
      const Flow reserves = dinitz(graph, from, to);
      auto [cnt, _] = countReachable<false>(map, graph, reserves, from);
      if (cnt < minPeople) {
        minFrom = from;
        minTo = to;
      }
    }
  }

  const Flow reserves = dinitz(graph, minFrom, minTo);
  return countReachable<true>(map, graph, reserves, minFrom);
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

template < typename C >
void test(C&& tests) {
  int fail = 0, ok = 0;

  for (auto&& [ ref_s, map ] : tests) {
    auto [ s, _ ] = critical_streets(map);
    if (s == ref_s) ok++;
    else {
      fail++;
      std::cout << "Got " << s << " but expected " << ref_s << std::endl;
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


