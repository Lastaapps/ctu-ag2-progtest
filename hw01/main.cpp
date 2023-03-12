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
using Connection = std::pair<Place, Place>;

struct Map {
  std::vector<Place> places;
  std::vector<Connection> connections;
};

#endif

using Point = uint32_t;
using Translation = std::unordered_map<Place, Point>;
using Lookup = std::vector<Point>;
using Graph = std::vector<std::vector<Point>>;

const Point FIRST_COMPONENT_ID = 1;

struct TrafficNetworkTester {
  explicit TrafficNetworkTester(const Map&);

  unsigned count_areas(const std::vector<Connection>& conns) const;

  private:
    Translation placeTrans;
    Lookup lookup;
    Graph componentNormalGraph;
    Graph componentInverseGraph;
};

template<bool useTwo>
std::pair<Graph, Lookup> solveGraph(const Graph& normal, const Graph& inverse, const Graph& addedNormal = Graph(), const Graph& addedInverse = Graph()) {

  const size_t items = normal.size() + (useTwo ? addedNormal.size() : 0);
  std::vector<Point> leftStack(items);

  {
    std::vector<Point> dfsStack(items);
    std::vector<bool> visited(items);

    for (Point start = 0; start < items; ++start) {
      if (visited[start]) {
        continue;
      }

      dfsStack.push_back(start);

      while(!dfsStack.empty()) {
        const Point point = dfsStack.back();
        dfsStack.pop_back();

        if (!useTwo || inverse.size() > point) {
          for (const Point& neighbour : inverse[point]) {
            if (visited[neighbour]) {
              continue;
            }

            visited[neighbour] = true;
            dfsStack.push_back(neighbour);
          }
        }

        if constexpr (useTwo) {
          for (const Point& neighbour : addedInverse[point]) {
            if (visited[neighbour]) {
              continue;
            }

            visited[neighbour] = true;
            dfsStack.push_back(neighbour);
          }
        }

        leftStack.push_back(point);
      }
    }
  }

  Point componentIdCounter = 0;
  std::vector<Point> lookup(items, -1);
  std::vector<Point> dfsStack(items);
  Graph componentGraph;

  for (const Point toProcess : leftStack) {
    if (toProcess >= 0) {
      continue;
    }
    const Point componentId = componentIdCounter++;

    dfsStack.push_back(toProcess);

    while(!dfsStack.empty()) {
      const Point point = dfsStack.back();
      dfsStack.pop_back();

      if (!useTwo || normal.size() > point) {
        for (const Point& neighbour : normal[point]) {
          if (lookup[neighbour] >= 0) {
            // TODO set
            componentGraph[componentId].push_back(lookup[neighbour]);
            continue;
          }

          lookup[neighbour] = componentId;
          dfsStack.push_back(neighbour);
        }
      }

      if constexpr (useTwo) {
        for (const Point& neighbour : addedNormal[point]) {
          if (lookup[neighbour] >= 0) {
            // TODO set
            componentGraph[componentId].push_back(lookup[neighbour]);
            continue;
          }

          lookup[neighbour] = componentId;
          dfsStack.push_back(neighbour);
        }
      }
    }
  }

  return {std::move(componentGraph), std::move(lookup)};
}

TrafficNetworkTester::TrafficNetworkTester(const Map& map) {
  // translate places to ints
  Point counter = 0;
  placeTrans.reserve(map.places.size());
  for (const auto& place : map.places) {
    placeTrans.emplace(place, counter++);
  }

  // build default graph
  Graph normalGraph (map.places.size());
  Graph inverseGraph(map.places.size());
  for (const auto& connection : map.connections) {
    const Point nameFrom = placeTrans[connection.first];
    const Point nameTo   = placeTrans[connection.second];
    normalGraph[nameFrom].push_back(nameTo);
    inverseGraph[nameTo].push_back(nameFrom);
  }

  // compose normal graph and lookup table
  auto [solvedGraph, solvedLookup] = solveGraph<false>(normalGraph, inverseGraph);
  componentNormalGraph = std::move(solvedGraph);
  lookup = std::move(solvedLookup);

  // compose inverse graph
  componentInverseGraph = Graph(componentNormalGraph.size());
  for (Point component = 0; component < componentNormalGraph.size(); ++component) {
    for(const Point& target : componentNormalGraph[component]) {
      componentInverseGraph[target].push_back(component);
    }
  }
}

Point translateAdded(const Place& place, const Translation& main, const Lookup& lookup, Translation& added, Point& counter) {
  const auto itr = main.find(place);
  if (itr != main.end()) {
    return lookup[itr -> second];
  }

  added.insert({place, counter});
  return counter++;
}

unsigned TrafficNetworkTester::count_areas(const std::vector<Connection> &conns) const {
  const size_t mySize = componentNormalGraph.size();
  Graph addedNormal(mySize + conns.size() * 2), addedInverse(mySize + conns.size() * 2);

  Point counter = componentNormalGraph.size();
  Translation addedTranslation;

  for (const auto& connection : conns) {
    const Point from = translateAdded(connection.first,  placeTrans, lookup, addedTranslation, counter);
    const Point to   = translateAdded(connection.second, placeTrans, lookup, addedTranslation, counter);

    addedNormal[from].push_back(to);
    addedInverse[to].push_back(from);
  }

  auto [solvedGraph, _] = solveGraph<true>(componentNormalGraph, componentInverseGraph, addedNormal, addedInverse);
  
  return solvedGraph.size();
}

#ifndef __PROGTEST__

using Test = std::pair<Map, std::vector<std::pair<unsigned, std::vector<Connection>>>>;

Test TESTS[] = {
  {
    { { "Dejvicka", "Hradcanska", "Malostranska", "Staromestska", "Mustek", "Muzeum" }, { // Map
                                                                                          { "Dejvicka", "Hradcanska" }, { "Hradcanska", "Malostranska" },
                                                                                          { "Malostranska", "Staromestska" }, { "Staromestska", "Mustek" },
                                                                                          { "Mustek", "Muzeum" }
                                                                                        } }, { // Queries
                                                                                          { 4, { { "Mustek", "Malostranska" } } },
                                                                                          { 4, { { "Malostranska", "Letnany" }, { "Letnany", "Dejvicka" } } },
                                                                                          { 1, { { "Malostranska", "Letnany" }, { "Letnany", "Dejvicka" }, { "Muzeum", "Hradcanska" } } },
                                                                                          { 1, { { "Muzeum", "Dejvicka" } } },
                                                                                          { 6, { { "Dejvicka", "Muzeum" } } },
                                                                                        }
  },
  {
    { { 
        "Na Sklonku", "Poliklinika Čumpelíkova", "Šumavská", "Nové Podolí", "Vozovna Střešovice (Muzeum MHD)",
        "Florenc", "Franty Kocourka", "Cukrovar Čakovice", "Praha-Dejvice", "Pod Říhákem",
        "Sukovská", "Novoborská", "U Průhonu", "Nádraží Modřany",
      }, {
        { "Pod Říhákem", "Novoborská" }, { "Nové Podolí", "Franty Kocourka" },
        { "Cukrovar Čakovice", "Florenc" }, { "Vozovna Střešovice (Muzeum MHD)", "Cukrovar Čakovice" },
        { "U Průhonu", "Praha-Dejvice" }, { "Sukovská", "Nové Podolí" },
        { "Poliklinika Čumpelíkova", "Nové Podolí" }, { "Florenc", "Na Sklonku" },
        { "Praha-Dejvice", "Cukrovar Čakovice" }, { "Franty Kocourka", "Šumavská" },
        { "Florenc", "Vozovna Střešovice (Muzeum MHD)" }, { "Pod Říhákem", "Florenc" },
        { "Novoborská", "Šumavská" }, { "Sukovská", "Šumavská" },
        { "Nové Podolí", "Pod Říhákem" }, { "Vozovna Střešovice (Muzeum MHD)", "Na Sklonku" },
        { "Cukrovar Čakovice", "Nádraží Modřany" }, { "Vozovna Střešovice (Muzeum MHD)", "Šumavská" },
        { "Novoborská", "Cukrovar Čakovice" }, { "Šumavská", "Na Sklonku" },
        { "Poliklinika Čumpelíkova", "Vozovna Střešovice (Muzeum MHD)" }, { "Nové Podolí", "Na Sklonku" },
        { "Pod Říhákem", "U Průhonu" }, { "Šumavská", "Sukovská" },
        { "Šumavská", "Nádraží Modřany" }, { "Nové Podolí", "Šumavská" },
        { "Poliklinika Čumpelíkova", "Šumavská" }, { "Šumavská", "Florenc" },
        { "Franty Kocourka", "Nové Podolí" }, { "U Průhonu", "Vozovna Střešovice (Muzeum MHD)" },
        { "Praha-Dejvice", "Florenc" }, { "Nové Podolí", "Praha-Dejvice" },
        { "Pod Říhákem", "Florenc" },
      } }, {
        { 4, {
             } },
          { 2, {
                 { "Nádraží Modřany", "Poliklinika Čumpelíkova" },
               } },
          { 2, {
                 { "Na Sklonku", "Poliklinika Čumpelíkova" },
               } },
          { 1, {
                 { "Nádraží Modřany", "Poliklinika Čumpelíkova" }, { "Na Sklonku", "Poliklinika Čumpelíkova" },
               } },
          { 4, {
                 { "Poliklinika Čumpelíkova", "Na Sklonku" },
               } },
          { 2, {
                 { "Cukrovar Čakovice", "Poliklinika Čumpelíkova" }, { "Poliklinika Čumpelíkova", "Na Sklonku" },
                 { "Nádraží Modřany", "Poliklinika Čumpelíkova" },
               } },
      }
  },
  {
    { { 
        "U Vojenské nemocnice", "Kuchyňka", "V Korytech", "Kelerka", "Vozovna Strašnice",
        "Geologická", "U Studánky", "U Jahodnice", "Hadovka", "Barrandovská",
        "K Netlukám", "Obchodní centrum Sárská", "Praha-Smíchov", "Sušická", "Moráň",
        "Praha-Bubny", "Rajská zahrada", "Strossmayerovo náměstí", "Průmstav",
      }, {
        { "U Vojenské nemocnice", "Vozovna Strašnice" }, { "K Netlukám", "Obchodní centrum Sárská" },
        { "Praha-Smíchov", "U Jahodnice" }, { "Praha-Smíchov", "K Netlukám" },
        { "Vozovna Strašnice", "Kelerka" }, { "Obchodní centrum Sárská", "Geologická" },
        { "K Netlukám", "Praha-Smíchov" }, { "V Korytech", "Geologická" },
        { "V Korytech", "Vozovna Strašnice" }, { "Vozovna Strašnice", "V Korytech" },
        { "U Vojenské nemocnice", "Kuchyňka" }, { "Kelerka", "Geologická" },
        { "Praha-Bubny", "Strossmayerovo náměstí" }, { "Kuchyňka", "V Korytech" },
        { "Praha-Smíchov", "Praha-Bubny" }, { "Obchodní centrum Sárská", "Moráň" },
        { "Kelerka", "V Korytech" }, { "Kelerka", "V Korytech" },
        { "Hadovka", "Rajská zahrada" }, { "V Korytech", "Geologická" },
        { "Sušická", "Praha-Smíchov" }, { "Barrandovská", "K Netlukám" },
        { "V Korytech", "Kelerka" }, { "K Netlukám", "V Korytech" },
        { "U Studánky", "Kuchyňka" }, { "Hadovka", "Barrandovská" },
        { "Praha-Bubny", "U Studánky" }, { "Moráň", "K Netlukám" },
        { "Strossmayerovo náměstí", "Kelerka" }, { "Barrandovská", "U Jahodnice" },
        { "V Korytech", "Kuchyňka" }, { "Průmstav", "Praha-Smíchov" },
        { "Geologická", "V Korytech" }, { "Rajská zahrada", "Kuchyňka" },
        { "U Jahodnice", "Kuchyňka" }, { "Praha-Smíchov", "Sušická" },
        { "K Netlukám", "Obchodní centrum Sárská" }, { "Geologická", "Kelerka" },
        { "Obchodní centrum Sárská", "K Netlukám" }, { "Obchodní centrum Sárská", "K Netlukám" },
        { "Hadovka", "U Studánky" }, { "K Netlukám", "Sušická" },
        { "Moráň", "U Vojenské nemocnice" }, { "Obchodní centrum Sárská", "Praha-Smíchov" },
        { "V Korytech", "U Studánky" }, { "Kuchyňka", "Geologická" },
        { "K Netlukám", "Moráň" }, { "Sušická", "U Vojenské nemocnice" },
        { "Kuchyňka", "U Vojenské nemocnice" },
      } }, {
        { 9, {
             } },
          { 5, {
                 { "Kuchyňka", "Kuchyňka" }, { "Strossmayerovo náměstí", "Průmstav" },
                 { "Průmstav", "V Korytech" }, { "K Netlukám", "Praha-Smíchov" },
                 { "Praha-Bubny", "Barrandovská" },
               } },
          { 9, {
                 { "Rajská zahrada", "Strossmayerovo náměstí" }, { "Sušická", "Obchodní centrum Sárská" },
                 { "Průmstav", "Strossmayerovo náměstí" }, { "Moráň", "Strossmayerovo náměstí" },
               } },
          { 5, {
                 { "Kelerka", "K Netlukám" }, { "U Studánky", "Sušická" },
                 { "U Studánky", "V Korytech" }, { "U Studánky", "Strossmayerovo náměstí" },
                 { "Kuchyňka", "V Korytech" }, { "Průmstav", "Rajská zahrada" },
               } },
          { 5, {
                 { "Vozovna Strašnice", "Obchodní centrum Sárská" }, { "Strossmayerovo náměstí", "Praha-Bubny" },
                 { "U Vojenské nemocnice", "V Korytech" }, { "U Jahodnice", "U Studánky" },
                 { "Rajská zahrada", "V Korytech" }, { "Obchodní centrum Sárská", "Sušická" },
               } },
          { 2, {
                 { "Barrandovská", "Praha-Smíchov" }, { "Geologická", "Hadovka" },
                 { "U Studánky", "Moráň" }, { "U Vojenské nemocnice", "Praha-Smíchov" },
               } },
      }
  },
  {
    { { 
        "Na Lukách", "Plánická", "U Mezníku", "Bílá Hora", "Psohlavců",
        "Koupaliště Čakovice", "Volha", "Dolnopočernický hřbitov", "Studentský dům", "U Rozcestí",
        "Koleje Jižní Město", "Kusá", "Vozovna Pankrác", "Ke Koulce", "Opatov",
        "Nádraží Žvahov - V Násypu", "Na Blanici", "Nádraží Libeň", "Ořechovka", "Ke Kateřinkám",
        "Divadlo pod Palmovkou", "Přístav Radotín", "Žákovská", "Pankrác", "Pod Děvínem",
      }, {
        { "Koleje Jižní Město", "U Rozcestí" }, { "Na Lukách", "Psohlavců" },
        { "U Rozcestí", "U Mezníku" }, { "Ke Koulce", "Kusá" },
        { "Nádraží Žvahov - V Násypu", "Nádraží Libeň" }, { "Koleje Jižní Město", "U Rozcestí" },
        { "Koupaliště Čakovice", "Psohlavců" }, { "Žákovská", "Pankrác" },
        { "Kusá", "U Rozcestí" }, { "Psohlavců", "Bílá Hora" },
        { "Volha", "Na Lukách" }, { "Na Lukách", "Plánická" },
        { "Vozovna Pankrác", "Opatov" }, { "Koupaliště Čakovice", "Bílá Hora" },
        { "Ořechovka", "Na Blanici" }, { "Psohlavců", "U Mezníku" },
        { "U Mezníku", "Bílá Hora" }, { "Divadlo pod Palmovkou", "U Rozcestí" },
        { "Vozovna Pankrác", "U Mezníku" }, { "Psohlavců", "Koupaliště Čakovice" },
        { "Na Blanici", "Nádraží Žvahov - V Násypu" }, { "Na Lukách", "Bílá Hora" },
        { "Kusá", "Koleje Jižní Město" }, { "Nádraží Libeň", "Ke Koulce" },
        { "Opatov", "Ke Koulce" }, { "Přístav Radotín", "Koupaliště Čakovice" },
        { "Kusá", "Koleje Jižní Město" }, { "Pankrác", "Žákovská" },
        { "Vozovna Pankrác", "Koleje Jižní Město" }, { "Plánická", "Na Lukách" },
        { "Bílá Hora", "Koupaliště Čakovice" }, { "Kusá", "Vozovna Pankrác" },
        { "Koupaliště Čakovice", "Bílá Hora" }, { "Psohlavců", "Koleje Jižní Město" },
        { "Bílá Hora", "U Mezníku" }, { "Psohlavců", "Bílá Hora" },
        { "Koleje Jižní Město", "Kusá" }, { "Přístav Radotín", "Opatov" },
        { "Plánická", "U Mezníku" }, { "Vozovna Pankrác", "Nádraží Žvahov - V Násypu" },
        { "Koupaliště Čakovice", "Bílá Hora" }, { "Nádraží Žvahov - V Násypu", "Opatov" },
        { "Vozovna Pankrác", "Opatov" }, { "Studentský dům", "Volha" },
        { "Žákovská", "Pankrác" }, { "U Mezníku", "Na Lukách" },
        { "Ke Kateřinkám", "Koleje Jižní Město" }, { "U Mezníku", "Plánická" },
        { "Opatov", "Na Blanici" }, { "U Mezníku", "Volha" },
        { "Kusá", "Psohlavců" }, { "Kusá", "Ke Koulce" },
        { "Dolnopočernický hřbitov", "Bílá Hora" },
      } }, {
        { 9, {
             } },
          { 8, {
                 { "Pod Děvínem", "U Rozcestí" }, { "Nádraží Žvahov - V Násypu", "Pod Děvínem" },
               } },
          { 7, {
                 { "Pankrác", "Pod Děvínem" }, { "U Rozcestí", "Žákovská" },
                 { "Pod Děvínem", "Nádraží Žvahov - V Násypu" },
               } },
      }
  },
};

template < typename C >
void test(C&& tests, bool exact) {
  int fail = 0, ok = 0;

  for (auto&& [ map, test_cases ] : tests) {
    TrafficNetworkTester T{map};
    for (auto&& [ ans, conns ] : test_cases)
      if (exact)
        (ans == T.count_areas(conns) ? ok : fail)++;
      else
        ((ans == 1) == (T.count_areas(conns) == 1) ? ok : fail)++;
  }

  if (fail)
    std::cout << fail << " of " << fail + ok << " tests failed!" << std::endl;
  else
    std::cout << "All " << ok << " tests succeded!" << std::endl;
}

int main() {
  test(TESTS, false);
  test(TESTS, true);
}

#endif


