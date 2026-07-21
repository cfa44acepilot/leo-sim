/*****************************************************************************
  filename route_diag.cpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator (monitor)
  date 2026-07-14
  Brief Description:
    Implementation of the renderer-vs-router link comparison. See route_diag.hpp
    for why it exists.

    Everything here is read-only against leo::Constellation. The one piece of
    state it creates is a private leo::Router built from the same inputs the
    simulator's refresh() uses, which is what lets has_edge() answer "could the
    router have used this link?" without exposing the simulator's own router.
 *****************************************************************************/

#include "route_diag.hpp"

#include <algorithm> /* std::reverse (the Dijkstra path comes out backwards) */
#include <cmath>     /* std::asin/sqrt (elevation, distances)                */
#include <cstdint>   /* std::uint8_t (the destination-visible bitmap)        */
#include <limits>    /* infinity as the Dijkstra "unreached" distance        */
#include <queue>     /* priority_queue for the blue-link chain search        */
#include <utility>   /* std::pair in the search frontier                     */
#include <vector>    /* the graphs and paths built here                      */

#include "router.hpp" /* leo::Router -- the CSR whose contents are the point  */

namespace monitor
{
namespace
{

/* Uplink elevation threshold. The simulator calls connect_ground_uplinks with a
   default-constructed UplinkSpec, so this mirrors that default; it is repeated
   (not shared) only because UplinkSpec's default IS the value, with no named
   constant to reach for. */
constexpr double kMinElevationDeg = 25.0;

/* Radians -> degrees, for the elevation readout. */
constexpr double kRadToDeg = 180.0 / 3.14159265358979;

/*---------------------------------------------------------------------------
  Function: dist_km
  Description: Straight-line distance between two nodes, in the ECEF frame the
               router weights its edges with -- so a distance printed here is
               the distance the router priced.
  Input: c    -- the constellation
         a, b -- node ids (satellite or ground; both carry position_ecef)
  Outputs: The distance in kilometers.
---------------------------------------------------------------------------*/
double dist_km(const leo::Constellation& c, /* source of both positions */
               leo::NodeId a,               /* one end                  */
               leo::NodeId b                /* the other                */
)
{
  const leo::Vec3& pa = c.position_ecef[a]; /* both frames are ECEF km */
  const leo::Vec3& pb = c.position_ecef[b];
  const double dx = pa.x - pb.x;
  const double dy = pa.y - pb.y;
  const double dz = pa.z - pb.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/*---------------------------------------------------------------------------
  Function: gate_ok
  Description: The ROUTER's satellite-to-satellite feasibility gate, re-run here
               on one pair. Deliberately the same two conditions Router::build
               applies, so a "gate passes but no edge" line in the report can
               only mean one thing: the pair was never wired as neighbors.
  Input: c    -- the constellation
         a, b -- satellite node ids
         spec -- the link budget being tested against
  Outputs: True if the pair is within range AND has a clear line of sight.
---------------------------------------------------------------------------*/
bool gate_ok(const leo::Constellation& c, /* positions to measure           */
             leo::NodeId a,               /* one end                        */
             leo::NodeId b,               /* the other                      */
             const leo::LinkSpec& spec    /* range + block radius to test   */
)
{
  if (dist_km(c, a, b) > spec.max_range_km)
  {
    return false; /* out of laser range: cheapest test first */
  }
  return leo::earth_clear(c.position_ecef[a], c.position_ecef[b],
                          spec.block_radius_km);
}

/*---------------------------------------------------------------------------
  Function: slot_kind
  Description: How two nodes are wired -- the wiring the router draws its edges
               from. Slots 0/1 are the in-plane fore/aft terminals, 2/3 the
               cross-plane pair; a ground endpoint is an uplink. Checked in BOTH
               directions because cross-plane picks can be asymmetric (routing
               treats links as undirected, so a one-sided pick still routes).
  Input: c    -- the constellation
         a, b -- node ids
  Outputs: A short label: "uplink", "in-plane", "cross-plane", or "none" -- the
           last meaning the two are not wired to each other at all.
---------------------------------------------------------------------------*/
const char* slot_kind(const leo::Constellation& c, /* the wiring being read */
                      leo::NodeId a,               /* one end               */
                      leo::NodeId b                /* the other             */
)
{
  if (c.is_ground(a) || c.is_ground(b))
  {
    return "uplink"; /* a ground endpoint can only ever be an uplink */
  }
  for (int s = 0; s < 4; ++s) /* 0,1 in-plane; 2,3 cross-plane */
  {
    const bool a_to_b = c.isl_neighbors[a][s] == b;
    const bool b_to_a = c.isl_neighbors[b][s] == a;
    if (a_to_b || b_to_a)
    {
      return s < 2 ? "in-plane" : "cross-plane";
    }
  }
  return "none";
}

/*---------------------------------------------------------------------------
  Function: elevation_deg
  Description: A satellite's elevation above a ground node's local horizon --
               the quantity connect_ground_uplinks thresholds. Uses geocentric
               "up" (the station's own position direction), matching ground.cpp,
               so the number here is the number that decided visibility.
  Input: c   -- the constellation
         g   -- the ground node id
         sat -- the satellite node id
  Outputs: Elevation in degrees; negative means below the horizon.
---------------------------------------------------------------------------*/
double elevation_deg(const leo::Constellation& c, /* positions           */
                     leo::NodeId g,               /* the ground station  */
                     leo::NodeId sat              /* the satellite       */
)
{
  const leo::Vec3& gp = c.position_ecef[g];   /* station, ECEF km   */
  const leo::Vec3& sp = c.position_ecef[sat]; /* satellite, ECEF km */
  const double gl = std::sqrt(gp.x * gp.x + gp.y * gp.y + gp.z * gp.z);
  const double dx = sp.x - gp.x; /* station -> satellite */
  const double dy = sp.y - gp.y;
  const double dz = sp.z - gp.z;
  const double dl = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (gl <= 0.0 || dl <= 0.0)
  {
    return -90.0; /* degenerate geometry: report it as invisible */
  }
  const double sin_elev = (gp.x * dx + gp.y * dy + gp.z * dz) / (gl * dl);
  return std::asin(std::clamp(sin_elev, -1.0, 1.0)) * kRadToDeg;
}

/*---------------------------------------------------------------------------
  Function: is_blue
  Description: Does the RENDERER draw an in-group (blue) link between i and j?
               Since the in-plane seeding moved into the core, that is simply
               "are they wired into each other's in-plane slots" --
               build_ingroup_lines draws slots 0/1 verbatim. Reading the SLOTS
               (rather than re-deriving the picks) is what makes this a real
               check: it compares the router's graph against the actual drawn
               geometry, not against a second guess at it.
  Input: c    -- the constellation
         i, j -- satellite node ids
  Outputs: True if a blue segment is drawn between them.
---------------------------------------------------------------------------*/
bool is_blue(const leo::Constellation& c, /* the wiring that is drawn */
             leo::NodeId i,               /* one end                  */
             leo::NodeId j                /* the other                */
)
{
  for (int s = 0; s < 2; ++s) /* slots 0,1 = the in-plane fore/aft terminals */
  {
    if (c.isl_neighbors[i][s] == j || c.isl_neighbors[j][s] == i)
    {
      return true;
    }
  }
  return false;
}

} /* namespace */

/* See the header for the contract. */
void print_route_diag(
    const leo::Constellation& c, /* propagated + linked, this tick */
    const leo::Snapshot& snap,   /* the route to explain           */
    const leo::LinkSpec& spec,   /* the budget it was routed under */
    std::FILE* out               /* report destination             */
)
{
  /* The router's own graph. Router::build is a pure function of (c, spec), so
     this CSR is identical to the one the simulator just routed on -- which is
     what makes has_edge() an authority here rather than an approximation. */
  leo::Router router; /* the graph under examination */
  router.build(c, spec);

  const leo::NodeId src = snap.src_node; /* ground source node      */
  const leo::NodeId dst = snap.dst_node; /* ground destination node */
  std::fprintf(out,
               "\n=== ROUTE DIAGNOSTIC =====================================\n"
               "src node %u, dst node %u | max_range %.0f km, block_radius "
               "%.0f km\n",
               static_cast<unsigned>(src), static_cast<unsigned>(dst),
               spec.max_range_km, spec.block_radius_km);

  /* --- 1: THE CHOSEN ROUTE ------------------------------------------------ */
  std::fprintf(out, "\n[1] CHOSEN ROUTE\n");
  if (!snap.found)
  {
    std::fprintf(out, "  no route found (reason code %d)\n",
                 static_cast<int>(snap.no_path_reason));
  }
  else
  {
    std::fprintf(out, "  %-6s %-6s %-9s %-9s %10s  %s\n", "hop", "from", "to",
                 "cat(to)", "dist km", "link");
    double sum_km = 0.0; /* path length, printed for context */
    for (std::size_t h = 0; h + 1 < snap.path.size(); ++h)
    {
      const leo::NodeId a = snap.path[h];     /* this hop's near end */
      const leo::NodeId b = snap.path[h + 1]; /* this hop's far end  */
      const double d = dist_km(c, a, b);
      sum_km += d;
      std::fprintf(out, "  %-6zu %-6u %-9u %-9u %10.1f  %s\n", h + 1,
                   static_cast<unsigned>(a), static_cast<unsigned>(b),
                   static_cast<unsigned>(c.is_ground(b) ? 0 : c.catalog_at(b)),
                   d, slot_kind(c, a, b));
    }
    std::fprintf(out, "  %zu hops, %.1f km total, cost %.3f ms\n", snap.hops,
                 sum_km, snap.cost_ms);
  }

  /* --- 2: THE ALTERNATIVE IN-PLANE CHAIN ----------------------------------
     Rather than take a chain on faith, DERIVE the one the user would see: the
     shortest chain of DRAWN blue links from a satellite the source can see to a
     satellite the destination can see. If the renderer and the router ever
     disagree again, this is the chain that would expose it. */
  std::fprintf(out, "\n[2] BEST CHAIN OF *DRAWN* BLUE (in-group) LINKS\n");
  std::vector<std::vector<leo::NodeId>> blue(
      c.num_satellites); /* drawn graph */
  for (leo::NodeId i = 0; i < c.num_satellites; ++i)
  {
    for (int s = 0; s < 2; ++s) /* the two drawn in-plane slots */
    {
      const leo::NodeId j = c.isl_neighbors[i][s];
      if (j != leo::INVALID_NODE)
      {
        blue[i].push_back(j);
      }
    }
  }

  /* Dijkstra over blue links only, seeded from every satellite the SOURCE can
     uplink to, stopping at the first satellite the DESTINATION can see. */
  const std::size_t src_g = src - c.num_satellites; /* ground list index */
  const std::size_t dst_g = dst - c.num_satellites;
  const std::vector<leo::NodeId>& src_vis = c.ground_uplinks[src_g];
  const std::vector<leo::NodeId>& dst_vis = c.ground_uplinks[dst_g];
  std::vector<std::uint8_t> is_dst_vis(c.num_satellites,
                                       0); /* O(1) goal test */
  for (const leo::NodeId n : dst_vis)
  {
    is_dst_vis[n] = 1;
  }

  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> best(c.num_satellites, inf); /* km along blue links */
  std::vector<leo::NodeId> came(c.num_satellites, leo::INVALID_NODE);
  using Item = std::pair<double, leo::NodeId>; /* (cost, node) frontier */
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
  for (const leo::NodeId s : src_vis)
  {
    best[s] = dist_km(c, src, s); /* start each seed at its uplink cost */
    pq.push({best[s], s});
  }
  leo::NodeId reached = leo::INVALID_NODE; /* first dst-visible sat we hit */
  while (!pq.empty())
  {
    const auto [d, u] = pq.top();
    pq.pop();
    if (d > best[u])
    {
      continue; /* a stale heap entry, superseded by a shorter one */
    }
    if (is_dst_vis[u])
    {
      reached = u;
      break;
    }
    for (const leo::NodeId v : blue[u])
    {
      const double nd = d + dist_km(c, u, v); /* relax along a drawn link */
      if (nd < best[v])
      {
        best[v] = nd;
        came[v] = u;
        pq.push({nd, v});
      }
    }
  }

  const bool have_chain = reached != leo::INVALID_NODE;
  if (!have_chain)
  {
    /* Not a failure of the fix: mutual pairing caps blue degree at 2, so the
       blue graph is a set of short chains and rarely spans two ground stations
       on its own. Reported so the absence is not mistaken for a bug. */
    std::fprintf(out,
                 "  NO chain of drawn blue links connects a source-visible "
                 "satellite to a destination-visible one\n"
                 "  (source sees %zu sats, destination sees %zu)\n",
                 src_vis.size(), dst_vis.size());
  }

  if (have_chain)
  {
    std::vector<leo::NodeId> chain; /* the sats, source end first */
    for (leo::NodeId n = reached; n != leo::INVALID_NODE; n = came[n])
    {
      chain.push_back(n);
    }
    std::reverse(chain.begin(), chain.end());

    std::fprintf(out, "  chain of %zu satellites (catalog ids):\n   ",
                 chain.size());
    for (const leo::NodeId n : chain)
    {
      std::fprintf(out, " %u", static_cast<unsigned>(c.catalog_at(n)));
    }
    std::fprintf(out, "\n\n");

    /* The heart of it: per consecutive pair, what the renderer shows against
       what the router holds. A "yes / NO" line is the bug. */
    std::fprintf(out, "  %-7s %-7s %-5s %-5s %-6s %-6s %-8s %-6s %s\n", "from",
                 "to", "grpA", "grpB", "blue?", "edge?", "dist km", "gate?",
                 "structural wiring");
    std::size_t mismatches = 0; /* drawn but not routable: must stay zero */
    for (std::size_t k = 0; k + 1 < chain.size(); ++k)
    {
      const leo::NodeId a = chain[k];
      const leo::NodeId b = chain[k + 1];
      const bool drawn = is_blue(c, a, b);     /* what the user sees   */
      const bool edge = router.has_edge(a, b); /* what the router has  */
      if (drawn && !edge)
      {
        ++mismatches;
      }
      std::fprintf(out, "  %-7u %-7u %-5u %-5u %-6s %-6s %8.1f %-6s %s\n",
                   static_cast<unsigned>(c.catalog_at(a)),
                   static_cast<unsigned>(c.catalog_at(b)),
                   static_cast<unsigned>(c.grid_coord[a].plane),
                   static_cast<unsigned>(c.grid_coord[b].plane),
                   drawn ? "yes" : "no", edge ? "yes" : "NO", dist_km(c, a, b),
                   gate_ok(c, a, b, spec) ? "pass" : "FAIL",
                   slot_kind(c, a, b));
    }
    std::fprintf(out,
                 "\n  drawn-but-not-routable pairs on this chain: %zu of "
                 "%zu\n",
                 mismatches, chain.size() - 1);

    /* --- 3: CAN THE GROUND ACTUALLY REACH THE CHAIN? ----------------------
       An in-plane chain the ground cannot touch at BOTH ends is no alternative
       at all, however the ISLs are wired. The search started and ended on
       visible satellites, so this confirms rather than surprises -- it is here
       to rule the question out, not to reveal something. */
    std::fprintf(out,
                 "\n[3] GROUND REACHABILITY OF THAT CHAIN (min elev %.0f "
                 "deg)\n",
                 kMinElevationDeg);
    const leo::NodeId first = chain.front(); /* source end of the chain */
    const leo::NodeId last = chain.back();   /* destination end         */
    std::fprintf(out,
                 "  source   -> cat %u: elev %6.2f deg, %8.1f km  [%s]\n"
                 "  cat %u -> dest   : elev %6.2f deg, %8.1f km  [%s]\n",
                 static_cast<unsigned>(c.catalog_at(first)),
                 elevation_deg(c, src, first), dist_km(c, src, first),
                 router.has_edge(src, first) ? "uplink IS in the router graph"
                                             : "NOT in the router graph",
                 static_cast<unsigned>(c.catalog_at(last)),
                 elevation_deg(c, dst, last), dist_km(c, dst, last),
                 router.has_edge(dst, last) ? "uplink IS in the router graph"
                                            : "NOT in the router graph");
  }

  /* --- 4: THE WHOLE-CONSTELLATION TALLY -----------------------------------
     The chain above answers the question for one path; this answers it for the
     entire shell, and does not depend on picking the "right" chain. Three
     counts settle it outright: how many links the renderer DRAWS that the
     router cannot use; how many of the drawn links are wired into the in-plane
     slots at all (the only place the router's in-plane edges can come from);
     and how many wired pairs survive the feasibility gate -- i.e. how much
     in-plane connectivity the router genuinely has. */
  std::fprintf(out, "\n[4] WHOLE-SHELL TALLY: DRAWN vs ROUTABLE\n");
  std::size_t drawn_total = 0;         /* blue segments on screen    */
  std::size_t drawn_no_edge = 0;       /* ...that the router lacks   */
  std::size_t drawn_slot_adjacent = 0; /* ...wired into slots 0/1    */
  for (leo::NodeId i = 0; i < c.num_satellites; ++i)
  {
    for (const leo::NodeId j : blue[i])
    {
      if (j < i)
      {
        continue; /* count each undirected pair once, from its lower end */
      }
      ++drawn_total;
      if (!router.has_edge(i, j))
      {
        ++drawn_no_edge;
      }
      const char* kind = slot_kind(c, i, j); /* "in-plane" == slots 0/1 */
      if (kind[0] == 'i')
      {
        ++drawn_slot_adjacent;
      }
    }
  }
  std::size_t slot_pairs = 0;          /* wired in-plane pairs        */
  std::size_t slot_pairs_gated = 0;    /* ...passing the gate         */
  std::size_t slot_pairs_routable = 0; /* ...actually in the CSR      */
  for (leo::NodeId i = 0; i < c.num_satellites; ++i)
  {
    for (int s = 0; s < 2; ++s) /* slots 0,1 = fore/aft */
    {
      const leo::NodeId j = c.isl_neighbors[i][s];
      if (j == leo::INVALID_NODE || j < i)
      {
        continue; /* unlinked, or already counted from the other end */
      }
      ++slot_pairs;
      if (gate_ok(c, i, j, spec))
      {
        ++slot_pairs_gated;
      }
      if (router.has_edge(i, j))
      {
        ++slot_pairs_routable;
      }
    }
  }
  std::fprintf(out,
               "  blue links DRAWN by the renderer      : %zu\n"
               "    of those, NOT in the router's graph : %zu  <-- the bug if "
               "> 0\n"
               "    of those, slot-adjacent (slots 0/1) : %zu\n"
               "  structural in-plane pairs (slots 0/1) : %zu\n"
               "    passing the router's gate           : %zu\n"
               "    present in the router's graph       : %zu\n",
               drawn_total, drawn_no_edge, drawn_slot_adjacent, slot_pairs,
               slot_pairs_gated, slot_pairs_routable);

  /* --- 5: WHAT WOULD THE ROUTE BE IF THE DRAWN LINKS WERE ROUTABLE? -------
     The tally proves agreement (or disagreement); this PRICES it. Reconstruct
     the router's own edge set (slots 0..3 through the gate, plus uplinks as-is
     -- exactly Router::build's rule), check the reconstruction against has_edge
     so it is not taken on faith, then route again with the drawn blue links
     forced in. Any difference is what a disagreement would be costing. */
  std::fprintf(out, "\n[5] COST OF THE DISAGREEMENT\n");
  std::vector<std::vector<leo::NodeId>> adj(c.node_count()); /* rebuilt CSR */
  std::size_t rebuilt = 0;   /* edges the reconstruction produced */
  std::size_t confirmed = 0; /* ...that has_edge agrees exist     */
  const auto add_edge = [&adj](leo::NodeId a, leo::NodeId b)
  {
    adj[a].push_back(b);
    adj[b].push_back(a); /* the router's CSR is symmetric */
  };
  for (leo::NodeId i = 0; i < c.num_satellites; ++i)
  {
    for (int s = 0; s < 4; ++s) /* all four ISL terminals */
    {
      const leo::NodeId j = c.isl_neighbors[i][s];
      if (j == leo::INVALID_NODE || !gate_ok(c, i, j, spec))
      {
        continue; /* the router would have rejected it too */
      }
      add_edge(i, j);
      ++rebuilt;
      if (router.has_edge(i, j))
      {
        ++confirmed;
      }
    }
  }
  for (std::size_t g = 0; g < c.ground_uplinks.size(); ++g)
  {
    /* Uplinks are admitted as-is: they are already elevation-filtered, and
       earth_clear would wrongly reject an endpoint sitting on the surface. */
    const leo::NodeId gn = static_cast<leo::NodeId>(c.num_satellites + g);
    for (const leo::NodeId s : c.ground_uplinks[g])
    {
      add_edge(gn, s);
    }
  }
  std::fprintf(out,
               "  reconstructed %zu sat-sat edges; %zu confirmed by has_edge "
               "(%s)\n",
               rebuilt, confirmed,
               rebuilt == confirmed ? "reconstruction is exact"
                                    : "MISMATCH -- do not trust [5]");

  /* Dijkstra on light-time, the router's own weight. Run twice: as-is (which
     must reproduce the snapshot's cost, and does -- that is the check that this
     whole section is trustworthy) and again with the drawn links added. */
  const leo::RouteSpec rspec; /* hop_cost 0, exactly as refresh() routes */
  const auto route_cost = [&](std::vector<std::vector<leo::NodeId>>& graph,
                              std::vector<leo::NodeId>& path) -> double
  {
    std::vector<double> d(c.node_count(), inf); /* seconds from src */
    std::vector<leo::NodeId> prev(c.node_count(), leo::INVALID_NODE);
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> q;
    d[src] = 0.0;
    q.push({0.0, src});
    while (!q.empty())
    {
      const auto [du, u] = q.top();
      q.pop();
      if (du > d[u])
      {
        continue; /* stale entry */
      }
      if (u == dst)
      {
        break; /* settled the destination: nothing shorter can follow */
      }
      for (const leo::NodeId v : graph[u])
      {
        const double nd = du + leo::edge_seconds(c, u, v, rspec);
        if (nd < d[v])
        {
          d[v] = nd;
          prev[v] = u;
          q.push({nd, v});
        }
      }
    }
    path.clear();
    if (d[dst] == inf)
    {
      return inf; /* unreachable on this graph */
    }
    for (leo::NodeId n = dst; n != leo::INVALID_NODE; n = prev[n])
    {
      path.push_back(n);
    }
    std::reverse(path.begin(), path.end());
    return d[dst];
  };

  std::vector<leo::NodeId> base_path; /* route on the router's graph as-is */
  std::vector<leo::NodeId> aug_path;  /* route once the drawn links are in */
  const double base_s = route_cost(adj, base_path);
  for (leo::NodeId i = 0; i < c.num_satellites; ++i)
  {
    for (const leo::NodeId j : blue[i])
    {
      if (j > i && !router.has_edge(i, j))
      {
        add_edge(i, j); /* force in a drawn link the router would not use */
      }
    }
  }
  const double aug_s = route_cost(adj, aug_path);

  std::fprintf(out, "  router's graph as it is  : ");
  if (base_s == inf)
  {
    std::fprintf(out, "no route\n");
  }
  else
  {
    std::fprintf(out, "%zu hops, %.3f ms\n", base_path.size() - 1,
                 base_s * 1000.0);
  }
  std::fprintf(out, "  + the drawn blue links   : ");
  if (aug_s == inf)
  {
    std::fprintf(out, "no route\n");
  }
  else
  {
    std::fprintf(out, "%zu hops, %.3f ms\n", aug_path.size() - 1,
                 aug_s * 1000.0);
    std::fprintf(out, "  route with them added    :");
    for (const leo::NodeId n : aug_path)
    {
      std::fprintf(out, " %u",
                   static_cast<unsigned>(c.is_ground(n) ? 0 : c.catalog_at(n)));
    }
    std::fprintf(out, "\n");
  }
  std::fprintf(out,
               "=========================================================\n");
}

} /* namespace monitor */
