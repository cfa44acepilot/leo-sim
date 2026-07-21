/*****************************************************************************
  filename route_diag.hpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator (monitor)
  date 2026-07-14
  Brief Description:
    A read-only comparison of what the ROUTER can use against what the RENDERER
    draws, for one route.

    It exists because those two once disagreed. The renderer used to derive
    in-plane links geometrically while the router built its in-plane edges from
    build_topology's slot-order wiring, and on real data the two sets were
    nearly disjoint -- the user could see a short blue link the router could not
    route over. The core now seeds slots 0/1 geometrically each tick, so the two
    agree by construction; this report is what proves it, and what would catch
    the disagreement coming back.

    The module only reports. It builds its OWN leo::Router from the same
    Constellation and LinkSpec the simulator uses (Router::build is a pure
    function of those), so nothing in src/ has to be touched or exposed.
 *****************************************************************************/

#ifndef LEO_MONITOR_ROUTE_DIAG_HPP
#define LEO_MONITOR_ROUTE_DIAG_HPP

#include <cstdio>  /* std::FILE -- the report goes to a stream, not a string */

#include "constellation.hpp"  /* Constellation: nodes, slots, uplinks         */
#include "isl_links.hpp"      /* LinkSpec: the feasibility gate being tested  */
#include "simulator.hpp"      /* Snapshot: the chosen route to explain        */

namespace monitor
{

/*---------------------------------------------------------------------------
  Function: print_route_diag
  Description: Explain a chosen route and test it against the in-plane links the
               renderer draws. Prints five sections: the chosen route hop by
               hop; the best chain of drawn blue links between the endpoints'
               visible satellites; that chain's ground reachability; a
               whole-shell tally of drawn-vs-routable links; and what the route
               would cost if every drawn link were routable. The tally is the
               decisive one -- it does not depend on picking the "right" chain.
  Input: c     -- the constellation, already propagated + linked this tick
         snap  -- the snapshot whose route is being explained
         spec  -- the SAME LinkSpec the simulator routed with
         out   -- destination stream (stdout in the headless path)
  Outputs: None; writes the report to `out`. Mutates nothing.
---------------------------------------------------------------------------*/
void print_route_diag(
    const leo::Constellation& c,  /* read-only: positions, slots, uplinks    */
    const leo::Snapshot& snap,    /* the route under explanation             */
    const leo::LinkSpec& spec,    /* the budget the router gated edges with  */
    std::FILE* out                /* where the report is written             */
);

}  /* namespace monitor */

#endif  /* LEO_MONITOR_ROUTE_DIAG_HPP */
