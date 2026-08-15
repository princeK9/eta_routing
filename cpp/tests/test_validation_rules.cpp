// Small standalone regression test for the validation rules, no framework
// needed (plain checks + iostream, consistent with this project's
// zero-new-dependency stance). Exists because SanityBoundsRule and
// ReachabilityRule didn't happen to trigger naturally in CLI demo runs
// (Yen's alternatives on this graph tend to stay within a few % of the
// best cost, and every demo query was reachable by construction) -
// NoClosedRoadsRule, by contrast, was already proven end-to-end via the
// CLI's --closed flag rejecting a real candidate. Run via `cmake --build`
// (target: test_validation_rules) then the resulting executable directly.
//
// Deliberately does NOT use assert(): this project builds Release by
// default (CMAKE_BUILD_TYPE=Release in CMakeLists.txt), which defines
// NDEBUG, which makes assert(...) a no-op - so any assert() wrapped around
// a call with a side effect (like `assert(rule.check(...))`) would silently
// never call it at all in this build. Learned that the hard way once
// already; every check below evaluates the call as its own statement
// first, then inspects the already-computed result.
#include <iostream>
#include <string>

#include "graph.hpp"
#include "route_validator.hpp"

namespace {
int g_failures = 0;

void expect(bool condition, const std::string& what) {
    if (condition) {
        std::cout << "  ok: " << what << "\n";
    } else {
        std::cout << "  FAIL: " << what << "\n";
        g_failures++;
    }
}
}  // namespace

int main() {
    Graph g;  // rules under test don't touch g's contents

    std::cout << "SanityBoundsRule:\n";
    {
        SanityBoundsRule rule(/*best_cost=*/1000.0, /*max_multiple=*/3.0);

        PathResult ok;
        ok.cost = 2500.0;  // 2.5x - under the bound
        std::string reason_ok;
        bool accepted = rule.check(g, ok, &reason_ok);
        expect(accepted == true, "accepts a 2.5x-cost candidate");

        PathResult too_far;
        too_far.cost = 3500.0;  // 3.5x - over the bound
        std::string reason_rejected;
        bool rejected_ok = rule.check(g, too_far, &reason_rejected);
        expect(rejected_ok == false, "rejects a 3.5x-cost candidate");
        expect(!reason_rejected.empty(), "gives a non-empty reason: \"" + reason_rejected + "\"");
    }

    std::cout << "ReachabilityRule:\n";
    {
        ReachabilityRule rule(/*source=*/10, /*target=*/20);

        PathResult ok;
        ok.path = {10, 15, 20};
        std::string reason_ok;
        bool accepted = rule.check(g, ok, &reason_ok);
        expect(accepted == true, "accepts a path that starts at source and ends at target");

        PathResult wrong_target;
        wrong_target.path = {10, 15, 21};  // doesn't end at 20
        std::string reason_wrong_target;
        bool rejected_wrong_target = rule.check(g, wrong_target, &reason_wrong_target);
        expect(rejected_wrong_target == false, "rejects a path ending at the wrong node");
        expect(!reason_wrong_target.empty(), "gives a non-empty reason: \"" + reason_wrong_target + "\"");

        PathResult empty;
        std::string reason_empty;
        bool rejected_empty = rule.check(g, empty, &reason_empty);
        expect(rejected_empty == false, "rejects an empty path");
    }

    std::cout << "RouteValidationChain:\n";
    {
        RouteValidationChain chain;
        chain.add_rule(std::make_unique<ReachabilityRule>(10, 20));
        chain.add_rule(std::make_unique<SanityBoundsRule>(1000.0, 3.0));

        PathResult bad;
        bad.path = {10, 99};   // fails reachability - chain should stop there
        bad.cost = 999999.0;   // would also fail sanity bounds, if the chain got that far
        std::string reason;
        bool result = chain.validate(g, bad, &reason);
        expect(result == false, "rejects a candidate that fails the first rule");
        expect(reason.rfind("reachability:", 0) == 0,
               "attributes the rejection to reachability, not sanity_bounds (proves short-circuiting): \"" + reason +
                   "\"");
    }

    if (g_failures == 0) {
        std::cout << "All validation rule checks passed.\n";
        return 0;
    }
    std::cout << g_failures << " check(s) FAILED.\n";
    return 1;
}
