#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "graph.hpp"
#include "pathfinder.hpp"

// One independent link in a validation chain-of-responsibility. Each rule
// answers exactly one yes/no question about a candidate route - it doesn't
// know what other rules exist, doesn't know its position in the chain, and
// doesn't decide what happens after it runs (accept the route? try the
// next candidate? that's RouteValidationChain's job, below). That
// decoupling is the pattern's entire value here: rules can be added,
// removed, or reordered without any other rule's code changing, and each
// one is independently constructible and testable. See NOTES.md for why
// this earns being called chain-of-responsibility rather than "a function
// with if-statements."
class ValidationRule {
public:
    virtual ~ValidationRule() = default;

    // Returns true if `candidate` satisfies this rule. On rejection, if
    // `reason` is non-null, writes a short human-readable explanation.
    virtual bool check(const Graph& g, const PathResult& candidate, std::string* reason) const = 0;
    virtual std::string name() const = 0;
};

// Rejects any route that uses one of a fixed set of "closed" road segments.
// Segments are identified by (source OSM id, dest OSM id) rather than CSR
// edge index, since edge indices are an artifact of how Graph::load happens
// to bucket a given run's edges, not something a caller should have to
// know. An empty closure list (the default when nothing is passed on the
// CLI) makes this rule accept everything - there's no live road-closure
// feed behind it, see NOTES.md.
class NoClosedRoadsRule : public ValidationRule {
public:
    explicit NoClosedRoadsRule(std::vector<std::pair<int64_t, int64_t>> closed_osm_edges);
    bool check(const Graph& g, const PathResult& candidate, std::string* reason) const override;
    std::string name() const override { return "no_closed_roads"; }

private:
    std::vector<std::pair<int64_t, int64_t>> closed_osm_edges_;
};

// Rejects a route whose cost exceeds `max_multiple` times the cheapest
// candidate found for this query - a sanity bound against detours that are
// technically loopless and technically distinct but practically useless
// (Yen's algorithm makes no promise that its 2nd/3rd/... path is a *good*
// alternative, only that it's the next-cheapest loopless one).
class SanityBoundsRule : public ValidationRule {
public:
    SanityBoundsRule(double best_cost, double max_multiple);
    bool check(const Graph& g, const PathResult& candidate, std::string* reason) const override;
    std::string name() const override { return "sanity_bounds"; }

private:
    double best_cost_;
    double max_multiple_;
};

// Defensive structural check: the candidate must be non-empty and must
// actually start/end at the requested source/target. Yen's algorithm
// should never produce anything else, but the validator is written to not
// assume its input is trustworthy just because of where it came from -
// the same reasoning as Graph::load skipping malformed CSV rows instead of
// trusting the exporter never emits one.
class ReachabilityRule : public ValidationRule {
public:
    ReachabilityRule(int32_t source, int32_t target);
    bool check(const Graph& g, const PathResult& candidate, std::string* reason) const override;
    std::string name() const override { return "reachability"; }

private:
    int32_t source_;
    int32_t target_;
};

// Owns an ordered list of rules and runs a candidate through all of them,
// short-circuiting on the first rejection - the chain-of-responsibility
// behavior: each link gets a chance to reject before the next one runs, and
// the chain doesn't care which rule (if any) actually did the rejecting.
class RouteValidationChain {
public:
    void add_rule(std::unique_ptr<ValidationRule> rule);

    // Returns true (and leaves *reason untouched) if every rule accepted
    // the candidate. Returns false and fills *reason with "<rule
    // name>: <reason>" for the first rule that rejected it.
    bool validate(const Graph& g, const PathResult& candidate, std::string* reason = nullptr) const;

private:
    std::vector<std::unique_ptr<ValidationRule>> rules_;
};
