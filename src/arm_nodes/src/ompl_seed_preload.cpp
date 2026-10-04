// Deterministic OMPL planning: pin OMPL's global RNG seed at process start.
//
// OMPL seeds its random number generator from the system clock the first time a
// generator is created, so the sampling-based planner (RRTConnect) takes a
// different — sometimes wildly different — path on every launch even for an
// identical scene. OMPL only honours ompl::RNG::setSeed() BEFORE the first
// generator exists; once planning has started it logs "Random number generation
// already started. Changing seed now will not lead to deterministic sampling."
// and ignores the call. That is exactly why a MoveIt planning-request adapter
// (which runs after the OMPL planner manager has already constructed an RNG)
// cannot make planning deterministic.
//
// This translation unit is built as a small shared library that is LD_PRELOADed
// into the move_group process (see arm_moveit_config/launch/move_group.launch.py).
// Its constructor runs at library-load time — before main(), and therefore before
// move_group constructs the OMPL planner manager and its first RNG — so the seed
// is pinned while OMPL is still pristine. The result: move_group plans the same
// trajectory for the same (start, goal, scene) on every run. Override the seed
// with the OMPL_SEED environment variable if needed.
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include <ompl/util/RandomNumbers.h>

namespace
{
__attribute__((constructor)) void pin_ompl_seed()
{
  std::uint_fast32_t seed = 42;
  if (const char * env = std::getenv("OMPL_SEED")) {
    char * end = nullptr;
    const unsigned long parsed = std::strtoul(env, &end, 10);
    if (end != env && parsed != 0UL) {
      seed = static_cast<std::uint_fast32_t>(parsed);
    }
  }
  ompl::RNG::setSeed(seed);
  std::fprintf(
    stderr, "[ompl_seed_preload] OMPL RNG seed pinned to %lu before first generator\n",
    static_cast<unsigned long>(seed));
}
}  // namespace
