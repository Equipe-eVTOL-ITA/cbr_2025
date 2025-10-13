#pragma once

#include "fase4/aux/ArenaPoint.hpp"

class PairOfArenaPoints {
public:
    ArenaPoint base;
    ArenaPoint deliver;

    PairOfArenaPoints(ArenaPoint base, ArenaPoint deliver) : base(base), deliver(deliver) {}

};
