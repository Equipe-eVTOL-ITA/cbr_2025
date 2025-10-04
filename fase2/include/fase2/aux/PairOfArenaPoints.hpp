#pragma once

#include "fase2/aux/ArenaPoint.hpp"

class PairOfArenaPoints {
public:
    ArenaPoint base;
    ArenaPoint deliver;

    PairOfArenaPoints(ArenaPoint base, ArenaPoint deliver) : base(base), deliver(deliver) {}

};
