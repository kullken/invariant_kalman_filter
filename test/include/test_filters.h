#pragma once

#include <string>

#include "iekf.h"


namespace invariant::test
{

struct FilterNamed
{
    std::string name;
    invariant::IEKF filter;
};

inline std::ostream &operator<<(std::ostream &os, const FilterNamed &param)
{
    return os << param.name;
}

} // namespace invariant::test
