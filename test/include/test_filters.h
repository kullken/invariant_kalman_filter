#pragma once

#include <string>

#include "iekf.h"


namespace invariant::test
{

struct TestFilter
{
    std::string description;
    invariant::IEKF filter;
};

inline std::ostream &operator<<(std::ostream &os, const TestFilter &param)
{
    return os << param.description;
}

} // namespace invariant::test
