#ifndef INVARIANT_TEST_CSV_HELPERS_H
#define INVARIANT_TEST_CSV_HELPERS_H

#include <vector>

#include "accuracy_test.h"

namespace invariant::test
{

void save_to_csv(const std::vector<Result>& results);

} // namespace invariant::test

#endif // INVARIANT_TEST_CSV_HELPERS_H