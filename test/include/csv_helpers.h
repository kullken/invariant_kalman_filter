#ifndef INVARIANT_TEST_CSV_HELPERS_H
#define INVARIANT_TEST_CSV_HELPERS_H

#include <string_view>
#include <vector>

#include "accuracy_test.h"

namespace invariant::test
{

void save_to_csv(const std::vector<Result>& results, std::string_view file_name_prefix="");

} // namespace invariant::test

#endif // INVARIANT_TEST_CSV_HELPERS_H