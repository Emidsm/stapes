#pragma once

#include <string_view>

#include <gtest/gtest.h>

namespace stapes::test {

/// Declares which requirements a test verifies, as a comma-separated list. The value
/// lands in the test's JSON result, where tools/traceability.py reads it.
inline void verifies(std::string_view requirement_ids) {
  ::testing::Test::RecordProperty("requirement", std::string(requirement_ids));
}

}  // namespace stapes::test
