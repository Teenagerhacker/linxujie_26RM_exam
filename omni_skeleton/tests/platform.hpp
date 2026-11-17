#pragma once
// Host-only replacements for HAL decoration/assertions. Production algorithms stay linked.
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#define HW_COMPONENTS_TOOLS_SYSTEM_HPP_
#define HW_COMPONENTS_TOOLS_ASSERT_HPP_
#define HW_OPTIMIZE_O2_START
#define HW_OPTIMIZE_O2_END
#define HW_OPTIMIZE_O2
#define HW_ASSERT(expr, ...) assert(expr)
