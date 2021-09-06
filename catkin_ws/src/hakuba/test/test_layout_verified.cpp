#include <gtest/gtest.h>
#include <ecl/threads.hpp>
#include <functional>
#include "../src/db/layout_verified.h"

using namespace std;

struct TestLayoutVerified: public ::testing::Test{};

struct alignas(8) A{
  uint16_t numSlots; // 2byte
  uint16_t freeSpaceOffset; // 2byte
  uint32_t _pad; // 4byte
};

TEST_F(TestLayoutVerified, test){
  static_assert(sizeof(LayoutVerified<A>::Bytes) == 24, "");
  static_assert(alignof(LayoutVerified<A>::Bytes) == 8, "");

  static_assert(sizeof(int) == 4, "");
  static_assert(sizeof(uint16_t) == 2, "");
  static_assert(sizeof(int[4]) == 16, "");
  static_assert(sizeof(int[5]) == 20, "");
  static_assert(sizeof(int[6]) == 24, "");

  static_assert(sizeof(A) == 8, "");
  static_assert(alignof(A) == 8, "");

  LayoutVerified<A>::Bytes bytes{1,2,3};
  static_assert(sizeof(bytes) == 24, "");
  bytes.push_back(4);
  static_assert(sizeof(bytes) == 24, "");
  // sizeof(Bytes) is independent from it's contents.
  // At here I wanna

  // Because Bytes are array of uint8_t, we can use
  // .size as bytes length.
  using LV = LayoutVerified<std::array<uint8_t, 2>>;
  LV::Bytes buf{1,2,3,4};
  std::pair<LV, LV::Bytes> out;

  LV::newFromPrefix(std::move(buf), out);
  LV::Bytes tmp{3,4};
  EXPECT_EQ(out.second, tmp);
  LV::Type tmp2{1,2};
  EXPECT_EQ(out.first.type, tmp2);
}

