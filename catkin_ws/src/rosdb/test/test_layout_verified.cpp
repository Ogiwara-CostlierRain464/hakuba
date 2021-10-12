#include <gtest/gtest.h>
#include <ecl/threads.hpp>
#include <functional>
#include "../src/layout_verified.h"

using namespace std;

struct TestLayoutVerified: public ::testing::Test{};

struct alignas(8) A{
  uint16_t numSlots; // 2byte
  uint16_t freeSpaceOffset; // 2byte
};

TEST_F(TestLayoutVerified, layout_test){
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
}

TEST_F(TestLayoutVerified, test){
  using LV = LayoutVerified<std::array<uint8_t, 2>>;
  LV::Bytes buf{1,2,3,4};
  LV::RefBytes buf_ref(buf.begin(), buf.end());
  std::pair<LV, LV::RefBytes> out = LV::newFromPrefix(buf_ref);

  // verify vector splitting works.
  LV::Bytes tmp{3, 4};
  LV::RefBytes tmp_ref(tmp.begin(), tmp.end());
  EXPECT_EQ(out.second, tmp_ref);
  LV::Type tmp2{1,2};
  LV::Type *result = out.first.type;
  EXPECT_EQ(*result, tmp2);

  // verify source change affects to layout.
  buf[0] = 5;
  EXPECT_EQ(out.first.type->at(0), 5);
}

