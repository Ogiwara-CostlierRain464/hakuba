#include <gtest/gtest.h>
#include <ecl/threads.hpp>
#include <functional>
#include "../src/db/slotted.h"

using namespace std;

struct TestSlotted: public ::testing::Test{};

TEST_F(TestSlotted, test){
  std::vector<uint8_t> page_data(128, 0);
  std::vector<
    std::reference_wrapper<uint8_t>>
  ref(page_data.begin(), page_data.end());

  auto slotted = Slotted(ref);
  slotted.header.type->freeSpaceOffset = slotted.body.size();
  slotted.header.type->numSlots = 0;
  std::vector<uint8_t> hello{'h', 'e', 'l', 'l', 'o'};
  std::vector<std::reference_wrapper<uint8_t>> hello_ref(
    hello.begin(), hello.end());
  EXPECT_TRUE(slotted.tryPushBack(hello));
  std::vector<uint8_t> world{'w', 'o'};
  EXPECT_TRUE(slotted.tryPushBack(world));
  EXPECT_EQ(slotted.dataAt(0), hello_ref);
  EXPECT_EQ((*(slotted.pointers().type))[0].len, 5);
  EXPECT_EQ((*(slotted.pointers().type))[0].offset, 115);
  EXPECT_EQ((*(slotted.pointers().type))[1].len, 2);
  EXPECT_EQ((*(slotted.pointers().type))[1].offset, 113);

}
