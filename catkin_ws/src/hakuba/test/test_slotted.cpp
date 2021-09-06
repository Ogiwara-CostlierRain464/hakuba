#include <gtest/gtest.h>
#include <ecl/threads.hpp>
#include <functional>
#include "../src/db/slotted.h"

using namespace std;

struct TestSlotted: public ::testing::Test{};

TEST_F(TestSlotted, test){
  std::vector<uint8_t> page_data(128, 8);

}
