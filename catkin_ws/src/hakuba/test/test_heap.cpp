#include <gtest/gtest.h>
#include <ecl/threads.hpp>
#include <functional>
#include "../src/db/heap.h"
#include "../src/db/repo.h"

using namespace std;

struct TestHeap: public ::testing::Test{};


TEST_F(TestHeap, test){
  std::vector<uint8_t> page_data(128, 0);
  std::vector<
    std::reference_wrapper<uint8_t>>
    ref(page_data.begin(), page_data.end());

  auto heap = HeapRepo::fromPage(ref, true);
  EXPECT_EQ(heap.header->freeSpaceOffset, 128 - 8);

  std::vector<uint8_t> hello{'h', 'e', 'l', 'l', 'o'};
  std::vector<std::reference_wrapper<uint8_t>> hello_ref(hello.begin(), hello.end());
  EXPECT_TRUE(heap.tryPushBack(hello));
  std::vector<uint8_t> world{'w', 'o'};
  EXPECT_TRUE(heap.tryPushBack(world));
  EXPECT_EQ(heap.itemAt(0), hello_ref);
  EXPECT_EQ((*(heap.itemIdArray().type))[0].len, 5);
  EXPECT_EQ((*(heap.itemIdArray().type))[0].offset, 115);
  EXPECT_EQ((*(heap.itemIdArray().type))[1].len, 2);
  EXPECT_EQ((*(heap.itemIdArray().type))[1].offset, 113);
}

