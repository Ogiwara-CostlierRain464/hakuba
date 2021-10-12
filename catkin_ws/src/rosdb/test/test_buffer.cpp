#include <iostream>
#include <gtest/gtest.h>
#include "../src/buffer.h"

using namespace std;

struct BufferTest: public ::testing::Test{};

TEST_F(BufferTest, test){
  std::array<uint8_t, PAGE_SIZE> hello{1,2};
  std::array<uint8_t, PAGE_SIZE> world{3,4};
  auto disk = DiskManager::open("/tmp/test2.data");
  BufferPool pool(1);
  BufferPoolManager buf_mgr(std::move(disk), std::move(pool));

  auto buffer = buf_mgr.createPage();
  try{
    buf_mgr.createPage();
    FAIL() << "Expected std::range_error";
  }catch(std::range_error const & err){
    EXPECT_EQ(err.what(),std::string("Pool size exceeded!"));
  }catch(...){
    FAIL() << "Expected std::range_error";
  }
  buffer->page = hello;
  buffer->isDirty = true;
  auto page1_id = buffer->pageId;

  buffer = buf_mgr.fetch_page(page1_id);
  EXPECT_EQ(buffer->page, hello);


}
