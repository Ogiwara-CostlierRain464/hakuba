#include <gtest/gtest.h>
#include <ecl/threads.hpp>
#include <functional>
#include "../src/db/index.h"
#include "../src/db/repo.h"

using namespace std;

struct TestIndex: public ::testing::Test{};

TEST_F(TestIndex, node){
  PageId page_id;

  {
    DiskManager disk;
    DiskManager::open("/tmp/test3.data", disk);
    BufferPool pool(10);
    BufferPoolManager buf_mgr(std::move(disk),
                              std::move(pool));

    // new page!
    auto buffer = buf_mgr.createPage();
    page_id = buffer->pageId;
    std::vector<std::reference_wrapper<uint8_t>>
      ref(buffer->page.begin(), buffer->page.end());

    auto node = IndexNodeRepo::fromPage(ref, true);
    ASSERT_EQ(node.body.header->freeSpaceOffset, 4096 - 16 -  8);
    std::vector<uint8_t> record{1,2,3};
    // Init
    ASSERT_TRUE(node.tryInsert(record));
    // After node operation, you have to mark
    // the page as dirty.
    buffer->setDirty();
    buf_mgr.flush();
  }

  {
    DiskManager disk;
    DiskManager::open("/tmp/test3.data", disk);
    BufferPool pool(10);
    BufferPoolManager buf_mgr(std::move(disk),
                              std::move(pool));

    // load existing page
    auto buffer = buf_mgr.fetch_page(page_id);
    std::vector<std::reference_wrapper<uint8_t>>
      ref(buffer->page.begin(), buffer->page.end());
    auto node = IndexNodeRepo::fromPage(ref, false);
    std::vector<std::reference_wrapper<uint8_t>> result;
    bool found = node.search([](
      const std::vector<std::reference_wrapper<uint8_t>> &record){
      return record.size() == 3;
    }, result);
    EXPECT_TRUE(found);
  }

}
