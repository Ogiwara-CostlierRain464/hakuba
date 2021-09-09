#include <gtest/gtest.h>
#include <ecl/threads.hpp>
#include <functional>
#include "../src/db/layout_verified.h"
#include "../src/db/disk.h"
#include "../src/db/buffer.h"
#include "../src/db/table.h"


using namespace std;

struct TestTable: public ::testing::Test{};

TEST_F(TestTable, node){
  PageId page_id;

  {
    DiskManager disk;
    DiskManager::open("/tmp/test3.data", disk);
    BufferPool pool(10);
    BufferPoolManager buf_mgr(std::move(disk),
                              std::move(pool));

    auto buffer = buf_mgr.createPage();
    page_id = buffer->pageId;
    std::vector<std::reference_wrapper<uint8_t>>
      ref(buffer->page.begin(), buffer->page.end());
    auto node = LinearListNode(ref);
    std::vector<uint8_t> record{1,2,3};
    don't forget to init Slotted header!
    ASSERT_TRUE(node.try_insert(record));
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

    auto buffer = buf_mgr.fetch_page(page_id);
    std::vector<std::reference_wrapper<uint8_t>>
      ref(buffer->page.begin(), buffer->page.end());
    auto node = LinearListNode(ref);
    std::vector<std::reference_wrapper<uint8_t>> result;
    bool found = node.search([](const std::vector<std::reference_wrapper<uint8_t>> &record){
      std::cout << record.size() << std::endl;
      return record.size() == 3;
    }, result);
//    EXPECT_TRUE(found);
  }

}