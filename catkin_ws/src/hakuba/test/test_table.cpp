#include <gtest/gtest.h>
#include <ecl/threads.hpp>
#include <functional>
#include "../src/db/layout_verified.h"
#include "../src/db/disk.h"
#include "../src/db/buffer.h"
#include "../src/db/table.h"
#include "../src/db/table_list.h"


using namespace std;

struct TestTable: public ::testing::Test{};

TEST_F(TestTable, table){
  {
    DiskManager disk = DiskManager::open("/tmp/db.data");
    BufferPool pool(2);
    BufferPoolManager buf_mgr(std::move(disk),
                              std::move(pool));
    auto buffer = buf_mgr.createPage();
    auto table_list =
      TableListRepo::fromPage(get_page_ref(buffer->page));
    size_t table_id;
    ASSERT_TRUE(table_list.tryPushBack(PageId(1), table_id));
    ASSERT_EQ(table_id, 0);
    auto buffer1 = buf_mgr.createPage();
    auto table = Table::fromNew(buf_mgr, buffer1);
    for(size_t i = 0; i < 40; i++){
      std::vector<uint8_t> data(100,i+1);
      table.insert(data);
    }

    EXPECT_EQ(table.currentPageId, PageId(1));
    buf_mgr.flush();
  }

  {
    DiskManager disk = DiskManager::open("/tmp/db.data");
    BufferPool pool(2);
    BufferPoolManager buf_mgr(std::move(disk),
                              std::move(pool));
    auto buf = buf_mgr.fetch_page(PageId(1));
    auto index = IndexNodeRepo::fromPage(get_page_ref(buf->page), false);
    EXPECT_EQ(index.body.itemAt(0)[0].get(), 1);
  }

  // clear file
  std::ofstream ofs;
  ofs.open("/tmp/db.data", std::ofstream::out | std::ofstream::trunc);
  ofs.close();
}