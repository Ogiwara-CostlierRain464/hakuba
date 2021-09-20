#include <gtest/gtest.h>
#include <ecl/threads.hpp>
#include <functional>
#include "../src/db/layout_verified.h"
#include "../src/db/disk.h"
#include "../src/db/buffer.h"
#include "../src/db/table.h"


using namespace std;

struct TestTable: public ::testing::Test{};

TEST_F(TestTable, table){

  PageId last_page_id;
  {
    DiskManager disk;
    DiskManager::open("/tmp/test3.data", disk);
    // 4096byte
    BufferPool pool(2);
    BufferPoolManager buf_mgr(std::move(disk),
                              std::move(pool));

    Table table(buf_mgr, PageId(0));
    for(size_t i = 0; i < 90; i++){
      std::vector<uint8_t> data(100,i+1);
      table.insert(data);
    }
    // wrong id!
    last_page_id = table.currentPageId;
  }
}