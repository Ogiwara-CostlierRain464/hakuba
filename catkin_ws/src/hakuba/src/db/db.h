#ifndef HAKUBA_DB_H
#define HAKUBA_DB_H

#include <string>
#include "table.h"

class DB{
public:
  DB(const std::string &path, size_t pool_size)
  : bufMgr(BufferPoolManager(
      DiskManager::open(path),
      BufferPool(pool_size)
    )),
    tableList(TableListRepo::fromPage(get_page_ref(
      (bufMgr.disk.nextPageId == 0
      ? bufMgr.createPage()
      : bufMgr.fetch_page(PageId(0))
      )->page
      ))),
    path(path)
    {}

  ~DB(){
    bufMgr.flush();
  }

  Table loadTable(size_t table_id){
    auto root_page_id = tableList.tableRootPageIdAt(table_id);
    return Table::fromExisting(bufMgr, root_page_id);
  }

  std::pair<Table, size_t> createTable(){
    auto new_buf = bufMgr.createPage();
    auto tmp = Table::fromNew(bufMgr, new_buf);
    size_t new_table_id;
    assert(tableList.tryPushBack(tmp.rootPageId, new_table_id) && "Can't create table any more");
    return std::make_pair(tmp, new_table_id);
  }

  void erase(){
    // clear file
    std::ofstream ofs;
    ofs.open(path, std::ofstream::out | std::ofstream::trunc);
    ofs.close();
    exit(0);
  }

private:
  std::string path;
  BufferPoolManager bufMgr;
  TableList tableList;
};

#endif //HAKUBA_DB_H
