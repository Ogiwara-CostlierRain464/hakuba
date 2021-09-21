#ifndef HAKUBA_REPO_H
#define HAKUBA_REPO_H

#include "buffer.h"
#include "index.h"
#include "layout_verified.h"
#include "table_list.h"

class HeapRepo{
public:
  using RefBytes = std::vector<std::reference_wrapper<uint8_t>>;

  static Heap fromPage(const RefBytes &page, bool init){
    using Layout = LayoutVerified<Heap::Header>;
    auto layout = Layout::newFromPrefix(page);
    auto tmp = Heap(layout.first.type, layout.second);

    if(init){
      *tmp.header = HeapHeader(0, tmp.body.size());
    }

    return tmp;
  }
};

class IndexNodeRepo{
public:
  using RefBytes = std::vector<std::reference_wrapper<uint8_t>>;
  /**
   * Create instance from page
   * You also do instance creation at here.
   * @param page
   * @param init
   * @return
   */
  static IndexNode fromPage(const RefBytes &page, bool init){
    using Layout = LayoutVerified<IndexNode::Header>;
    auto layout = Layout::newFromPrefix(page);
    auto heap = HeapRepo::fromPage(layout.second, init);
    IndexNode tmp(layout.first.type, heap);

    if(init){
      *tmp.header = IndexNodeHeader();
    }

    return tmp;
  }
};

class TableListRepo{
public:
  using RefBytes = std::vector<std::reference_wrapper<uint8_t>>;

  static TableList fromPage(const RefBytes &page){
    using Layout = LayoutVerified<TableListHeader>;
    auto layout = Layout::newFromPrefix(page);
    return TableList(layout.first.type, layout.second);
  }
};

#endif //HAKUBA_REPO_H
