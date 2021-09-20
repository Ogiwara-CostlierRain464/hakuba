#ifndef HAKUBA_REPO_H
#define HAKUBA_REPO_H

#include "buffer.h"
#include "index.h"
#include "layout_verified.h"

class HeapRepo{
public:
  using RefBytes = std::vector<std::reference_wrapper<uint8_t>>;

  static Heap fromPage(RefBytes &page, bool init){
    using Layout = LayoutVerified<Heap::Header>;
    Layout::RefBytes ref(page.begin(), page.end());
    auto layout = Layout::newFromPrefix(ref);
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
  static IndexNode fromPage(RefBytes &page, bool init){
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

#endif //HAKUBA_REPO_H
