#ifndef HAKUBA_REPO_H
#define HAKUBA_REPO_H

#include "buffer.h"

class IndexNodeRepo{
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
    auto heap = heapFromPage(layout.second, init);
    IndexNode tmp(layout.first.type, heap);

    if(init){
      *tmp.header = IndexNodeHeader();
    }

    return tmp;
  }

  static Heap heapFromPage(RefBytes &page, bool init){
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

#endif //HAKUBA_REPO_H
