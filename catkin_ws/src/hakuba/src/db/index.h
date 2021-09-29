#ifndef HAKUBA_INDEX_H
#define HAKUBA_INDEX_H

#include "buffer.h"
#include "heap.h"


struct IndexNodeHeader{
  // Page Id as identity.
  PageId nextId{PageId::INVALID_PAGE_ID};

  IndexNodeHeader() = default;
};

// Entity
class IndexNode{
public:
  using Header = IndexNodeHeader;
  typedef std::vector<std::reference_wrapper<uint8_t>> RefBytes;

  IndexNodeHeader* header;
  Heap body;

  IndexNode(IndexNodeHeader *header_,
            const Heap &body_)
            : header(header_), body(body_){}

  IndexNode(const IndexNode &other) = default;


  // node itself don't have any method!
  // it is just a container
  // how to handle search algorithm?
  bool tryInsert(const std::vector<uint8_t> &record){
    // Handle it's capacity by itself.
    // if capacity is over, notify insert has failed.
    return body.tryPushBack(record);
  }
  // after call this function, you should write data into DiskMgr.

  bool search(const std::function<bool(const RefBytes &)> &fn, RefBytes &out){
    for(size_t i = 0; i < body.header->numItems; i++){
      auto slot = body.itemAt(i);
      if(fn(slot)){
        out = slot;
        return true;
      }
    }
    return false;
  }
};

#endif //HAKUBA_INDEX_H
