#ifndef HAKUBA_TABLE_H
#define HAKUBA_TABLE_H

#include <functional>
#include "disk.h"
#include "buffer.h"
#include "slotted.h"
#include "layout_verified.h"


// Index represented as linear list.
// each data are retrieved by seeking each page
// how to utilize page system?
// to make structure simple,
// Root is working as meta
// each node points to next node
// In KOBA789's impl, LayoutVerified is used.
// Just an easiest way to convert from struct to array is use C++ casting
// However, I'm not sure this works in another platform.
/**
 * KOBA789's implmentation is mixing domain entity and I/O
 * how can we separate this?
 * Also, KOBA789's impl can hold only one table!
 *
 * you need: Slotted, Pointer, Pointers, LayoutAligned.
 *
 * As same as B+tree node, LinearListNode have to handle Slotted.
 * Size of slot is depend on inserted element
 *
 * Also, in KOBA789's impl, you have to remember Page ID by your self.
 *
 * But anyway, at first, I impl this by single table.
 *
 * How to apply changes to buffer page?
 * Table insert -> Insert to last node(check doubling?)
 * If Last node has capacity, then just insert.
 * Else, create new node and insert to it.
 * Table handle's DiskBufferManager.
 * However, Linear List Node only handle's Buffer, which don't
 * handle I/O by itself.
 *
 * PageID0: TableListNode, which hold table information
 * when this is fulled, create new node.
 *
 *
 */

struct LinkedListHeader{
  PageId nextPageId;
};

struct LinearListNode{
  using Layout = LayoutVerified<LinkedListHeader>;
  Layout header;
  Slotted body;

private:
  explicit LinearListNode(const std::pair<Layout, Layout::RefBytes> &tmp)
  : header(tmp.first), body(Slotted(tmp.second))
  {}
public:
  explicit LinearListNode(const Layout::RefBytes &page)
  : LinearListNode(Layout::newFromPrefix(page)){}

  // node itself don't have any method!
  // it is just a container
  // how to handle search algorithm?
  bool try_insert(const Layout::Bytes &record){
    // Handle it's capacity by itself.
    // if capacity is over, notify insert has failed.
    return body.tryPushBack(record);
  }
  // after call this function, you should write data into DiskMgr.

  bool search(const std::function<bool(const Layout::RefBytes&)> &fn, Layout::RefBytes &out){
    for(size_t i = 0; i < body.header.type->numSlots; i++){
      auto slot = body.dataAt(i);
      if(fn(slot)){
        out = slot;
        return true;
      }
    }
    return false;
  }
};

struct Table{
  // have BufferPoolMgr, and top PageId
  // constitute from nodes, and when one node filled,
  // then create new page and node.
};



#endif //HAKUBA_TABLE_H
