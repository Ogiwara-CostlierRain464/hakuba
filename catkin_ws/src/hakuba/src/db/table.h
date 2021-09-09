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
  bool tryInsert(const Layout::Bytes &record){
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

class Table{
public:
  typedef std::vector<std::reference_wrapper<uint8_t>> RefBytes;
  typedef std::vector<uint8_t> Record;

  // have BufferPoolMgr, and top PageId
  // constitute from nodes, and when one node filled,
  // then create new page and node.
  BufferPoolManager &bufMgr;
  PageId rootPageId;
  PageId currentPageId{rootPageId};

  explicit Table(BufferPoolManager &buf_mgr,
        PageId root_page_id)
  : bufMgr(buf_mgr), rootPageId(root_page_id)
  {}

  void insert(const Record &record){
    auto buffer = bufMgr.fetch_page(currentPageId);
    RefBytes ref(buffer->page.begin(), buffer->page.end());
    auto current_node = LinearListNode(ref);
    auto result = current_node.tryInsert(record);
    if(result){
      buffer->setDirty();
      //bufMgr.flush();
    }else{
      // When current page id is filled,
      // then move to next page.
      auto new_buffer = bufMgr.createPage();
      auto new_page_id = new_buffer->pageId;
      current_node.header.type->nextPageId = new_page_id;
      currentPageId = new_page_id;
      // or should we use goto?
      // TODO: set next page id of new_buffer as INVALID_PAGE_ID
      insert(record);
    }
  }

  bool search(const std::function<bool(const RefBytes&)> &fn, RefBytes &out){
    // search from root page. when not found within
    PageId seek_page_id = rootPageId;
    for(;;){
      auto buffer = bufMgr.fetch_page(rootPageId);
      RefBytes ref(buffer->page.begin(), buffer->page.end());
      auto node = LinearListNode(ref);
      RefBytes result;
      bool found = node.search(fn, out);
      if(found){
        return true;
      }else if(node.header.type->nextPageId != PageId::INVALID_PAGE_ID){
        seek_page_id = node.header.type->nextPageId;
        continue;
      }else{
        assert(node.header.type->nextPageId == PageId::INVALID_PAGE_ID);
        return false;
      }
    }
  }
};



#endif //HAKUBA_TABLE_H
