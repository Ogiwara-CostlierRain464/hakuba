#ifndef HAKUBA_TABLE_H
#define HAKUBA_TABLE_H

#include <functional>
#include "disk.h"
#include "buffer.h"
#include "slotted.h"
#include "layout_verified.h"
#include "index.h"
#include "repo.h"


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
    auto current_node = IndexNodeRepo::fromPage(ref, false);
    auto result = current_node.tryInsert(record);
    if(result){
      buffer->setDirty();
      //bufMgr.flush();
    }else{
      // When current page id is filled,
      // then move to next page.
      auto new_buffer = bufMgr.createPage();
      RefBytes new_ref(new_buffer->page.begin(), new_buffer->page.end());
      auto new_node = IndexNodeRepo::fromPage(new_ref, true);
      new_node.header->nextId = new_buffer->pageId;
      currentPageId = new_buffer->pageId;
      insert(record);
    }
  }

  bool search(const std::function<bool(const RefBytes&)> &fn, RefBytes &out){
    // search from root page. when not found within
    PageId seek_page_id = rootPageId;
    for(;;){
      auto buffer = bufMgr.fetch_page(rootPageId);
      RefBytes ref(buffer->page.begin(), buffer->page.end());
      auto node = IndexNodeRepo::fromPage(ref, false);
      RefBytes result;
      bool found = node.search(fn, out);
      if(found){
        return true;
      }else if(node.header->nextId != PageId::INVALID_PAGE_ID){
        seek_page_id = node.header->nextId;
        continue;
      }else{
        assert(node.header->nextId == PageId::INVALID_PAGE_ID);
        return false;
      }
    }
  }
};



#endif //HAKUBA_TABLE_H
