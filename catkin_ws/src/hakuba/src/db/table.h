#ifndef HAKUBA_TABLE_H
#define HAKUBA_TABLE_H

#include <functional>
#include <iterator>
#include "disk.h"
#include "buffer.h"
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

class Table;

class TableIterator: public std::iterator<std::input_iterator_tag,
  std::vector<std::reference_wrapper<uint8_t>>>{
  friend Table;
private:
  PageId seekPageId;
  std::shared_ptr<Buffer> seekPageBuffer{nullptr};
  size_t itemIdIndex;
  Table* table;

  TableIterator();

  TableIterator(Table* table,
                PageId seek_page_id,
                size_t item_id_index)
  : table(table),
    seekPageId(seek_page_id) , itemIdIndex(item_id_index){}


public:
  TableIterator(const TableIterator& other);

  TableIterator& operator++();
  std::vector<std::reference_wrapper<uint8_t>> operator*();
  bool operator!=(const TableIterator& other);
  bool operator==(const TableIterator& other);
};

class Table{
  friend TableIterator;
public:
  typedef std::vector<std::reference_wrapper<uint8_t>> RefBytes;
  typedef std::vector<uint8_t> Record;

  // have BufferPoolMgr, and top PageId
  // constitute from nodes, and when one node filled,
  // then create new page and node.
  std::reference_wrapper<BufferPoolManager> bufMgr;
  PageId rootPageId;
  PageId currentPageId{rootPageId};
  std::shared_ptr<Buffer> currentPage{nullptr};

  // Do copy!
  Table(const Table &other) = default;

private:
  explicit Table(BufferPoolManager &buf_mgr,
        PageId root_page_id)
  : bufMgr(buf_mgr), rootPageId(root_page_id)
  {}

public:
  static Table fromNew(BufferPoolManager &buf_mgr,
                      std::shared_ptr<Buffer> &buffer){
    // Do init
    auto root_index =
      IndexNodeRepo::fromPage(get_page_ref(buffer->page), true);
    Table table(buf_mgr, buffer->pageId);
    table.currentPage = buffer;
    return table;
  }

  static Table fromExisting(BufferPoolManager &buf_mgr,
                           PageId root_page_id){
    return Table(buf_mgr, root_page_id);
  }

  void insert(const Record &record){
    if(!currentPage){
      currentPage = bufMgr.get().fetch_page(currentPageId);
    }
    auto current_node =
      IndexNodeRepo::fromPage(get_page_ref(currentPage->page), false);
    auto result = current_node.tryInsert(record);
    if(result){
      currentPage->setDirty();
      //bufMgr.flush();
    }else{
      // When current page id is filled,
      // then move to next page.
      auto new_buffer = bufMgr.get().createPage();
      RefBytes new_ref(new_buffer->page.begin(), new_buffer->page.end());
      auto new_node = IndexNodeRepo::fromPage(new_ref, true);
      current_node.header->nextId = new_buffer->pageId;
      currentPageId = new_buffer->pageId;
      currentPage = new_buffer;
      insert(record);
    }
  }

  bool search(const std::function<bool(const RefBytes&)> &fn, RefBytes &out){
    // search from root page. when not found within
    PageId seek_page_id = rootPageId;
    for(;;){
      auto buffer = bufMgr.get().fetch_page(seek_page_id);
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

  TableIterator begin(){
    return TableIterator(this, rootPageId, 0);
  }

  TableIterator end(){
    return TableIterator();
  }
};



#endif //HAKUBA_TABLE_H
