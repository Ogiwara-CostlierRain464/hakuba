#include "table.h"
#include <cassert>

// This is equal to end of iter.
TableIterator::TableIterator()
  : table(nullptr),
    seekPageId(PageId::INVALID_PAGE_ID), itemIdIndex(0){}

TableIterator::TableIterator(const TableIterator& other)
=default;

TableIterator& TableIterator::operator++(){


  auto node =
    IndexNodeRepo::fromPage(get_page_ref(seekPageBuffer->page), false);

  itemIdIndex++;
  if(itemIdIndex == node.body.header->numItems){
    seekPageBuffer = nullptr;
    seekPageId = node.header->nextId;
    itemIdIndex = 0;
    if(seekPageId == PageId::INVALID_PAGE_ID){
      table = nullptr;
    }
  }
  return *this;
}

Table::RefBytes TableIterator::operator*(){
  if(seekPageId == PageId::INVALID_PAGE_ID){
    // end of iter.
    return Table::RefBytes{};
  }

  if(!seekPageBuffer){
    assert(seekPageId != PageId::INVALID_PAGE_ID);
    seekPageBuffer = table->bufMgr.get().fetch_page(seekPageId);
  }
  auto node =
    IndexNodeRepo::fromPage(get_page_ref(seekPageBuffer->page), false);
  assert(itemIdIndex < node.body.header->numItems);
  return node.body.itemAt(itemIdIndex);
}

bool TableIterator::operator!=(const TableIterator& other){
  return table != other.table
         or seekPageId != other.seekPageId
         or itemIdIndex != other.itemIdIndex;
}

bool TableIterator::operator==(const TableIterator& other){
  return !this->operator!=(other);
}