#ifndef HAKUBA_HEAP_H
#define HAKUBA_HEAP_H
#include <cstddef>
#include "layout_verified.h"

struct alignas(8) HeapHeader{
  uint16_t numItems{};
  uint16_t freeSpaceOffset{};

  HeapHeader(uint16_t num_items,
             uint16_t free_space_offset)
  : numItems(num_items),
  freeSpaceOffset(free_space_offset){}
};

struct ItemId{
  uint16_t offset;
  uint16_t len;

  ItemId(uint16_t offset_, uint16_t len_)
    : offset(offset_), len(len_){}

  // return range
  // TODO: replace with custom Iter base Range class
  std::pair<size_t, size_t> range(){
    return std::make_pair(offset, offset+len);
  }
};

struct Heap{
  typedef std::vector<std::reference_wrapper<uint8_t>> RefBytes;
  using Header = HeapHeader;

  HeapHeader *header;
  RefBytes body;

  Heap(HeapHeader *header_, RefBytes body_)
  : header(header_), body(body_){}

  Heap(const Heap &other) = default;

  size_t itemCounts() const{
    return header->numItems;
  }

  size_t freeSpace() const{
    return header->freeSpaceOffset - itemIdArraySize();
  }

  size_t itemIdArraySize() const{
    return sizeof(ItemId) * itemCounts();
  }

  LayoutVerified<ItemId[]> itemIdArray(){
    RefBytes slice(body.begin(), body.begin() + itemIdArraySize());
    return LayoutVerified<ItemId[]>::newSlice(slice);
  }

  RefBytes itemAt(ItemId itemId){
    auto range = itemId.range();
    // range: 31730, 31730+24914
    // how this is happening?
    return RefBytes(
      body.begin() + range.first,
      body.begin() + range.second
      );
  }

  RefBytes itemAt(size_t index){
    return itemAt(itemIdAt(index));
  }

  ItemId itemIdAt(size_t index){
    auto itemIds = itemIdArray().type;
    return (*(itemIds))[index];
  }


  /**
 * Try to push back data into this page
 * @param data
 * @return false if failed.
 */
  bool tryPushBack(const std::vector<uint8_t> &data){
    // alloc pointer and insert data
    if(freeSpace() < sizeof(ItemId) + data.size()){
      return false;
    }
    if(freeSpace() == 12){
      auto a = 1+1;
      assert(true);
    }

    auto next_item_id_index = itemCounts();
    header->freeSpaceOffset -= data.size();
    header->numItems += 1;
    ItemId &next_item_id = (*(itemIdArray().type))[next_item_id_index];
    next_item_id.offset = header->freeSpaceOffset;
    next_item_id.len = data.size();
    // copy data into page
    RefBytes destination = this->itemAt(next_item_id);
    assert(destination.size() == data.size() && "wrong impl");
    for(size_t i = 0; i < destination.size(); i++){
      destination[i].get() = data[i];
    }
    return true;
  }
};

#endif //HAKUBA_HEAP_H
