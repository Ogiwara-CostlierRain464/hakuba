#ifndef HAKUBA_SLOTTED_H
#define HAKUBA_SLOTTED_H

#include "layout_verified.h"

struct alignas(8) SlottedHeader{
  uint16_t numSlots{}; // 2byte
  uint16_t freeSpaceOffset{}; // 2byte
};

struct Pointer{
  uint16_t offset;
  uint16_t len;

  Pointer(uint16_t offset_, uint16_t len_)
  : offset(offset_), len(len_){}

  // return range
  // TODO: replace with custom Iter base Range class
  std::pair<size_t, size_t> range(){
    return std::make_pair(offset, offset+len);
  }
};

typedef LayoutVerified<Pointer[]> Pointers;

struct Slotted{
  using Layout = LayoutVerified<SlottedHeader>;

  Layout header;
  Layout::RefBytes body;

  Slotted(const Layout::RefBytes &bytes){
    auto layout = Layout::newFromPrefix(bytes);
    header = layout.first;
    body = layout.second;
  }

  // Call this when created from new page.
  void init(){
    header.type->numSlots = 0;
    header.type->freeSpaceOffset = body.size();
  }

  size_t capacity() const{
    return body.size();
  }

  size_t numSlots() const{
    return header.type->numSlots;
  }

  size_t freeSpace() const{
    return header.type->freeSpaceOffset;
  }

  size_t pointersSize() const{
    return sizeof(Pointer) * numSlots();
  }

  Pointers pointers() const{
    Layout::RefBytes slice(body.begin(), body.begin() + pointersSize());
    return Pointers::newSlice(slice);
  }

  Pointer pointerAt(size_t index) const{
    return (*(pointers().type))[index];
  }

  Layout::RefBytes data(Pointer pointer){
    auto range = pointer.range();
    return Layout::RefBytes(
      body.begin() + range.first,
      body.begin() + range.second);
  }

  /**
   * Try to push back data into this page
   * @param data
   * @return false if failed.
   */
  bool tryPushBack(const std::vector<uint8_t> &data){
    // alloc pointer and insert data
    if(freeSpace() < sizeof(Pointer) + data.size()){
      return false;
    }
    auto next_pointer_index = numSlots();
    header.type->freeSpaceOffset -= data.size();
    header.type->numSlots += 1;
    Pointer &next_pointer = (*(pointers().type))[next_pointer_index];
    next_pointer.offset = header.type->freeSpaceOffset;
    next_pointer.len = data.size();
    // copy data into page
    Layout::RefBytes destination = this->data(next_pointer);
    assert(destination.size() == data.size() && "wrong impl");
    for(size_t i = 0; i < destination.size(); i++){
      destination[i].get() = data[i];
    }
    return true;
  }

  Layout::RefBytes dataAt(size_t index){
    auto pointers_ = pointers().type;
    return data((*(pointers_))[index]);
  }
};


#endif //HAKUBA_SLOTTED_H
