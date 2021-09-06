#ifndef HAKUBA_SLOTTED_H
#define HAKUBA_SLOTTED_H

#include "layout_verified.h"

struct SlottedHeader{
  uint16_t numSlots;
  uint16_t freeSpaceOffset;
  uint32_t _pad;
};

struct Pointer{
  uint16_t offset;
  uint16_t len;

  // return range
  // TODO: replace with custom Iter base Range class
  std::pair<size_t, size_t> range(){
    return std::make_pair(offset, offset+len);
  }
};

typedef LayoutVerified<std::vector<Pointer>> Pointers;

struct Slotted{
  using Layout = LayoutVerified<SlottedHeader>;

  Layout header;
  Layout::RefBytes body;

  explicit Slotted(const Layout::RefBytes &bytes){
    auto out = LayoutVerified<SlottedHeader>::newFromPrefix(bytes);
    header = std::move(out.first);
    body = std::move(out.second);
  }

  size_t capacity() const{
    return body.size();
  }

  size_t numSlots() const{
    // there should be some way to convert!
    return header.type->numSlots;
  }

  size_t freeSpace() const{
    return header.type->freeSpaceOffset;
  }

  size_t pointersSize() const{
    return sizeof(Pointer) * numSlots();
  }

  Pointers pointers(){
    Layout::RefBytes slice(body.begin(), body.begin() + pointersSize());
    return Pointers::newSlice(slice);
  }

  Layout::RefBytes data(Pointer pointer){
    auto range = pointer.range();
    return Layout::RefBytes(
      body.begin() + range.first,
      body.end() + range.second);
  }

  void push_back(){

  }

  Layout::RefBytes operator[](size_t index){
    return data(pointers().type->at(index));
  }
};


#endif //HAKUBA_SLOTTED_H
