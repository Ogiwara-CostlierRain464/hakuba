#ifndef HAKUBA_LAYOUT_VERIFIED_H
#define HAKUBA_LAYOUT_VERIFIED_H

#include <tuple>
#include <vector>

template<typename T>
struct LayoutVerified{
  typedef std::vector<uint8_t> Bytes ;
  Bytes bytes{};
  // Phantom Data
  T type{};

  explicit LayoutVerified(Bytes &&bytes_){
    if(bytes_.size() != sizeof(T) or alignof(Bytes) % alignof(T) != 0){
      assert(false && "Alignment statement not satisfied!");
    }
    bytes = std::move(bytes_);
  }

  static void new_from_prefix(Bytes &&bytes,
                              std::pair<LayoutVerified<T>, Bytes> &out){
    if (!(bytes.size() < sizeof(T) or alignof(Bytes) % alignof(T) != 0)){
      assert(false && "Alignment statement not satisfied!");
    }
    std::vector<Bytes> bytes2(bytes.begin(), bytes.begin() + sizeof(T));
    std::vector<Bytes> suffix(bytes.begin() + sizeof(T), bytes.end());

    return std::make_pair(LayoutVerified<T>(std::move(bytes2)), std::move(suffix));
  }

  static void new_slice(Bytes &&bytes,
                        LayoutVerified<T> &out){
    assert(sizeof(T) != 0);
    if(bytes.size() % sizeof(T) != 0 or alignof(Bytes) % alignof(T) != 0){
      assert(false && "Alignment statement not satisfied!");
    }

    out = LayoutVerified<T>(std::move(bytes));
  }
};

struct Pointer{
  uint16_t offset;
  uint16_t len;

  // return range
  std::pair<size_t, size_t> range(){
    return std::make_pair(offset, offset+len);
  }
};

typedef LayoutVerified<std::vector<Pointer>> Pointers;

#endif //HAKUBA_LAYOUT_VERIFIED_H
