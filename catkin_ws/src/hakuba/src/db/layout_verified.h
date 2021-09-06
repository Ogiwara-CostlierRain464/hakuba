#ifndef HAKUBA_LAYOUT_VERIFIED_H
#define HAKUBA_LAYOUT_VERIFIED_H

#include <tuple>
#include <vector>

template<typename T>
struct LayoutVerified{
  typedef std::vector<uint8_t> Bytes;
  typedef T Type;

  /**
   * `bytes` is array of 1 byte, so bytes.size() is exactly
   * same as byte count.
   */
  Bytes bytes{};
  // Phantom Data
  T type{};

  LayoutVerified() = default;

  explicit LayoutVerified(Bytes &&bytes_){
    if(bytes_.size() != sizeof(T) or alignof(Bytes) % alignof(T) != 0){
      assert(false && "Alignment statement not satisfied!");
    }
    bytes = std::move(bytes_);
    // unsafe
    type = *reinterpret_cast<const T*>(bytes.data());
  }

  /**
   *
   * @param bytes
   * @param out
   */
  static void newFromPrefix(Bytes &&bytes,
                              std::pair<LayoutVerified<T>, Bytes> &out){
    if ((bytes.size() < sizeof(T)) or (alignof(Bytes) % alignof(T) != 0)){
      assert(false && "Alignment statement not satisfied!");
    }

    Bytes bytes2(bytes.begin(), bytes.begin() + sizeof(T));
    Bytes suffix(bytes.begin() + sizeof(T), bytes.end());

    out = std::make_pair(LayoutVerified<T>(std::move(bytes2)), std::move(suffix));
  }

  static void newSlice(Bytes &&bytes,
                        LayoutVerified<T> &out){
    assert(sizeof(T) != 0);
    if(bytes.size() % sizeof(T) != 0 or alignof(Bytes) % alignof(T) != 0){
      assert(false && "Alignment statement not satisfied!");
    }

    out = LayoutVerified<T>(std::move(bytes));
  }
};

#endif //HAKUBA_LAYOUT_VERIFIED_H

