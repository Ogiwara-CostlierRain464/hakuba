#ifndef HAKUBA_LAYOUT_VERIFIED_H
#define HAKUBA_LAYOUT_VERIFIED_H

#include <tuple>
#include <vector>

template<typename T>
struct LayoutVerified{
  typedef LayoutVerified<T> Self;
  typedef std::vector<uint8_t> Bytes;
  typedef std::vector<std::reference_wrapper<uint8_t>> RefBytes;
  typedef T Type;

  /**
   * `bytes` is array of 1 byte, so bytes.size() is exactly
   * same as byte count.
   */
  RefBytes bytes{};
  // Phantom Data
  T* type{};

  LayoutVerified() = default;
  LayoutVerified(const LayoutVerified<T> &other) = default;
  LayoutVerified(LayoutVerified<T> &&other) = default;
  LayoutVerified<T>& operator=(const LayoutVerified<T> &other) = default;
  LayoutVerified<T>& operator=(LayoutVerified<T> &&other) = default;


  /**
   * Check layout(memory size & alignment) and cast safely.
   * This is copied from zerocopy::LayoutVerified in Rust.
   * @param bytes_
   */
  explicit LayoutVerified(const RefBytes &bytes_){
    if(bytes_.size() != sizeof(T) or alignof(Bytes) % alignof(T) != 0){
      assert(false && "Alignment statement not satisfied!");
    }
    bytes = RefBytes(bytes_.begin(), bytes_.end());
    // unsafe
    type = reinterpret_cast<T*>(&bytes[0].get());
  }

  /**
   *
   * @param bytes
   * @param out
   */
  static std::pair<Self, RefBytes> newFromPrefix(const RefBytes &bytes){
    if ((bytes.size() < sizeof(T)) or (alignof(Bytes) % alignof(T) != 0)){
      assert(false && "Alignment statement not satisfied!");
    }

    RefBytes bytes2(bytes.begin(), bytes.begin() + sizeof(T));
    RefBytes suffix(bytes.begin() + sizeof(T), bytes.end());

    return std::make_pair(LayoutVerified<T>(bytes2), suffix);
  }

  static Self newSlice(const RefBytes &bytes){
    assert(sizeof(T) != 0);
    if(bytes.size() % sizeof(T) != 0 or alignof(Bytes) % alignof(T) != 0){
      assert(false && "Alignment statement not satisfied!");
    }

    return LayoutVerified<T>(bytes);
  }
};

#endif //HAKUBA_LAYOUT_VERIFIED_H

