#ifndef HAKUBA_TUPLE_H
#define HAKUBA_TUPLE_H

#include <vector>
#include <cstddef>
#include "memcmpable.h"

namespace tuple{
  void encode(std::vector<std::vector<uint8_t>> &elems,
              std::vector<uint8_t> &bytes){
    for(auto &elem: elems){
      auto len = memcmpable::encoded_size(elem.size());
      bytes.reserve(len);
      memcmpable::encode(elem.data(), elem.size(),bytes);
    }
  }

  void decode(const std::vector<uint8_t> &bytes,
              std::vector<std::vector<uint8_t>> &elems){
    std::vector<uint8_t> rest = bytes;
    size_t rest_len = rest.size();
    while (not rest.empty()){
      std::vector<uint8_t> elem{};
      memcmpable::decode(rest.data(), rest_len, elem);
      elems.push_back(elem);
    }
  }
}

#endif //HAKUBA_TUPLE_H
