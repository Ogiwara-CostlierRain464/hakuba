#ifndef HAKUBA_MEMCMPABLE_H
#define HAKUBA_MEMCMPABLE_H

#include <cstddef>
#include <vector>

namespace memcmpable {
  const size_t ESCAPE_LENGTH = 9;

  size_t encoded_size(size_t len){
    return (len + (ESCAPE_LENGTH - 1)) / (ESCAPE_LENGTH - 1) * ESCAPE_LENGTH;
  }

  void encode(const uint8_t src[], size_t len, std::vector<uint8_t> &dst){
    for(;;){
      auto copy_len = std::min(ESCAPE_LENGTH-1, len);
//    dst = std::vector<uint8_t>(src, src+copy_len-1);
      std::copy(src, src+copy_len, std::back_inserter(dst));
      src = src+copy_len;
      len = len-copy_len;
      if(len == 0){
        auto pad_size = ESCAPE_LENGTH - 1 - copy_len;
        if(pad_size > 0){
          // do padding
          dst.resize(dst.size() + pad_size, 0);
        }
        dst.push_back(copy_len);
        break;
      }
      dst.push_back(ESCAPE_LENGTH);
    }
  }

  void decode(uint8_t src[], size_t &src_len, std::vector<uint8_t> &dst){
    for(;;){
      assert(src_len >= ESCAPE_LENGTH);
      uint8_t extra = src[ESCAPE_LENGTH-1];
      auto len_ = std::min(ESCAPE_LENGTH-1, (size_t) extra);
//    dst = std::vector<uint8_t>(src, src+len_-1);
      std::copy(src, src+len_, std::back_inserter(dst));
      // move elements in src
      for(size_t i = 0; i < src_len-ESCAPE_LENGTH; i++){
        src[i] = src[ESCAPE_LENGTH + i];
      }
      src_len = src_len-ESCAPE_LENGTH;

      if(extra < ESCAPE_LENGTH){
        break;
      }
    }
  }
}

#endif //HAKUBA_MEMCMPABLE_H
