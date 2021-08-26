#ifndef HAKUBA_BUFFER_H
#define HAKUBA_BUFFER_H

#include <memory>
#include <vector>
#include <unordered_map>
#include "disk.h"

struct BufferId{
    size_t body{};

    BufferId() = default;
    explicit BufferId(size_t body_): body(body_){}
    BufferId( const BufferId &other ) = default;
};

typedef std::array<uint8_t, PAGE_SIZE> Page;

struct Buffer{
    PageId pageId{};
    Page page{};
    bool isDirty{false};

    Buffer()= default;
    Buffer(const Buffer &other) = delete;
};

struct Frame{
    uint64_t  usageCount{};
    std::shared_ptr<Buffer> buffer{};

    Frame(const Frame &other) = delete;
};

struct BufferPool{
  std::vector<Frame> buffers;
  BufferId nextVictimId;

  explicit BufferPool(size_t pool_size){
      std::vector<Frame> buffers_{};
      buffers_.resize(pool_size);
      buffers = std::move(buffers_);
      nextVictimId = BufferId();
  }

  size_t size() const{
      return buffers.size();
  }

  BufferId evict(){
    // Clock-sweep algorithm
    size_t pool_size = this->size();
    size_t consecutive_pinned = 0;
    BufferId victim_id;
    for(;;){
      BufferId next_victim_id = this->nextVictimId;
      Frame &frame = this->operator[](next_victim_id);
      if(frame.usageCount == 0){
        victim_id = nextVictimId;
        break;
      }
      if(frame.buffer.operator bool()){
        frame.usageCount--;
        consecutive_pinned = 0;
      }else{
        consecutive_pinned++;
        if(consecutive_pinned >= pool_size){
          assert(false && "Pool size exceeded!");
        }
      }
      this->nextVictimId = incrementId(nextVictimId);
    }
    return victim_id;
  }

  BufferId incrementId(BufferId bufferId) const{
    return BufferId((bufferId.body + 1) % size());
  }

  Frame& operator[](BufferId index){
      return this->buffers[index.body];
  }
};

struct BufferPoolManager{
  DiskManager disk;
  BufferPool pool;
  std::unordered_map<PageId, BufferId> pageTable{};

  BufferPoolManager(DiskManager &&disk_,
                    BufferPool &&pool_):
  disk(std::move(disk_)),
  pool(std::move(pool_)){}

  std::shared_ptr<Buffer> fetch_page(){

  }
};



#endif //HAKUBA_BUFFER_H
