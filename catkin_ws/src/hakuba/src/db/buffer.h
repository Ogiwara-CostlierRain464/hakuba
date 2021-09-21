#ifndef HAKUBA_BUFFER_H
#define HAKUBA_BUFFER_H

#include <memory>
#include <vector>
#include <unordered_map>
#include <stdexcept>
#include "disk.h"

struct BufferId{
    size_t body{};

    BufferId() = default;
    explicit BufferId(size_t body_): body(body_){}
    BufferId( const BufferId &other ) = default;
};

typedef std::array<uint8_t, PAGE_SIZE> Page;

static std::vector<std::reference_wrapper<uint8_t>> get_page_ref(Page &page){
  return std::vector<std::reference_wrapper<uint8_t>>(page.begin(), page.end());
}

struct Buffer{
    PageId pageId{};
    Page page{};
    bool isDirty{false};

    Buffer()= default;
    Buffer(const Buffer &other) = delete;

    void setDirty(){
      isDirty = true;
    }
};

struct Frame{
    uint64_t usageCount{};
    std::shared_ptr<Buffer> buffer{std::make_shared<Buffer>()};

    Frame() = default;
    Frame(const Frame &other) = delete;
    Frame(Frame &&other) = default;
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
      // no place except here uses this buffer!
      if(frame.buffer.use_count() == 1){
        frame.usageCount--;
        consecutive_pinned = 0;
      }else{
        consecutive_pinned++;
        if(consecutive_pinned >= pool_size){
//          assert(false && "Pool size exceeded!");
          throw std::range_error("Pool size exceeded!");
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

  std::shared_ptr<Buffer> fetch_page(PageId page_Id){
    if(pageTable.find(page_Id) != pageTable.end()){
      BufferId buffer_Id = pageTable.at(page_Id);
      Frame &frame = pool[buffer_Id];
      frame.usageCount++;
      return frame.buffer;
    }
    auto buffer_Id = pool.evict();
    Frame &frame = pool[buffer_Id];
    auto evict_page_id = frame.buffer->pageId;
    {
      auto &buffer = frame.buffer;
      if(buffer->isDirty){
        this->disk.writePageData(evict_page_id, buffer->page);
      }
      buffer->pageId = page_Id;
      buffer->isDirty = false;
      this->disk.readPageData(page_Id, buffer->page);
      frame.usageCount = 1;
    }
    auto page = frame.buffer;
    this->pageTable.erase(evict_page_id);
    this->pageTable[page_Id] = buffer_Id;
    return page;
  }

  std::shared_ptr<Buffer> createPage(){
    auto buffer_id = pool.evict();
    auto &frame = pool[buffer_id];
    auto evict_page_id = frame.buffer->pageId;
    auto &buffer = frame.buffer;
    if(buffer->isDirty){
      disk.writePageData(evict_page_id, buffer->page);
    }
    pageTable.erase(evict_page_id);
    auto page_id = disk.allocatePage();
    *buffer = std::move(Buffer());
    // is it sure to dealloc array?
    buffer->pageId = page_id;
    buffer->isDirty = true;
    frame.usageCount = 1;
    auto page = frame.buffer;
    pageTable.erase(evict_page_id);
    pageTable[page_id] = buffer_id;
    return page;
  }

  void flush(){
    for(auto &pair : pageTable){
      PageId page_id = pair.first;
      BufferId buffer_id = pair.second;
      auto &frame = pool[buffer_id];
      auto &page = frame.buffer->page;
      // Why not only dirty page?
      if(frame.buffer->isDirty){
        disk.writePageData(page_id, page);
        frame.buffer->isDirty = false;
      }
    }
    disk.sync();
  }
};



#endif //HAKUBA_BUFFER_H
