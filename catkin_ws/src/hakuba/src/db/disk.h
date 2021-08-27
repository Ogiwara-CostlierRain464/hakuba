#ifndef HAKUBA_DISK_H
#define HAKUBA_DISK_H

#include <cstddef>
#include <limits>
#include <fstream>
#include <iostream>
#include <array>

struct PageId{
    uint64_t body{};

    PageId() = default;

    explicit PageId(uint64_t body_): body(body_)
    {}

    static PageId INVALID_PAGE_ID;

    bool operator==(const PageId &rhs) const{
      return body == rhs.body;
    }
};

namespace std{
  template <>
  struct hash<PageId>{
    std::size_t operator()(const PageId &p) const{
      using std::size_t;
      using std::hash;
      using std::string;

      return hash<uint64_t>()(p.body);
    }
  };
}

PageId PageId::INVALID_PAGE_ID = PageId(std::numeric_limits<uint64_t>::max());

const uint64_t PAGE_SIZE = 4096;

struct DiskManager{
    std::fstream heapFile{};
    uint64_t nextPageId{};

    DiskManager(){}

private:
    DiskManager(std::fstream &&heap_file, uint64_t next_page_id)
    : heapFile(std::move(heap_file)), nextPageId(next_page_id){}

public:
    static void create(std::fstream &&heap_file, DiskManager &out){
        // getting file length
        heap_file.seekg(0, std::ios::beg);
        long begin = heap_file.tellg();
        heap_file.seekg(0, std::ios::end);
        long end = heap_file.tellg();
        uint64_t next_page_id = (end-begin) / PAGE_SIZE;
        // return to initial pos.
        heap_file.seekg(0, std::ios::beg);
        out = DiskManager(std::move(heap_file), next_page_id);
    }

    static void open(const std::string &heap_file_path, DiskManager &out){
        // By default, C++ fstream creates file if not exists.
        std::fstream heap_file;
        heap_file.open(heap_file_path,
                       std::ios::in | std::ios::out | std::ios::binary );
        assert(heap_file.is_open() && "Could not create disk file!");
        DiskManager::create(std::move(heap_file), out);
    }

    void readPageData(PageId pageId, std::array<uint8_t, PAGE_SIZE> data){
        uint64_t offset = PAGE_SIZE * pageId.body;
        assert(offset <= std::numeric_limits<int64_t>::max());
        heapFile.seekg((int64_t) offset, std::ios::beg);
        heapFile.read(reinterpret_cast<char *>(data.data()), PAGE_SIZE);
    }

    void writePageData(PageId pageId, std::array<uint8_t, PAGE_SIZE> data){
        uint64_t offset = PAGE_SIZE * pageId.body;
        assert(offset <= std::numeric_limits<int64_t>::max());
        heapFile.seekg((int64_t) offset, std::ios::beg);
        heapFile.write(reinterpret_cast<const char *>(data.data()), PAGE_SIZE);
    }

    PageId allocatePage(){
        nextPageId++;
        return PageId(nextPageId);
    }

    void sync(){
        heapFile.flush();
        heapFile.sync();
    }

    void drop(){
        heapFile.close();
    }
};

#endif //HAKUBA_DISK_H
