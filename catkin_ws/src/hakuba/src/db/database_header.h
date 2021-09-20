#ifndef HAKUBA_DATABASE_HEADER_H
#define HAKUBA_DATABASE_HEADER_H

#include "disk.h"
#include "layout_verified.h"

struct TableListHeader{
  uint16_t numTables{};
  uint16_t freeSpaceOffset{};

  TableListHeader(uint16_t num_tables,
                  uint16_t free_space_offset)
  : numTables(num_tables),
  freeSpaceOffset(free_space_offset){}
};

struct TableDescriber{
  PageId rootPageId{};

  explicit TableDescriber(PageId root_page_id)
    : rootPageId(root_page_id){}
};

class TableList{
  typedef std::vector<std::reference_wrapper<uint8_t>> RefBytes;
  TableListHeader *header;
  RefBytes body;

  TableList(TableListHeader *header_, RefBytes body_)
  : header(header_), body(body_){}

  TableList(const TableList &other) = default;

  size_t freeSpace() const{
    return header->freeSpaceOffset;
  }

  LayoutVerified<TableDescriber[]> describerArray(){
    RefBytes slice(body.begin(),
    body.begin() + sizeof(TableDescriber) * header->numTables);
    return LayoutVerified<TableDescriber[]>::newSlice(slice);
  }

  bool tryPushBack(PageId root_page_id){
    if(freeSpace() < sizeof(TableListHeader)){
      return false;
    }
    auto next_describer_id = header->numTables;
    header->freeSpaceOffset -= sizeof(TableListHeader);
    header->numTables++;
    TableDescriber &describer = (*(describerArray().type))[next_describer_id];
    describer.rootPageId = root_page_id;
    return true;
  }

};

#endif //HAKUBA_DATABASE_HEADER_H
