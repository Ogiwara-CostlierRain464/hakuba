#ifndef HAKUBA_TABLE_LIST_H
#define HAKUBA_TABLE_LIST_H

#include "disk.h"
#include "layout_verified.h"

/**
 * This header should be initialized as 0.
 */
struct TableListHeader{
  uint16_t numTables{0};
  uint16_t occupySpaceOffset{0};
};

struct TableDescriber{
  PageId rootPageId{};

  explicit TableDescriber(PageId root_page_id)
    : rootPageId(root_page_id){}
};

class TableList{
public:
  typedef std::vector<std::reference_wrapper<uint8_t>> RefBytes;
  TableListHeader *header;
  RefBytes body;

  TableList(TableListHeader *header_, RefBytes body_)
  : header(header_), body(body_){}

  TableList(const TableList &other) = default;

  size_t freeSpace() const{
    return body.size() - header->occupySpaceOffset;
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
    header->occupySpaceOffset += sizeof(TableListHeader);
    header->numTables++;
    TableDescriber &describer = (*(describerArray().type))[next_describer_id];
    describer.rootPageId = root_page_id;
    return true;
  }

};

#endif //HAKUBA_TABLE_LIST_H
