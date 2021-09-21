#ifndef HAKUBA_DB_H
#define HAKUBA_DB_H

#include <string>
#include "table.h"

class DB{
  DiskManager disk
  BufferPool pool;

  DB(const std::string &path, size_t pool_size){

  }
};

#endif //HAKUBA_DB_H
