#ifndef ROSDB_GENERIC_DATABASE_H
#define ROSDB_GENERIC_DATABASE_H


#include <ros/time.h>
#include <ros/serialization.h>
#include <topic_tools/shape_shifter.h>
#include "db.h"
#include "tuple.h"

class GenericTable;

class GenericTableIterator: public std::iterator<std::input_iterator_tag,
  std::pair<ros::Time, topic_tools::ShapeShifter >>{
  friend GenericTable;

private:
  TableIterator iterator{TableIterator::end()};

  GenericTableIterator();
  explicit GenericTableIterator(const TableIterator &iter);

public:
  GenericTableIterator(const GenericTableIterator &other);

  static GenericTableIterator end(){
    return GenericTableIterator();
  }
  GenericTableIterator& operator++();
  std::pair<ros::Time,  topic_tools::ShapeShifter > operator*();
  bool operator!=(const GenericTableIterator &other);
  bool operator==(const GenericTableIterator &other);
};

class GenericTable{
public:
  explicit GenericTable(const Table &table)
    : table(table){}

  void insert(const ros::Time &time,
              const topic_tools::ShapeShifter &data){
    namespace ser = ros::serialization;
    uint32_t key_serial_size = ser::serializationLength(time);
    std::vector<uint8_t> key_vec(key_serial_size);
    ser::OStream stream(key_vec.data(), key_serial_size);
    ser::serialize(stream, time);

    std::vector<uint8_t> val_vec(data.size());
    ser::OStream stream2(val_vec.data(), val_vec.size());
    data.write(stream2);

    std::vector<std::vector<uint8_t>> bytes{};
    bytes.push_back(key_vec); bytes.push_back(val_vec);
    std::vector<uint8_t> record{};
    ::tuple::encode(bytes, record);

    table.insert(record);
  }

  bool search(const std::function<bool(const ros::Time&,const topic_tools::ShapeShifter&)> &fn,
              ros::Time &out_time,
              topic_tools::ShapeShifter &out_data){
    namespace ser = ros::serialization;
    Table::RefBytes out;
    bool found = table.search([&](const Table::RefBytes& record){
      std::vector<std::vector<uint8_t>> bytes{};
      std::vector<uint8_t> record_(record.begin(), record.end());
      ::tuple::decode(record_, bytes);

      ros::Time time;
      topic_tools::ShapeShifter data;
      uint32_t key_serial_size = ser::serializationLength(time);
      ser::IStream key_stream(bytes[0].data(), key_serial_size);
      ser::deserialize(key_stream, time);

      uint32_t val_serial_size = ser::serializationLength(data);
      ser::IStream val_stream(bytes[1].data(), val_serial_size);
      ser::deserialize(val_stream, data);

      return fn(time, data);
    }, out);

    if(found){
      std::vector<std::vector<uint8_t>> bytes{};
      std::vector<uint8_t> record_(out.begin(), out.end());
      ::tuple::decode(record_, bytes);

      uint32_t key_serial_size = ser::serializationLength(out_time);
      ser::IStream key_stream(bytes[0].data(), key_serial_size);
      ser::deserialize(key_stream, out_time);

      uint32_t val_serial_size = ser::serializationLength(out_data);
      ser::IStream val_stream(bytes[1].data(), val_serial_size);
      ser::deserialize(val_stream, out_data);

      return true;
    }else{
      return false;
    }
  }

  GenericTableIterator begin(){
    return GenericTableIterator(table.begin());
  }

  GenericTableIterator end(){
    return GenericTableIterator();
  }

private:
  Table table;
};

class GenericDB{
public:
  explicit GenericDB(const std::string &path,
                     size_t pool_size =  100'000'000 / PAGE_SIZE)
  // 100MB
    : db(std::move(DB(path, pool_size))){}

  GenericTable loadTable(size_t table_id){
    auto table = db.loadTable(table_id);
    return GenericTable(table);
  }

  std::pair<GenericTable, size_t> createTable(){
    auto pair = db.createTable();
    return std::make_pair(GenericTable(pair.first), pair.second);
  }

  void erase(){
    db.erase();
  }

  void flush(){
    db.flush();
  }

private:
  DB db;
};

#endif //ROSDB_GENERIC_DATABASE_H
