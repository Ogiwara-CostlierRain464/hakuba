#ifndef HAKUBA_TIMESERIES_DATABASE_H
#define HAKUBA_TIMESERIES_DATABASE_H

#include <ros/time.h>
#include <ros/serialization.h>
#include "db.h"
#include "tuple.h"

template<class T>
class TimeSeriesTable;

template<class T>
class TimeSeriesTableIterator: public std::iterator<std::input_iterator_tag,
  std::pair<ros::Time,T>>{
  friend TimeSeriesTable<T>;

private:
  TableIterator iterator{TableIterator::end()};

  TimeSeriesTableIterator();
  TimeSeriesTableIterator(const TableIterator &iter);
  TimeSeriesTableIterator(const TimeSeriesTableIterator<T> &other);

public:
  static TimeSeriesTableIterator end(){
    return TimeSeriesTableIterator<T>();
  }
  TimeSeriesTableIterator& operator++();
  std::pair<ros::Time,T> operator*();
  bool operator!=(const TimeSeriesTableIterator<T> &other);
  bool operator==(const TimeSeriesTableIterator<T> &other);
};

template<class T>
class TimeSeriesTable{
public:
  explicit TimeSeriesTable(const Table &table)
  : table(table){}

  void insert(const ros::Time &time, const T& data){
    namespace ser = ros::serialization;
    uint32_t key_serial_size = ser::serializationLength(time);
    std::vector<uint8_t> key_vec(key_serial_size);
    ser::OStream stream(key_vec.data(), key_serial_size);
    ser::serialize(stream, time);

    uint32_t val_serial_size = ser::serializationLength(data);
    std::vector<uint8_t> val_vec(val_serial_size);
    ser::OStream stream2(val_vec.data(), val_serial_size);
    ser::serialize(stream2, data);

    std::vector<std::vector<uint8_t>> bytes{};
    bytes.push_back(key_vec); bytes.push_back(val_vec);
    std::vector<uint8_t> record{};
    ::tuple::encode(bytes, record);

    table.insert(record);
  }

  bool search(const std::function<bool(const ros::Time&,const T&)> &fn,
              ros::Time &out_time,
              T &out_data){
    namespace ser = ros::serialization;
    Table::RefBytes out;
    bool found = table.search([&](const Table::RefBytes& record){
      std::vector<std::vector<uint8_t>> bytes{};
      std::vector<uint8_t> record_(record.begin(), record.end());
      ::tuple::decode(record_, bytes);

      ros::Time time;
      T data;
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

  TimeSeriesTableIterator<T> begin(){
    return TimeSeriesTableIterator<T>(table.begin());
  }

  TimeSeriesTableIterator<T> end(){
    return TimeSeriesTableIterator<T>();
  }

private:
  Table table;
};

/**
 * Wrapper class of DB.
 */
class TimeSeriesDB{
public:
  explicit TimeSeriesDB(const std::string &path)
  // 100MB
  : db(std::move(DB(path, 100'000'000 / PAGE_SIZE))){}

  template<class T>
  TimeSeriesTable<T> loadTable(size_t table_id){
    auto table = db.loadTable(table_id);
    return TimeSeriesTable<T>(table);
  }

  template<class T>
  std::pair<TimeSeriesTable<T>, size_t> createTable(){
    auto pair = db.createTable();
    return std::make_pair(TimeSeriesTable<T>(pair.first), pair.second);
  }

  void erase(){
    db.erase();
  }

private:
  DB db;
};

#endif //HAKUBA_TIMESERIES_DATABASE_H
