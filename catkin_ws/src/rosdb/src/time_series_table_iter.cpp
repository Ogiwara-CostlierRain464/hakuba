#include "timeseries_database.h"

using namespace std;


template<class T>
TimeSeriesTableIterator<T>::TimeSeriesTableIterator()= default;

template<class T>
TimeSeriesTableIterator<T>::TimeSeriesTableIterator(const TableIterator &iter)
: iterator(iter){};

template<class T>
TimeSeriesTableIterator<T>::TimeSeriesTableIterator(const TimeSeriesTableIterator<T> &other)= default;

template<class T>
TimeSeriesTableIterator<T>&
  TimeSeriesTableIterator<T>::operator++(){
  ++iterator;
  return *this;
}

template<class T>
std::pair<ros::Time,T> TimeSeriesTableIterator<T>::operator*(){
  namespace ser = ros::serialization;

  auto ref_bytes = iterator.operator*();
  assert(!ref_bytes.empty() && "You can't call in this situation.");

  std::vector<std::vector<uint8_t>> bytes{};
  std::vector<uint8_t> record_(ref_bytes.begin(), ref_bytes.end());
  ::tuple::decode(record_, bytes);
  ros::Time time;
  T data;
  uint32_t key_serial_size = ser::serializationLength(time);
  ser::IStream key_stream(bytes[0].data(), key_serial_size);
  ser::deserialize(key_stream, time);

  uint32_t val_serial_size = ser::serializationLength(data);
  ser::IStream val_stream(bytes[1].data(), val_serial_size);
  ser::deserialize(val_stream, data);

  return std::make_pair(time, data);
}

template<class T>
bool TimeSeriesTableIterator<T>::operator!=(const TimeSeriesTableIterator<T> &other){
  return iterator.operator!=(other.iterator);
}

template<class T>
bool TimeSeriesTableIterator<T>::operator==(const TimeSeriesTableIterator<T> &other){
  return not this->operator!=(other);
}
