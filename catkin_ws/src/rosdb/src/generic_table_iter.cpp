#include "generic_database.h"

using namespace std;

GenericTableIterator::GenericTableIterator()= default;

GenericTableIterator::GenericTableIterator(const TableIterator &iter)
  : iterator(iter){};

GenericTableIterator::GenericTableIterator(const GenericTableIterator &other)= default;

GenericTableIterator&
GenericTableIterator::operator++(){
  ++iterator;
  return *this;
}

std::pair<ros::Time, topic_tools::ShapeShifter> GenericTableIterator::operator*(){
  namespace ser = ros::serialization;

  auto ref_bytes = iterator.operator*();
  assert(!ref_bytes.empty() && "You can't call in this situation.");

  std::vector<std::vector<uint8_t>> bytes{};
  std::vector<uint8_t> record_(ref_bytes.begin(), ref_bytes.end());
  ::tuple::decode(record_, bytes);
  ros::Time time;
  topic_tools::ShapeShifter data;
  uint32_t key_serial_size = ser::serializationLength(time);
  ser::IStream key_stream(bytes[0].data(), key_serial_size);
  ser::deserialize(key_stream, time);

  uint32_t val_serial_size = ser::serializationLength(data);
  ser::IStream val_stream(bytes[1].data(), val_serial_size);
  ser::deserialize(val_stream, data);

  return std::make_pair(time, data);
}

bool GenericTableIterator::operator!=(const GenericTableIterator &other){
  return iterator.operator!=(other.iterator);
}

bool GenericTableIterator::operator==(const GenericTableIterator &other){
  return not this->operator!=(other);
}
