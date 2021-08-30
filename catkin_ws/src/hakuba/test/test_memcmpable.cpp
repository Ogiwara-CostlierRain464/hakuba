#include <gtest/gtest.h>
#include <ecl/threads.hpp>
#include <functional>
#include "../src/db/memcmpable.h"

using namespace std;

struct TestMemcmpable: public ::testing::Test{};

TEST_F(TestMemcmpable, test){
  std::string org1 = "helloworld";
  std::string org2 = "foobar";
  std::vector<uint8_t> enc{};
  encode((uint8_t *) org1.c_str(), org1.length(), enc);
  encode((uint8_t *) org2.c_str(), org2.length(), enc);
  uint8_t rest[enc.size()];
  std::copy(enc.begin(), enc.end(), rest);
  std::vector<uint8_t> dec1{};
  size_t src_len = enc.size();
  decode(rest, src_len, dec1);
  std::vector<uint8_t> tmp(org1.begin(), org1.end());
  EXPECT_EQ(tmp, dec1);
  std::vector<uint8_t> dec2{};
  decode(rest, src_len, dec2);
  tmp = std::vector<uint8_t>(org2.begin(), org2.end());
  EXPECT_EQ(tmp, dec2);
}