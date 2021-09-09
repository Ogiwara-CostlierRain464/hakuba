#include <iostream>
#include <gtest/gtest.h>
#include "../src/db/disk.h"

using namespace std;

struct DiskTest: public ::testing::Test{};

TEST_F(DiskTest, test){
    DiskManager disk;
    DiskManager::open("/tmp/test.data", disk);
    auto page_one = disk.allocatePage();
    std::array<uint8_t, PAGE_SIZE> data{1,2};
    disk.writePageData(page_one, data);
    auto page_two = disk.allocatePage();
    std::array<uint8_t, PAGE_SIZE> data2{3,4,5,6};
    disk.writePageData(page_two, data2);
    disk.sync();
    disk.drop();

    DiskManager disk2;
    DiskManager::open("/tmp/test.data", disk2);
    std::array<uint8_t, PAGE_SIZE> buf{};
    disk2.readPageData(page_one, buf);

    EXPECT_EQ(buf[0], 1);
    EXPECT_EQ(buf[1], 2);
    disk2.readPageData(page_two, buf);
    EXPECT_EQ(buf[0], 3);
    EXPECT_EQ(buf[1], 4);
}