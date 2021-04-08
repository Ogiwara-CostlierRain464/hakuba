#ifndef SRC_DB_H
#define SRC_DB_H

#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/LaserScan.h>
#include <functional>
#include <ecl/threads.hpp>
#include <ecl/utilities.hpp>
#include <ecl/utilities/function_objects.hpp>
#include <utility>
#include <immintrin.h>

template <typename T>
class TimeSeriesTable{
public:
    typedef std::map<ros::Time, T> TableType;

    explicit TimeSeriesTable()= default;

    ros::Time getLatestTime(){
        latestTimeMutex.lock();
        auto copy = latestTime;
        latestTimeMutex.unlock();
        return copy;
    }

    void insertLatest(ros::Time &at, T& data){
        tableMutex.lock();
        latestTimeMutex.lock();
        table[at] = data;
        latestTime = at;
        tableMutex.unlock();
        latestTimeMutex.unlock();
    }

    void getLatest(T& out_data){
        tableMutex.lock();
        out_data = table[getLatestTime()];
        tableMutex.unlock();
    }


private:
    ros::Time latestTime = ros::Time::now();

    ecl::Mutex latestTimeMutex{};
    ecl::Mutex tableMutex{};

    TableType table;
};

class IncrementalView{
public:
    typedef std::map<std::pair<double, double>, int> TableType;

    explicit IncrementalView(std::function< void(IncrementalView*) > query_)
    : query(std::move(query_)), queryThread(ecl::Thread(ecl::generateFunctionObject(&IncrementalView::onQueryThread, *this)))
    {}

    // special range query for GridMap
    void insert(std::pair<double, double> at, int occupy){
        tableMutex.lock();
        table[at] = occupy;
        tableMutex.unlock();
    }

    void rangeQuery(const std::function<void (TableType&)> &range_query){
        // by using this kind of interface, creation on table array is always
        // required, which will lead to low performance.
        tableMutex.lock();
        range_query(table);
        tableMutex.unlock();
    }

private:
    void onQueryThread(){
        auto ptr = this;
        assert(ptr != nullptr);

        query(ptr);
    }

    const std::function<void (IncrementalView*)> query;
    ecl::Thread queryThread;

    TableType table;
    ecl::Mutex tableMutex;
};

#endif //SRC_DB_H
