#ifndef SRC_DB_H
#define SRC_DB_H

#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/LaserScan.h>
#include <functional>
#include <ecl/threads.hpp>
#include <ecl/utilities.hpp>
#include <ecl/utilities/function_objects.hpp>
#include <immintrin.h>

template <typename T>
class TimeSeriesTable{
public:
    typedef std::map<ros::Time, T> TableType;

    explicit TimeSeriesTable(const ros::Duration& max_storage_time =
            ros::Duration().fromSec(10)):
            maxStorageTime(max_storage_time)
            {}

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
    ros::Duration maxStorageTime;

    ecl::Mutex latestTimeMutex{};
    ecl::Mutex tableMutex{};

    TableType table;
};

class IncrementalView{
public:
    typedef std::map<std::pair<double, double>, int> TableType;

    explicit IncrementalView(const std::function< void(IncrementalView*) > &query_)
    : query(query_), queryThread(ecl::Thread(ecl::generateFunctionObject(&IncrementalView::onQueryThread, *this)))
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

    const std::function<void (IncrementalView*)> &query;
    ecl::Thread queryThread;

    TableType table;
    ecl::Mutex tableMutex;
};

struct DB{
    // tf stores data by std::set (what a naive implementation...)
    // which is implemented by Rb tree
    // lookup runs for findClosest md::mutex suffixMutex{};ethod
    // which uses iterator to find.
    // each elements are sorted by time.
    // How to make DB?
    // How to make it generalized?
    // anyway, at first, ...
    // 1: pull up transform data from tf
    // pulling up just requires tf listener!
    // 2: store another type datum.
    // at first, just use tf for transformation.
    // key point is LaserScan data.
    // how to save LaserScan? just embedding to memory?
    // should I take care of logger already at this point?
    // I'm still not sure how long should I save scan data,
    // so just save for 10 second as same as tf does.
    // Most important thing that should be save is...GridMap.
    // Does GridMap should be Materialized View?
    // when to calculate transformation of LaserScan?
    // calculation of LaserScan can be a System transaction.
    // so it can be running as different thread.
    // try to run callback function differ from main thread not as usual.
    // 1. data insert method should not be called from main thread.
    // 2. GridMap update method should not be called from main thread or data inserting method
    // 3. GridMap read method should be called from main thread.
    // (a) callback thread: obtain LaserScan, and add to LaserScan table
    // (b) update GridMap thread: check latest LaserScan. If latest time has changed, calculate absolute position of PointCloud and apply to GridMap
    // (c) main thread: get map from GridMap and make decision.
    // so at here, that key point is at least three different thread touches one DB.
    // DB structure should be protected by nature.
    // how to protect Db structure? Mutex? Silo?
    // LaserScan table: inserted by (a), read by (b). Protect Linked list and latest_time by mutex.
    // GridMap table: updated by (b), read by (c).
    // Offer user to create table, create view.
    // TimeSeriesTable: store data keyed by ros::Time, and has latest_update_time field.
    // IncrementalView: created by callback function. IF: space scan? reading operation is implemented by user side.
    // at here, you have to do range scan, not only one single point.
    // for convenience, we provide this as a special view, a GridMapView
};

#endif //SRC_DB_H
