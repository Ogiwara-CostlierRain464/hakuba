#ifndef SRC_DB_H
#define SRC_DB_H

#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <functional>
#include <ecl/threads.hpp>
#include <ecl/utilities.hpp>
#include <ecl/utilities/function_objects.hpp>
#include <utility>
#include <immintrin.h>

template <typename T>
class TimeSeriesTable{
public:
    typedef std::map<ros::Time, T> TableType ;

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

    void rangeQuery(const std::function<void (TableType&)> &range_query){
        tableMutex.lock();
        range_query(table);
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

// return result as stream
// return big batch
// find the data which is near at this point
/**
 * To implement this data, we have to implement storage storing
 * Because memory pressure requirement is critical.
 *
 * - Read "DBML impl by Rust?"
 * - How to impl memory pressure management? (because data should be saved 10s~1h back)
 *
 * but anyway, at first, just implement IF.
 *
 * after that, let's discuss about the performance.
 *
 * save data of odom and LiDAR
 * find data by range query ( near at this boor ball, near with this scan data. )
 * performance of this find query is critical!
 *
 *
 */

void user(){
    TimeSeriesTable<sensor_msgs::LaserScan> scanTable;
    TimeSeriesTable<nav_msgs::Odometry> odomTable;

    sensor_msgs::LaserScan current_scan;
    nav_msgs::Odometry current_odom;

    bool scan_flag = false;
    bool odom_flag = false;

    scanTable.rangeQuery([&current_scan, &scan_flag](TimeSeriesTable<sensor_msgs::LaserScan>::TableType
    &table){
        for(auto & it : table){
            if(it.second.ranges == current_scan.ranges){
                scan_flag = true;
            }
        }
    });
    odomTable.rangeQuery([&current_odom, &odom_flag](TimeSeriesTable<nav_msgs::Odometry>::TableType
    &table){
        for(auto & it : table){
            if(it.second.pose == current_odom.pose){
                odom_flag = true;
            }
        }
    });

    if(scan_flag and odom_flag){
        // do something at here
    }


}

#endif //SRC_DB_H
