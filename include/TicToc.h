//
// This file is part of UL-SLAM based on ORB-SLAM3
//

#ifndef ORB_SLAM3_TICTOC_H
#define ORB_SLAM3_TICTOC_H
#include <ctime>
#include <cstdlib>
#include <chrono>

class TicToc
{
public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};
#endif //ORB_SLAM3_TICTOC_H
