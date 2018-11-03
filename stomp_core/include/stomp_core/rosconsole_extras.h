///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2017, Bence Magyar
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of the copyright holder nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#ifndef ROSCONSOLE_EXTRAS_H
#define ROSCONSOLE_EXTRAS_H

#include <vector>
#include <deque>
#include <ros/console.h>
#include <iostream>
#include <Eigen/Eigen>

#define ROS_RED_STREAM(SSS) ROS_INFO_STREAM("\033[31;1m" << SSS << "\033[0m\n")
#define ROS_GREEN_STREAM(SSS) ROS_INFO_STREAM("\033[32;1m" << SSS << "\033[0m\n")
#define ROS_YELLOW_STREAM(SSS) ROS_INFO_STREAM("\033[33;1m" << SSS << "\033[0m\n")
#define ROS_BLUE_STREAM(SSS) ROS_INFO_STREAM("\033[34;1m" << SSS << "\033[0m\n")
#define ROS_MAGENTA_STREAM(SSS) ROS_INFO_STREAM("\033[35;1m" << SSS << "\033[0m\n")
#define ROS_CYAN_STREAM(SSS) ROS_INFO_STREAM("\033[36;1m" << SSS << "\033[0m\n")
#define ROS_WHITE_STREAM(SSS) ROS_INFO_STREAM("\033[30;47m" << SSS << "\033[0m\n")

template <typename T>
std::ostream& operator<<(std::ostream& stream, const std::vector<T>& list)
{
    stream << "[";
    for (const T& elem : list)
        stream << elem << ", ";
    if (not list.empty())
        stream.seekp(-2, std::ios_base::end);  // remove last ", "
    stream << "]";
    return stream;
}

template <typename T>
std::ostream& operator<<(std::ostream& stream, const std::deque<T>& list)
{
    stream << "[";
    for (const T& elem : list)
        stream << elem << ", ";
    if (not list.empty())
        stream.seekp(-2, std::ios_base::end);  // remove last ", "
    stream << "]";
    return stream;
}

std::string shape(const Eigen::MatrixXd& mat)
{
    std::stringstream ss;
    ss << "(" << mat.rows() << "," << mat.cols() << ")";
    return ss.str();
}

#include <chrono>
using namespace std::chrono;
class MeasureTime
{
  typedef ros::Time clock_type;
public:
  MeasureTime(std::string name, int seq=-1)
  : start_time_(clock_type::now())
  , name_(name)
  , seq_(seq)
  {}

  ~MeasureTime()
  {
    //auto wd = microseconds(clock_type::now() - start_time_).count();
    //ROS_CYAN_STREAM(name_ << seq_ << " elapsed time: " << (start_time_ - clock_type::now()) << "s, ");// << seconds << " us");
  }
private:
  //const clock_type::time_point start_time_;
  const clock_type start_time_;
  const std::string name_;
  const int seq_;
};


/* free test code here
ROS_RED_STREAM("bold red text");
ROS_YELLOW_STREAM("yellow stuff");
ROS_GREEN_STREAM("Green stuff");
ROS_BLUE_STREAM("blue");
ROS_CYAN_STREAM("cyan is different fromb blue");
ROS_MAGENTA_STREAM("magenta is not pink or is it?");
ROS_WHITE_STREAM("white bricks");
*/

#endif
