/* Auto-generated by genmsg_cpp for file /home/parallels/groovy_workspace/oryx_srr/oryxsrr_msgs/msg/SoftwareStop.msg */
#ifndef ORYXSRR_MSGS_MESSAGE_SOFTWARESTOP_H
#define ORYXSRR_MSGS_MESSAGE_SOFTWARESTOP_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"

namespace oryxsrr_msgs
{
template <class ContainerAllocator>
struct SoftwareStop_ {
  typedef SoftwareStop_<ContainerAllocator> Type;

  SoftwareStop_()
  : header()
  , stop(false)
  , message()
  {
  }

  SoftwareStop_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , stop(false)
  , message(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef uint8_t _stop_type;
  uint8_t stop;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _message_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  message;


  typedef boost::shared_ptr< ::oryxsrr_msgs::SoftwareStop_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::oryxsrr_msgs::SoftwareStop_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SoftwareStop
typedef  ::oryxsrr_msgs::SoftwareStop_<std::allocator<void> > SoftwareStop;

typedef boost::shared_ptr< ::oryxsrr_msgs::SoftwareStop> SoftwareStopPtr;
typedef boost::shared_ptr< ::oryxsrr_msgs::SoftwareStop const> SoftwareStopConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::oryxsrr_msgs::SoftwareStop_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::oryxsrr_msgs::SoftwareStop_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace oryxsrr_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::oryxsrr_msgs::SoftwareStop_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::oryxsrr_msgs::SoftwareStop_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::oryxsrr_msgs::SoftwareStop_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1e356d29f1193a9ede4461043af863ca";
  }

  static const char* value(const  ::oryxsrr_msgs::SoftwareStop_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x1e356d29f1193a9eULL;
  static const uint64_t static_value2 = 0xde4461043af863caULL;
};

template<class ContainerAllocator>
struct DataType< ::oryxsrr_msgs::SoftwareStop_<ContainerAllocator> > {
  static const char* value() 
  {
    return "oryxsrr_msgs/SoftwareStop";
  }

  static const char* value(const  ::oryxsrr_msgs::SoftwareStop_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::oryxsrr_msgs::SoftwareStop_<ContainerAllocator> > {
  static const char* value() 
  {
    return "#Header\n\
Header header\n\
\n\
#Flag for if stop or release\n\
bool stop\n\
\n\
#Message explaining the stop call\n\
string message\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::oryxsrr_msgs::SoftwareStop_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::oryxsrr_msgs::SoftwareStop_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::oryxsrr_msgs::SoftwareStop_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::oryxsrr_msgs::SoftwareStop_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.stop);
    stream.next(m.message);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SoftwareStop_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::oryxsrr_msgs::SoftwareStop_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::oryxsrr_msgs::SoftwareStop_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "stop: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.stop);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.message);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ORYXSRR_MSGS_MESSAGE_SOFTWARESTOP_H

