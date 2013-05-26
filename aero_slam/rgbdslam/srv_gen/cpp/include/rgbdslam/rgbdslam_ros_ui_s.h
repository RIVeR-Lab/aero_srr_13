/* Auto-generated by genmsg_cpp for file /home/rj/Development/groovy/rgbdslam_freiburg/rgbdslam/srv/rgbdslam_ros_ui_s.srv */
#ifndef RGBDSLAM_SERVICE_RGBDSLAM_ROS_UI_S_H
#define RGBDSLAM_SERVICE_RGBDSLAM_ROS_UI_S_H
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

#include "ros/service_traits.h"




namespace rgbdslam
{
template <class ContainerAllocator>
struct rgbdslam_ros_ui_sRequest_ {
  typedef rgbdslam_ros_ui_sRequest_<ContainerAllocator> Type;

  rgbdslam_ros_ui_sRequest_()
  : command()
  , value()
  {
  }

  rgbdslam_ros_ui_sRequest_(const ContainerAllocator& _alloc)
  : command(_alloc)
  , value(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _command_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  command;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _value_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  value;


  typedef boost::shared_ptr< ::rgbdslam::rgbdslam_ros_ui_sRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rgbdslam::rgbdslam_ros_ui_sRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct rgbdslam_ros_ui_sRequest
typedef  ::rgbdslam::rgbdslam_ros_ui_sRequest_<std::allocator<void> > rgbdslam_ros_ui_sRequest;

typedef boost::shared_ptr< ::rgbdslam::rgbdslam_ros_ui_sRequest> rgbdslam_ros_ui_sRequestPtr;
typedef boost::shared_ptr< ::rgbdslam::rgbdslam_ros_ui_sRequest const> rgbdslam_ros_ui_sRequestConstPtr;


template <class ContainerAllocator>
struct rgbdslam_ros_ui_sResponse_ {
  typedef rgbdslam_ros_ui_sResponse_<ContainerAllocator> Type;

  rgbdslam_ros_ui_sResponse_()
  {
  }

  rgbdslam_ros_ui_sResponse_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::rgbdslam::rgbdslam_ros_ui_sResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rgbdslam::rgbdslam_ros_ui_sResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct rgbdslam_ros_ui_sResponse
typedef  ::rgbdslam::rgbdslam_ros_ui_sResponse_<std::allocator<void> > rgbdslam_ros_ui_sResponse;

typedef boost::shared_ptr< ::rgbdslam::rgbdslam_ros_ui_sResponse> rgbdslam_ros_ui_sResponsePtr;
typedef boost::shared_ptr< ::rgbdslam::rgbdslam_ros_ui_sResponse const> rgbdslam_ros_ui_sResponseConstPtr;

struct rgbdslam_ros_ui_s
{

typedef rgbdslam_ros_ui_sRequest Request;
typedef rgbdslam_ros_ui_sResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct rgbdslam_ros_ui_s
} // namespace rgbdslam

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rgbdslam::rgbdslam_ros_ui_sRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rgbdslam::rgbdslam_ros_ui_sRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rgbdslam::rgbdslam_ros_ui_sRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "406bad1a44daaa500258274f332bb924";
  }

  static const char* value(const  ::rgbdslam::rgbdslam_ros_ui_sRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x406bad1a44daaa50ULL;
  static const uint64_t static_value2 = 0x0258274f332bb924ULL;
};

template<class ContainerAllocator>
struct DataType< ::rgbdslam::rgbdslam_ros_ui_sRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rgbdslam/rgbdslam_ros_ui_sRequest";
  }

  static const char* value(const  ::rgbdslam::rgbdslam_ros_ui_sRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rgbdslam::rgbdslam_ros_ui_sRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string command\n\
string value\n\
\n\
";
  }

  static const char* value(const  ::rgbdslam::rgbdslam_ros_ui_sRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rgbdslam::rgbdslam_ros_ui_sResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rgbdslam::rgbdslam_ros_ui_sResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rgbdslam::rgbdslam_ros_ui_sResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::rgbdslam::rgbdslam_ros_ui_sResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::rgbdslam::rgbdslam_ros_ui_sResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rgbdslam/rgbdslam_ros_ui_sResponse";
  }

  static const char* value(const  ::rgbdslam::rgbdslam_ros_ui_sResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rgbdslam::rgbdslam_ros_ui_sResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
";
  }

  static const char* value(const  ::rgbdslam::rgbdslam_ros_ui_sResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::rgbdslam::rgbdslam_ros_ui_sResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rgbdslam::rgbdslam_ros_ui_sRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.command);
    stream.next(m.value);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct rgbdslam_ros_ui_sRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rgbdslam::rgbdslam_ros_ui_sResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct rgbdslam_ros_ui_sResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<rgbdslam::rgbdslam_ros_ui_s> {
  static const char* value() 
  {
    return "406bad1a44daaa500258274f332bb924";
  }

  static const char* value(const rgbdslam::rgbdslam_ros_ui_s&) { return value(); } 
};

template<>
struct DataType<rgbdslam::rgbdslam_ros_ui_s> {
  static const char* value() 
  {
    return "rgbdslam/rgbdslam_ros_ui_s";
  }

  static const char* value(const rgbdslam::rgbdslam_ros_ui_s&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<rgbdslam::rgbdslam_ros_ui_sRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "406bad1a44daaa500258274f332bb924";
  }

  static const char* value(const rgbdslam::rgbdslam_ros_ui_sRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<rgbdslam::rgbdslam_ros_ui_sRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rgbdslam/rgbdslam_ros_ui_s";
  }

  static const char* value(const rgbdslam::rgbdslam_ros_ui_sRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<rgbdslam::rgbdslam_ros_ui_sResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "406bad1a44daaa500258274f332bb924";
  }

  static const char* value(const rgbdslam::rgbdslam_ros_ui_sResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<rgbdslam::rgbdslam_ros_ui_sResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rgbdslam/rgbdslam_ros_ui_s";
  }

  static const char* value(const rgbdslam::rgbdslam_ros_ui_sResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // RGBDSLAM_SERVICE_RGBDSLAM_ROS_UI_S_H

