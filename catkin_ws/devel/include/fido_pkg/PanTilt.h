// Generated by gencpp from file fido_pkg/PanTilt.msg
// DO NOT EDIT!


#ifndef FIDO_PKG_MESSAGE_PANTILT_H
#define FIDO_PKG_MESSAGE_PANTILT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace fido_pkg
{
template <class ContainerAllocator>
struct PanTilt_
{
  typedef PanTilt_<ContainerAllocator> Type;

  PanTilt_()
    : pan(0)
    , tilt(0)  {
    }
  PanTilt_(const ContainerAllocator& _alloc)
    : pan(0)
    , tilt(0)  {
    }



   typedef int16_t _pan_type;
  _pan_type pan;

   typedef int16_t _tilt_type;
  _tilt_type tilt;




  typedef boost::shared_ptr< ::fido_pkg::PanTilt_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fido_pkg::PanTilt_<ContainerAllocator> const> ConstPtr;

}; // struct PanTilt_

typedef ::fido_pkg::PanTilt_<std::allocator<void> > PanTilt;

typedef boost::shared_ptr< ::fido_pkg::PanTilt > PanTiltPtr;
typedef boost::shared_ptr< ::fido_pkg::PanTilt const> PanTiltConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::fido_pkg::PanTilt_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::fido_pkg::PanTilt_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace fido_pkg

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'fido_pkg': ['/home/turtlebot/catkin_ws/src/fido_pkg/msg'], 'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::fido_pkg::PanTilt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fido_pkg::PanTilt_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fido_pkg::PanTilt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fido_pkg::PanTilt_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fido_pkg::PanTilt_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fido_pkg::PanTilt_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::fido_pkg::PanTilt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e5401181ff33a9d514daeb8a647b3152";
  }

  static const char* value(const ::fido_pkg::PanTilt_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe5401181ff33a9d5ULL;
  static const uint64_t static_value2 = 0x14daeb8a647b3152ULL;
};

template<class ContainerAllocator>
struct DataType< ::fido_pkg::PanTilt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fido_pkg/PanTilt";
  }

  static const char* value(const ::fido_pkg::PanTilt_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::fido_pkg::PanTilt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 pan\n\
int16 tilt \n\
";
  }

  static const char* value(const ::fido_pkg::PanTilt_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::fido_pkg::PanTilt_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pan);
      stream.next(m.tilt);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct PanTilt_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fido_pkg::PanTilt_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::fido_pkg::PanTilt_<ContainerAllocator>& v)
  {
    s << indent << "pan: ";
    Printer<int16_t>::stream(s, indent + "  ", v.pan);
    s << indent << "tilt: ";
    Printer<int16_t>::stream(s, indent + "  ", v.tilt);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FIDO_PKG_MESSAGE_PANTILT_H