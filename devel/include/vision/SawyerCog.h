// Generated by gencpp from file vision/SawyerCog.msg
// DO NOT EDIT!


#ifndef VISION_MESSAGE_SAWYERCOG_H
#define VISION_MESSAGE_SAWYERCOG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose.h>

namespace vision
{
template <class ContainerAllocator>
struct SawyerCog_
{
  typedef SawyerCog_<ContainerAllocator> Type;

  SawyerCog_()
    : sawyer_end()  {
    }
  SawyerCog_(const ContainerAllocator& _alloc)
    : sawyer_end(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _sawyer_end_type;
  _sawyer_end_type sawyer_end;





  typedef boost::shared_ptr< ::vision::SawyerCog_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vision::SawyerCog_<ContainerAllocator> const> ConstPtr;

}; // struct SawyerCog_

typedef ::vision::SawyerCog_<std::allocator<void> > SawyerCog;

typedef boost::shared_ptr< ::vision::SawyerCog > SawyerCogPtr;
typedef boost::shared_ptr< ::vision::SawyerCog const> SawyerCogConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vision::SawyerCog_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vision::SawyerCog_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::vision::SawyerCog_<ContainerAllocator1> & lhs, const ::vision::SawyerCog_<ContainerAllocator2> & rhs)
{
  return lhs.sawyer_end == rhs.sawyer_end;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::vision::SawyerCog_<ContainerAllocator1> & lhs, const ::vision::SawyerCog_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace vision

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::vision::SawyerCog_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vision::SawyerCog_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vision::SawyerCog_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vision::SawyerCog_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vision::SawyerCog_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vision::SawyerCog_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vision::SawyerCog_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c11a2e283bb2bfb477384cd2ce2a8c12";
  }

  static const char* value(const ::vision::SawyerCog_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc11a2e283bb2bfb4ULL;
  static const uint64_t static_value2 = 0x77384cd2ce2a8c12ULL;
};

template<class ContainerAllocator>
struct DataType< ::vision::SawyerCog_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vision/SawyerCog";
  }

  static const char* value(const ::vision::SawyerCog_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vision::SawyerCog_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Pose sawyer_end\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::vision::SawyerCog_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vision::SawyerCog_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.sawyer_end);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SawyerCog_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vision::SawyerCog_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vision::SawyerCog_<ContainerAllocator>& v)
  {
    s << indent << "sawyer_end: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.sawyer_end);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VISION_MESSAGE_SAWYERCOG_H