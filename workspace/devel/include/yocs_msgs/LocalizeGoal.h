// Generated by gencpp from file yocs_msgs/LocalizeGoal.msg
// DO NOT EDIT!


#ifndef YOCS_MSGS_MESSAGE_LOCALIZEGOAL_H
#define YOCS_MSGS_MESSAGE_LOCALIZEGOAL_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace yocs_msgs
{
template <class ContainerAllocator>
struct LocalizeGoal_
{
  typedef LocalizeGoal_<ContainerAllocator> Type;

  LocalizeGoal_()
    : command(0)
    , distortion(0.0)  {
    }
  LocalizeGoal_(const ContainerAllocator& _alloc)
    : command(0)
    , distortion(0.0)  {
  (void)_alloc;
    }



   typedef int8_t _command_type;
  _command_type command;

   typedef float _distortion_type;
  _distortion_type distortion;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(STAND_AND_LOCALIZE)
  #undef STAND_AND_LOCALIZE
#endif
#if defined(_WIN32) && defined(SPIN_AND_LOCALIZE)
  #undef SPIN_AND_LOCALIZE
#endif

  enum {
    STAND_AND_LOCALIZE = 10,
    SPIN_AND_LOCALIZE = 20,
  };


  typedef boost::shared_ptr< ::yocs_msgs::LocalizeGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::yocs_msgs::LocalizeGoal_<ContainerAllocator> const> ConstPtr;

}; // struct LocalizeGoal_

typedef ::yocs_msgs::LocalizeGoal_<std::allocator<void> > LocalizeGoal;

typedef boost::shared_ptr< ::yocs_msgs::LocalizeGoal > LocalizeGoalPtr;
typedef boost::shared_ptr< ::yocs_msgs::LocalizeGoal const> LocalizeGoalConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::yocs_msgs::LocalizeGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::yocs_msgs::LocalizeGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::yocs_msgs::LocalizeGoal_<ContainerAllocator1> & lhs, const ::yocs_msgs::LocalizeGoal_<ContainerAllocator2> & rhs)
{
  return lhs.command == rhs.command &&
    lhs.distortion == rhs.distortion;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::yocs_msgs::LocalizeGoal_<ContainerAllocator1> & lhs, const ::yocs_msgs::LocalizeGoal_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace yocs_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::yocs_msgs::LocalizeGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::yocs_msgs::LocalizeGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::yocs_msgs::LocalizeGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::yocs_msgs::LocalizeGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::yocs_msgs::LocalizeGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::yocs_msgs::LocalizeGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::yocs_msgs::LocalizeGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6383995712d8736902464bbea6b0fb6d";
  }

  static const char* value(const ::yocs_msgs::LocalizeGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6383995712d87369ULL;
  static const uint64_t static_value2 = 0x02464bbea6b0fb6dULL;
};

template<class ContainerAllocator>
struct DataType< ::yocs_msgs::LocalizeGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "yocs_msgs/LocalizeGoal";
  }

  static const char* value(const ::yocs_msgs::LocalizeGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::yocs_msgs::LocalizeGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"int8 command\n"
"float32 distortion\n"
"\n"
"int8 STAND_AND_LOCALIZE=10\n"
"int8 SPIN_AND_LOCALIZE =20\n"
;
  }

  static const char* value(const ::yocs_msgs::LocalizeGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::yocs_msgs::LocalizeGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.command);
      stream.next(m.distortion);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LocalizeGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::yocs_msgs::LocalizeGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::yocs_msgs::LocalizeGoal_<ContainerAllocator>& v)
  {
    s << indent << "command: ";
    Printer<int8_t>::stream(s, indent + "  ", v.command);
    s << indent << "distortion: ";
    Printer<float>::stream(s, indent + "  ", v.distortion);
  }
};

} // namespace message_operations
} // namespace ros

#endif // YOCS_MSGS_MESSAGE_LOCALIZEGOAL_H
