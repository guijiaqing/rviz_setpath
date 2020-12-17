#include <setpath_pad.h>

namespace rviz_set_path
{

PubMark::PubMark(ros::NodeHandle &nh):
nh_(nh),
tf_(ros::Duration(5))
{
    send_points_client_ = nh_.serviceClient<sfm_lib::targets_list>("send_targetPoints");
    btn_click_pub_ = nh_.advertise<sfm_lib::DisplayInput>("display_input", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
}
    
PubMark::~PubMark()
{
}
void PubMark::init_pointsAndLine(){
   //初始化
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "map";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "marker_points";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    //分配3个id
    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;
    //初始化形状
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    //初始化大小
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;
    //初始化颜色
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;
    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
}
void PubMark::get_now_pose(){//获取当前位置
    tf::StampedTransform robot_pose_tf;
    try
    {
      tf_.waitForTransform("map", "base_footprint", ros::Time::now(), ros::Duration(0.1));
      tf_.lookupTransform("map", "base_footprint", ros::Time(0), robot_pose_tf);
      geometry_msgs::Point p;
      p.x = robot_pose_tf.getOrigin().x();
      p.y = robot_pose_tf.getOrigin().y();
      p.z = 0;
      points.points.push_back(p);
      line_strip.points.push_back(p);
      line_list.points.push_back(p);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR_STREAM("Couldn't transform from "<<"map"<<" to "<< "odom");
    }


}
void PubMark::pub_markers(QVector<PathPoint> &markPointV){//
  //cout<<"size:"<<markPointV.size()<<endl;
  init_pointsAndLine();//初始化
  get_now_pose();//获取当前位置
  QVector<PathPoint>::iterator point_vecit;//数据迭代器  
  for(point_vecit=markPointV.begin();point_vecit!=markPointV.end();point_vecit++){
    geometry_msgs::Point p;
    p.x = point_vecit->x;
    p.y = point_vecit->y;
    p.z = 0;
    points.points.push_back(p);
    line_strip.points.push_back(p);
    line_list.points.push_back(p);
    //cout<<"point_index:"<<point_vecit->index<<endl;
  }
  if(markPointV.size()>0){
    marker_pub_.publish(points);
    marker_pub_.publish(line_strip);
    //marker_pub_.publish(line_list);
    //cout<<"have pub"<<endl;
  }else{
    marker_pub_.publish(points);
    marker_pub_.publish(line_strip);
    //marker_pub_.publish(line_list);
  }
  line_strip.points.clear();
  points.points.clear();
  line_list.points.clear();
}
void PubMark::call_sendPoints(QVector<PathPoint> &markPointV){
  sfm_lib::targets_list points_list;
  QVector<PathPoint>::iterator point_vecit;//数据迭代器
  for(point_vecit=markPointV.begin();point_vecit!=markPointV.end();point_vecit++){
    geometry_msgs::Pose point ;
    point.position.x = point_vecit->x;
    point.position.y = point_vecit->y;
    point.orientation = tf::createQuaternionMsgFromYaw(point_vecit->yaw);
    points_list.request.target_points.push_back(point);
  }
  if(points_list.request.target_points.size()>1){
    if(send_points_client_.call(points_list))
    {

    }
  }
}
void PubMark::pub_btn_signal(TaskInput btn_signal_input){//按钮输入
  sfm_lib::DisplayInput btn_signal;
  btn_signal.signal = btn_signal_input;
  btn_click_pub_.publish(btn_signal);
}
       
} // namespace name
   
