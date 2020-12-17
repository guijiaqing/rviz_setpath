#include <setpath_pad.h>

namespace rviz_set_path
{

DoNav::DoNav(ros::NodeHandle &nh):
nh_(nh),
tf_(ros::Duration(5)),
donavc_("move_base", true)
{
   isCancel_ = isPause_ = false;
  //cancelid_.id = "0";
  //cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
}
    
DoNav::~DoNav()
{
  
}
void DoNav::start_nav(QVector<PathPoint> &markPointV){//开始导航

 if(!isPause_){
   cout<<"非暂停状态，导航开始,初始化"<<endl;
   sum_distance = 0.001;
   point_index_ = seg_distance = i_distance = have_dodis = 0;
   clear_navdata();
   packNavPoint(markPointV);
 }else{
   
 }
 isCancel_ = false;
 move_base_sendgoal();
 //cout<<"start p:"<<isPause_<<",c:"<<isCancel_<<endl;
}
void DoNav::pause_nav(){//暂停导航
  isPause_ = true;
  donavc_.cancelAllGoals();
  //cout<<"pause p:"<<isPause_<<",c:"<<isCancel_<<endl;
}
void DoNav::cancel_nav(){//取消导航
  if(isPause_){
    isPause_ = false;
  }else{
    isCancel_ = true;
    donavc_.cancelAllGoals();
  }
  Q_EMIT station_signal(Station::CANCEL);//发出取消的信号
  //cout<<"cancel p:"<<isPause_<<",c:"<<isCancel_<<endl;
}

void DoNav::packNavPoint(QVector<PathPoint> &markPointV){//打包成pos点的列表
  cp_prePose();//储备现在位置
  QVector<PathPoint>::iterator point_vecit;//数据迭代器  
  for(point_vecit=markPointV.begin();point_vecit!=markPointV.end();point_vecit++){
    init_pose();
    float x , y , yaw, distance;
    target_pose_.pose.position.x = pose_now_.pose.position.x;
    target_pose_.pose.position.y = pose_now_.pose.position.y;
    x = point_vecit->x - target_pose_.pose.position.x;
    y = point_vecit->y - target_pose_.pose.position.y;
    distance =  sqrt(x*x + y*y);
    sum_distance += distance;
    yaw = acos(x / distance);
    if(y<0)
    {
        yaw = -yaw;
    }
    target_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);//只通过y即绕z的旋转角度计算四元数，用于平面小车。返回四元数 
    navpoints_.push_back(target_pose_);

    target_pose_.pose.position.x = point_vecit->x;
    target_pose_.pose.position.y = point_vecit->y;

    navpoints_.push_back(target_pose_);
    cp_prePose(target_pose_);
    //cout<<"point_index:"<<point_vecit->index<<endl;
  }
  //cout<<navpoints_.size()<<endl;
}

void DoNav::cp_prePose(){//备份当前位置
    tf::StampedTransform robot_pose_tf;
    try
    {
      tf_.waitForTransform("map", "base_footprint", ros::Time::now(), ros::Duration(5));
      tf_.lookupTransform("map", "base_footprint", ros::Time(0), robot_pose_tf);
      pose_now_.pose.position.x = robot_pose_tf.getOrigin().x();
      pose_now_.pose.position.y = robot_pose_tf.getOrigin().y();
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR_STREAM("Couldn't transform from "<<"map"<<" to "<< "odom");
    }

}
void DoNav::cp_prePose(geometry_msgs::PoseStamped &pose_now){//备份前一个位置
    pose_now_.pose.position.x = pose_now.pose.position.x;
    pose_now_.pose.position.y = pose_now.pose.position.y;
}

void DoNav::init_pose()
{
    //构造header
    target_pose_.header.seq = 0;
    target_pose_.header.stamp =ros::Time::now();//如果有问题就使用Time(0)获取时间戳，确保类型一致
    target_pose_.header.frame_id = "map";
    //构造pose
    target_pose_.pose.position.x = 0;
    target_pose_.pose.position.y = 0;
    target_pose_.pose.position.z = 0.0;
    target_pose_.pose.orientation.x = 0.0;
    target_pose_.pose.orientation.y = 0.0;
    target_pose_.pose.orientation.z = 0.0;
    target_pose_.pose.orientation.w = 1.0;
}
void DoNav::clear_navdata(){
  navpoints_.clear();
  init_pose();
}
void DoNav::move_base_sendgoal()
{
  move_base_msgs::MoveBaseGoal goal;
	//Use the map frame to define goal pose
  goal.target_pose.header.frame_id = "map";
	//Set the time stamp to "now"
  goal.target_pose.header.stamp = ros::Time::now();
	//Set the goal pose to the i-th waypoint
  goal.target_pose.pose = navpoints_[point_index_].pose;
	//Start the robot moving toward the goal
  donavc_.sendGoal(goal, boost::bind(&DoNav::doneCb, this, _1, _2),
                   boost::bind(&DoNav::activeCb,this),
                   boost::bind(&DoNav::feedbackCb,this, _1));

}
void DoNav::doneCb(const actionlib::SimpleClientGoalState &state,const move_base_msgs::MoveBaseResultConstPtr &result){//一次导航完成执行
  if(!isPause_){
    //cout<<"导航结束，不在暂停状态"<<endl;
    have_dodis += seg_distance;//完成一段添加实时距离表示已经完成的
    i_distance = 0;//实时距离重新计算
    point_index_++;//下一个导航点
  }
  if(point_index_ < navpoints_.size() && !(isCancel_ || isPause_)){
     //cout<<"导航结束，进行下一个点"<<endl;
     move_base_sendgoal();
  }else{
     //cout<<"导航结束，没有后续点"<<endl;
     if(isCancel_){
       Q_EMIT station_signal(Station::CANCEL);//发出取消的信号
     }else if(isPause_){
       Q_EMIT station_signal(Station::PAUSE);//发出暂停的信号
     }else{
       Q_EMIT station_signal(Station::FINISHED);//发出完成的信号
       isCancel_ = false;
       isPause_ = false;
     }
  }
}
void DoNav::activeCb(){//激活触发一次
  cout<<"导航开始激发一次"<<endl;
  if(point_index_ < navpoints_.size()){//记录本次出发点，用来计算进度
    target_pose_.pose.position.x = navpoints_[point_index_].pose.position.x;
    target_pose_.pose.position.y = navpoints_[point_index_].pose.position.y;
  }
  float x , y ;
  tf::StampedTransform robot_pose_tf;
  try
  {
    tf_.waitForTransform("map", "base_footprint", ros::Time::now(), ros::Duration(5));
    tf_.lookupTransform("map", "base_footprint", ros::Time(0), robot_pose_tf);
    x = robot_pose_tf.getOrigin().x() - target_pose_.pose.position.x;
    y = robot_pose_tf.getOrigin().y() - target_pose_.pose.position.y;
    if(!isPause_){
      seg_distance = sqrt(x*x + y*y);
      cout<<"非暂停状态，段距离计算一次！"<<endl;
    }
  }
  catch(tf::TransformException ex)
  {
    ROS_ERROR_STREAM("Couldn't transform from "<<"map"<<" to "<< "odom");
  }
  isPause_ = false;
  Q_EMIT station_signal(Station::NORM);//发布状态，正常
}
void DoNav::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){//期间一直触发
    float x , y ;
    tf::StampedTransform robot_pose_tf;
    try
    {
      tf_.waitForTransform("map", "base_footprint", ros::Time::now(), ros::Duration(5));
      tf_.lookupTransform("map", "base_footprint", ros::Time(0), robot_pose_tf);
      x = robot_pose_tf.getOrigin().x() - target_pose_.pose.position.x;
      y = robot_pose_tf.getOrigin().y() - target_pose_.pose.position.y;
      if(!isPause_){
        i_distance = sqrt(x*x + y*y);
        Q_EMIT percent_nav_signal((have_dodis + seg_distance - i_distance) / sum_distance);
        cout<<"非暂停状态，比例计算一次！"<<endl;
      }
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR_STREAM("Couldn't transform from "<<"map"<<" to "<< "odom");
    }

}
       
} // namespace name
   
