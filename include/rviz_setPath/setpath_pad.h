#ifndef SETPATH_PAD_H
#define SETPATH_PAD_H

//所需要包含的头文件
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件
#include <stdio.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalID.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>
#include <QVector>
#include <QScrollArea>
#include <QProgressBar>

#include <geometry_msgs/Twist.h>
#include <QDebug>
#include <vector>

#include <QThread>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>
#include <sfm_lib/DisplayInput.h>
#include <sfm_lib/targets_list.h>
#include <sfm_lib/StateControl.h>
#endif

//class QLineEdit;


using namespace std;

namespace rviz_set_path
{
class PubMark;
class DoNav;
class MyProgressBar;

enum OperaType{
  DEL=0,//删除
  UP=1,//上移
  DOWN=2//下移
};

enum Station{
  NORM=0,//正常
  PAUSE=1,//暂停
  CANCEL=2,//取消
  FINISHED = 3//完成
};

class PathPoint{//数据类
  public:
    PathPoint(){};
    ~PathPoint(){};
  public:
    int index;
    float x;
    float y;    
    float yaw;
};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> DoNavClient;
// 所有的plugin都必须是rviz::Panel的子类
class SetPanel: public rviz::Panel
{
// 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
Q_OBJECT
public:
    // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
    SetPanel( QWidget* parent = 0 );
    ~SetPanel();
    // 重载rviz::Panel积累中的函数，用于保存、加载配置文件中的数据，在我们这个plugin
    // 中，数据就是topic的名称
    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

private:
    void ros_init();//ros部分初始化
    void ui_init();//UI部分初始化
    void setTips(QString info);//设置提示文字
    void add_Navpoint(int index,float x,float y,float yaw);//添加航路点
    void add_VBox_by_vector();//从vector数据来创建界面点
    void delete_all_point(int index = -1);//删除操作
    void remove_all_HBox(int index = -1);//删除界面元素
    void remove_all_vector(int index = -1);//删除所有数据
    //回调函数
    void pointCb(const geometry_msgs::PointStampedConstPtr &clicked_point);

// 公共槽.
public Q_SLOTS:
    void setPath();//发送一路的点
    void get_pregress_slots(float pre);//接受进度函数

// 内部槽.
protected Q_SLOTS:
    void addPoint();                 //添加航路点
    void clearPoint();                 //清空航路点
    void startNav();                 //开始导航
    void pauseNav();                 //暂停导航
    void cancelNav();                 //取消导航
    void returnNav();                 //一键返航
    void commNav();                 //下发任务
    void recvbBtnSlot(int type,int index);    //接收按钮槽
    void recvChangeSlot(int index,float x,float y,float yaw);    //接收修改值槽
    void recvErrorSlot(QString errorstr);    //接收修改值槽
    void flash_Slot(void);//更新显示的槽函数
    void get_nav_station_Slot(int station);//接收导航状态的槽函数

Q_SIGNALS:
    void flash_signal(void);//更新显示的信号

// 内部变量.
protected:
    //控件元素
    QVBoxLayout *parent1_v = NULL;//整体空间
    QVBoxLayout *viewbox_v = NULL;//点消息空间
    QLabel *tips_lab = NULL;//提示
    QPushButton *start_btn = NULL;//开始下发
    QPushButton *pause_btn = NULL;//暂停
    QPushButton *cancel_btn = NULL;//取消
    QPushButton *add_btn = NULL;//添加
    QPushButton *delete_all_btn = NULL;//清空
    QPushButton *return_btn = NULL;//一键返航
    MyProgressBar *progressbar = NULL;//进度条
    //QScrollArea *scrollArea=NULL;//滚动区
    QPushButton *comm_btn = NULL;//下发任务
    PubMark *pub_display = NULL;//显示操作指针
    //DoNav *do_nav = NULL;//导航操作指针

    QVector<PathPoint> point_vec;//数据
    QVector<PathPoint>::iterator point_vecit;//数据迭代器
    //int nav_station_;//导航状态的变量
    PathPoint pose_org_;//初始位置，用于返航
    //bool return_flag_;//返航状态

    // ROS节点句柄
    ros::NodeHandle nh_;
    tf::TransformListener tf_;
    ros::Subscriber clicked_point_sub_;

};

class MyQHBoxLayout : public QHBoxLayout
{
  Q_OBJECT
  public:
     MyQHBoxLayout(int index,float coordx, float coordy,float yaw);
     ~MyQHBoxLayout();
     void SetIndex(int index);//设置index_
     int GetIndex();//获取index_
     float GetCoordx();//获取coordx_
     float GetCoordy();//获取coordy_
     float GetCoordyaw();//获取coordyaw_
     //void SetCoordx(float x);//设置坐标x
     //void SetCoordy(float y);//设置坐标y

  private Q_SLOTS:
     void changeX();//修改x坐标
     void changeY();//修改x坐标
     void changeYaw();//修改x坐标

  public Q_SLOTS:
      void del();//删除点
      void up();//上一一位
      void down();//下移一位 
  Q_SIGNALS:
    void btn_signal(int type,int index);//变换类型和当前的序号
    void coordChange_signal(int index,float x,float y,float yaw);//修改坐标点信号
    void error_signal(QString info);//错误提示信号

  private:
    QLabel *index_lab = NULL;//序号
    QPushButton *delbtn=NULL;//删除
    QPushButton *upbtn=NULL;//上移
    QPushButton *downbtn=NULL;//下移
    QLineEdit *coordx_edit=NULL;//x坐标
    QLineEdit *coordy_edit=NULL;//y坐标
    QLineEdit *coordyaw_edit=NULL;//y坐标
    
    float coordx_=0;
    float coordy_=0;
    float coordyaw_=0;
    int index_=0;
    
  

};

class MyProgressBar : public QHBoxLayout
{
  Q_OBJECT
  public:
     MyProgressBar(int min_value,int max_value, int init_value);
     ~MyProgressBar();
     void set_progressV(int value);//设置值

  private:
     QProgressBar *progressBar = NULL;//进度条
     QLabel *txt_progress = NULL;//文字
  public Q_SLOTS:
     void set_progress_value(int value);

  private:
     void init_progressclas();//初始化进度条类
     void set_txt(int value);//设置标签值

};

class DoNav : public QObject
{
    Q_OBJECT
    public:
       DoNav(ros::NodeHandle &nh);
       ~DoNav();
    private:
        ros::NodeHandle nh_;
        ros::Publisher cancel_pub_;
        geometry_msgs::PoseStamped pose_now_;//当前位置，前一个位置
        geometry_msgs::PoseStamped target_pose_;//目标点
        QVector<geometry_msgs::PoseStamped> navpoints_;
        DoNavClient donavc_;
        bool isCancel_;//取消标志位
        bool isPause_;//暂停标志位
        actionlib_msgs::GoalID cancelid_;
        tf::TransformListener tf_;
        float sum_distance,seg_distance,i_distance,have_dodis;//总距离，段距离，实时距离,已经完成的距离
        uint32_t point_index_;

    public:
        void start_nav(QVector<PathPoint> &markPointV);//开始导航
        void pause_nav();//暂停导航
        void cancel_nav();//取消导航

    Q_SIGNALS:
        void percent_nav_signal(float per);//百分比的反馈
        void station_signal(int station);//状态反馈

    private:
        void packNavPoint(QVector<PathPoint> &markPointV);//打包成pos点的列表
        void doneCb(const actionlib::SimpleClientGoalState &state,const move_base_msgs::MoveBaseResultConstPtr &result);//一次导航完成执行
        void activeCb();//激活触发一次
        void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);//期间一直触发
        void cp_prePose();//备份当前位置
        void cp_prePose(geometry_msgs::PoseStamped &pose_now);//备份前一个位置
        void init_pose();//初始化过渡变量位姿
        void clear_navdata();//清除导航数据
        void move_base_sendgoal();//发送导航目标


};

class PubMark : public QObject
{
    Q_OBJECT
    public:
       PubMark(ros::NodeHandle &nh);
       ~PubMark();
    private:
      //QVector<PathPoint> markPointV_;
      visualization_msgs::Marker points, line_strip, line_list;
      ros::Publisher marker_pub_;
      ros::Publisher btn_click_pub_;
      ros::ServiceClient  send_points_client_;//发送目标点集
      ros::NodeHandle nh_;
      tf::TransformListener tf_;
      void init_pointsAndLine();//初始化
      void get_now_pose();//获取当前位置

    public:
      void call_sendPoints(QVector<PathPoint> &markPointV);//发布显示点
      void pub_markers(QVector<PathPoint> &markPointV);//发布显示点
      void pub_btn_signal(TaskInput btn_signal_input);//按钮输入
};

} // end namespace rviz_teleop_commander

#endif // TELEOP_PANEL_H
