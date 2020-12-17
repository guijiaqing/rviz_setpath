

#include <setpath_pad.h>

namespace rviz_set_path
{

// 构造函数，初始化变量
SetPanel::SetPanel( QWidget* parent )
  : rviz::Panel( parent )
{
    std::cout<<"dispal enter"<<std::endl;
    ros_init();//ros部分初始化
    ROS_INFO("ros init finished");
    ui_init();//UI部分初始化
    ROS_INFO("ui init finished");

    connect( this, SIGNAL( flash_signal() ), this, SLOT( flash_Slot() ));


    // 创建一个定时器，用来定时发布消息
    QTimer* output_timer = new QTimer( this );

    // 设置定时器的回调函数，按周期调用sendVel()
    //connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

    // 设置定时器的周期，100ms
    output_timer->start( 100 );
}

SetPanel::~SetPanel()
{
  if(parent1_v!=NULL)
    delete parent1_v;
  if(viewbox_v!=NULL)
    delete viewbox_v;
  if(tips_lab!=NULL)
    delete tips_lab;
  if(start_btn!=NULL)
    delete start_btn;
  if(pause_btn!=NULL)
    delete pause_btn;
  if(cancel_btn!=NULL)
    delete cancel_btn;
  if(add_btn!=NULL)
    delete add_btn;
  if(delete_all_btn!=NULL)
    delete delete_all_btn;
  if(pub_display!=NULL)
    delete pub_display;
  if(progressbar!=NULL)
    delete progressbar;
  if(return_btn!=NULL)
    delete return_btn;
  if(comm_btn!=NULL)
    delete comm_btn;
  //if(robot_!=NULL)
    //delete robot_;
  
}


//私有函数
void SetPanel::ros_init(){//ros部分初始化
    clicked_point_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &SetPanel::pointCb, this);
    pub_display = new PubMark(nh_);
  
    tf::StampedTransform robot_pose_tf;
    try
    {
      tf_.waitForTransform("map", "base_footprint", ros::Time::now(), ros::Duration(1));
      tf_.lookupTransform("map", "base_footprint", ros::Time(0), robot_pose_tf);
      pose_org_.x = robot_pose_tf.getOrigin().x();
      pose_org_.y = robot_pose_tf.getOrigin().y();
      pose_org_.index = 0;
      //pose_org_.pose.position.x = robot_pose_tf.getOrigin().x();
      //pose_org_.pose.position.y = robot_pose_tf.getOrigin().y();
      //pose_org_.pose.orientation.x = robot_pose_tf.getRotation().x();
      //pose_org_.pose.orientation.y = robot_pose_tf.getRotation().y();
      //pose_org_.pose.orientation.z = robot_pose_tf.getRotation().z();
      //pose_org_.pose.orientation.w = robot_pose_tf.getRotation().w();
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR_STREAM("Couldn't transform from "<<"map"<<" to "<< "odom");
    }
}
void SetPanel::ui_init(){//UI部分初始化
    parent1_v = new QVBoxLayout();
    viewbox_v = new QVBoxLayout();

    tips_lab = new QLabel("tips:");
    start_btn = new QPushButton("开始");
    pause_btn = new QPushButton("暂停");
    cancel_btn = new QPushButton("取消");
    add_btn = new QPushButton("添加");
    delete_all_btn = new QPushButton("清空");
    return_btn = new QPushButton("返航");
    comm_btn = new QPushButton("下发");
    start_btn->setFixedSize(70,25);
    pause_btn->setFixedSize(70,25);
    cancel_btn->setFixedSize(70,25);
    add_btn->setFixedSize(70,25);
    delete_all_btn->setFixedSize(70,25);
    return_btn->setFixedSize(70,25);
    comm_btn->setFixedSize(70,25);

    parent1_v->addWidget(new QLabel("路径点：(序号，坐标x，坐标y)"),1, Qt::AlignLeft | Qt::AlignTop);
    parent1_v->addLayout(viewbox_v);

    QHBoxLayout *hbtn_layout = new QHBoxLayout();
    hbtn_layout->addStretch();
    hbtn_layout->addWidget(add_btn);// ,1, Qt::AlignRight);
    hbtn_layout->addWidget(delete_all_btn);// ,1, Qt::AlignRight);
    hbtn_layout->addWidget(comm_btn);
    hbtn_layout->addWidget(return_btn);// ,1, Qt::AlignRight);
    hbtn_layout->addWidget(cancel_btn);// ,1, Qt::AlignRight);
    hbtn_layout->addWidget(pause_btn);// ,1, Qt::AlignRight);
    hbtn_layout->addWidget(start_btn);// ,1, Qt::AlignRight);

    parent1_v->addStretch();
    parent1_v->addLayout(hbtn_layout);
    progressbar = new MyProgressBar(0,100,0);
    parent1_v->addLayout(progressbar);

    parent1_v->addWidget(tips_lab ,1, Qt::AlignLeft | Qt::AlignBottom);

    setLayout( parent1_v );
    //viewbox_v->addLayout(new MyQHBoxLayout(0,1.1,2.2));

    // 设置信号与槽的连接
    connect( start_btn, SIGNAL( clicked() ), this, SLOT( startNav() ));  
    connect( pause_btn, SIGNAL( clicked() ), this, SLOT( pauseNav() ));  
    connect( cancel_btn, SIGNAL( clicked() ), this, SLOT( cancelNav() ));  
    connect( add_btn, SIGNAL( clicked() ), this, SLOT( addPoint() ));    
    connect( delete_all_btn, SIGNAL( clicked() ), this, SLOT( clearPoint() )); 
    connect( return_btn, SIGNAL( clicked() ), this, SLOT( returnNav() )); 
    connect( comm_btn, SIGNAL( clicked() ), this, SLOT( commNav() )); 
}
void SetPanel::setTips(QString info)
{
  tips_lab->setText("tips:" + info);
}

void SetPanel::add_Navpoint(int index,float x,float y,float yaw){//添加航路点
   PathPoint onePoint;
   onePoint.index = index;
   onePoint.x = x;
   onePoint.y = y;
   onePoint.yaw= yaw;
   point_vec.insert(index,onePoint);//添加一个点
   //cout<<"添加了一个v["<<index<<"]"<<endl;
   if(index<point_vec.size()-1){
     for(point_vecit = point_vec.begin() + index + 1;point_vecit!=point_vec.end();point_vecit++){
       //cout<<"更改序号由"<<point_vecit->index<<"到";
       point_vecit->index = point_vecit->index + 1;
       //cout<<point_vecit->index<<endl;
     }
   }
   remove_all_HBox();
   add_VBox_by_vector();
   setTips("添加了一个航路点");
}
void SetPanel::add_VBox_by_vector(){//从vector数据来创建界面点
  for(point_vecit=point_vec.begin();point_vecit!=point_vec.end();point_vecit++){
    //cout<<point_vecit->index<<","<<point_vecit->x<<","<<point_vecit->y<<endl;
    MyQHBoxLayout *mypointbox = new MyQHBoxLayout(point_vecit->index,point_vecit->x,point_vecit->y,point_vecit->yaw);
    viewbox_v->addLayout(mypointbox);
    connect(mypointbox, SIGNAL(btn_signal(int,int)), this, SLOT( recvbBtnSlot(int,int) ));
    connect(mypointbox, SIGNAL(coordChange_signal(int,float,float,float)), this, SLOT( recvChangeSlot(int,float,float,float) ));
    connect(mypointbox, SIGNAL(error_signal(QString)), this, SLOT( recvErrorSlot(QString) ));
  }  
}



void SetPanel::delete_all_point(int index){//删除操作
  remove_all_HBox(index);
  remove_all_vector(index);
}
void SetPanel::remove_all_HBox(int index){
  int indexcount = viewbox_v->children().size();
  if(index < 0){
    for(int i=0;i<indexcount;i++){
      delete (MyQHBoxLayout*)(viewbox_v->children()[0]);
    }
  }else{
    if(index < viewbox_v->children().size()){
      for(int i=0;i<viewbox_v->children().size();i++){
        MyQHBoxLayout *thislayout = (MyQHBoxLayout*)(viewbox_v->children()[i]);
        if(index == thislayout->GetIndex()){
          delete thislayout;
          setTips("删除了第" + QString::number(index) + "个点");
          break;
        }
      }
    }else{
      setTips("错误");
    }
  }
}//删除界面元素
void SetPanel::remove_all_vector(int index){
  if(index < 0){
    for(;point_vec.size()>0;){
      point_vec.remove(0);
    }
  }else{
    if(index < point_vec.size()){
      point_vec.remove(index);
      for(int i=index;i<point_vec.size();i++){
        point_vec[i].index = point_vec[i].index - 1;
      }
    }else{
      setTips("错误");
    }
  }
}//删除所有数据

void SetPanel::pointCb(const geometry_msgs::PointStampedConstPtr &clicked_point){
  int index = viewbox_v->children().size();
  add_Navpoint(index,clicked_point->point.x,clicked_point->point.y,0);
  Q_EMIT flash_signal();//发送更新信号
}

//槽函数
void SetPanel::addPoint(){
  int index = viewbox_v->children().size();
  add_Navpoint(index,0,0,0);
  Q_EMIT flash_signal();//发送更新信号
}
void SetPanel::get_pregress_slots(float pre){//接受进度函数
  progressbar->set_progressV(int(pre * 100));
}
void SetPanel::clearPoint(){   //清空
  delete_all_point();
  setTips("清空");
  Q_EMIT flash_signal();//发送更新信号
}
void SetPanel::startNav(){  //开始导航
  pub_display->pub_btn_signal(TaskInput::STA_TASK);
}
void SetPanel::pauseNav(){                 //暂停导航
  pub_display->pub_btn_signal(TaskInput::PAUSE_TASK);
   setTips("暂停导航");
}
void SetPanel::cancelNav(){                 //取消导航
pub_display->pub_btn_signal(TaskInput::END_TASK);
   setTips("取消导航");
}
void SetPanel::returnNav(){                 //一键返航
  pub_display->pub_btn_signal(TaskInput::RETRUN_DOCK);
}
//下发任务
void SetPanel::commNav(){
  ROS_INFO("enter send lists");
  pub_display->call_sendPoints(point_vec);
}  
void SetPanel::recvbBtnSlot(int type,int index){    //接收按钮槽
  if(type == OperaType::DEL){
    int indexcount = viewbox_v->children().size();
    if(index < indexcount){
      remove_all_HBox();
      remove_all_vector(index);
      add_VBox_by_vector();
      setTips("删除第" + QString::number(index) + "个航路点,还有" + QString::number(viewbox_v->children().size()) + "个航路点");
    }else{
      setTips("删除错误");
    }
  }else if(type == OperaType::UP){//上移
    if(index != 0){
      float x = point_vec[index].x;
      float y = point_vec[index].y;
      float yaw = point_vec[index].yaw;
      remove_all_vector(index);
      add_Navpoint(index - 1,x,y,yaw);
      setTips("由第" + QString::number(index) + "个航路点上移到第"+ QString::number(index-1) +"个航路点");
    }else{
      setTips("已经是第" + QString::number(index) + "个航路点，无法移动");
    }
  }else if(type == OperaType::DOWN){//下移
    if(index != viewbox_v->children().size() - 1){
      setTips("由第" + QString::number(index) + "个航路点上移到第"+ QString::number(index+1) +"个航路点");
    }else{
      setTips("已经是第" + QString::number(index) + "个航路点，无法移动");
    }
  }
  Q_EMIT flash_signal();//发送更新信号  
}
void SetPanel::recvChangeSlot(int index,float x,float y,float yaw){//修改坐标点信号
   point_vec[index].index = index;
   point_vec[index].x = x;
   point_vec[index].y = y;
   point_vec[index].yaw = yaw;
   Q_EMIT flash_signal();//发送更新信号
   setTips(QString::number(index) + "修改坐标为:" + QString::number(x) +","+ QString::number(y)+","+ QString::number(yaw) );
}
void SetPanel::recvErrorSlot(QString info){//错误提示信号
   setTips(info);
}

void SetPanel::flash_Slot(void){//更新显示的槽函数
  pub_display->pub_markers(point_vec);

  
}


// 重载父类的功能
void SetPanel::save( rviz::Config config ) const
{
    //rviz::Panel::save( config );
    //config.mapSetValue( "Topic", output_topic_ );
}

// 重载父类的功能，加载配置数据
void SetPanel::load( const rviz::Config& config )
{
    //rviz::Panel::load( config );
    //QString topic;
    //if( config.mapGetString( "Topic", &topic ))
    //{
        //output_topic_editor_->setText( topic );
        //updateTopic();
    //}
}


MyQHBoxLayout::MyQHBoxLayout(int index,float coordx, float coordy,float yaw):
  coordx_(coordx),
  coordy_(coordy),
  coordyaw_(yaw),
  index_(index)
{
  index_lab = new QLabel(QString::number(index_,10));
  this->addWidget(index_lab);
  coordx_edit = new QLineEdit(QString::number(coordx_));
  coordy_edit = new QLineEdit(QString::number(coordy_));
  coordyaw_edit = new QLineEdit(QString::number(coordyaw_));
  this->addWidget( coordx_edit );
  this->addWidget( coordy_edit );
  this->addWidget( coordyaw_edit );
  upbtn = new QPushButton("上移");
  downbtn = new QPushButton("下移");
  delbtn = new QPushButton("删除");
  upbtn->setFixedSize(70,25);
  downbtn->setFixedSize(70,25);
  delbtn->setFixedSize(70,25);
  this->addWidget( upbtn );
  //this->addWidget( downbtn );
  this->addWidget( delbtn );

  connect( coordx_edit, SIGNAL( editingFinished() ), this, SLOT( changeX() ));//输入完成x
  connect( coordy_edit, SIGNAL( editingFinished() ), this, SLOT( changeY() ));//输入完成x
  connect( coordyaw_edit, SIGNAL( editingFinished() ), this, SLOT( changeYaw() ));//输入完成x
  connect( upbtn, SIGNAL( clicked() ), this, SLOT( up() ));//上移
  connect( downbtn, SIGNAL( clicked() ), this, SLOT( down() ));//下移
  connect( delbtn, SIGNAL( clicked() ), this, SLOT( del() ));//下移
}
MyQHBoxLayout::~MyQHBoxLayout()
{
  if(delbtn!=NULL)
    delete delbtn;
  if(upbtn!=NULL)
    delete upbtn;
  if(downbtn!=NULL)
    delete downbtn;
  if(coordx_edit!=NULL)
    delete coordx_edit;
  if(coordy_edit!=NULL)
    delete coordy_edit;
  if(coordyaw_edit!=NULL)
    delete coordyaw_edit;
  if(index_lab!=NULL)
    delete index_lab;
}
void MyQHBoxLayout::del(){//删除点
  Q_EMIT btn_signal(OperaType::DEL,index_);
}
void MyQHBoxLayout::up(){//上移一位
  Q_EMIT btn_signal(OperaType::UP,index_);
}
void MyQHBoxLayout::down(){//下移一位
  Q_EMIT btn_signal(OperaType::DOWN,index_);
}
void MyQHBoxLayout::SetIndex(int index){//设置index_
  index_ = index;
  index_lab->setText(QString::number(index_));
}
int MyQHBoxLayout::GetIndex(){//获取index_
  return index_;
}
float MyQHBoxLayout::GetCoordx(){//获取coordx_
  return coordx_;
}
float MyQHBoxLayout::GetCoordy(){//获取coordy_
  return coordy_;
}
float MyQHBoxLayout::GetCoordyaw(){//获取coordyaw_
  return coordyaw_;
}

void MyQHBoxLayout::changeX(){//修改x坐标
  try{
    QString coordx = coordx_edit->text();
    coordx_ = coordx.toFloat();
    Q_EMIT coordChange_signal(index_,coordx_,coordy_,coordyaw_);//发送修改信号
  }catch(int e){
    Q_EMIT error_signal("x坐标，数字格式有误");
  }
}
void MyQHBoxLayout::changeY(){//修改y坐标
  try{
    QString coordy = coordy_edit->text();
    coordy_ = coordy.toFloat();
    Q_EMIT coordChange_signal(index_,coordx_,coordy_,coordyaw_);//发送修改信号
  }catch(int e){
    Q_EMIT error_signal("y坐标数字格式有误");
  }
}
void MyQHBoxLayout::changeYaw(){//修改y坐标
  try{
    QString coordyaw = coordyaw_edit->text();
    coordyaw_ = coordyaw.toFloat();
    Q_EMIT coordChange_signal(index_,coordx_,coordy_,coordyaw_);//发送修改信号
  }catch(int e){
    Q_EMIT error_signal("yaw数字格式有误");
  }
}

MyProgressBar::MyProgressBar(int min_value,int max_value, int init_value){
  progressBar = new QProgressBar();
  txt_progress = new QLabel();
  progressBar->setRange(min_value, max_value);
  progressBar->setValue(init_value);
  set_txt(init_value);
  init_progressclas();
}
MyProgressBar::~MyProgressBar(){
  if(progressBar != NULL)
    delete progressBar;
  if(txt_progress != NULL)
    delete txt_progress;
}
void MyProgressBar::init_progressclas(){//初始化进度条类
  this->addWidget(progressBar);
  //this->addWidget(txt_progress,1, Qt::AlignRight);
}
void MyProgressBar::set_txt(int value){//设置标签值
  txt_progress->setText(QString::number(value) + "%");
}
void MyProgressBar::set_progressV(int value){//设置值
  progressBar->setValue(value);
  set_txt(value);
}

} // end namespace rviz_teleop_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_set_path::SetPanel,rviz::Panel )
// END_TUTORIAL
