#ifndef KHNP_MAIN_H
#define KHNP_MAIN_H


#include "utility.h"

///// Qt GUI elements
#include <QtCore>
#include <QApplication>
#include <QtWidgets>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QImage>
#include <QPixmap>
#include <QPushButton>
#include <QIcon>

//QT utils
#include <QTimer> // important, or GUI freezes
#include <QFrame>
#include <QString>
#include <QPalette>
#include <QFont>

///// common headers
#include <ros/ros.h>
#include <ros/package.h> // get package_path
#include <iostream> //cout
#include <fstream>
#include <string>
#include <math.h> // pow
#include <vector>

///// Utils
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h> // to Quaternion_to_euler

///// time for Gazebo
#include <rosgraph_msgs/Clock.h>

///// headers
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <std_msgs/Empty.h>

///// image processing
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


using namespace std;


////////////////////////////////////////////////////////////////////////////////////////////////////
class khnp_comp: public QWidget{
  private:
    // no meaning for private, just separated QT variables
    QTimer *QT_timer_thread;
    QHBoxLayout *main_hbox;
    QVBoxLayout *left_vbox, *right_vbox;
    QHBoxLayout *right_hbox1, *right_hbox2, *right_hbox3, *right_hbox4, *right_hbox5, *right_hbox6, *right_hbox_result;
    QLabel *left_text1, *left_text2, *left_3rd_img, *left_1st_img;
    QLabel *right_text1, *right_text2, *right_text3, *right_text4, *right_text9;
    QLabel *right_text5, *right_text6, *right_text7, *right_text8, *right_text10;
    QLabel *right_creator, *right_logo, *right_result;
    QPushButton *refresh_button;

    QPalette palette;
    QFont font;
    QColor cyan=QColor(121,215,252);
    QColor palegreen=QColor(172,252,186);
    QColor palepurple=QColor(228,191,255);
    QColor lightred=QColor(255,77,115);
    int iconsize=100;

    cv::Mat third_cam_cv_img, first_cam_cv_img, logo_img;
    string package_path;

    void QT_initialize();
    void qt_img_update(QLabel *label, cv::Mat img);
    void finish_result();
    void nothing();

    void qt_timer_func();

  public:
    // no meaning for public, just separate ROS and main variables
    gazebo_msgs::ModelStates states;
    gazebo_msgs::ModelState third_cam_pose, uav_pose;
    gazebo_msgs::SetModelState model_move_srv;
    std_msgs::Empty empty_msg;
    rosgraph_msgs::Clock real_current_time, fixed_current_time, fixed_course_time;

    bool initialized=false, qt_initialized=false, state_check=false, clock_check=false, third_cam_check=false, first_cam_check=false;
    bool first_clock_in=false, if_finished=false;
    std::string robot_name, cube_name, third_cam_name, third_cam_topic, first_cam_topic;
    int robot_idx=0, cube_idx=0, img_width, img_height, current_score=0, falldown_score=0;
    position3d tolerance={0.15, 0.6, 0.5};
    position3d cube_tolerance={0.85, 0.65, 1.5};


    ///// ros and tf
    ros::NodeHandle nh;
    ros::Subscriber states_sub, third_cam_sub, first_cam_sub, clock_sub;
    ros::Publisher spawning_msg_pub;
    ros::ServiceClient model_mover;
    ros::Timer main_timer;

    void main_timer_func(const ros::TimerEvent& event);
    void cam_move(geometry_msgs::Pose pose);
    void states_callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void third_cam_callback(const sensor_msgs::CompressedImage::ConstPtr& msg);
    void first_cam_callback(const sensor_msgs::CompressedImage::ConstPtr& msg);
    void clock_callback(const rosgraph_msgs::Clock::ConstPtr& msg);

    khnp_comp(ros::NodeHandle& n, QWidget *parent=0) : nh(n), QWidget(parent){
      ///// params
      nh.param("/img_width", img_width, 480);
      nh.param("/img_height", img_height, 320);
      nh.param<std::string>("/robot_name", robot_name, "/");
      nh.param<std::string>("/cube_name", cube_name, "b2");
      nh.param<std::string>("/third_cam_name", third_cam_name, "third_camera");
      nh.param<std::string>("/third_cam_topic", third_cam_topic, "/third_camera/rgb/image_raw/compressed");
      nh.param<std::string>("/first_cam_topic", first_cam_topic, "/d455/depth/rgb_image_raw/compressed");

      ///// Init
      course_initilization();
      package_path = ros::package::getPath("khnp_competition");
      QT_initialize();
      third_cam_pose.model_name=third_cam_name;
      uav_pose.model_name=robot_name;

      ///// sub pub
      states_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 3, &khnp_comp::states_callback, this);
      third_cam_sub = nh.subscribe<sensor_msgs::CompressedImage>(third_cam_topic, 10, &khnp_comp::third_cam_callback, this);
      first_cam_sub = nh.subscribe<sensor_msgs::CompressedImage>(first_cam_topic, 10, &khnp_comp::first_cam_callback, this);
      clock_sub = nh.subscribe<rosgraph_msgs::Clock>("/clock", 3, &khnp_comp::clock_callback, this);

      spawning_msg_pub = nh.advertise<std_msgs::Empty>("/spawning_model", 1);
      model_mover = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

      //// timers
      main_timer = nh.createTimer(ros::Duration(1/15.0), &khnp_comp::main_timer_func, this); 
      QT_timer_thread = new QTimer(this);
      connect(QT_timer_thread, &QTimer::timeout, [=](){qt_timer_func();} );
      QT_timer_thread->start(80);

      ROS_WARN("class heritated, starting node...");
    }
    ~khnp_comp(){}
};











////////////////////// definitions, can be separated to .cpp files
/////// initializations
void khnp_comp::QT_initialize(){
  logo_img = cv::imread(package_path + "/resources/khnp.jpg");
  cv::resize( logo_img, logo_img, cv::Size( logo_img.cols/2, logo_img.rows/2 ), 0, 0, CV_INTER_NN );
  cv::cvtColor(logo_img, logo_img, CV_BGR2RGB);

  left_text1 = new QLabel();
  left_text2 = new QLabel();
  left_3rd_img = new QLabel();
  left_1st_img = new QLabel();
  refresh_button = new QPushButton();
  right_text1 = new QLabel();
  right_text2 = new QLabel();
  right_text3 = new QLabel();
  right_text4 = new QLabel();
  right_text5 = new QLabel();
  right_text6 = new QLabel();
  right_text7 = new QLabel();
  right_text8 = new QLabel();
  right_text9 = new QLabel();
  right_text10 = new QLabel();
  right_creator = new QLabel();
  right_logo = new QLabel();
  right_result = new QLabel();
  right_hbox1 = new QHBoxLayout();
  right_hbox2 = new QHBoxLayout();
  right_hbox3 = new QHBoxLayout();
  right_hbox4 = new QHBoxLayout();
  right_hbox5 = new QHBoxLayout();
  right_hbox6 = new QHBoxLayout();
  right_hbox_result = new QHBoxLayout();
  left_vbox = new QVBoxLayout();
  right_vbox = new QVBoxLayout();
  main_hbox = new QHBoxLayout();


  refresh_button->setText("click here only when GUI freezed");
  font = refresh_button->font();
  font.setPointSize(10);
  refresh_button->setFont(font);

  left_text1->setText(tr("3rd person view image"));
  left_text1->setAlignment(Qt::AlignCenter);
  left_text1->setAutoFillBackground(true);
  left_text1->setFixedSize(QSize(img_width,50));
  palette = left_text1->palette();
  palette.setColor(QPalette::Window, cyan);
  left_text1->setPalette(palette);
  font = left_text1->font();
  font.setPointSize(14);
  left_text1->setFont(font);
  left_text1->setFrameStyle(QFrame::Panel | QFrame::Raised);
  left_text1->setLineWidth(3);

  left_text2->setText(tr("1st person view image"));
  left_text2->setAlignment(Qt::AlignCenter);
  left_text2->setAutoFillBackground(true);
  left_text2->setFixedSize(QSize(img_width,50));
  palette = left_text2->palette();
  palette.setColor(QPalette::Window, cyan);
  left_text2->setPalette(palette);
  font = left_text2->font();
  font.setPointSize(14);
  left_text2->setFont(font);
  left_text2->setFrameStyle(QFrame::Panel | QFrame::Raised);
  left_text2->setLineWidth(3);



  right_text1->setText(tr("Current score"));
  right_text1->setAlignment(Qt::AlignCenter);
  right_text1->setAutoFillBackground(true);
  right_text1->setFixedSize(QSize(260,50));
  palette = right_text1->palette();
  palette.setColor(QPalette::Window, palegreen);
  right_text1->setPalette(palette);
  font = right_text1->font();
  font.setPointSize(14);
  right_text1->setFont(font);
  right_text1->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text1->setLineWidth(3);

  right_text2->setText(tr("Current stage"));
  right_text2->setAlignment(Qt::AlignCenter);
  right_text2->setAutoFillBackground(true);
  right_text2->setFixedSize(QSize(260,50));
  palette = right_text2->palette();
  palette.setColor(QPalette::Window, palegreen);
  right_text2->setPalette(palette);
  font = right_text2->font();
  font.setPointSize(14);
  right_text2->setFont(font);
  right_text2->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text2->setLineWidth(3);

  right_text3->setText(tr("Course left time"));
  right_text3->setAlignment(Qt::AlignCenter);
  right_text3->setAutoFillBackground(true);
  right_text3->setFixedSize(QSize(260,50));
  palette = right_text3->palette();
  palette.setColor(QPalette::Window, palegreen);
  right_text3->setPalette(palette);
  font = right_text3->font();
  font.setPointSize(14);
  right_text3->setFont(font);
  right_text3->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text3->setLineWidth(3);

  right_text4->setText(tr("Current total time"));
  right_text4->setAlignment(Qt::AlignCenter);
  right_text4->setAutoFillBackground(true);
  right_text4->setFixedSize(QSize(260,50));
  palette = right_text4->palette();
  palette.setColor(QPalette::Window, palegreen);
  right_text4->setPalette(palette);
  font = right_text4->font();
  font.setPointSize(14);
  right_text4->setFont(font);
  right_text4->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text4->setLineWidth(3);

  right_text9->setText(tr("Fall down Penalty"));
  right_text9->setAlignment(Qt::AlignCenter);
  right_text9->setAutoFillBackground(true);
  right_text9->setFixedSize(QSize(260,50));
  palette = right_text9->palette();
  palette.setColor(QPalette::Window, palegreen);
  right_text9->setPalette(palette);
  font = right_text9->font();
  font.setPointSize(14);
  right_text9->setFont(font);
  right_text9->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text9->setLineWidth(3);

  right_text5->setText(QString::number(0.0,'f',7));
  right_text5->setAlignment(Qt::AlignCenter);
  right_text5->setAutoFillBackground(true);
  right_text5->setFixedSize(QSize(260,50));
  font = right_text5->font();
  font.setPointSize(13);
  right_text5->setFont(font);

  right_text6->setText(tr("Starting"));
  right_text6->setAlignment(Qt::AlignCenter);
  right_text6->setAutoFillBackground(true);
  right_text6->setFixedSize(QSize(260,50));
  font = right_text6->font();
  font.setPointSize(13);
  right_text6->setFont(font);

  right_text7->setText(QString::number(0.0,'f',7));
  right_text7->setAlignment(Qt::AlignCenter);
  right_text7->setAutoFillBackground(true);
  right_text7->setFixedSize(QSize(260,50));
  font = right_text7->font();
  font.setPointSize(13);
  right_text7->setFont(font);

  right_text8->setText(QString::number(0.0,'f',7));
  right_text8->setAlignment(Qt::AlignCenter);
  right_text8->setAutoFillBackground(true);
  right_text8->setFixedSize(QSize(260,50));
  font = right_text8->font();
  font.setPointSize(13);
  right_text8->setFont(font);

  right_text10->setText(QString::number(0.0,'f',7));
  right_text10->setAlignment(Qt::AlignCenter);
  right_text10->setAutoFillBackground(true);
  right_text10->setFixedSize(QSize(260,50));
  font = right_text10->font();
  font.setPointSize(13);
  right_text10->setFont(font);

  QString creator = "Maintainers (Report any bugs please)\n\nEungchang Mason Lee (email: engcang93@gmail.com)\nJunho Choi (email: cjh6685kr@gmail.com)";
  right_creator->setText(creator);
  right_creator->setAlignment(Qt::AlignCenter);
  right_creator->setAutoFillBackground(true);
  right_creator->setFixedSize(QSize(360,90));
  palette = right_creator->palette();
  palette.setColor(QPalette::Window, palepurple);
  right_creator->setPalette(palette);
  font = right_creator->font();
  font.setPointSize(9);
  right_creator->setFont(font);
  right_creator->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_creator->setLineWidth(3);

  QImage imgIn= QImage((uchar*) logo_img.data, logo_img.cols, logo_img.rows, logo_img.step, QImage::Format_RGB888);
  QPixmap pixmap = QPixmap::fromImage(imgIn);
  right_logo->setPixmap(pixmap);



  left_vbox->addWidget(left_text1);
  left_vbox->addWidget(left_3rd_img);
  left_vbox->addWidget(left_text2);
  left_vbox->addWidget(left_1st_img);

  right_hbox1->addWidget(right_text1);
  right_hbox1->addWidget(right_text5);
  right_hbox2->addWidget(right_text2);
  right_hbox2->addWidget(right_text6);
  right_hbox3->addWidget(right_text3);
  right_hbox3->addWidget(right_text7);
  right_hbox4->addWidget(right_text4);
  right_hbox4->addWidget(right_text8);
  right_hbox6->addWidget(right_text9);
  right_hbox6->addWidget(right_text10);
  right_hbox5->addWidget(right_creator);
  right_hbox5->addWidget(right_logo);
  right_hbox5->setAlignment(Qt::AlignCenter);
  right_hbox_result->addWidget(right_result);
  right_hbox_result->setAlignment(Qt::AlignCenter);

  right_vbox->addWidget(refresh_button);
  right_vbox->addLayout(right_hbox1);
  right_vbox->addLayout(right_hbox2);
  right_vbox->addLayout(right_hbox3);
  right_vbox->addLayout(right_hbox4);
  right_vbox->addLayout(right_hbox6);
  right_vbox->addLayout(right_hbox5);
  right_vbox->setAlignment(Qt::AlignCenter);
  right_vbox->addLayout(right_hbox_result);


  main_hbox->addLayout(left_vbox);
  main_hbox->addLayout(right_vbox);

  setLayout(main_hbox);

  show();
  setWindowTitle(tr("KHNP competition window"));

  connect(falldown_button, &QPushButton::clicked, this, &khnp_comp::falldown_button_callback);
  connect(pause_button, &QPushButton::clicked, this, &khnp_comp::pause_button_callback);
  connect(reset_button, &QPushButton::clicked, this, &khnp_comp::reset_button_callback);
  connect(skip_button, &QPushButton::clicked, this, &khnp_comp::skip_button_callback);
  connect(refresh_button, &QPushButton::clicked, this, &khnp_comp::nothing);

  qt_initialized=true;
}







/////// timer functions
void khnp_comp::main_timer_func(const ros::TimerEvent& event){
  if (initialized && qt_initialized && state_check && third_cam_check && first_cam_check){
    cam_move(states.pose[robot_idx]);
    if(!if_finished){

      ///// time check and finish course
      ///// or, check if passing finish line
      ///// or, check if passing waypoint

    }
  }
  else if (!third_cam_check){
    spawning_msg_pub.publish(empty_msg);
  }
  else{
    cout << initialized << qt_initialized << state_check << third_cam_check << first_cam_check << endl;
  }
}

void khnp_comp::qt_timer_func(){
  if (initialized && qt_initialized && state_check && third_cam_check && first_cam_check){
    if (!third_cam_cv_img.empty() and !first_cam_cv_img.empty()){
      qt_img_update(left_3rd_img, third_cam_cv_img);
      qt_img_update(left_1st_img, first_cam_cv_img);
    }
    if(!if_finished){
      ros::Duration temp2 = courseAB[current_map].time_limit - (real_current_time.clock-fixed_course_time.clock);
      right_text7->setText(QString::number(temp2.sec + temp2.nsec*1e-9,'f',7));

      double temp = (real_current_time.clock-fixed_current_time.clock).toSec();
      right_text8->setText(QString::number(temp,'f',7));
    }
  }
}






/////// methods
void khnp_comp::cam_move(geometry_msgs::Pose pose){
  third_cam_pose.pose = pose;
  third_cam_pose.pose.position.z += 5.0;
  third_cam_pose.pose.orientation.x = 0.0; third_cam_pose.pose.orientation.y = 0.0; third_cam_pose.pose.orientation.z = 0.0; third_cam_pose.pose.orientation.w = 1.0;
  model_move_srv.request.model_state = third_cam_pose;
  model_mover.call(model_move_srv);
}

void khnp_comp::qt_img_update(QLabel *label, cv::Mat img){
  cv::Mat vis_img;
  if (img.cols != img_width or img.rows != img_height){
    cv::resize(img, vis_img, cv::Size(img_width, img_height));
    cv::cvtColor(vis_img, vis_img, CV_BGR2RGB);
  }
  else{
    cv::cvtColor(img, vis_img, CV_BGR2RGB);
  }
  try{
    QImage imgIn= QImage((uchar*) vis_img.data, vis_img.cols, vis_img.rows, vis_img.step, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(imgIn);
    label->setPixmap(pixmap);
  }
  catch(...){
    return;
  }
}

void khnp_comp::if_time_over(double time_left){
  if (time_left<=0){
    if (move_to_next_map()){
      courseAB[current_map-1].if_passed_map=false;
      char stg_string[20];
      sprintf(stg_string, "%s- %d", courseAB[current_map].name.c_str(), current_course+1);
      right_text6->setText(QString::fromStdString(stg_string));
    }
    if_felldown_flag=false;
  }
}
void khnp_comp::if_started_course(geometry_msgs::Pose pose){
  if(within_range(pose.position, courseAB[current_map].courses[current_course].start_position, tolerance)){
    char stg_string[20];
    sprintf(stg_string, "%s- %d", courseAB[current_map].name.c_str(), current_course+1);
    right_text6->setText(QString::fromStdString(stg_string));
  }
}
void khnp_comp::if_passed_course(geometry_msgs::Pose pose){
  if(within_range(pose.position, courseAB[current_map].courses[current_course].finish_position, tolerance)){
    if(courseAB[current_map].name == manipulator_map.name){
      if (within_range(states.pose[cube_idx].position, courseAB[current_map].courses[current_course].finish_position, cube_tolerance)){
        current_score+=courseAB[current_map].courses[current_course].score;
        right_text5->setText(QString::number(current_score,'f',7));
      }
/*      for (int i = 0; i < cubes_names.size(); ++i){*/
/*        other_pose.model_name = cubes_names[i];*/
/*        other_pose.pose.position.x = cubes_poses[i].x; other_pose.pose.position.y = cubes_poses[i].y; other_pose.pose.position.z = cubes_poses[i].z;*/
/*        other_pose.pose.orientation.x = 0.0; other_pose.pose.orientation.y = 0.0; other_pose.pose.orientation.z = 0.0; other_pose.pose.orientation.w = 1.0;*/
/*        other_pose.twist.linear.x = 0.0; other_pose.twist.linear.y = 0.0; other_pose.twist.linear.z = 0.0;*/
/*        model_move_srv.request.model_state = other_pose;*/
/*        model_mover.call(model_move_srv);*/
/*      }*/
    }
    else{
      current_score+=courseAB[current_map].courses[current_course].score;
      right_text5->setText(QString::number(current_score,'f',7));
    }
    if (current_course+1 < courseAB[current_map].courses.size()){
      current_course+=1;
    }
    else if(current_map+1 < courseAB.size()){
      current_map+=1;
      current_course=0;
      fixed_course_time=real_current_time;
    }
    if (current_course+1 >= courseAB[current_map].courses.size() && current_map+1 >= courseAB.size()){
      finish_result();
    }
  }
}

void khnp_comp::nothing(){
  repaint();
}

void khnp_comp::finish_result(){
  if(!if_finished){
    right_text1->setText(tr("Total score"));
    palette = right_text1->palette();
    palette.setColor(QPalette::Window, lightred);
    right_text1->setPalette(palette);

    right_text4->setText(tr("Finished time"));
    palette = right_text4->palette();
    palette.setColor(QPalette::Window, lightred);
    right_text4->setPalette(palette);
    
    char result_string[300];
    string _one_passed=(courseAB[0].if_passed_map?"Passed":"Failed (or partially)");
    string _two_passed=(courseAB[1].if_passed_map?"Passed":"Failed (or partially)");
    string _three_passed=(courseAB[2].if_passed_map?"Passed":"Failed (or partially)");
    string _four_passed=(courseAB[3].if_passed_map?"Passed":"Failed (or partially)");
    string _five_passed=(courseAB[4].if_passed_map?"Passed":"Failed (or partially)");
    if (!skip_check){
      sprintf(result_string, "Results:\n\n%s: %s \n%s: %s \n%s: %s \n%s: %s \n%s: %s \nPenalty (falldown): %d", //
        courseAB[0].name.c_str(), _one_passed.c_str(), courseAB[1].name.c_str(), _two_passed.c_str(), //
        courseAB[2].name.c_str(), _three_passed.c_str(), courseAB[3].name.c_str(), _four_passed.c_str(), // 
        courseAB[4].name.c_str(), _five_passed.c_str(), falldown_score);
    }
    else{
      sprintf(result_string, "Results:\nYou skipped at least once, score invalid!\n%s: %s \n%s: %s \n%s: %s \n%s: %s \n%s: %s \nPenalty (falldown): %d", //
        courseAB[0].name.c_str(), _one_passed.c_str(), courseAB[1].name.c_str(), _two_passed.c_str(), //
        courseAB[2].name.c_str(), _three_passed.c_str(), courseAB[3].name.c_str(), _four_passed.c_str(), // 
        courseAB[4].name.c_str(), _five_passed.c_str(), falldown_score);
    }
    QString result = QString::fromStdString(result_string);
    right_result->setText(result);
    right_result->setAlignment(Qt::AlignCenter);
    right_result->setAutoFillBackground(true);
    right_result->setFixedSize(QSize(400,180));
    palette = right_result->palette();
    palette.setColor(QPalette::Window, lightred);
    right_result->setPalette(palette);
    font = right_result->font();
    font.setPointSize(11);
    right_result->setFont(font);
    right_result->setFrameStyle(QFrame::Panel | QFrame::Raised);
    right_result->setLineWidth(3);

    double temp = (real_current_time.clock-fixed_current_time.clock).toSec();
    right_text8->setText(QString::number(temp,'f',7));

/*    for (int i = 0; i < cubes_names.size(); ++i){*/
/*      other_pose.model_name = cubes_names[i];*/
/*      other_pose.pose.position.x = cubes_poses[i].x; other_pose.pose.position.y = cubes_poses[i].y; other_pose.pose.position.z = cubes_poses[i].z;*/
/*      other_pose.pose.orientation.x = 0.0; other_pose.pose.orientation.y = 0.0; other_pose.pose.orientation.z = 0.0; other_pose.pose.orientation.w = 1.0;*/
/*      other_pose.twist.linear.x = 0.0; other_pose.twist.linear.y = 0.0; other_pose.twist.linear.z = 0.0;*/
/*      model_move_srv.request.model_state = other_pose;*/
/*      model_mover.call(model_move_srv);*/
/*    }*/
    for (int i = 0; i < spheres_names.size(); ++i){
      other_pose.model_name = spheres_names[i];
      other_pose.pose.position.x = spheres_poses[i].x; other_pose.pose.position.y = spheres_poses[i].y; other_pose.pose.position.z = spheres_poses[i].z;
      other_pose.pose.orientation.x = 0.0; other_pose.pose.orientation.y = 0.0; other_pose.pose.orientation.z = 0.0; other_pose.pose.orientation.w = 1.0;
      other_pose.twist.linear.x = 0.0; other_pose.twist.linear.y = 0.0; other_pose.twist.linear.z = 0.0;
      model_move_srv.request.model_state = other_pose;
      model_mover.call(model_move_srv);
      spheres_throw_vec[i].if_throw=false;
    }

    if_finished=true;

    ROS_WARN("Finished!!!");
  }
}








/////// callbacks
void khnp_comp::clock_callback(const rosgraph_msgs::Clock::ConstPtr& msg){
  real_current_time = *msg;
  if(!first_clock_in){
    fixed_current_time=real_current_time;
    fixed_course_time=real_current_time;
    first_clock_in=true;
  }
}
void khnp_comp::states_callback(const gazebo_msgs::ModelStates::ConstPtr& msg){
  states = *msg;
  for (int i = 0; i < states.name.size(); ++i)
  {
    if (states.name[i]==robot_name){
      robot_idx=i;
      state_check=true;
    }
    else if (states.name[i]==cube_name){
      cube_idx=i;
    }
  }
}
void khnp_comp::third_cam_callback(const sensor_msgs::CompressedImage::ConstPtr& msg){
  cv_bridge::CvImagePtr third_cam_cv_img_ptr = cv_bridge::toCvCopy(*msg);
  third_cam_cv_img_ptr->image.copyTo(third_cam_cv_img);
  if(!third_cam_check){
    ROS_WARN("%s initialized!", third_cam_name.c_str());
    third_cam_check=true;
  }
}
void khnp_comp::first_cam_callback(const sensor_msgs::CompressedImage::ConstPtr& msg){
  cv_bridge::CvImagePtr first_cam_cv_img_ptr = cv_bridge::toCvCopy(*msg);
  first_cam_cv_img_ptr->image.copyTo(first_cam_cv_img);
  first_cam_check=true;
}





#endif