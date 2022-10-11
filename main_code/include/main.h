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

///// time for Gazebo
#include <rosgraph_msgs/Clock.h>
///// ROS headers
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <std_msgs/Empty.h>
///// OpenCV image processing
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
    QHBoxLayout *left_hbox1, *right_hbox_stat1, *right_hbox_stat2, *right_hbox_stat3;
    QHBoxLayout *right_hbox_wp1, *right_hbox_wp2, *right_hbox_wp3, *right_hbox_wp4, *right_hbox_wp5, *right_hbox_wp6, *right_hbox_wp7, *right_hbox_wp8, *right_hbox_wp9, *right_hbox_wp10;
    QLabel *left_text1, *left_text2, *left_3rd_img, *left_1st_img;
    QLabel *right_text1, *right_text2, *right_text3, *right_text4;
    QLabel *right_text11, *right_text12, *right_text13, *right_text14, *right_text15, *right_text16, *right_text17, *right_text18, *right_text19, *right_text20;
    QLabel *right_text21, *right_text22, *right_text23, *right_text24, *right_text25, *right_text26, *right_text27, *right_text28, *right_text29, *right_text30;
    QLabel *right_partition, *right_creator, *right_logo;
    QPushButton *refresh_button, *zoom_button;
    vector<QLabel*> waypoint_time_text_vec;

    QPalette palette;
    QFont font;
    QColor cyan=QColor(121,215,252);
    QColor palegreen=QColor(172,252,186);
    QColor palepurple=QColor(228,191,255);
    QColor beige=QColor(255,203,130);
    QColor lightred=QColor(255,77,115);

    cv::Mat third_cam_cv_img, first_cam_cv_img, logo_img;
    string package_path;

    void QT_initialize();
    void qt_img_update(QLabel *label, cv::Mat img);
    void finish_result();
    void refreshing();
    void zoom_in_out();
    void qt_timer_func(); // QT main thread

  public:
    // no meaning for public, just separate ROS and main variables
    gazebo_msgs::ModelState third_cam_pose;
    geometry_msgs::Pose uav_pose;
    gazebo_msgs::SetModelState model_move_srv;
    gazebo_msgs::ApplyBodyWrench model_force_srv;
    std_msgs::Empty empty_msg;
    rosgraph_msgs::Clock current_time, fixed_time;

    bool qt_initialized=false, state_check=false, clock_check=false, third_cam_check=false, first_cam_check=false;
    bool if_finished=false;
    std::string robot_name, third_cam_name, third_cam_topic, first_cam_topic;
    int img_width, img_height;
    double cam_z_offset=5.0;

    //finishing
    double max_time_t=3600.0;
    vector<double> finish_point;
    //class
    int current_class=11;
    //waypoints
    vector<double> waypoints;
    vector<double> waypoints_times;
    vector<double> waypoints_times_tmp;
    //wind points
    vector<double> wind_spec;

    ///// ros and tf
    ros::NodeHandle nh;
    ros::Subscriber states_sub, third_cam_sub, first_cam_sub, clock_sub;
    ros::Publisher spawning_msg_pub;
    ros::ServiceClient model_mover, model_pusher;
    ros::Timer main_timer;

    void cam_move(const geometry_msgs::Pose &pose);
    bool if_passed_finish(const geometry_msgs::Pose &pose, const vector<double> &pt);
    void if_waypoints(const geometry_msgs::Pose &pose);
    void if_wind_disturbance(const geometry_msgs::Pose &pose);
    void main_timer_func(const ros::TimerEvent& event);
    void states_callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void third_cam_callback(const sensor_msgs::CompressedImage::ConstPtr& msg);
    void first_cam_callback(const sensor_msgs::CompressedImage::ConstPtr& msg);
    void clock_callback(const rosgraph_msgs::Clock::ConstPtr& msg);

    khnp_comp(ros::NodeHandle& n, QWidget *parent=0) : nh(n), QWidget(parent){
      ///// params
      nh.param<int>("/img_width", img_width, 480);
      nh.param<int>("/img_height", img_height, 360);
      nh.param<std::string>("/robot_name", robot_name, "/");
      nh.param<std::string>("/third_cam_name", third_cam_name, "third_camera");
      nh.param<std::string>("/third_cam_topic", third_cam_topic, "/third_camera/rgb/image_raw/compressed");
      nh.param<std::string>("/first_cam_topic", first_cam_topic, "/d455/depth/rgb_image_raw/compressed");
      nh.param<double>("/max_time_t", max_time_t, 3600.0);
      nh.getParam("/finish_point", finish_point);
      nh.getParam("/waypoints", waypoints);
      nh.getParam("/windpoints", wind_spec);

      ///// Init
      waypoints_times = vector<double>(waypoints.size()/3, 0.0);
      waypoints_times_tmp = vector<double>(waypoints.size()/3, 0.0);
      package_path = ros::package::getPath("khnp_competition");
      third_cam_pose.model_name=third_cam_name;
      model_force_srv.request.body_name="iris::iris_khnp::base_link";
      model_force_srv.request.duration=ros::Duration(0.2);
      QT_initialize();

      ///// sub pub
      states_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 3, &khnp_comp::states_callback, this);
      third_cam_sub = nh.subscribe<sensor_msgs::CompressedImage>(third_cam_topic, 10, &khnp_comp::third_cam_callback, this);
      first_cam_sub = nh.subscribe<sensor_msgs::CompressedImage>(first_cam_topic, 10, &khnp_comp::first_cam_callback, this);
      clock_sub = nh.subscribe<rosgraph_msgs::Clock>("/clock", 3, &khnp_comp::clock_callback, this);

      spawning_msg_pub = nh.advertise<std_msgs::Empty>("/spawning_model", 1);
      model_mover = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
      model_pusher = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

      //// timers
      main_timer = nh.createTimer(ros::Duration(1/24.4), &khnp_comp::main_timer_func, this); 
      QT_timer_thread = new QTimer(this);
      connect(QT_timer_thread, &QTimer::timeout, [=](){qt_timer_func();} );
      QT_timer_thread->start(42);

      ROS_WARN("class heritated, starting node...");
    }
    ~khnp_comp(){}
};











////////////////////// definitions, can be separated to .cpp files
/////// initializations
void khnp_comp::QT_initialize(){
  ///// Every components
  main_hbox = new QHBoxLayout();
  left_vbox = new QVBoxLayout();
  right_vbox = new QVBoxLayout();
  left_hbox1 = new QHBoxLayout();
  right_hbox_stat1 = new QHBoxLayout();
  right_hbox_stat2 = new QHBoxLayout();
  right_hbox_stat3 = new QHBoxLayout();
  right_hbox_wp1 = new QHBoxLayout(); right_hbox_wp2 = new QHBoxLayout(); right_hbox_wp3 = new QHBoxLayout();
  right_hbox_wp4 = new QHBoxLayout(); right_hbox_wp5 = new QHBoxLayout(); right_hbox_wp6 = new QHBoxLayout();
  right_hbox_wp7 = new QHBoxLayout(); right_hbox_wp8 = new QHBoxLayout(); right_hbox_wp9 = new QHBoxLayout(); right_hbox_wp10 = new QHBoxLayout();  
  left_text1 = new QLabel();
  left_text2 = new QLabel();
  left_3rd_img = new QLabel();
  left_1st_img = new QLabel();
  right_text1 = new QLabel(); right_text2 = new QLabel(); right_text3 = new QLabel(); right_text4 = new QLabel();
  right_text11 = new QLabel(); right_text12 = new QLabel(); right_text13 = new QLabel(); right_text14 = new QLabel(); right_text15 = new QLabel();
  right_text16 = new QLabel(); right_text17 = new QLabel(); right_text18 = new QLabel(); right_text19 = new QLabel(); right_text20 = new QLabel();
  right_text21 = new QLabel(); right_text22 = new QLabel(); right_text23 = new QLabel(); right_text24 = new QLabel(); right_text25 = new QLabel();
  right_text26 = new QLabel(); right_text27 = new QLabel(); right_text28 = new QLabel(); right_text29 = new QLabel(); right_text30 = new QLabel();
  right_partition = new QLabel();
  right_creator = new QLabel();
  right_logo = new QLabel();
  refresh_button = new QPushButton();
  zoom_button = new QPushButton();

  ///// Left components
  left_text1->setText(tr("3rd person view image"));
  left_text1->setAlignment(Qt::AlignCenter);
  left_text1->setAutoFillBackground(true);
  left_text1->setFixedSize(QSize(img_width*0.7,45));
  palette = left_text1->palette();
  palette.setColor(QPalette::Window, cyan);
  left_text1->setPalette(palette);
  font = left_text1->font();
  font.setPointSize(14);
  left_text1->setFont(font);
  left_text1->setFrameStyle(QFrame::Panel | QFrame::Raised);
  left_text1->setLineWidth(3);

  zoom_button->setText("Click to zoom in");
  zoom_button->setAutoFillBackground(true);
  zoom_button->setFixedSize(QSize(img_width*0.27,45));
  font = zoom_button->font();
  font.setPointSize(10);
  zoom_button->setFont(font);
  zoom_button->setStyleSheet("background-color: #FFD36B");

  left_text2->setText(tr("1st person view image"));
  left_text2->setAlignment(Qt::AlignCenter);
  left_text2->setAutoFillBackground(true);
  left_text2->setFixedSize(QSize(img_width,45));
  palette = left_text2->palette();
  palette.setColor(QPalette::Window, cyan);
  left_text2->setPalette(palette);
  font = left_text2->font();
  font.setPointSize(14);
  left_text2->setFont(font);
  left_text2->setFrameStyle(QFrame::Panel | QFrame::Raised);
  left_text2->setLineWidth(3);

  ///// Right components
  refresh_button->setText("Click here only when GUI freezes");
  font = refresh_button->font();
  font.setPointSize(10);
  refresh_button->setFont(font);

  right_text1->setText(tr("Current TIME"));
  right_text1->setAlignment(Qt::AlignCenter);
  right_text1->setAutoFillBackground(true);
  right_text1->setFixedSize(QSize(260,40));
  palette = right_text1->palette();
  palette.setColor(QPalette::Window, palegreen);
  right_text1->setPalette(palette);
  font = right_text1->font();
  font.setPointSize(14);
  right_text1->setFont(font);
  right_text1->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text1->setLineWidth(3);

  right_text2->setText(QString::number(0.0,'f',2));
  right_text2->setAlignment(Qt::AlignCenter);
  right_text2->setAutoFillBackground(true);
  right_text2->setFixedSize(QSize(260,40));
  font = right_text2->font();
  font.setPointSize(13);
  right_text2->setFont(font);

  right_text3->setText(tr("Current CLASS"));
  right_text3->setAlignment(Qt::AlignCenter);
  right_text3->setAutoFillBackground(true);
  right_text3->setFixedSize(QSize(260,40));
  palette = right_text3->palette();
  palette.setColor(QPalette::Window, palegreen);
  right_text3->setPalette(palette);
  font = right_text3->font();
  font.setPointSize(14);
  right_text3->setFont(font);
  right_text3->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text3->setLineWidth(3);

  right_text4->setText(QString::number(current_class,'f',0));
  right_text4->setAlignment(Qt::AlignCenter);
  right_text4->setAutoFillBackground(true);
  right_text4->setFixedSize(QSize(260,40));
  font = right_text4->font();
  font.setPointSize(13);
  right_text4->setFont(font);

  right_partition->setText(tr("Waypoint status are:"));
  right_partition->setAlignment(Qt::AlignCenter);
  right_partition->setAutoFillBackground(true);
  right_partition->setFixedHeight(30);
  font = right_partition->font();
  font.setPointSize(10);
  right_partition->setFont(font);
  right_partition->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_partition->setLineWidth(3);

  right_text11->setText(tr("Waypoint1"));
  right_text11->setAlignment(Qt::AlignCenter);
  right_text11->setAutoFillBackground(true);
  right_text11->setFixedSize(QSize(260,40));
  palette = right_text11->palette();
  palette.setColor(QPalette::Window, beige);
  right_text11->setPalette(palette);
  font = right_text11->font();
  font.setPointSize(12);
  right_text11->setFont(font);
  right_text11->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text11->setLineWidth(2);

  right_text12->setText(tr("NaN"));
  right_text12->setAlignment(Qt::AlignCenter);
  right_text12->setAutoFillBackground(true);
  right_text12->setFixedSize(QSize(260,40));
  font = right_text12->font();
  font.setPointSize(12);
  right_text12->setFont(font);
  waypoint_time_text_vec.push_back(right_text12);

  right_text13->setText(tr("Waypoint2"));
  right_text13->setAlignment(Qt::AlignCenter);
  right_text13->setAutoFillBackground(true);
  right_text13->setFixedSize(QSize(260,40));
  palette = right_text13->palette();
  palette.setColor(QPalette::Window, beige);
  right_text13->setPalette(palette);
  font = right_text13->font();
  font.setPointSize(12);
  right_text13->setFont(font);
  right_text13->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text13->setLineWidth(2);

  right_text14->setText(tr("NaN"));
  right_text14->setAlignment(Qt::AlignCenter);
  right_text14->setAutoFillBackground(true);
  right_text14->setFixedSize(QSize(260,40));
  font = right_text14->font();
  font.setPointSize(12);
  right_text14->setFont(font);
  waypoint_time_text_vec.push_back(right_text14);

  right_text15->setText(tr("Waypoint3"));
  right_text15->setAlignment(Qt::AlignCenter);
  right_text15->setAutoFillBackground(true);
  right_text15->setFixedSize(QSize(260,40));
  palette = right_text15->palette();
  palette.setColor(QPalette::Window, beige);
  right_text15->setPalette(palette);
  font = right_text15->font();
  font.setPointSize(12);
  right_text15->setFont(font);
  right_text15->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text15->setLineWidth(2);

  right_text16->setText(tr("NaN"));
  right_text16->setAlignment(Qt::AlignCenter);
  right_text16->setAutoFillBackground(true);
  right_text16->setFixedSize(QSize(260,40));
  font = right_text16->font();
  font.setPointSize(12);
  right_text16->setFont(font);
  waypoint_time_text_vec.push_back(right_text16);

  right_text17->setText(tr("Waypoint4"));
  right_text17->setAlignment(Qt::AlignCenter);
  right_text17->setAutoFillBackground(true);
  right_text17->setFixedSize(QSize(260,40));
  palette = right_text17->palette();
  palette.setColor(QPalette::Window, beige);
  right_text17->setPalette(palette);
  font = right_text17->font();
  font.setPointSize(12);
  right_text17->setFont(font);
  right_text17->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text17->setLineWidth(2);

  right_text18->setText(tr("NaN"));
  right_text18->setAlignment(Qt::AlignCenter);
  right_text18->setAutoFillBackground(true);
  right_text18->setFixedSize(QSize(260,40));
  font = right_text18->font();
  font.setPointSize(12);
  right_text18->setFont(font);
  waypoint_time_text_vec.push_back(right_text18);

  right_text19->setText(tr("Waypoint5"));
  right_text19->setAlignment(Qt::AlignCenter);
  right_text19->setAutoFillBackground(true);
  right_text19->setFixedSize(QSize(260,40));
  palette = right_text19->palette();
  palette.setColor(QPalette::Window, beige);
  right_text19->setPalette(palette);
  font = right_text19->font();
  font.setPointSize(12);
  right_text19->setFont(font);
  right_text19->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text19->setLineWidth(2);

  right_text20->setText(tr("NaN"));
  right_text20->setAlignment(Qt::AlignCenter);
  right_text20->setAutoFillBackground(true);
  right_text20->setFixedSize(QSize(260,40));
  font = right_text20->font();
  font.setPointSize(12);
  right_text20->setFont(font);
  waypoint_time_text_vec.push_back(right_text20);

  right_text21->setText(tr("Waypoint6"));
  right_text21->setAlignment(Qt::AlignCenter);
  right_text21->setAutoFillBackground(true);
  right_text21->setFixedSize(QSize(260,40));
  palette = right_text21->palette();
  palette.setColor(QPalette::Window, beige);
  right_text21->setPalette(palette);
  font = right_text21->font();
  font.setPointSize(12);
  right_text21->setFont(font);
  right_text21->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text21->setLineWidth(2);

  right_text22->setText(tr("NaN"));
  right_text22->setAlignment(Qt::AlignCenter);
  right_text22->setAutoFillBackground(true);
  right_text22->setFixedSize(QSize(260,40));
  font = right_text22->font();
  font.setPointSize(12);
  right_text22->setFont(font);
  waypoint_time_text_vec.push_back(right_text22);

  right_text23->setText(tr("Waypoint7"));
  right_text23->setAlignment(Qt::AlignCenter);
  right_text23->setAutoFillBackground(true);
  right_text23->setFixedSize(QSize(260,40));
  palette = right_text23->palette();
  palette.setColor(QPalette::Window, beige);
  right_text23->setPalette(palette);
  font = right_text23->font();
  font.setPointSize(12);
  right_text23->setFont(font);
  right_text23->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text23->setLineWidth(2);

  right_text24->setText(tr("NaN"));
  right_text24->setAlignment(Qt::AlignCenter);
  right_text24->setAutoFillBackground(true);
  right_text24->setFixedSize(QSize(260,40));
  font = right_text24->font();
  font.setPointSize(12);
  right_text24->setFont(font);
  waypoint_time_text_vec.push_back(right_text24);

  right_text25->setText(tr("Waypoint8"));
  right_text25->setAlignment(Qt::AlignCenter);
  right_text25->setAutoFillBackground(true);
  right_text25->setFixedSize(QSize(260,40));
  palette = right_text25->palette();
  palette.setColor(QPalette::Window, beige);
  right_text25->setPalette(palette);
  font = right_text25->font();
  font.setPointSize(12);
  right_text25->setFont(font);
  right_text25->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text25->setLineWidth(2);

  right_text26->setText(tr("NaN"));
  right_text26->setAlignment(Qt::AlignCenter);
  right_text26->setAutoFillBackground(true);
  right_text26->setFixedSize(QSize(260,40));
  font = right_text26->font();
  font.setPointSize(12);
  right_text26->setFont(font);
  waypoint_time_text_vec.push_back(right_text26);

  right_text27->setText(tr("Waypoint9"));
  right_text27->setAlignment(Qt::AlignCenter);
  right_text27->setAutoFillBackground(true);
  right_text27->setFixedSize(QSize(260,40));
  palette = right_text27->palette();
  palette.setColor(QPalette::Window, beige);
  right_text27->setPalette(palette);
  font = right_text27->font();
  font.setPointSize(12);
  right_text27->setFont(font);
  right_text27->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text27->setLineWidth(2);

  right_text28->setText(tr("NaN"));
  right_text28->setAlignment(Qt::AlignCenter);
  right_text28->setAutoFillBackground(true);
  right_text28->setFixedSize(QSize(260,40));
  font = right_text28->font();
  font.setPointSize(12);
  right_text28->setFont(font);
  waypoint_time_text_vec.push_back(right_text28);

  right_text29->setText(tr("Waypoint10"));
  right_text29->setAlignment(Qt::AlignCenter);
  right_text29->setAutoFillBackground(true);
  right_text29->setFixedSize(QSize(260,40));
  palette = right_text29->palette();
  palette.setColor(QPalette::Window, beige);
  right_text29->setPalette(palette);
  font = right_text29->font();
  font.setPointSize(12);
  right_text29->setFont(font);
  right_text29->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text29->setLineWidth(2);

  right_text30->setText(tr("NaN"));
  right_text30->setAlignment(Qt::AlignCenter);
  right_text30->setAutoFillBackground(true);
  right_text30->setFixedSize(QSize(260,40));
  font = right_text30->font();
  font.setPointSize(12);
  right_text30->setFont(font);
  waypoint_time_text_vec.push_back(right_text30);

  QString creator = "Maintainers (Report any bugs please)\n\nEungchang Mason Lee (email: engcang93@gmail.com)\nJunho Choi (email: cjh6685kr@gmail.com)";
  right_creator->setText(creator);
  right_creator->setAlignment(Qt::AlignCenter);
  right_creator->setAutoFillBackground(true);
  right_creator->setFixedSize(QSize(335,90));
  palette = right_creator->palette();
  palette.setColor(QPalette::Window, palepurple);
  right_creator->setPalette(palette);
  font = right_creator->font();
  font.setPointSize(9);
  right_creator->setFont(font);
  right_creator->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_creator->setLineWidth(3);

  logo_img = cv::imread(package_path + "/resources/khnp.jpg");
  cv::resize( logo_img, logo_img, cv::Size( logo_img.cols/2, logo_img.rows/2 ));
  cv::cvtColor(logo_img, logo_img, CV_BGR2RGB);
  QImage imgIn= QImage((uchar*) logo_img.data, logo_img.cols, logo_img.rows, logo_img.step, QImage::Format_RGB888);
  QPixmap pixmap = QPixmap::fromImage(imgIn);
  right_logo->setPixmap(pixmap);

  ///// Setting layout
  left_hbox1->addWidget(left_text1);
  left_hbox1->addWidget(zoom_button);
  left_vbox->addLayout(left_hbox1);
  left_vbox->addWidget(left_3rd_img);
  left_vbox->addWidget(left_text2);
  left_vbox->addWidget(left_1st_img);
  left_vbox->setAlignment(Qt::AlignCenter);

  right_hbox_stat1->addWidget(right_text1);
  right_hbox_stat1->addWidget(right_text2);
  right_hbox_stat2->addWidget(right_text3);
  right_hbox_stat2->addWidget(right_text4);
  right_hbox_wp1->addWidget(right_text11);
  right_hbox_wp1->addWidget(right_text12);
  right_hbox_wp2->addWidget(right_text13);
  right_hbox_wp2->addWidget(right_text14);
  right_hbox_wp3->addWidget(right_text15);
  right_hbox_wp3->addWidget(right_text16);
  right_hbox_wp4->addWidget(right_text17);
  right_hbox_wp4->addWidget(right_text18);
  right_hbox_wp5->addWidget(right_text19);
  right_hbox_wp5->addWidget(right_text20);
  right_hbox_wp6->addWidget(right_text21);
  right_hbox_wp6->addWidget(right_text22);
  right_hbox_wp7->addWidget(right_text23);
  right_hbox_wp7->addWidget(right_text24);
  right_hbox_wp8->addWidget(right_text25);
  right_hbox_wp8->addWidget(right_text26);
  right_hbox_wp9->addWidget(right_text27);
  right_hbox_wp9->addWidget(right_text28);
  right_hbox_wp10->addWidget(right_text29);
  right_hbox_wp10->addWidget(right_text30);
  right_hbox_stat3->addWidget(right_creator);
  right_hbox_stat3->addWidget(right_logo);
  right_hbox_stat3->setAlignment(Qt::AlignCenter);

  right_vbox->addWidget(refresh_button);
  right_vbox->addLayout(right_hbox_stat1);
  right_vbox->addLayout(right_hbox_stat2);
  right_vbox->addWidget(right_partition);  
  right_vbox->addLayout(right_hbox_wp1);
  right_vbox->addLayout(right_hbox_wp2);
  right_vbox->addLayout(right_hbox_wp3);
  right_vbox->addLayout(right_hbox_wp4);
  right_vbox->addLayout(right_hbox_wp5);
  right_vbox->addLayout(right_hbox_wp6);
  right_vbox->addLayout(right_hbox_wp7);
  right_vbox->addLayout(right_hbox_wp8);
  right_vbox->addLayout(right_hbox_wp9);
  right_vbox->addLayout(right_hbox_wp10);
  right_vbox->addLayout(right_hbox_stat3);
  right_vbox->setAlignment(Qt::AlignCenter);

  main_hbox->addLayout(left_vbox);
  main_hbox->addLayout(right_vbox);

  setLayout(main_hbox);

  show();
  setWindowTitle(tr("KHNP competition window"));

  connect(refresh_button, &QPushButton::clicked, this, &khnp_comp::refreshing);
  connect(zoom_button, &QPushButton::clicked, this, &khnp_comp::zoom_in_out);

  qt_initialized=true;
}







/////// timer functions
void khnp_comp::main_timer_func(const ros::TimerEvent& event){
  if (qt_initialized && state_check && third_cam_check && first_cam_check){
    cam_move(uav_pose);
    if(!if_finished){

      ///// time check and finish course
      if ((current_time.clock-fixed_time.clock).toSec() >= max_time_t)
      {
        finish_result();
      }
      ///// or, check if passing finish line
      else if (if_passed_finish(uav_pose, finish_point))
      {        
        finish_result();
      }
      ///// or, check if passing waypoint
      else
      {
        if_waypoints(uav_pose);
        if_wind_disturbance(uav_pose);
      }

    }
  }
  else if (!third_cam_check){
    spawning_msg_pub.publish(empty_msg);
  }
  else{
    cout << qt_initialized << state_check << third_cam_check << first_cam_check << endl;
  }
}

void khnp_comp::qt_timer_func(){
  if (qt_initialized && state_check && third_cam_check && first_cam_check){
    if (!third_cam_cv_img.empty() and !first_cam_cv_img.empty()){
      qt_img_update(left_3rd_img, third_cam_cv_img);
      qt_img_update(left_1st_img, first_cam_cv_img);
    }
    if(!if_finished){
      double temp = (current_time.clock-fixed_time.clock).toSec();
      right_text2->setText(QString::number(temp,'f',2));
    }
  }
}






/////// methods
void khnp_comp::refreshing(){
  repaint();
}
void khnp_comp::zoom_in_out(){
  if (cam_z_offset==5.0)
  {    
    zoom_button->setText("Click to zoom out");
    zoom_button->setStyleSheet("background-color: #FF7A7A");
    cam_z_offset=2.0;
  }
  else if (cam_z_offset==2.0)
  {
    zoom_button->setText("Click to zoom in");
    zoom_button->setStyleSheet("background-color: #FFD36B");
    cam_z_offset=5.0;
  }
}
void khnp_comp::cam_move(const geometry_msgs::Pose &pose){
  third_cam_pose.pose = pose;
  third_cam_pose.pose.position.z += cam_z_offset;
  third_cam_pose.pose.orientation.x = 0.0; third_cam_pose.pose.orientation.y = 0.0; third_cam_pose.pose.orientation.z = 0.0; third_cam_pose.pose.orientation.w = 1.0;
  model_move_srv.request.model_state = third_cam_pose;
  model_mover.call(model_move_srv);
}
bool khnp_comp::if_passed_finish(const geometry_msgs::Pose &pose, const vector<double> &pt){
  if (fabs(pose.position.x-pt[0]) < 0.25 && fabs(pose.position.y-pt[1]) < 0.8 && fabs(pose.position.z-pt[2]) < 1.25)
    return true;
  else
    return false;
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
void khnp_comp::finish_result(){
  if(!if_finished){
    right_text1->setText(tr("Finished TIME"));
    palette = right_text1->palette();
    palette.setColor(QPalette::Window, lightred);
    right_text1->setPalette(palette);

    double temp = (current_time.clock-fixed_time.clock).toSec();
    right_text2->setText(QString::number(temp,'f',2));

    right_text3->setText(tr("Finished CLASS"));
    palette = right_text3->palette();
    palette.setColor(QPalette::Window, lightred);
    right_text3->setPalette(palette);

    right_text4->setText(QString::number(current_class,'f',0));

    ofstream result_file(package_path + "/result/result.txt");
    string contents = "Final CLASS: " + to_string(current_class) +
    "\nFinal Time: " + to_string((current_time.clock-fixed_time.clock).toSec());
    for (int i = 0; i < waypoints_times.size(); ++i)
    {
      contents = contents + "\nWaypoint " + to_string(i) + " time: " + to_string(waypoints_times[i]);
    }
    result_file << contents;
    result_file.close();
    
    ROS_WARN("Finished!!!");
    ROS_WARN("Finished!!!");
    ROS_WARN("Finished!!!");
    if_finished=true;
  }
}
void khnp_comp::if_waypoints(const geometry_msgs::Pose &pose){
  double x = pose.position.x;
  double y = pose.position.y;
  double z = pose.position.z;
  bool in_waypoint_=false;

  for (int i = 0; i < waypoints.size()/3; ++i)
  {
    if(sqrt(pow(x-waypoints[i*3],2)+pow(y-waypoints[i*3+1],2)) < 0.6 && fabs(z-waypoints[i*3+2]) < 0.45)
    {
      if (waypoints_times_tmp[i] < 0.0)
      { //nothing      
      }
      else if (waypoints_times_tmp[i]==0.0)
      {
        waypoints_times_tmp[i] = (current_time.clock-fixed_time.clock).toSec();
      }
      else if ( (current_time.clock-fixed_time.clock).toSec() - waypoints_times_tmp[i] >= 1.5 )
      {
        waypoints_times[i] = (current_time.clock-fixed_time.clock).toSec();
        waypoint_time_text_vec[i]->setText(QString::number(waypoints_times[i],'f',2));
        waypoints_times_tmp[i] = -1.0;
        current_class--;
        right_text4->setText(QString::number(current_class,'f',0));
      }
      in_waypoint_=true;
      break;
    }
  }
  if (!in_waypoint_)
  {
    for (int i = 0; i < waypoints_times_tmp.size(); ++i)
    {
      if (waypoints_times_tmp[i] > 0.0) waypoints_times_tmp[i] = 0.0;
    }
  }
}
void khnp_comp::if_wind_disturbance(const geometry_msgs::Pose &pose){
  double x = pose.position.x;
  double y = pose.position.y;
  double z = pose.position.z;

  for (int i = 0; i < wind_spec.size()/5; ++i)
  {
    if (fabs(x-wind_spec[i*5]) < 0.8 && fabs(y-wind_spec[i*5+1]) < 0.8 && fabs(z-wind_spec[i*5+2]) < 1.25)
    {
      if (wind_spec[i*5+3]==0.0)
      {
        model_force_srv.request.wrench.force.x = 1.0 * wind_spec[i*5+4];
        model_force_srv.request.wrench.force.y = 0.0;
      }
      else if (wind_spec[i*5+3]==1.0)
      {
        model_force_srv.request.wrench.force.x = 0.0;
        model_force_srv.request.wrench.force.y = -1.0 * wind_spec[i*5+4];
      }
      else if (wind_spec[i*5+3]==2.0)
      {
        model_force_srv.request.wrench.force.x = 0.0;
        model_force_srv.request.wrench.force.y = 1.0 * wind_spec[i*5+4];
      }
      else if (wind_spec[i*5+3]==3.0)
      {
        model_force_srv.request.wrench.force.x = -1.0 * wind_spec[i*5+4];
        model_force_srv.request.wrench.force.y = 0.0;
      }
      model_force_srv.request.wrench.force.z = 0.0;
      model_pusher.call(model_force_srv);
      break; // no need to inspect more
    }
    else
    {
      model_force_srv.request.wrench.force.x = 0.0;
      model_force_srv.request.wrench.force.y = 0.0;
      model_force_srv.request.wrench.force.z = 0.0;
    }
  }
}








/////// callbacks
void khnp_comp::clock_callback(const rosgraph_msgs::Clock::ConstPtr& msg){
  current_time = *msg;
  if(!clock_check){
    fixed_time=current_time;
  }
  clock_check=true;
}
void khnp_comp::states_callback(const gazebo_msgs::ModelStates::ConstPtr& msg){
  for (int i = 0; i < msg->name.size(); ++i)
  {
    if (msg->name[i]==robot_name){
      uav_pose = msg->pose[i];
      state_check=true;
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