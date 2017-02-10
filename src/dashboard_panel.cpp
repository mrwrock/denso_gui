#include "dashboard_panel.h"
#include <QDebug>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <QFileDialog>
#include <QDateTime>
#include <QTimer>



namespace denso_gui
{
	DensoDashboardPanel::DensoDashboardPanel( QWidget* parent )
	  : rviz::Panel( parent )
	  , denso_gui()
	{
	  ui_.setupUi(this);
		  //SIGNAL connections
		  connect(ui_.launch_node_button, SIGNAL(clicked()), this, SLOT(onLaunchNode()));
		  connect(ui_.shutdown_button, SIGNAL(clicked()), this, SLOT(onShutdownCommand()));
		  connect(ui_.clear_error_button, SIGNAL(clicked()), this, SLOT(onClearErrorCommand()));
		  connect(ui_.ptp_command_button, SIGNAL(clicked()), this, SLOT(onPTPCommand()));
		  connect(ui_.cp_command_button, SIGNAL(clicked()), this, SLOT(onCPCommand()));
		  connect(ui_.speed_button, SIGNAL(clicked()), this, SLOT(onSpeedCommand()));
		  connect(ui_.string_button, SIGNAL(clicked()), this, SLOT(onStringCommand()));
		  connect(ui_.joint_command_button, SIGNAL(clicked()), this, SLOT(onJointCommand()));


	}

	DensoDashboardPanel::~DensoDashboardPanel(){}

	void DensoDashboardPanel::onLaunchNode(){

		  denso_gui.init();
	}
	void DensoDashboardPanel::onPTPCommand(){

		  denso_gui.send_ptp_command(ui_.x_box->text().toFloat(),ui_.y_box->text().toFloat(),ui_.z_box->text().toFloat(),ui_.rx_box->text().toFloat(),ui_.ry_box->text().toFloat(),ui_.rz_box->text().toFloat(),ui_.fig_box->text().toFloat());
	}
	void DensoDashboardPanel::onCPCommand(){

		  denso_gui.send_cp_command(ui_.x_box->text().toFloat(),ui_.y_box->text().toFloat(),ui_.z_box->text().toFloat(),ui_.rx_box->text().toFloat(),ui_.ry_box->text().toFloat(),ui_.rz_box->text().toFloat(),ui_.fig_box->text().toFloat());
	}
	void DensoDashboardPanel::onShutdownCommand(){

		denso_gui.send_shutdown();
		
	}
	void DensoDashboardPanel::onClearErrorCommand(){

		denso_gui.clear_errors();

	}
	void DensoDashboardPanel::onJointCommand(){

		denso_gui.send_joint_command(ui_.j1_box->text().toFloat(),ui_.j2_box->text().toFloat(),ui_.j3_box->text().toFloat(),ui_.j4_box->text().toFloat(),ui_.j5_box->text().toFloat(),ui_.j6_box->text().toFloat());

	}
	void DensoDashboardPanel::onStringCommand(){

		denso_gui.send_string(ui_.string_box->text().toStdString());

	}
	void DensoDashboardPanel::onSpeedCommand(){

		denso_gui.send_speed(ui_.speed_box->text().toFloat());
	
	}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(denso_gui::DensoDashboardPanel,rviz::Panel)
