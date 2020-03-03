#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/force_torque_sensor_interface.h>

#include <kinova_hardware_interface/array_torque_sensors.h>

class KinovaMico : public hardware_interface::RobotHW
{
public:
  KinovaMico()
  {

    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_a("m1n6s200_joint_1", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b("m1n6s200_joint_2", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_b);

    hardware_interface::JointStateHandle state_handle_c("m1n6s200_joint_3", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_c);

    hardware_interface::JointStateHandle state_handle_d("m1n6s200_joint_4", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(state_handle_d);

    hardware_interface::JointStateHandle state_handle_e("m1n6s200_joint_5", &pos[4], &vel[4], &eff[4]);
    jnt_state_interface.registerHandle(state_handle_e);

    hardware_interface::JointStateHandle state_handle_f("m1n6s200_joint_6", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface.registerHandle(state_handle_f);

    registerInterface(&jnt_state_interface);

    // connect and register the joint torque interface
    hardware_interface::JointHandle tor_handle_a(jnt_state_interface.getHandle("m1n6s200_joint_1"), &cmd[0]);
    jnt_tor_interface.registerHandle(tor_handle_a);

    hardware_interface::JointHandle tor_handle_b(jnt_state_interface.getHandle("m1n6s200_joint_2"), &cmd[1]);
    jnt_tor_interface.registerHandle(tor_handle_b);

    hardware_interface::JointHandle tor_handle_c(jnt_state_interface.getHandle("m1n6s200_joint_3"), &cmd[2]);
    jnt_tor_interface.registerHandle(tor_handle_c);

    hardware_interface::JointHandle tor_handle_d(jnt_state_interface.getHandle("m1n6s200_joint_4"), &cmd[3]);
    jnt_tor_interface.registerHandle(tor_handle_d);

    hardware_interface::JointHandle tor_handle_e(jnt_state_interface.getHandle("m1n6s200_joint_5"), &cmd[4]);
    jnt_tor_interface.registerHandle(tor_handle_e);

    hardware_interface::JointHandle tor_handle_f(jnt_state_interface.getHandle("m1n6s200_joint_6"), &cmd[5]);
    jnt_tor_interface.registerHandle(tor_handle_f);

    registerInterface(&jnt_tor_interface);

    // connect and register force/torque sensor interface
    hardware_interface::ForceTorqueSensorHandle ft_handle("ft_sensor", "id", frc, trq);
    ft_sensor_interface.registerHandle(ft_handle);

    registerInterface(&ft_sensor_interface);

    // connect and register torque sensors array interface
    hardware_interface::ArrayTorqueSensorsHandle trq_arr_handle("arr_trq_sensors", "id_arr", trq_arr);
    trq_arr_interface.registerHandle(trq_arr_handle);

    registerInterface(&trq_arr_interface);
  }

  void read()
  {
    // std::cout << "I'm reading" << std::endl;
  }

  void write()
  {
    // std::cout << "I'm writing" << std::endl;
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::EffortJointInterface jnt_tor_interface;

  hardware_interface::ForceTorqueSensorInterface ft_sensor_interface;

  hardware_interface::ArrayTorqueSensorsInterface trq_arr_interface;

  double cmd[6];
  double pos[6];
  double vel[6];
  double eff[6];
  double frc[3];
  double trq[3];
  double trq_arr[6];
};

#define RATE 500.0 //ka++

int main(int argc, char **argv)
{
  ros::init(argc, argv, "KinovaMico");
  ros::NodeHandle nh;
  ros::Rate loop_rate(RATE);

  ros::CallbackQueue my_callback_queue;    //ka++
  nh.setCallbackQueue(&my_callback_queue); //ka++

  KinovaMico robot;
  //initialize control manager
  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(2, &my_callback_queue);
  spinner.start();
  //ros::waitForShutdown();//ka++

  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(RATE);

  while (ros::ok())
  {
    //time loop starts
    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;
    // std::cout << "Period: " << period << std::endl;

    prev_time = time; //ka++

    // read robot data
    robot.read();

    // update data
    cm.update(time, period);

    //send data to the robot
    robot.write();

    //ros::spinOnce();

    rate.sleep();
  }

  spinner.stop(); //ka++
  return 0;
}
