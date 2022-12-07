#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "gazebo_msgs/msg/link_states.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Core>


#define M_PI 3.14159265358979323846

using Vector24d = Eigen::Matrix<double, 24, 1>;
using Vector12d = Eigen::Matrix<double, 12, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Matrix12d = Eigen::Matrix<double, 12, 12>;
using Matrix24d = Eigen::Matrix<double, 24, 24>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;

using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using MatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;


class SubscriberNode : public rclcpp::Node
{
public:
  bool isInitialised() const { if(IMU_init && DVL_init && Pressure_init)return true; else return false ;}
  SubscriberNode() : Node("subscriber")
  {
                                                                //      topic name        queue size
    IMU_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("/rexrov/imu",     100,
                                                                       //                 callback function           number of parameters
                                                                       std::bind(&SubscriberNode::callbackIMU, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "subscriber IMU initialized");


    DVL_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/rexrov/dvl_twist", 100,
                                                                                                std::bind(&SubscriberNode::callbackDVL, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "subscriber DVL initialized");

    Pressure_subscriber_ = this->create_subscription<sensor_msgs::msg::FluidPressure>("/rexrov/pressure", 100,

                                                                                        std::bind(&SubscriberNode::callbackPressure, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "subscriber Pressure initialized");

    Truth_subscriber_ = this->create_subscription<gazebo_msgs::msg::LinkStates>("/gazebo/link_states", 100,

                                                                                std::bind(&SubscriberNode::callbackGroundTruth, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "subscriber ground truth initialized");

    State_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/kalmen_filter/state", 100);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
                                     std::bind(&SubscriberNode::publishNews, this));
    
    // timer_prediction_ = this->create_wall_timer(std::chrono::milliseconds(5),
    //                                  std::bind(&SubscriberNode::prediction_step, this));

   
    
    RCLCPP_INFO(this->get_logger(), "publisher started");
    Client_ = this->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");
    while(!Client_->wait_for_service(std::chrono::seconds(1))){
      RCLCPP_WARN(this->get_logger(),"waiting for service to be up....");
          }
    RCLCPP_INFO(this->get_logger(), "client created");
    }
  void update_simulator_state(double px, double py, double pz, double pitch, double yaw, double roll)
  {
    auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    request->state.pose.position.set__x(px);
    request->state.pose.position.set__y(py);
    request->state.pose.position.set__z(pz);
    request->state.set__name("submarine_z");
    tf2::Quaternion Q;

    Q.setRPY(pitch, yaw, roll);

    Q = Q.normalize();

    request->state.pose.orientation.set__w(Q.getW());
    request->state.pose.orientation.set__x(Q.getX());
    request->state.pose.orientation.set__y(Q.getY());
    request->state.pose.orientation.set__z(Q.getZ());

    Client_->async_send_request(request);
  }

protected:

  VectorXd getState(){return state_vector;}
  MatrixXd getCov(){return cov_matrix;}
  void setState(const VectorXd &current_state){state_vector=current_state;}
  void setCov(const MatrixXd &current_cov){cov_matrix=current_cov;}
  double wrapAngle(double angle)
  {
    angle = fmod(angle, (2.0 * M_PI));
    if (angle <= -M_PI)
    {
      angle += (2.0 * M_PI);
    }
    else if (angle > M_PI)
    {
      angle -= (2.0 * M_PI);
    }
    return angle;
  }

  double limit_angle(double angle)
  {
    if (angle > M_PI)
    angle = M_PI;
    else if(angle < -M_PI)
    angle = -M_PI;
    return angle;
  }
double rounding(double n){
  return(round(n*10000.0)/10000.0);
}
private:

  const double INIT_POS_STD = 0;
  const double INIT_VEL_STD = 15;
  const double LINEAR_ACCEL_STD = 0.1;
  const double GPS_POS_STD = 0.0;
  const double dt = 0.02;
  const double ORIANTATION_STD = 0.01 / 180.0 * M_PI;
  const double INIT_PSI_STD = 45.0 / 180.0 * M_PI;
  const double INIT_PSI_ACCEL_STD = 45.0 / 180.0 * M_PI;
  const double IMU_STD = 0.001;
  const double DVL_STD = 0.001;
  const double PRESSURE_STD = 0.001;

  void prediction_step(const sensor_msgs::msg::Imu::SharedPtr msg)
  {


    VectorXd current_state = getState();
    MatrixXd current_cov   = getCov();

    MatrixXd F = Matrix12d::Zero();

    F(0 , 0)=  1;  
    F(1 , 1)=  1;  
    F(2 , 2)=  1;  
    F(3 , 3)=  1;  
    F(4 , 4)=  1;  
    F(5 , 5)=  1;  
    F(6 , 6)=  1;  
    F(7 , 7)=  1;  
    F(8 , 8)=  1;  
    F(9 , 9)=  1;  
    F(10,10)=  1;  
    F(11,11)=  1;
    F(0 , 3)= dt;
    F(1 , 4)= dt;
    F(2 , 5)= dt;
    F(6 , 9)= dt;
    F(7 ,10)= dt;
    F(8 ,11)= dt;
  
    double LinearXAccel = msg->linear_acceleration.x;
    double LinearYAccel = msg->linear_acceleration.y;
    double LinearZAccel = msg->linear_acceleration.z;

    MatrixXd Q = MatrixXd::Zero(6,6);

    Q(0, 0) = (LinearXAccel    *    LinearXAccel);
    Q(1, 1) = (LinearYAccel    *    LinearYAccel);
    Q(2, 2) = (LinearZAccel    *    LinearZAccel);
    Q(3, 3) = (ORIANTATION_STD * ORIANTATION_STD);
    Q(4, 4) = (ORIANTATION_STD * ORIANTATION_STD);
    Q(5, 5) = (ORIANTATION_STD * ORIANTATION_STD);

    MatrixXd L = MatrixXd::Zero(12,6);

    L(0, 0) =(0.5 * dt * dt);
    L(1, 1) =(0.5 * dt * dt);
    L(2, 2) =(0.5 * dt * dt);
    L(3, 0) =             dt;
    L(4, 1) =             dt;
    L(5, 2) =             dt;
    L(6, 3) =(0.5 * dt * dt);
    L(7, 4) =(0.5 * dt * dt);
    L(8, 5) =(0.5 * dt * dt);
    L(9, 3) =             dt;
    L(10,4) =             dt;
    L(11,5) =             dt;

    current_state = F * current_state;
    
    // current_state(6) = wrapAngle(current_state(6));
    // current_state(7) = wrapAngle(current_state(7));
    // current_state(8) = wrapAngle(current_state(8));

    current_cov = F * current_cov * F.transpose() + L * Q * L.transpose();

    // ----------------------------------------------------------------------- //

    setState(current_state);
    setCov(current_cov);

    // --------------------------------------
  }

  void prediction_step_extended(const sensor_msgs::msg::Imu::SharedPtr msg)
  {

    VectorXd current_state = getState();
    MatrixXd current_cov = getCov();

 

    double px      = current_state(0);
    double py      = current_state(1);
    double pz      = current_state(2);
    double vx      = current_state(3);
    double vy      = current_state(4);
    double vz      = current_state(5);
    double thetax  = current_state(6);
    double thetay  = current_state(7);
    double thetaz  = current_state(8);
    double thetavx = current_state(9);
    double thetavy = current_state(10);
    double thetavz = current_state(11);

    double LinearXAccel = msg->linear_acceleration.x;
    double LinearYAccel = msg->linear_acceleration.y;
    double LinearZAccel = msg->linear_acceleration.z;

    double px_new = px + dt*vx*cos(thetaz) - dt*vy*sin(thetaz);
    double py_new = py + dt*vy*cos(thetaz) + dt*vx*sin(thetaz);
    double pz_new = pz + dt*vz;

    // tf2::Quaternion q(
    //     msg->orientation.x,
    //     msg->orientation.y,
    //     msg->orientation.z,
    //     msg->orientation.w);
    // tf2::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);
    // if (dt * vx * cos(thetaz) != 0){
    //   std::cout << "px_new       reading  :" << px + dt * vx * cos(thetaz) - dt * vy * sin(thetaz) << "\n";
    //   std::cout << "py_new       reading  :" << py + dt * vy * cos(thetaz) + dt * vx * sin(thetaz) << "\n";
    //   std::cout << "theta                 :" << thetaz<< "\n";
    // }   



    double vx_new = vx + dt * LinearXAccel;
    double vy_new = vy + dt * LinearYAccel;
    double vz_new = vz ;
   

   //assume vx , vy and vz 0 to debug the other trades of the filter 

    // double vx_new = 0; 
    // double vy_new = 0; 
    // double vz_new = 0;

    double thetax_new = wrapAngle(thetax + dt * msg->angular_velocity.x);
    double thetay_new = wrapAngle(thetay + dt * msg->angular_velocity.y);
    double thetaz_new = wrapAngle(thetaz + dt * msg->angular_velocity.z);
    
 

    current_state << px_new , py_new , pz_new  ,
                     vx_new , vy_new , vz_new  ,
                     thetax_new , thetay_new , thetaz_new  ,
                     thetavx, thetavy, thetavz ;

    MatrixXd F = Matrix12d::Zero();
    MatrixXd Q = Matrix12d::Zero();
  
    F(0, 0)   = 1;
    F(1, 1)   = 1;
    F(2, 2)   = 1;
    F(3, 3)   = 1;
    F(4, 4)   = 1;
    F(5, 5)   = 1;
    F(6, 6)   = 1;
    F(7, 7)   = 1;
    F(8, 8)   = 1;
    F(9, 9)   = 1;
    F(10, 10) = 1;
    F(11, 11) = 1;

    F(0, 3) =  dt * cos(thetaz);
    F(0, 4) = -dt * sin(thetaz);

    F(1, 3) =  dt * sin(thetaz);
    F(1, 4) =  dt * cos(thetaz);

    F(0, 6) = -dt*vx*sin(thetaz)-dt*vy*cos(thetaz);
    F(1, 6) = -dt*vy*sin(thetaz)+dt*vx*cos(thetaz);

    F(2, 5) = dt;
    F(6, 9) = dt;
    F(7,10) = dt;
    F(8,11) = dt;


    Q(3, 3)   = dt*dt*(LinearXAccel * LinearXAccel);
    Q(4, 4)   = dt*dt*(LinearYAccel * LinearYAccel);
    Q(5, 5)   = dt*dt*(LinearZAccel * LinearZAccel);

    Q(6, 6) = dt * dt * (ORIANTATION_STD * ORIANTATION_STD);
    Q(7, 7) = dt * dt * (ORIANTATION_STD * ORIANTATION_STD);
    Q(8, 8) = dt * dt * (ORIANTATION_STD * ORIANTATION_STD);

    Q(9, 9)   = dt*dt*(ORIANTATION_STD * ORIANTATION_STD);
    Q(10, 10) = dt*dt*(ORIANTATION_STD * ORIANTATION_STD);
    Q(11, 11) = dt*dt*(ORIANTATION_STD * ORIANTATION_STD);

    current_cov = F * current_cov * F.transpose() + Q;

    // ----------------------------------------------------------------------- //

    setState(current_state);
    setCov(current_cov);



    // --------------------------------------
  }

  void callbackIMU(const sensor_msgs::msg::Imu::SharedPtr msg)
  {

    // RCLCPP_INFO(this->get_logger(), "Imu  seq: [%d]", msg->header.frame_id);
    // RCLCPP_INFO(this->get_logger(), "Imu  Oriantation x:[%f], y: [%f], z: [%f] ,w: [%f]",
    //             msg->orientation.x,
    //             msg->orientation.y,
    //             msg->orientation.z,
    //             msg->orientation.w);
    // RCLCPP_INFO(this->get_logger(), "INIT: %d", isInitialised());

    
    
  
    if (isInitialised())
    {

      prediction_step_extended(msg);

      VectorXd current_state = getState();
      MatrixXd current_cov = getCov();
    
      VectorXd z = Vector6d();
      MatrixXd H = MatrixXd::Zero(6, 12);
      MatrixXd R = Matrix6d::Zero();

      tf2::Quaternion q(
          msg->orientation.x,
          msg->orientation.y,
          msg->orientation.z,
          msg->orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

 

      // double roll = msg->orientation.x;
      // double pitch= msg->orientation.y;
      // double yaw  = msg->orientation.z;

      // std::cout << "roll:" << roll << " pitch:" << pitch << " yaw:" << yaw<<"\n";

      z<< wrapAngle(roll),
          wrapAngle(pitch),
          wrapAngle(yaw),
          msg->angular_velocity.x,
          msg->angular_velocity.y,
          msg->angular_velocity.z;

      // std::cout << "yaw speed:"<< msg->angular_velocity.z<<"\n";

      H(0,  6) = 1;
      H(1,  7) = 1;
      H(2,  8) = 1;
      H(3,  9) = 1;
      H(4, 10) = 1;
      H(5, 11) = 1;

      R(0, 0) = IMU_STD * IMU_STD;
      R(1, 1) = IMU_STD * IMU_STD;
      R(2, 2) = IMU_STD * IMU_STD;
      
      //adding zeros to angular velocity stds to get direct feedback from readings
      
      R(3, 3) = 0 ;
      R(4, 4) = 0 ;
      R(5, 5) = 0 ;

      VectorXd z_hat = H * current_state;
      VectorXd y = z - z_hat;

      y(0)=wrapAngle(y(0));
      y(1)=wrapAngle(y(1));
      y(2)=wrapAngle(y(2));

      MatrixXd S = H * current_cov * H.transpose() + R;
      MatrixXd K = current_cov * H.transpose() * S.inverse();

      current_state = current_state + K * y;
      current_cov = (MatrixXd::Identity(12, 12) - K * H) * current_cov;

      setState(current_state);
      setCov(current_cov);
   
      // state.pose.pose.orientation.set__x(msg->orientation.x);
      // state.pose.pose.orientation.set__y(msg->orientation.y);
      // state.pose.pose.orientation.set__z(msg->orientation.z);
      // state.pose.pose.orientation.set__w(msg->orientation.w);

      // state.twist.twist.angular.set__x(msg->angular_velocity.x);
      // state.twist.twist.angular.set__y(msg->angular_velocity.y);
      // state.twist.twist.angular.set__z(msg->angular_velocity.z);

    }
    else
    {

      VectorXd current_state = getState();
      MatrixXd current_cov = getCov();

      current_cov(0,  0) = GPS_POS_STD  * GPS_POS_STD;
      current_cov(1,  1) = GPS_POS_STD  * GPS_POS_STD;
      current_cov(2,  2) = GPS_POS_STD  * GPS_POS_STD;
      current_cov(3,  3) = INIT_VEL_STD * INIT_VEL_STD;
      current_cov(4,  4) = INIT_VEL_STD * INIT_VEL_STD;
      current_cov(5,  5) = INIT_VEL_STD * INIT_VEL_STD;
      current_cov(6,  6) = INIT_PSI_STD * INIT_PSI_STD;
      current_cov(7,  7) = INIT_PSI_STD * INIT_PSI_STD;
      current_cov(8,  8) = INIT_PSI_STD * INIT_PSI_STD;
      current_cov(9,  9) = INIT_PSI_ACCEL_STD * INIT_PSI_ACCEL_STD;
      current_cov(10,10) = INIT_PSI_ACCEL_STD * INIT_PSI_ACCEL_STD;
      current_cov(11,11) = INIT_PSI_ACCEL_STD * INIT_PSI_ACCEL_STD;

      tf2::Quaternion q(
          msg->orientation.x,
          msg->orientation.y,
          msg->orientation.z,
          msg->orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);



      // double roll = msg->orientation.x;
      // double pitch = msg->orientation.y;
      // double yaw = msg->orientation.z;

      current_state(6) = wrapAngle(roll);
      current_state(7) = wrapAngle(pitch);
      current_state(8) = wrapAngle(yaw);
      current_state(9)  = msg->angular_velocity.x;
      current_state(10) = msg->angular_velocity.y;
      current_state(11) = msg->angular_velocity.z;

      state.pose.pose.orientation.set__x(roll);
      state.pose.pose.orientation.set__y(pitch);
      state.pose.pose.orientation.set__z(yaw);
    

      state.twist.twist.angular.set__x(msg->angular_velocity.x);
      state.twist.twist.angular.set__y(msg->angular_velocity.y);
      state.twist.twist.angular.set__z(msg->angular_velocity.z);

      setState(current_state);
      setCov(current_cov);

      IMU_init = true;
        }
  }
  void callbackDVL(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
  {
    
    // RCLCPP_INFO(this->get_logger(), "DVL linear velocity in x: [%f], y: [%f], z: [%f]",
    //             msg->twist.twist.linear.x,
    //             msg->twist.twist.linear.y,
    //             msg->twist.twist.linear.z);

    if (isInitialised())
    {

      // state.twist.twist.linear.set__x(msg->twist.twist.linear.x);
      // state.twist.twist.linear.set__y(msg->twist.twist.linear.y);
      // state.twist.twist.linear.set__z(msg->twist.twist.linear.z);

      VectorXd current_state = getState();
      MatrixXd current_cov = getCov();
   

      VectorXd z = Vector3d();
      MatrixXd H = MatrixXd::Zero(3, 12);
      MatrixXd R = Matrix3d::Zero();

      z <<msg->twist.twist.linear.z,
          msg->twist.twist.linear.y,
          msg->twist.twist.linear.x;

      // std::cout << "vz        reading:" << msg->twist.twist.linear.z << "\n";
      // std::cout << "vy        reading:" << msg->twist.twist.linear.y << "\n";
      // std::cout << "vx        reading:" << msg->twist.twist.linear.x << "\n";

      H(0, 3)  = 1;
      H(1, 4)  = 1;
      H(2, 5)  = 1;
 

      R(0, 0) = DVL_STD * DVL_STD;
      R(1, 1) = DVL_STD * DVL_STD;
      R(2, 2) = DVL_STD * DVL_STD;
 

      VectorXd z_hat = H * current_state;
      VectorXd y = z - z_hat;
      MatrixXd S = H * current_cov * H.transpose() + R;
      MatrixXd K = current_cov * H.transpose() * S.inverse();

      current_state = current_state + K * y;
      current_cov = (MatrixXd::Identity(12, 12) - K * H) * current_cov;

      setState(current_state);
      setCov(current_cov);
  
    }
    else
    {

      VectorXd current_state = getState();
      MatrixXd current_cov = getCov();

      current_state(3) = msg->twist.twist.linear.z;
      current_state(4) = msg->twist.twist.linear.y;
      current_state(5) = msg->twist.twist.linear.x;
    

      setState(current_state);
      setCov(current_cov);
     
      DVL_init = true;
     
    }
    }
  void callbackPressure(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
  {

      // RCLCPP_INFO(this->get_logger(), "pressure reading :[%f]" , msg->fluid_pressure);
      // RCLCPP_INFO(this->get_logger(), "pressure variance:[%f]" , msg->variance);

      if (isInitialised())
      {
        VectorXd current_state = getState();
        MatrixXd current_cov = getCov();
  

        double Pgauge = (msg->fluid_pressure) - (93.068);
        // depth in meter
        double h = rounding(Pgauge / (0.99983 * 9.806));
        
        VectorXd z = VectorXd(1); 
        z(0)=-h;
        MatrixXd H = MatrixXd::Zero(1,12);
        MatrixXd R = MatrixXd::Zero(1,1);
        R(0, 0) = PRESSURE_STD * PRESSURE_STD;
        H(0,2) = 1;
        VectorXd z_hat = H * current_state;
        VectorXd y = z - z_hat;
        MatrixXd S = H * current_cov * H.transpose() + R;

        MatrixXd K = current_cov * H.transpose() * S.inverse();

        current_state = current_state + K * y;
        current_cov = (MatrixXd::Identity(12, 12) - K * H) * current_cov;

        // std::cout << "pressure reading:" << msg->fluid_pressure << "\n";
        // std::cout << "Z        reading:" << z<< "\n";
        // std::cout << "Z_hat    reading:" << z_hat << "\n";
        // std::cout << "H        reading:" << H << "\n";
        // std::cout << "y        reading:" << y << "\n";
        // std::cout << "s        reading:" << S << "\n";
        // std::cout << "k        reading:" << K.transpose() << "\n";

        setState(current_state);
        setCov(current_cov);
  
      }
      else
      {

        VectorXd current_state = getState();
        MatrixXd current_cov =getCov();
        float Pgauge = (msg->fluid_pressure) - (93.068);
        // depth in meter
        float h = Pgauge / (0.99983 * 9.806);

        current_state(2) = -h;

        // std::cout<<"pressure reading:"<<h<<"\n";

        setState(current_state);
        setCov(current_cov);
  
        Pressure_init = true;
        
      }
      }

  void callbackGroundTruth(const gazebo_msgs::msg::LinkStates::SharedPtr msg)
  {

    // RCLCPP_INFO(this->get_logger(), "position in x:[%f], y: [%f], z: [%f] ",
    //             msg->pose.data()->position.x,
    //             msg->pose.data()->position.y,
    //             msg->pose.data()->position.z);

    // RCLCPP_INFO(this->get_logger(), "oriantation in x:[%f], y: [%f], z: [%f] ",
    //             msg->pose.data()->orientation.x,
    //             msg->pose.data()->orientation.y,
    //             msg->pose.data()->orientation.z);

    // RCLCPP_INFO(this->get_logger(), "twist linear in x:[%f], y: [%f], z: [%f] ",
    //             msg->twist.data()->linear.x,
    //             msg->twist.data()->linear.y,
    //             msg->twist.data()->linear.z);

    // RCLCPP_INFO(this->get_logger(), "twist angular in x:[%f], y: [%f], z: [%f] ",
    //             msg->twist.data()->angular.x,
    //             msg->twist.data()->angular.y,
    //             msg->twist.data()->angular.z);
  }
  void publishNews()
  {
    // RCLCPP_INFO(this->get_logger(), "position      in x:[%f],y:[%f],z:[%f]",
    //             state.pose.pose.position.x,
    //             state.pose.pose.position.y,
    //             state.pose.pose.position.z );
    // RCLCPP_INFO(this->get_logger(), "oriantation   in x:[%f],y:[%f],z:[%f]",
    //             state.pose.pose.orientation.x,
    //             state.pose.pose.orientation.y,
    //             state.pose.pose.orientation.z);
    // RCLCPP_INFO(this->get_logger(), "linear twist  in x:[%f],y:[%f],z:[%f]",
    //             state.twist.twist.linear.x,
    //             state.twist.twist.linear.y,
    //             state.twist.twist.linear.z);
    // RCLCPP_INFO(this->get_logger(), "angular twist in x:[%f],y:[%f],z:[%f]", 
    //             state.twist.twist.angular.x,
    //             state.twist.twist.angular.y,
    //             state.twist.twist.angular.z);






    RCLCPP_INFO(this->get_logger(), "position      in x:[%f],y:[%f],z:[%f]",
                state_vector(0),
                state_vector(1),
                state_vector(2));
    RCLCPP_INFO(this->get_logger(), "linear twist    in x:[%f],y:[%f],z:[%f]",
                state_vector(3),
                state_vector(4),
                state_vector(5));
    RCLCPP_INFO(this->get_logger(), "oriantation  in x:[%f],y:[%f],z:[%f]",
                state_vector(6),
                state_vector(7),
                state_vector(8));
    RCLCPP_INFO(this->get_logger(), "angular twist in x:[%f],y:[%f],z:[%f]",
                state_vector(9),
                state_vector(10),
                state_vector(11));

    update_simulator_state(state_vector(0),
                           state_vector(1),
                           state_vector(2),
                           state_vector(6),
                           state_vector(7),
                           state_vector(8));
    // tf2::Quaternion myQuaternion;

    // myQuaternion.setRPY(state_vector(6), state_vector(7), state_vector(8));

    // myQuaternion = myQuaternion.normalize();

    State_publisher_->publish(state);
  }

  nav_msgs::msg::Odometry state ;
  Vector12d state_vector = Vector12d::Zero();
  Matrix12d cov_matrix = Matrix12d::Zero();
  bool IMU_init =      false;
  bool DVL_init =      false;
  bool Pressure_init = false;
  
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr IMU_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr DVL_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr Pressure_subscriber_;
  rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr Truth_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr State_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_prediction_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr Client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubscriberNode>();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
