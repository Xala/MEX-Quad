#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <boost/circular_buffer.hpp>
#include <mavros/OverrideRCIn.h>
#include <mavros/RCIn.h>

mavros::RCIn RCI;
mavros::OverrideRCIn RCO;

class OverrideNode
{
private:
    ros::NodeHandle n;
    ros::Publisher pub_RC;
    ros::Subscriber sub_RC;
    ros::Subscriber sub_RC_To_Quad;
    ros::Rate loop_rate;

public:
    OverrideNode()
        : n("~")
        , loop_rate(1)
    {
        init();
    }

    ~OverrideNode()
    {
    }

    void init()
    {
        pub_RC = n.advertise<mavros::OverrideRCIn>("/mavros/rc/override", 1000);
        sub_RC = n.subscribe("/mavros/rc/in", 1, &OverrideNode::rcOverideOk, this);
        sub_RC_To_Quad = n.subscribe("/rc/in", 1, &OverrideNode::rcCallback, this);
        loop_rate = ros::Rate(5);
    }

    void rcOverideOk(const mavros::RCIn::ConstPtr &msg)
    {
        RCI.channels = msg->channels;
    }

    void rcCallback(const mavros::RCIn::ConstPtr &msg)
    {
        if (RCI.channels[5] > 2000)
        {
            RCO.channels[0] = msg->channels[0];
            RCO.channels[1] = msg->channels[1];
            RCO.channels[2] = msg->channels[2];
            RCO.channels[3] = msg->channels[3];
            RCO.channels[4] = msg->channels[4];
            RCO.channels[6] = msg->channels[6];
            RCO.channels[7] = msg->channels[7];
            RCO.channels[5] = RCI.channels[5];
            pub_RC.publish(RCO);
        }
        else
        {
            RCO.channels[0] = 65535;
            RCO.channels[1] = 65535;
            RCO.channels[2] = 65535;
            RCO.channels[3] = 65535;
            RCO.channels[4] = 65535;
            RCO.channels[5] = 65535;
            RCO.channels[6] = 65535;
            RCO.channels[7] = 65535;
            pub_RC.publish(RCO);
        }
    }

    void run()
    {
        while(ros::ok())
        {
            ros::spinOnce();
            //calc();
            loop_rate.sleep();
        }
    }



  /*void calc()
  {
    static bool timedOut = true;

    if ((ros::Time::now()-last_input).toSec() > timeout)
    {
        if (!timedOut)
        {
            timedOut = true;
            ROS_DEBUG("timeout");
            motorL.I = 0;
            motorR.I = 0;
        }
        ras_arduino_msgs::PWM pwm;
        pub_pwm.publish(pwm);
    }
    else
    {
        timedOut = false;
        double elapsed = loop_rate.expectedCycleTime().toSec();
        double wDesiredL = (twi.linear.x-(0.5*0.238*twi.angular.z))/(0.0975/2);
        double wDesiredR = (twi.linear.x+(0.5*0.238*twi.angular.z))/(0.0975/2);
        double wMeasuredL = ((double) (enc.delta_encoder1)*2*M_PI*10)/360;
        double wMeasuredR = ((double) (enc.delta_encoder2)*2*M_PI*10)/360;

        double errorL = wDesiredL - wMeasuredL;
        double errorR = wDesiredR - wMeasuredR;

        ras_arduino_msgs::PWM pwm;
        pwm.PWM1 =   motorL.update(elapsed, errorL);
        pwm.PWM2 = - motorR.update(elapsed, errorR);
        pub_pwm.publish(pwm);

        if (output_measured_angular)
        {
            std_msgs::Float64 ld; ld.data = wDesiredL;
            std_msgs::Float64 rd; rd.data = wDesiredR;
            std_msgs::Float64 lm; lm.data = wMeasuredL;
            std_msgs::Float64 rm; rm.data = wMeasuredR;

            pub_left_desiredAngular.publish(ld);
            pub_right_desiredAngular.publish(rd);
            pub_left_measuredAngular.publish(lm);
            pub_right_measuredAngular.publish(rm);
        }


    }*/
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "failsafe");
  OverrideNode my_node;
  my_node.run();
}
