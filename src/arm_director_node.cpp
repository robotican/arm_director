/*******************************************************************************
* Copyright (c) 2018, RoboTICan, LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of RoboTICan nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
/* Author: Elhay Rauper*/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <dxl_interface/motors_builder.h>

#define LOOP_HZ 100.0

// ************* ROTATION1 ************
#define ROTATION1_ID 1
#define ROTATION1_MAX_VEL 0.5
#define ROTATION1_MAX_POS M_PI / 4
#define ROTATION1_MIN_POS -M_PI / 4
#define ROTATION1_AXIS 2

// ************* SHOULDER1 ************
#define SHOULDER1_ID 2
#define SHOULDER1_MAX_VEL 0.5
#define SHOULDER1_MAX_POS M_PI / 4
#define SHOULDER1_MIN_POS -M_PI / 4
#define SHOULDER1_AXIS 3

// ************* SHOULDER2 ************
#define SHOULDER2_ID 3
#define SHOULDER2_MAX_VEL 0.5
#define SHOULDER2_MAX_POS M_PI / 4
#define SHOULDER2_MIN_POS -M_PI / 4
#define SHOULDER2_AXIS 3

// ************* ROTATION2 ************
#define ROTATION2_ID 4
#define ROTATION2_MAX_VEL 0.5
#define ROTATION2_MAX_POS M_PI / 4
#define ROTATION2_MIN_POS -M_PI / 4
#define ROTATION2_AXIS 2

// ************* SHOULDER3 ************
#define SHOULDER3_ID 5
#define SHOULDER3_MAX_VEL 0.5
#define SHOULDER3_MAX_POS M_PI / 4
#define SHOULDER3_MIN_POS -M_PI / 4
#define SHOULDER3_AXIS 1

// ************* WRIST ************
#define WRIST_ID 6
#define WRIST_MAX_VEL 0.5
#define WRIST_MAX_POS M_PI / 4
#define WRIST_MIN_POS -M_PI / 4
#define WRIST_AXIS  0


// ************* GRIPPER ************
#define GRIPPER_ID 20
#define GRIPPER_MAX_VEL 0.5
#define GRIPPER_MAX_POS 0.100 //0.109
#define GRIPPER_MIN_POS 0
#define GRIPPER_OPEN_AXIS  4
#define GRIPPER_CLOSE_AXIS  5


//************** DEAD_BAND *************
#define FINE_BTN 6
#define ROUGH_BTN 7


dxl::MotorsBuilder *motors;

double gripper_pos_cmd = 0;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    if (joy_msg->buttons[FINE_BTN] && !joy_msg->buttons[ROUGH_BTN])
    {
        // shoulder 3
        dxl::Motor shoulder3;
        motors->getMotor(SHOULDER3_ID, shoulder3);

        double shoulder3_vel_cmd = joy_msg->axes[SHOULDER3_AXIS] * SHOULDER3_MAX_VEL;

        if (shoulder3.position >= SHOULDER3_MAX_POS && shoulder3_vel_cmd > 0)
        {
            ROS_WARN("shoulder3 reached max pos limit: %f", shoulder3.position * 180 / M_PI);
            shoulder3_vel_cmd = 0;
        }
        else if (shoulder3.position <= SHOULDER3_MIN_POS && shoulder3_vel_cmd < 0)
        {
            ROS_WARN("shoulder3 reached min pos limit: %f", shoulder3.position * 180 / M_PI);
            shoulder3_vel_cmd = 0;
        }

        motors->setMotorVelocity(SHOULDER3_ID, shoulder3_vel_cmd);


        // wrist
        dxl::Motor wrist;
        motors->getMotor(WRIST_ID, wrist);

        double wrist_vel_cmd = joy_msg->axes[WRIST_AXIS] * WRIST_MAX_VEL;

        if (wrist.position >= WRIST_MAX_POS && wrist_vel_cmd > 0)
        {
            ROS_WARN("wrist reached max pos limit: %f", wrist.position * 180 / M_PI);

            wrist_vel_cmd = 0;
        }
        else if (wrist.position <= WRIST_MIN_POS && wrist_vel_cmd < 0)
        {
            ROS_WARN("wrist reached min pos limit: %f", wrist.position * 180 / M_PI);

            wrist_vel_cmd = 0;
        }
        motors->setMotorVelocity(WRIST_ID, wrist_vel_cmd);

        // rotation 2
        dxl::Motor rotation2;
        motors->getMotor(ROTATION2_ID, rotation2);

        double rotation2_vel_cmd = joy_msg->axes[ROTATION2_AXIS] * ROTATION2_MAX_VEL;

        if (rotation2.position >= ROTATION2_MAX_POS && rotation2_vel_cmd > 0)
        {
            ROS_WARN("rotation2 reached max pos limit: %f", rotation2.position * 180 / M_PI);

            rotation2_vel_cmd = 0;
        }
        else if (rotation2.position <= ROTATION2_MIN_POS && rotation2_vel_cmd < 0)
        {
            ROS_WARN("rotation2 reached min pos limit: %f", rotation2.position * 180 / M_PI);

            rotation2_vel_cmd = 0;
        }

        motors->setMotorVelocity(ROTATION2_ID, rotation2_vel_cmd);


        // shoulder2
        dxl::Motor shoulder2;
        motors->getMotor(SHOULDER2_ID, shoulder2);

        double shoulder2_vel_cmd = joy_msg->axes[SHOULDER2_AXIS] * SHOULDER2_MAX_VEL;

        if (shoulder2.position >= SHOULDER2_MAX_POS && shoulder2_vel_cmd > 0)
        {
            ROS_WARN("shoulder2 reached max pos limit: %f", shoulder2.position * 180 / M_PI);
            shoulder2_vel_cmd = 0;
        }
        else if (shoulder2.position <= SHOULDER2_MIN_POS && shoulder2_vel_cmd < 0)
        {
            ROS_WARN("shoulder2 reached min limit: %f", shoulder2.position * 180 / M_PI);
            shoulder2_vel_cmd = 0;
        }


        // example of rebooting motor
        if (joy_msg->buttons[3])
        {
            ROS_WARN("REBOOTING");
            if (motors->rebootMotor(ROTATION1_ID))
            {
                ros::Duration(0.1).sleep(); // must wait some time until motor is back to life

                if (motors->setMotorTorque(ROTATION1_ID, true))
                {
                    ROS_INFO("MOTOR TORQUE SUCCESS");
                }
                else
                {
                    ROS_INFO("MOTOR TORQUE FAILED");
                }
            }
        }

        motors->setMotorVelocity(SHOULDER2_ID, shoulder2_vel_cmd);

        motors->setMotorVelocity(SHOULDER1_ID, 0);
        motors->setMotorVelocity(ROTATION1_ID, 0);

    }
    else if(!joy_msg->buttons[FINE_BTN] && joy_msg->buttons[ROUGH_BTN])
    {
        // rostation1
        dxl::Motor rotation1;
        motors->getMotor(ROTATION1_ID, rotation1);

        double rotation1_vel_cmd = joy_msg->axes[ROTATION1_AXIS] * ROTATION1_MAX_VEL;

        if (rotation1.position >= ROTATION1_MAX_POS && rotation1_vel_cmd > 0)
        {
            ROS_WARN("rotation1 reached max limit: %f", rotation1.position * 180 / M_PI);
            rotation1_vel_cmd = 0;
        }
        else if (rotation1.position <= ROTATION1_MIN_POS && rotation1_vel_cmd < 0)
        {
            ROS_WARN("rotation1 reached min limit: %f", rotation1.position * 180 / M_PI);
            rotation1_vel_cmd = 0;
        }

        motors->setMotorVelocity(ROTATION1_ID, rotation1_vel_cmd);

        // shoulder1
        dxl::Motor shoulder1;
        motors->getMotor(SHOULDER1_ID, shoulder1);

        double shoulder1_vel_cmd = joy_msg->axes[SHOULDER1_AXIS] * SHOULDER1_MAX_VEL;

        if (shoulder1.position >= SHOULDER1_MAX_POS && shoulder1_vel_cmd > 0)
        {
            ROS_WARN("shoulder1 reached max limit: %f", shoulder1.position * 180 / M_PI);
            shoulder1_vel_cmd = 0;
        }
        else if (shoulder1.position <= SHOULDER1_MIN_POS && shoulder1_vel_cmd < 0)
        {
            ROS_WARN("shoulder1 reached min limit: %f", shoulder1.position * 180 / M_PI);
            shoulder1_vel_cmd = 0;
        }

        motors->setMotorVelocity(SHOULDER1_ID, shoulder1_vel_cmd);

        motors->setMotorVelocity(SHOULDER2_ID, 0);
        motors->setMotorVelocity(ROTATION2_ID, 0);
        motors->setMotorVelocity(WRIST_ID, 0);
        motors->setMotorVelocity(SHOULDER3_ID, 0);
    }
    else
    {
        motors->setMotorVelocity(SHOULDER1_ID, 0);
        motors->setMotorVelocity(SHOULDER2_ID, 0);
        motors->setMotorVelocity(ROTATION1_ID, 0);

        motors->setMotorVelocity(ROTATION2_ID, 0);
        motors->setMotorVelocity(WRIST_ID, 0);
        motors->setMotorVelocity(SHOULDER3_ID, 0);
    }

    // gripper
    dxl::Motor gripper;
    motors->getMotor(GRIPPER_ID, gripper);


    gripper_pos_cmd += joy_msg->axes[GRIPPER_OPEN_AXIS] * GRIPPER_MAX_VEL * 0.005;
    gripper_pos_cmd -= joy_msg->axes[GRIPPER_CLOSE_AXIS] * GRIPPER_MAX_VEL * 0.005;
    if (gripper_pos_cmd > GRIPPER_MAX_POS) gripper_pos_cmd=GRIPPER_MAX_POS;
    if (gripper_pos_cmd < GRIPPER_MIN_POS) gripper_pos_cmd=GRIPPER_MIN_POS;


    if (gripper.position >= GRIPPER_MAX_POS && gripper_pos_cmd > gripper.position)
    {
        ROS_WARN("gripper reached max limit: %f", gripper.position * 180 / M_PI);
        gripper_pos_cmd = GRIPPER_MAX_POS;
    }
    else if (gripper.position <= GRIPPER_MIN_POS && gripper_pos_cmd < gripper.position)
    {
        ROS_WARN("gripper reached min limit: %f", gripper.position * 180 / M_PI);
        gripper_pos_cmd = GRIPPER_MIN_POS;
    }

    motors->setMotorPosition(GRIPPER_ID, gripper_pos_cmd);

    //ROS_INFO("VEL %f, POS %f", wrist_vel_cmd * 180 / M_PI, motor.position * 180 / M_PI);


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_director_node");
    ros::NodeHandle nh;

    ros::Subscriber joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);


    dxl::MotorsBuilder motors_builder(nh);

    motors = &motors_builder;


    ros::Rate rate2(LOOP_HZ*2.0);


    motors_builder.read();


    motors_builder.setMotorPosition(GRIPPER_ID, 0.05);
    motors_builder.setMotorVelocity(GRIPPER_ID, 0.1);

    gripper_pos_cmd = 0.05;



    while (ros::ok())
    {
        // read all motors state
        motors_builder.read();

        // sleep shortly between read and write to prevent
        // motor communication errors
        rate2.sleep();

        // write commands to motors
        motors_builder.write();

        // sleep shortly between write and read to prevent
        // motor communication errors
        rate2.sleep();

        ros::spinOnce();
    }


    // on exit set all motors to 0 as safety measure
    motors_builder.setMotorVelocity(ROTATION1_ID, 0);
    motors_builder.setMotorVelocity(SHOULDER1_AXIS, 0);
    motors_builder.setMotorVelocity(SHOULDER2_AXIS, 0);
    motors_builder.setMotorVelocity(ROTATION2_ID, 0);
    motors_builder.setMotorVelocity(SHOULDER3_AXIS, 0);
    motors_builder.setMotorVelocity(WRIST_ID, 0);
    motors_builder.setMotorVelocity(GRIPPER_ID, 0);

    return 0;
}