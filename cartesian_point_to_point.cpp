#include <cmath>
#include <iostream>
#include <algorithm>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

int main(int argc, char** argv) {
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname> <x> <y> <z>" << std::endl;
        return -1;
    }

    try {
        franka::Robot robot(argv[1]);
        setDefaultBehavior(robot);

        constexpr double X_MIN = -0.855;
        constexpr double X_MAX = 0.855;
        constexpr double Y_MIN = -0.855;
        constexpr double Y_MAX = 0.855;
        constexpr double Z_MIN = 0.0;
        constexpr double Z_MAX = 1.0;

        double x = std::stod(argv[2]);
        double y = std::stod(argv[3]);
        double z = std::stod(argv[4]);

        if (x < X_MIN || x > X_MAX || y < Y_MIN || y > Y_MAX || z < Z_MIN || z > Z_MAX) {
            std::cerr << "Error: Position (" << x << ", " << y << ", " << z << ") is out of the robot's workspace." << std::endl;
            return -1;
        }

        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        std::cout << "WARNING: This example will move the robot! "
                  << "Please make sure to have the user stop button at hand!" << std::endl
                  << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration." << std::endl;

        robot.setCollisionBehavior({{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                                   {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                                   {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                                   {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

        std::array<double, 16> initial_pose;
        double time = 0.0;
        double total_time = 10.0;  
        double max_velocity = 0.1;  
        double max_acceleration = 0.2;  

        robot.control([&initial_pose, &x, &y, &z, &total_time, &time, &max_velocity, &max_acceleration](const franka::RobotState& robot_state,
                                                                                                      franka::Duration period) -> franka::CartesianPose {
            static double time = 0.0;
            time += period.toSec();

            if (time == 0.0) {
                initial_pose = robot_state.O_T_EE;
            }

            std::array<double, 7> current_joint_positions = robot_state.q;
            std::array<double, 7> current_joint_velocities = robot_state.dq;

            static std::array<double, 7> previous_joint_positions = current_joint_positions;
            static std::array<double, 7> previous_joint_velocities = current_joint_velocities;

            constexpr double max_joint_velocity = 2.0;   
            constexpr double max_joint_acceleration = 1.0; 

            double initial_x = initial_pose[12];
            double initial_y = initial_pose[13];
            double initial_z = initial_pose[14];

            double dx = x - initial_x;
            double dy = y - initial_y;
            double dz = z - initial_z;

            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            double velocity = std::min(max_velocity, distance / (total_time - time));
            double acceleration = std::min(max_acceleration, velocity / (total_time - time));

            double delta_x = initial_x + dx * (time / total_time) - initial_x;
            double delta_y = initial_y + dy * (time / total_time) - initial_y;
            double delta_z = initial_z + dz * (time / total_time) - initial_z;

            std::array<double, 16> new_pose = initial_pose;
            new_pose[12] = initial_x + delta_x;
            new_pose[13] = initial_y + delta_y;
            new_pose[14] = initial_z + delta_z;

            std::array<double, 7> new_joint_velocities = current_joint_velocities;
            for (size_t i = 0; i < 7; i++) {
            double joint_velocity = (current_joint_positions[i] - previous_joint_positions[i]) / period.toSec();

            if (joint_velocity > max_joint_velocity) {
                joint_velocity = max_joint_velocity;
            } else if (joint_velocity < -max_joint_velocity) {
                joint_velocity = -max_joint_velocity;
            }

            double joint_acceleration = (joint_velocity - previous_joint_velocities[i]) / period.toSec();

            if (std::abs(joint_acceleration) > max_joint_acceleration) {
               joint_velocity = previous_joint_velocities[i] + std::copysign(max_joint_acceleration * period.toSec(), joint_acceleration);
            }

            previous_joint_positions[i] = current_joint_positions[i];
            previous_joint_velocities[i] = joint_velocity;
            }

            std::cout << "Time: " << time
                      << ", Position: (" << new_pose[12] << ", " << new_pose[13] << ", " << new_pose[14]
                      << "), Velocity: " << velocity << ", Acceleration: " << acceleration << std::endl;

            if (time >= total_time) {
                std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
                return franka::MotionFinished(new_pose);
            }

            return new_pose;
        });

    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}