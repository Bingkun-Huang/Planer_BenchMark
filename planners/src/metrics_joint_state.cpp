#include "metrics/metrics.h"

double calculateAvgJointMovement(const trajectory_msgs::JointTrajectory& trajectory);
double calculateNJSFromAgentPositions(const std::vector<Eigen::Vector3d>& positions, double freq_hz, double outlier_threshold);
double calculateEnergyConsumption(const trajectory_msgs::JointTrajectory& trajectory);
double compareTrajectoryWithStraightLine(const trajectory_msgs::JointTrajectory& traj);
double calculateTrajectoryLength(const trajectory_msgs::JointTrajectory& trajectory);
double calculateEndEffectorDistance(const std::vector<Eigen::Vector3d>& agent_positions) ;
void printTrajectory(const trajectory_msgs::JointTrajectory& traj);
double computeTCPSpeedVarianceCM(const std::vector<Eigen::Vector3d>& positions, double freq_hz = 10.0);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collect_and_stop");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber goal_sub = nh.subscribe("/goal_position",     1, goalCallback);
    ros::Subscriber dist_sub = nh.subscribe("/distance_to_goal", 1, distCallback);
    ros::Subscriber js_sub   = nh.subscribe("/joint_states",     10, jointStateCallback);
    ros::Subscriber agent_sub   = nh.subscribe("/agent_position",    10, agentPositionCallback);

    std::string home_dir = std::getenv("HOME");
    std::string log_path = home_dir + "/benchmark_ws/planning_metrics.txt";
    std::ofstream log_file(log_path, std::ios::app);
    if (!log_file.is_open()) 
    {
        ROS_WARN("Cannot open log file: %s", log_path.c_str());
    }
    double freq_hz = 10.0; // 
    ros::Rate loop(10);
    while (ros::ok()) 
    {
        ros::spinOnce();

        // stop collecting data when distance < 0.006 
        if (collecting_data && (g_distance_to_goal < 0.006)) {
            double planning_time = (ros::Time::now() - start_collect_time).toSec();

            size_t traj_size = global_traj.points.size();
            ROS_INFO("---------------------------------");
            ROS_INFO("Collected trajectory size=%zu (points)", traj_size);
            if (traj_size < 2) 
            {
                ROS_WARN("Not enough data => Metrics will be 0 or invalid!");
            }
            //printTrajectory(global_traj);

            double avg_movement = calculateAvgJointMovement(global_traj);
            double njs          = calculateNJSFromAgentPositions(tcp_positions,10.0, 0.08);
            double energy       = calculateEnergyConsumption(global_traj);
            double frechet      = compareTrajectoryWithStraightLine(global_traj);
            double tcp_var = computeTCPSpeedVarianceCM(tcp_positions, freq_hz);
            double distance_m = calculateEndEffectorDistance(tcp_positions);

            ROS_INFO("---------------------------------");
            ROS_INFO("PlanningTime: %.3f s", planning_time);
            ROS_INFO("AvgJointMovement: %.4f", avg_movement);
            ROS_INFO("NJS: %.4f", njs);
            //ROS_INFO("Energy: %.4f", energy);
            ROS_INFO("Frechet: %.4f", frechet);
            ROS_INFO("TCP Speed Variance: %.4f", tcp_var);
            ROS_INFO("TCP traveled distance = %.3f m", distance_m);
            ROS_INFO("---------------------------------");

            if (log_file.is_open()) 
            {
                log_file << "----------------------------------------\n"
                         << "Planner ID: Multi_agent\n"
                         << "Planning Time: " << std::fixed << std::setprecision(4) << planning_time << " s\n"
                         << "Avg Joint Movement: " << std::fixed << std::setprecision(4) << avg_movement << " rad\n"
                         << "NJS: " << std::fixed << std::setprecision(4) << njs << "\n"
                         //<< "Energy: " << std::fixed << std::setprecision(4) << energy << "\n"
                         //<< "Frechet Distance: " << std::fixed << std::setprecision(4) << frechet << "\n"
                         << "TCP Speed Variance: " << std::fixed << std::setprecision(4) << tcp_var << " cm^2/s^2\n"
                         << "TCP traveled Distance: " << std::fixed << std::setprecision(4) << distance_m << " m\n"
                         << "----------------------------------------\n";
                log_file.flush();
            }


            collecting_data = false;
            goal_received = false;
            g_distance_to_goal = 999.0; 
        }

        loop.sleep();
    }

    if (log_file.is_open()) {
        log_file.close();
    }

    ros::shutdown();
    return 0;
}