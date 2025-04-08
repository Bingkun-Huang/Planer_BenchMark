#include "planners/planners.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(kinematic_model));

    std::string planner_ID, planning_pipeline;
    ros::param::param<std::string>("planning_pipeline", planning_pipeline, "ompl");
    ros::param::param<std::string>("default_planner_config", planner_ID, "RRT");
    move_group.setPlannerId(planner_ID);


    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::Point>("goal_position", 10, goalCallback);
    ros::Rate rate(10);

    int total_attempts = 0, successful_plans = 0;
    std::string home_dir = std::getenv("HOME");
    std::string log_path = home_dir + "/benchmark_ws/planning_metrics.txt";
    std::ofstream log_file(log_path, std::ios::app);

    while (ros::ok()) {
        ros::spinOnce();

        if (goal_received) {
            total_attempts++;
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            std::vector<double> start_positions;
            move_group.getCurrentState()->copyJointGroupPositions(joint_model_group, start_positions);
            ros::Time t0 = ros::Time::now();

            bool success = planToGoal(goal_pose, move_group, my_plan, planning_pipeline, joint_model_group);
            double planning_time = getPlanningTime(my_plan);

            if (success) 
            {
                successful_plans++;
                std::vector<double> end_positions = my_plan.trajectory_.joint_trajectory.points.back().positions;
                double avg_joint_movement = calculateAvgJointMovement(my_plan.trajectory_.joint_trajectory);
                double frechet = compareTrajectoryWithStraightLine(my_plan.trajectory_.joint_trajectory);
                double energy = calculateEnergyConsumption(my_plan.trajectory_.joint_trajectory);
                double njs = calculateNormalizedJerkScoreCartesian(move_group, my_plan.trajectory_.joint_trajectory);
                double ee_distance = calculateEndEffectorDistance(move_group, my_plan.trajectory_.joint_trajectory);
                double tcp_speed_var = calculateVarianceOfTCPSpeed(my_plan.trajectory_.joint_trajectory, robot_state, joint_model_group);
                ROS_WARN("----------------------------------------");
                ROS_INFO("Planning Time: %.4f s", planning_time);
                ROS_INFO("Avg Joint Movement: %.4f rad", avg_joint_movement);
                ROS_INFO("NJS: %.4f", njs);
                //ROS_INFO("[Metrics] Energy: %.4f", energy);
                ROS_INFO("Frechet: %.4f", frechet);
                ROS_INFO("TCP Speed Variance: %.4f cm^2/s^2", tcp_speed_var);
                ROS_INFO("TCP traveled Distance: %.4f m", ee_distance);
                // ROS_INFO("[Metrics] Success Rate: %.2f%%", 100.0 * successful_plans / total_attempts);
                ROS_WARN("----------------------------------------");

                if (log_file.is_open()) 
{
    std::string planner_id_for_log = (planning_pipeline == "ompl") ? move_group.getPlannerId() : planning_pipeline;

    log_file << "----------------------------------------\n"
             << "Planner ID: " << planner_id_for_log << "\n"
             << "Planning Time: " << std::fixed << std::setprecision(4) << planning_time << " s\n"
             << "Avg Joint Movement: " << avg_joint_movement << " rad\n"
             << "NJS: " << njs << "\n"
             //<< "Energy: " << energy << "\n"
             //<< "Frechet Distance: " << frechet << "\n"
             << "TCP Speed Variance: " << tcp_speed_var << " cm^2/s^2\n"
             << "TCP traveled Distance: " << ee_distance << " m\n"
             //<< "Success Rate: " << (100.0 * successful / total) << "%\n"
             << "----------------------------------------\n";
    log_file.flush();
    
}
                visual_tools.publishTrajectoryLine(my_plan.trajectory_, move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP));
                visual_tools.trigger();
                visual_tools.prompt("Press 'next' in RViz to execute the motion");
                move_group.execute(my_plan);
            }
            goal_received = false;
        }
        rate.sleep();
    }

    if (log_file.is_open()) 
    {
        log_file.close();
    }

    ros::shutdown();
    return 0;
}
