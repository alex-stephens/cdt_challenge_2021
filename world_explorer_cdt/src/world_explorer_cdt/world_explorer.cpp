#include <world_explorer_cdt/world_explorer.h>

WorldExplorer::WorldExplorer(ros::NodeHandle &nh)
{
    // Read parameters
    readParameters(nh);

    // Setup subscriber
    map_sub_       = nh.subscribe(input_map_topic_, 1, &WorldExplorer::elevationMapCallback, this);    
    frontiers_sub_ = nh.subscribe(input_frontiers_topic_, 1, &WorldExplorer::frontiersCallback, this);
    graph_sub_     = nh.subscribe(input_graph_topic_, 1, &WorldExplorer::graphCallback, this);

    position_stat_sub_ = nh.subscribe(input_pos_ctrl_topic_, 1, &WorldExplorer::positionCtrlCallback, this);

    // Setup publisher
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>(output_goal_topic_, 10);
    plan_pub_ = nh.advertise<nav_msgs::Path>(output_plan_topic_, 10);

    // Setup service server
    explore_serv_ = nh.advertiseService("start_stop_world_explorer", &WorldExplorer::toggleExploration, this);

    // Execute main loop
    run();
}

bool WorldExplorer::toggleExploration(cdt_msgs::ToggleExploration::Request &req,
                                        cdt_msgs::ToggleExploration::Response & res)
{
    if(req.button_press)
        start_exploring_ = !start_exploring_;

    res.exploration_running = start_exploring_;

    if(start_exploring_)
        ROS_INFO("Button press detected, starting exploration.");
    else if(!start_exploring_)
        ROS_INFO("Button press detected, stopping exploration.");
    else
        ROS_INFO("Button press detected, nothing changing. Something weird is going on.");
    
    return true;
}

void WorldExplorer::readParameters(ros::NodeHandle &nh)
{
    // Depending on the parameter (required or optional) the API differs:

    // input_topic is required (no default topic)
    if (!nh.getParam("input_map_topic", input_map_topic_))
    {
        ROS_ERROR("Could not read parameter `input_map_topic`.");
        exit(-1);
    }
    if (!nh.getParam("input_frontiers_topic", input_frontiers_topic_))
    {
        ROS_ERROR("Could not read parameter `input_frontiers_topic`.");
        exit(-1);
    }
    if (!nh.getParam("input_graph_topic", input_graph_topic_))
    {
        ROS_ERROR("Could not read parameter `input_graph_topic`.");
        exit(-1);
    }
    if (!nh.getParam("input_pos_ctrl_topic", input_pos_ctrl_topic_))
    {
        ROS_ERROR("Could not read parameter `input_pos_ctrl_topic`.");
        exit(-1);
    }    
    if (!nh.getParam("input_base_frame", base_frame_))
    {
        ROS_ERROR("Could not read parameter `input_base_frame`.");
        exit(-1);
    }
    if (!nh.getParam("goal_frame", goal_frame_))
    {
        ROS_ERROR("Could not read parameter `goal_frame`.");
        exit(-1);
    }    
    if (!nh.getParam("planning_time", planning_time_))
    {
        ROS_ERROR("Could not read parameter `planning_time`.");
        exit(-1);
    }        
    if (!nh.getParam("path_execute_limit", path_execute_limit_))
    {
        ROS_ERROR("Could not read parameter `path_execute_limit`.");
        exit(-1);
    }            

    // Setup local planner
    local_planner_.setPlanningTime(planning_time_);
    
    // output topic are optional. They will use default values
    nh.param("output_goal_topic", output_goal_topic_, std::string("/world_explorer/goal"));
    nh.param("output_plan_topic", output_plan_topic_, std::string("/current_plan"));
    nh.param("update_rate", rate_, 1.0); // 1 second default rate
}

void WorldExplorer::elevationMapCallback(const grid_map_msgs::GridMap& in_grid_map)
{
    // Convert grid map and store in internal variable
    grid_map::GridMapRosConverter::fromMessage(in_grid_map, elevation_map_);
    elevation_map_.convertToDefaultStartIndex();
    received_trav_map_ = true;

    // set traversability map in local planner
    local_planner_.setMap(elevation_map_);
    traversability_ = elevation_map_;
}

void WorldExplorer::frontiersCallback(const cdt_msgs::Frontiers& in_frontiers)
{
    // Storing the frontiers is straightforward since the structure is the same
    frontiers_ = in_frontiers;    
    received_frontiers_ = true;
}

void WorldExplorer::graphCallback(const cdt_msgs::Graph& in_graph)
{
    // Set the graph structure to graph planner
    // int num_nodes = graph_.nodes.size();
    // if (num_nodes != in_graph.nodes.size()){
    //     graph_navigation_ = false;
    // }
    graph_planner_.setGraph(in_graph);
    graph_ = in_graph;
}

void WorldExplorer::positionCtrlCallback(const std_msgs::Float32& in_status)
{
    pos_ctrl_status_ = in_status.data;
}

void WorldExplorer::run()
{
    ros::Rate rate(rate_);

    while(ros::ok())
    {
        // Check new data
        if(received_frontiers_ && received_trav_map_ && start_exploring_){
            plan();
            received_frontiers_ = false;
        }

        // Do not remove the spinOnce!
        ros::spinOnce();
        rate.sleep();
    }

}

void WorldExplorer::plan()
{
 

    // We only run the planning if there are frontiers available
    if(frontiers_.frontiers.size() > 0)
    {
        ROS_INFO("Exploooooriiiiiiiiiiiiiing");
        ROS_DEBUG_STREAM("Pos controller status: " << pos_ctrl_status_);

        // Get current position
        double robot_x, robot_y, robot_theta;
        getRobotPose2D(robot_x, robot_y, robot_theta);
        
        // Analyze and sort frontiers
        std::vector<Eigen::Vector2d> goals = local_planner_.searchFrontiers(frontiers_, robot_x, robot_y, robot_theta, graph_);


        // TODO Choose a frontier, work it off if it is valid and send it to the position controller
        // here we just use the first one as an example
        int goal_ind = -1;
        Eigen::Vector2d pose_goal;
        bool have_plan = false;
        do {
            // Local Planner (RRT)
            goal_ind++;
            if (goal_ind >= goals.size()) {
                ROS_INFO("Can't find a suitable frontier with a path");
                break;
            }
            ROS_INFO("Checking %d", goal_ind);
            pose_goal = goals.at(goal_ind);
            if (local_planner_.isInTraversablity(pose_goal))
                have_plan = local_planner_.planPath(robot_x, robot_y, robot_theta, pose_goal, route_);
        } while (!have_plan);


        if (!have_plan) {
            pose_goal = goals.at(0);
            ROS_INFO("Plotting a path with Dijkstra");
            if (!graph_navigation_)
            {
                graph_planner_.planPath(robot_x, robot_y, robot_theta, pose_goal, route_);
                graph_navigation_ = true;
            } else {
                ROS_INFO("Following old plan");
            }
            double dist = 0;
            for (int i = 0; i < route_.size(); i++) {
                dist = std::hypot(robot_x - route_.back().x(), robot_y - route_.back().y());
                if (dist < 0.3) {
                    route_.pop_back();
                } else {
                    break;
                }
            }
        } else {
            graph_navigation_ = false;
        }

        // TODO Graph Planner
        //graph_planner_.planPath(robot_x, robot_y, robot_theta, pose_goal, route_);

        // If we have route targets (frontiers), work them off and send to position controller

        if(route_.size() > 0)
        {
            ROS_INFO("Navigating");
            // Create goal message
            float step = 0.2;
            Eigen::Isometry3d pose1;
            pose1.setIdentity();
            pose1.translate(Eigen::Vector3d(robot_x, robot_y, 0));
            Eigen::Isometry3d pose2;

            geometry_msgs::PoseStamped target;

            int i = 0;
            if (graph_navigation_) {
                // i = route_.size()-1;
                // double min_dist = 10000000;
                // int nav_goal_id = -1;
                // int nearest_goal_id = -1;
                // for(; i >=0; i--) {
                //     double dist = std::hypot(robot_x - route_[i].x(), robot_y - route_[i].y());
                //     pose2.setIdentity();
                //     pose2.translate(Eigen::Vector3d(route_[i].x(), route_[i].y(), 0));
                //     if (nav_goal_id == -1 && dist < 1.6 && local_planner_.isStraightPathValid(pose1, pose2, step)) {
                //         nav_goal_id = i; // set it to the first goal within 
                //     }
                //     if (dist < min_dist) {
                //         min_dist = dist;
                //         nearest_goal_id = i;
                //     }
                // }
                // if (nav_goal_id < 0) { // if we did not find one within lookahead
                //     if (nearest_goal_id >= 0 ) {
                //         nav_goal_id = nearest_goal_id; // go to nearest
                //     } else {
                //         nav_goal_id = 0; // or to start
                //         graph_navigation_ = false; // and replan
                //         ROS_INFO("Forcing replan");
                //     }
                // }
                // i = nav_goal_id;
                // double dist = 0;
                // for (int i = 0; i < route_.size(); i++) {
                //     dist = std::hypot(robot_x - route_.back.x(), robot_y - route_.back.y());
                //     if (dist < 0.3) {
                //         route_.pop_back();
                //     } else {
                //         break;
                //     }
                // }
                target.pose.position.x = route_.back().x();
                target.pose.position.y = route_.back().y();
                target.pose.position.z = 0.25;
                target.header.frame_id = goal_frame_;

            } else {



                bool push_goal_forwards = true;
                while (push_goal_forwards && i < route_.size()) 
                {
                    pose2.setIdentity();
                    pose2.translate(Eigen::Vector3d(route_[i].x(), route_[i].y(), 0));
                    if (!local_planner_.isStraightPathValid(pose1, pose2, step))
                    {
                        break;
                    }
                    i++;
                } 

                if (i == route_.size()) {
                    i--;
                }

                // target.pose.position.x = route_.begin()->x();
                // target.pose.position.y = route_.begin()->y();
                target.pose.position.x = route_[i].x();
                target.pose.position.y = route_[i].y();
                target.pose.position.z = 0.25;
                target.header.frame_id = goal_frame_;
            }

            goal_pub_.publish(target);
            ROS_DEBUG_STREAM("Sending target " << route_.begin()->transpose());


            // Visualize route (plan)
            nav_msgs::Path plan;
            plan.header.stamp = ros::Time::now(); // Should fix this
            plan.header.frame_id = goal_frame_;
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = robot_x;
            pose.pose.position.y = robot_y;
            pose.pose.position.z = 0.25; // This is to improve the visualization only
            if (graph_navigation_) {
                for(auto carrot : route_){
                    geometry_msgs::PoseStamped pose;
                    pose.pose.position.x = carrot.x();
                    pose.pose.position.y = carrot.y();
                    pose.pose.position.z = 0.25; // This is to improve the visualization only
                    plan.poses.push_back(pose);
                }
                plan.poses.push_back(pose);
                std::reverse(plan.poses.begin(),plan.poses.end());
            } else {
                plan.poses.push_back(pose);
                for(auto carrot : route_){
                    geometry_msgs::PoseStamped pose;
                    pose.pose.position.x = carrot.x();
                    pose.pose.position.y = carrot.y();
                    pose.pose.position.z = 0.25; // This is to improve the visualization only
                    plan.poses.push_back(pose);
                }
            }
            
            plan_pub_.publish(plan);
        } 
    }
    else
    {
        ROS_INFO("No frontiers to go to.");
        // TODONE: Implement something to indicate it ended and optionally go to the home position
        double robot_x, robot_y, robot_theta;
        getRobotPose2D(robot_x, robot_y, robot_theta);
        auto home_pose = graph_.nodes[0].pose;

        Eigen::Vector2d pose_goal = Eigen::Vector2d(home_pose.position.x, home_pose.position.y);
        graph_planner_.planPath(robot_x, robot_y, robot_theta, pose_goal, route_);
        double dist = 0;
        for (int i = 0; i < route_.size(); i++) {
            dist = std::hypot(robot_x - route_.back().x(), robot_y - route_.back().y());
            if (dist < 0.3) {
                route_.pop_back();
            } else {
                break;
            }
        }
        if(route_.size() > 0)
        {
            // Create goal message
            geometry_msgs::PoseStamped target;
            target.pose.position.x = route_.back().x();
            target.pose.position.y = route_.back().y();
            target.pose.position.z = 0.25;
            target.header.frame_id = goal_frame_;
            goal_pub_.publish(target);
            ROS_DEBUG_STREAM("Sending target " << route_.back().transpose());

            // Visualize route (plan)
            nav_msgs::Path plan;
            plan.header.stamp = ros::Time::now(); // Should fix this
            plan.header.frame_id = goal_frame_;
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = robot_x;
            pose.pose.position.y = robot_y;
            pose.pose.position.z = 0.25; // This is to improve the visualization only

            for(auto carrot : route_){
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = carrot.x();
                pose.pose.position.y = carrot.y();
                pose.pose.position.z = 0.25; // This is to improve the visualization only
                plan.poses.push_back(pose);
            }
            plan.poses.push_back(pose);
            std::reverse(plan.poses.begin(),plan.poses.end());

            plan_pub_.publish(plan);
        } 
    }
}

void WorldExplorer::getRobotPose2D(double &x, double &y, double &theta)
{
    // Get current pose using the tf listener
    tf::StampedTransform base_to_map_transform;
    tf_listener_.waitForTransform(goal_frame_, base_frame_,  ros::Time(0), ros::Duration(0.5));
    try
    {
        tf_listener_.lookupTransform(goal_frame_, base_frame_, ros::Time(0), base_to_map_transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    // Extract components from robot pose
    x = base_to_map_transform.getOrigin().getX();
    y = base_to_map_transform.getOrigin().getY();

    // Extract orientation is more involved, since it is a quaternion
    // We'll get some help from Eigen
    // First we create an Eigen quaternion
    Eigen::Quaterniond q(base_to_map_transform.getRotation().getW(),
                         base_to_map_transform.getRotation().getX(),
                         base_to_map_transform.getRotation().getY(),
                         base_to_map_transform.getRotation().getZ());
    // We convert it to an Axis-Angle representation
    // This representation is given by an axis wrt to some coordinate frame, and a rotation along that axis
    Eigen::AngleAxisd axis_angle(q);

    // The value corresponding to the z component is the orientation wrt to the z axis (planar rotation)
    // We need to extract the z component of the axis and multiply it by the angle
    theta = axis_angle.axis().z() * axis_angle.angle();
}
