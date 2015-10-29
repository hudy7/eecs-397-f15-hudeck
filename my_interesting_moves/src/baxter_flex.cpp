#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <baxter_traj_streamer/trajAction.h>

using namespace std;

//7-dof vector
#define VECTOR_DIM 7 


void doneCb(const actionlib::SimpleClientGoalState& state,
        const baxter_traj_streamer::trajResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d; traj_id = %d",result->return_val,result->traj_id);
}


int main(int argc, char** argv) {
        ros::init(argc, argv, "traj_action_client_node"); 
        ros::NodeHandle nh;         
        int g_count = 0;
        

        Vectorq7x1 q_flex;
        q_flex<< -0.907528, -0.111813,   2.06622,    1.8737,    -1.295,   2.00164,  -2.87179;

        Eigen::VectorXd q_in_vecxd;
        Vectorq7x1 q_vec_right_arm;

        std::vector<Eigen::VectorXd> des_path;

        //initially empty trajectory
        trajectory_msgs::JointTrajectory des_trajectory;

        cout<<"Instantiating a new trajectory "<<endl;
        

        //this is where you instantiate a new right arm move by passing in a node handle
        Baxter_traj_streamer baxter_traj_streamer(&nh);

        cout<<"Starting call backs"<<endl;

		for (int i=0;i<100;i++) {
        	ros::spinOnce();
       		ros::Duration(0.01).sleep();
       	}

       	cout<<"getting current right-arm pose:"<<endl;
	    q_vec_right_arm =  baxter_traj_streamer.get_qvec_right_arm();  
	    cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;    
	    q_in_vecxd = q_vec_right_arm; // start from here;
	    des_path.push_back(q_in_vecxd); //put all zeros here
	    q_in_vecxd = q_flex; // conversion; not sure why I needed to do this...but des_path.push_back(q_in_vecxd) likes it
	    des_path.push_back(q_in_vecxd); //twice, to define a trajectory 

	    cout << "stuffing traj: " << endl;
    	baxter_traj_streamer.stuff_trajectory(des_path, des_trajectory); //convert from vector of 7dof poses to trajectory message        
        // here is a "goal" object compatible with the server, as defined in example_action_server/action
        baxter_traj_streamer::trajGoal goal; 
        // does this work?  copy traj to goal:
        goal.trajectory = des_trajectory;
        
        actionlib::SimpleActionClient<baxter_traj_streamer::trajAction> action_client("trajActionServer", true);
        
        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running


        if (!server_exists) {
            ROS_WARN("could not connect to server; will wait forever");
            return 0; // bail out; optionally, could print a warning message and retry
        }
        server_exists = action_client.waitForServer(); //wait forever 
        
       
        ROS_INFO("Connection to Action Server exists");  

        
        g_count++;
        goal.traj_id = g_count; // this merely sequentially numbers the goals sent
        ROS_INFO("sending traj_id %d",g_count);
        //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
        action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired
        //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
        
        bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
        
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result for goal number %d",g_count);
            return 0;
        }
        else {
            ROS_INFO("finished before timeout");
        }
        
        

    return 0;
}



        























