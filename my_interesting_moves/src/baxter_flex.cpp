#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_interesting_moves/right_arm_moves.h>
#include <my_interesting_moves/trajAction.h>

using namespace std;

//7-dof vector
#define VECTOR_DIM 7 

void doneCallBack(const actionlib::SimpleClientGoalState& state, const my_interesting_moves::trajResultConstPtr& result){

}


int main(int argc, char** argv) {
        ros::init(argc, argv, "traj_action_client_node"); 
        ros::NodeHandle nh;         
        int g_count = 0;
        int ans;

        Vectorq7x1 q_flex;
        q_flex<< -0.907528, -0.111813,   2.06622,    1.8737,    -1.295,   2.00164,  -2.87179;

        Eigen::VectorXd q_in_vecxd;
        Vectorq7x1 q_vec_right_arm;

        std::vector<Eigen::VectorXd> des_path;

        //initially empty trajectory
        trajectory_msgs::JointTrajectory des_trajectory;

        cout<<"Instantiating a new trajectory (enter) 1"<<endl;
        cin>> ans;

        //this is where you instantiate a new right arm move by passing in a node handle
        My_interesting_moves right_arm_moves(&nh);

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

	    


    }        























