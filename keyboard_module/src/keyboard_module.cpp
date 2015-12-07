#include <ros/ros.h>
#include <iostream>
#include <string>
using namespace std;


int main (int argc, char ** argv){

	string keyboard_input;

	cout << "Type 'p' to pause, 'c' to continue and 'g' to check if gripper is ok" << endl;

	while(true){ // ros::ok()

		cin >> keyboard_input;
		if(keyboard_input == "p"){
			cout<< endl << "Currently paused" << endl;
			cin >> keyboard_input;
			

		}

		if(keyboard_input == "c"){
			cout << endl << "Continuing process" << endl;
			cin >> keyboard_input;
			
		}

		if(keyboard_input == "g"){
			cout << endl << "Checking to see if gripper is okay" << endl;
			cin >> keyboard_input;
			
		}

        if(keyboard_input != "p" && keyboard_input != "c" && keyboard_input != "g"){
        	cout << "Invalid keyboard input! Pausing" << endl;
        	keyboard_input = "p";
        }
		
	}





}

