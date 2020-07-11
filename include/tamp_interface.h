#include <iostream>
#include <ros/ros.h>
#include "tamp_msgs/tampSimulationReq.h"
#include "tamp_msgs/tampSimulationRes.h"
#include "tamp_msgs/baxterControlCommand.h"
#include "tamp_msgs/knowledge.h"
#include <chrono>
#include <stdio.h>
#include "std_msgs/String.h"
#include <boost/algorithm/string.hpp>
#include"tamp_msgs/trajquest.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "control_msgs/GripperCommandAction.h"
#include "control_msgs/GripperCommandGoal.h"
#include "tamp_msgs/ackquest.h"
#include"std_msgs/Bool.h"


#define RST  "\x1B[0m"
#define KBLU  "\x1B[34m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define FBLU(x) KBLU x RST
#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define BOLD(x) "\x1B[1m" x RST

using namespace std;
using namespace std::chrono;

class agents_tasks{
public:
	vector<string> agents; 			//! the different agents combination for performing actions like: leftArm, rightArm, leftArm+rightArm.
	int agentsNumber; // Left: 0, right=1, left+right=2
	vector<string> collaborators;
	string lastAssignedAction;
	bool isActionSuccessfullyDone;
	bool isBusy;
	microseconds microSec_StartingTime;
	bool emergencyFlag; // stop robot emergency flag

	agents_tasks(){
//		agents=NULL;
//		collaborators=NULL;
		lastAssignedAction="";
		isActionSuccessfullyDone=false;
		isBusy=false;
		agentsNumber=0;
		microSec_StartingTime=duration_cast< microseconds >(system_clock::now().time_since_epoch());
		emergencyFlag=false;
	};
	~agents_tasks(){};

	void Print(void){
		cout<<"Agents: ";
		for(int i=0;i<agents.size();i++){
			cout<<agents[i]<<" ";
		}
		cout<<endl;

		cout<<"collaborators: ";
		for(int i=0;i<collaborators.size();i++){
			cout<<collaborators[i]<<" ";
		}
		cout<<endl;

		cout<<"Last Assigned Action: "<<lastAssignedAction<<endl;
		cout<<"Is Action successfully done? "<< isActionSuccessfullyDone<<endl;
		cout<<"Is Agent busy? "<<isBusy<<endl;
		cout<<"emergency Flag: "<<emergencyFlag<<endl;
		cout<<"Agent Number: "<<agentsNumber<<endl;
		cout<<"Last action command time: "<<microSec_StartingTime.count()<<endl;

	};
};


class tamp_interface
{
public:
	tamp_interface();
	~tamp_interface();

    //actuatuion commands
    void arrivingCommands(const std_msgs::String::ConstPtr& msg);
    void setAgentsList();
	void sendGraspingCommand(agents_tasks& agent);
	void sendUnGraspingCommand(agents_tasks& agent);
	void sendHoldingCommand(agents_tasks& agent);
	void sendStoppingCommand(agents_tasks& agent);
	void sendApproachingCommand(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent);
	void sendRestingCommand(agents_tasks& agent);
	void controlAckPub(agents_tasks& agent);
    void PublishRobotAck(agents_tasks& agent);
    bool checkIfReachedToTarget(geometry_msgs::PoseStamped rp, std::vector<double> tar);
    void checkrcommand(agents_tasks& agent);
    void checklcommand(agents_tasks& agent);




    //simulation commands
	void arrivingSimulationCommand(const tamp_msgs::tampSimulationReq& msg);
	void simulateGraspingCommand(const tamp_msgs::tampSimulationReq& msg);
	void simulateUnGraspingCommand(const tamp_msgs::tampSimulationReq& msg);
	void simulateHoldingCommand(const tamp_msgs::tampSimulationReq& msg);
	void simulateStoppingCommand(const tamp_msgs::tampSimulationReq& msg);
	void simulateApproachingCommand(const tamp_msgs::tampSimulationReq& msg);
	void simulateRestingcommand(const tamp_msgs::tampSimulationReq& msg);
	void simulateArmMotionPlanner(std::vector<string> ResponsibleAgents,std::vector<double> target,  bool results,double time);
    void simulateCheckrcommand(const tamp_msgs::tampSimulationReq& msg);
    void simulateChecklcommand(const tamp_msgs::tampSimulationReq& msg);


	std::vector<agents_tasks> agents_list;
	       

private:
	string lastagent_;
	double dis_,drot_,ptol_,rtol_;
	ros::NodeHandle nh;
	ros::Subscriber leftArmSub;
	ros::Subscriber rightArmSub;
	ros::Subscriber simulationCommandSub;
	ros::Subscriber actuationCommandSub;
	ros::Publisher  leftArmPub;
	ros::Publisher  rightArmPub;
	ros::Publisher simulationResponsePub;
	ros::Publisher controlCommandPub;
	ros::Publisher robotAckPub,eliminateObject,taskDone;
	ros::ServiceClient tampKnowledgeClient;
	ros::ServiceClient tampMotionPlannerClient,tampMotionAckClient;
	microseconds microSec_time;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripperActionClinetRight,gripperActionClinetLeft;
};
