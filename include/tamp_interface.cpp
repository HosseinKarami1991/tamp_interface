#include "tamp_interface.h"

tamp_interface::tamp_interface():gripperActionClinetRight(nh,"/robot/end_effector/right_gripper/gripper_action",true),
gripperActionClinetLeft(nh,"/robot/end_effector/left_gripper/gripper_action",true){
cout<<"tamp_interface::tamp_interface()"<<endl;





  simulationCommandSub = nh.subscribe("simulation_command",10, &tamp_interface::arrivingSimulationCommand,this);
  actuationCommandSub = nh.subscribe("robot_command",10,&tamp_interface::arrivingCommands,this);
  simulationResponsePub = nh.advertise<tamp_msgs::tampSimulationRes>("simulation_response",80);
  controlCommandPub	= nh.advertise<tamp_msgs::baxterControlCommand>("robot_control_command",80);
  robotAckPub = nh.advertise<std_msgs::String>("robot_ack",80);
  eliminateObject = nh.advertise<std_msgs::String>("eliminate_object",80);
  tampKnowledgeClient= nh.serviceClient<tamp_msgs::knowledge>("tamp_knowledge_service");
  tampMotionPlannerClient= nh.serviceClient<tamp_msgs::trajquest>("tamp_motion_service");
  tampMotionAckClient= nh.serviceClient<tamp_msgs::ackquest>("tamp_ack_service");
   taskDone = nh.advertise<std_msgs::Bool>("task_done",80);




 




 setAgentsList();
}

tamp_interface::~tamp_interface(){}

void tamp_interface::setAgentsList(){
	cout<<"tamp_interface::setAgentsList"<<endl;
    agents_tasks agent1;
	agent1.agents.push_back("LeftArm");
	agent1.agentsNumber=0;
	agent1.collaborators.clear();
	agents_list.push_back(agent1);

	agents_tasks agent2;
	agent2.agents.push_back("RightArm");
	agent2.agentsNumber=1;
	agent2.collaborators.clear();
	agents_list.push_back(agent2);

	agents_tasks agent3;
	agent3.agents.push_back("LeftArm");
	agent3.agents.push_back("RightArm");
	agent3.agentsNumber=2;
	agent3.collaborators.clear();
	agents_list.push_back(agent3);




}


//**********************************Simulation Commands*****************************************************//

void tamp_interface::arrivingSimulationCommand(const tamp_msgs::tampSimulationReq& msg){


	cout<<BOLD(FBLU("tamp_interface::arrivingSimulationCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	// arrive the simulation command here
	// base on the arriving command call different functions
	cout<<msg.ActionName<<" ";
	for(int i=0;i<msg.ResponsibleAgents.size();i++)
		cout<<msg.ResponsibleAgents[i]<<" ";
	for(int i=0;i<msg.ColleagueAgents.size();i++)
		cout<<msg.ColleagueAgents[i]<<" ";
	for(int i=0;i<msg.ActionParametersName.size();i++)
		cout<<msg.ActionParametersName[i]<<" ";
	cout<<endl;


	string tempActionName=msg.ActionName;
	if(tempActionName=="Grasp"|| tempActionName=="UnGrasp")
		simulateGraspingCommand(msg);
	else if(tempActionName=="Stop")
		simulateStoppingCommand(msg);
	else if(tempActionName=="HoldOn")
		simulateHoldingCommand(msg);
	else if(tempActionName=="Approach")
		simulateApproachingCommand(msg);
	else if(tempActionName=="Rest")
		simulateRestingcommand(msg);
	else if(tempActionName=="checkifr")
		simulateCheckrcommand(msg);
	else if(tempActionName=="checkifl")
		simulateChecklcommand(msg);
	else
	{
		cout<<"The arriving msg name is wrong: "<<tempActionName <<endl;
		exit(1);
	}



}

void tamp_interface::simulateGraspingCommand(const tamp_msgs::tampSimulationReq& msg){

	cout<<BOLD(FBLU("tamp_interface::simulateGraspingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	tamp_msgs::tampSimulationRes tempResponseMsg;

	tempResponseMsg.success=1;
	tempResponseMsg.time=0.2;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	simulationResponsePub.publish(tempResponseMsg);


}



void tamp_interface::simulateHoldingCommand(const tamp_msgs::tampSimulationReq& msg){

	cout<<BOLD(FBLU("tamp_interface::simulateHoldingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	tamp_msgs::tampSimulationRes tempResponseMsg;

	tempResponseMsg.success=1;
	tempResponseMsg.time=0.02;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	simulationResponsePub.publish(tempResponseMsg);


}

void tamp_interface::simulateStoppingCommand(const tamp_msgs::tampSimulationReq& msg){

	cout<<BOLD(FBLU("tamp_interface::simulateStoppingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	tamp_msgs::tampSimulationRes tempResponseMsg;

	tempResponseMsg.success=1;
	tempResponseMsg.time=0.02;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	simulationResponsePub.publish(tempResponseMsg);

}

void tamp_interface::simulateApproachingCommand(const tamp_msgs::tampSimulationReq& msg){

    cout<<BOLD(FBLU("tamp_interface::simulateApproachingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
    
	//! parse the input command
	//vector<string> parameter1,parameter2,parameter1Info,parameter2Info;
	/// Transport wTo wTg
	//boost::split(parameter1,msg.ActionParametersName[0], boost::is_any_of("-"));// wTo: Point3, Cylinder2-ConnectionFrame
	//boost::split(parameter2,msg.ActionParametersName[1], boost::is_any_of("-"));// wTg: Point4, Plate1_connectionFrame

   
    //! call the knowledge base
	tamp_msgs::knowledge knowledge_msg;

	knowledge_msg.request.reqType=msg.ActionParametersName[0];
//	if(parameter1.size()>1)
//		knowledge_msg.request.Name=parameter1[1];
//	else
	knowledge_msg.request.Name="";

	//knowledge_msg.request.requestInfo=msg.ActionParameterInfo[0]; // objectPose
    std::vector<double> target;
	if(tampKnowledgeClient.call(knowledge_msg)){

		//int vectorSize=knowledge_msg.response.pose.size();
         target=knowledge_msg.response.pose;
		if(target.size()==0){
			

			cout<<FBLU("Time2: ")<<to_string(ros::Time::now().toSec())<<endl;
			tamp_msgs::tampSimulationRes tempResponseMsg;

			tempResponseMsg.success=false;
			tempResponseMsg.time=0;//sec
			tempResponseMsg.ActionName=msg.ActionName;

			for(int i=0;i<msg.ResponsibleAgents.size();i++)
				tempResponseMsg.ResponsibleAgents.push_back(msg.ResponsibleAgents[i]);
			for(int i=0;i<msg.ActionParameterInfo.size();i++)
				tempResponseMsg.ActionParameterInfo.push_back(msg.ActionParameterInfo[i]);
			for(int i=0;i<msg.ActionParametersName.size();i++)
				tempResponseMsg.ActionParametersName.push_back(msg.ActionParametersName[i]);
			for(int i=0;i<msg.ColleagueAgents.size();i++)
				tempResponseMsg.ColleagueAgents.push_back(msg.ColleagueAgents[i]);





			simulationResponsePub.publish(tempResponseMsg);
			cout<<FBLU("Time Published: ")<<to_string(ros::Time::now().toSec())<<endl;









		}

		else{




				bool simulationResult;
				string arm;
				double  actionTime;
				tamp_msgs::trajquest trajsrv;
				 geometry_msgs::Pose tarpos;
				if(msg.ResponsibleAgents[0]=="LeftArm"){

			       arm="left";
			    tarpos.orientation.x=target[3];
			    tarpos.orientation.y=target[4];
			    tarpos.orientation.z=target[5]; 
				}
				else if(msg.ResponsibleAgents[0]=="RightArm"){

			     arm="right";
			      tarpos.orientation.x=target[3];
			    tarpos.orientation.y=target[4];
			    tarpos.orientation.z=target[5]; 
				}
				
			    cout<<FBLU("Time1: ")<<to_string(ros::Time::now().toSec())<<endl;
				//simulateArmMotionPlanner(arm,target,simulationResult, actionTime);
			    
			    trajsrv.request.arm = arm;
			   
			    tarpos.position.x =target[0]; 
			    tarpos.position.y =target[1]; 
			    tarpos.position.z =target[2];
			   
			    trajsrv.request.execute=false;
			    trajsrv.request.targetpos = tarpos;
			     std::vector<string> graspif;
			     	boost::split(graspif, msg.ActionParametersName[0], boost::is_any_of("-"));
			   if(graspif.size()>1){
			   		 if(graspif[1]=="grasp"||graspif[1]=="postgrasp"){
			       knowledge_msg.request.reqType=graspif[0];

			    	if(tampKnowledgeClient.call(knowledge_msg)){
			         std::vector<string> objtoremove = knowledge_msg.response.names;
			         trajsrv.request.objecttoremve = objtoremove[0];
					
					
				     }
				     else{
					cout<<" The knowledge base does not responded"<<endl;
				     }


			    }
			   }
			   

			  
			        trajsrv.request.position_tolerance.data = 0.01;
			        trajsrv.request.orientation_tolerance.data = 0.2;

			  

			    
			    trajsrv.request.withcollision=true;
			  bool results;
			  double time;
			  if(tampMotionPlannerClient.call(trajsrv)){


			  	results = trajsrv.response.success;
			  	time = trajsrv.response.time;
			  }







			    cout<<FBLU("Time2: ")<<to_string(ros::Time::now().toSec())<<endl;
				tamp_msgs::tampSimulationRes tempResponseMsg;

				tempResponseMsg.success=results;
				tempResponseMsg.time=time;//sec
				tempResponseMsg.ActionName=msg.ActionName;

				for(int i=0;i<msg.ResponsibleAgents.size();i++)
					tempResponseMsg.ResponsibleAgents.push_back(msg.ResponsibleAgents[i]);
				for(int i=0;i<msg.ActionParameterInfo.size();i++)
					tempResponseMsg.ActionParameterInfo.push_back(msg.ActionParameterInfo[i]);
				for(int i=0;i<msg.ActionParametersName.size();i++)
					tempResponseMsg.ActionParametersName.push_back(msg.ActionParametersName[i]);
				for(int i=0;i<msg.ColleagueAgents.size();i++)
					tempResponseMsg.ColleagueAgents.push_back(msg.ColleagueAgents[i]);





				simulationResponsePub.publish(tempResponseMsg);
				cout<<FBLU("Time Published: ")<<to_string(ros::Time::now().toSec())<<endl;




		}
		
	}
	else
	{
		cout<<" The knowledge base does not responded"<<endl;
	}

	


	
   

	
}

void tamp_interface::simulateRestingcommand(const tamp_msgs::tampSimulationReq& msg){

	cout<<BOLD(FBLU("tamp_interface::simulateRestingcommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	tamp_msgs::tampSimulationRes tempResponseMsg;

	tempResponseMsg.success=1;
	tempResponseMsg.time=0.2;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	simulationResponsePub.publish(tempResponseMsg);

}


void tamp_interface::simulateCheckrcommand(const tamp_msgs::tampSimulationReq& msg){
	tamp_msgs::tampSimulationRes tempResponseMsg;

	
	tempResponseMsg.time=0.02;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	
	if(lastagent_=="RightArm"){
      tempResponseMsg.success=1;
	}
	else{
		tempResponseMsg.success=0;
	}
	simulationResponsePub.publish(tempResponseMsg);

}
void tamp_interface::simulateChecklcommand(const tamp_msgs::tampSimulationReq& msg){
	tamp_msgs::tampSimulationRes tempResponseMsg;

	
	tempResponseMsg.time=0.02;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	
	if(lastagent_=="RightArm"){
      tempResponseMsg.success=0;
	}
	else{
		tempResponseMsg.success=1;
	}
	simulationResponsePub.publish(tempResponseMsg);

}

/*void tamp_interface::simulateArmMotionPlanner(std::vector<string> ResponsibleAgents,std::vector<double> target,bool results,double time){

     cout<<BOLD(FBLU("tamp_interface::simulateArmMotionPlanner"))<<endl;

    tamp_msgs::trajquest trajsrv;
    trajsrv.request.arm = ResponsibleAgents[0];
    geometry_msgs::Pose tarpos;

    if(ResponsibleAgents[0]=="RightArm"){
       tarpos.orientation.x=0.5;
    tarpos.orientation.y=1.9;
    tarpos.orientation.z=1.0; 

    }
    else{
          tarpos.orientation.x=-0.8;
    tarpos.orientation.y=1.2;
    tarpos.orientation.z=-1.6; 


    }
    tarpos.position.x =target[0]; 
    tarpos.position.y =target[1]; 
    tarpos.position.z =target[2];
    
    trajsrv.request.execute=false;
    trajsrv.request.targetpos = tarpos;
    trajsrv.request.position_tolerance.data = 0.05;
    trajsrv.request.orientation_tolerance.data = 1.5;
    trajsrv.request.withcollision=true;


  if(tampMotionPlannerClient.call(trajsrv)){


  	results = trajsrv.response.success;
  	time = trajsrv.response.time;
  }





}
*/










//**********************************Actuation Commands*****************************************************//


void tamp_interface::arrivingCommands(const std_msgs::String::ConstPtr& commandmsg){

	cout<<BOLD(FBLU("tamp_interface::arrivingCommands"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	string input=commandmsg-> data.c_str();
	ROS_INFO("Arrived robot command: %s",input.c_str());
	int agentNumber;
	vector<string> msg, msgAction,msgAgents,msgColleagues;
	string cmndType, reachingPoint;
	boost::split(msg, input, boost::is_any_of(" "));
	boost::split(msgAction, msg[0], boost::is_any_of("_"));

	boost::split(msgAgents, msg[1], boost::is_any_of("+"));

	if(msg.size()==3)
	boost::split(msgColleagues, msg[2], boost::is_any_of("+"));
	else if(msg.size()>3)
		cout<<"Error in arriving msg size: "<<msg.size()<<input<<endl;

	//! first find which agents should perform the action and assign it.

	if(msgAgents.size()==1)
	{
		if(msgAgents[0]=="LeftArm")
		{
			agentNumber=0;
		}
		else if(msgAgents[0]=="RightArm")
		{
			agentNumber=1;
		}
		else
		{
			cout<<"The agents definition is not correct: "<<msgAgents.size()<<", "<<msg[1]<<endl;
		}
	}
	else if(msgAgents.size()==2)
	{
		if((msgAgents[0]=="LeftArm" && msgAgents[1]=="RightArm") ||(msgAgents[1]=="LeftArm" && msgAgents[0]=="RightArm"))
		{
			agentNumber=2;
		}
		else
		{
			cout<<"The agents definition is not correct: "<<msgAgents.size()<<", "<<msg[1]<<endl;
		}
	}
	else
	{
		cout<<"The agents size is not correct: "<<msgAgents.size()<<", "<<msg[1]<<endl;
	}

	if(agents_list[agentNumber].isBusy==false || msg[0]=="Stop")
	{


		agents_list[agentNumber].isBusy=true;

		agents_list[agentNumber].lastAssignedAction=msg[0];

		agents_list[agentNumber].microSec_StartingTime=duration_cast< microseconds >(system_clock::now().time_since_epoch());
		agents_list[agentNumber].collaborators=msgColleagues;

	}
	else
	{
		cout<<"The agent you assigned is busy now and it can not perform a new action "<<endl;
	}


	if(msgAction[0]=="Approach")
	{
		sendApproachingCommand(commandmsg,agents_list[agentNumber]);
	}
	
	else if(msgAction[0]=="Rest")
	{
		sendRestingCommand(agents_list[agentNumber]);
	}
	
	
	else if(msgAction[0]=="Grasp")
	{
		sendGraspingCommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="UnGrasp")
	{
		sendUnGraspingCommand(agents_list[agentNumber]);
	}
	
	else if(msgAction[0]=="Stop")
	{
		sendStoppingCommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="HoldOn")
	{
		sendHoldingCommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="checkifr")
	{
		checkrcommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="checkifl")
	{
		checklcommand(agents_list[agentNumber]);
	}

	else
	{
		cout<<"Error in arriving msg action:"<<msgAction[0]<<endl;
	}

  




}

void tamp_interface::sendGraspingCommand(agents_tasks& agent){

	if(agent.agents[0]=="RightArm"){


		control_msgs::GripperCommandGoal grippgoal;
		gripperActionClinetRight.waitForServer();
		grippgoal.command.position = 10.0;
		gripperActionClinetRight.sendGoal(grippgoal);
		bool finished_before_timeout = gripperActionClinetRight.waitForResult(ros::Duration(10.0));

		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = gripperActionClinetRight.getState();
			ROS_INFO("Right Gripper Action finished: %s",state.toString().c_str());
			agent.isBusy=false;
			agent.isActionSuccessfullyDone=true;
			agent.emergencyFlag=false;    
            PublishRobotAck(agent);
		}
		else
			ROS_INFO("Action did not finish before the time out.");

	}

	else if(agent.agents[0]=="LeftArm"){
         
        control_msgs::GripperCommandGoal grippgoal;
		gripperActionClinetLeft.waitForServer();
		grippgoal.command.position = 10.0;
		gripperActionClinetLeft.sendGoal(grippgoal);
		bool finished_before_timeout = gripperActionClinetLeft.waitForResult(ros::Duration(10.0));

		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = gripperActionClinetLeft.getState();
			ROS_INFO("Left Gripper Action finished: %s",state.toString().c_str());
			agent.isBusy=false;
			agent.isActionSuccessfullyDone=true;
			agent.emergencyFlag=false;    
            PublishRobotAck(agent);

			
		}
		else
			ROS_INFO("Action did not finish before the time out.");




	}
    



}

void tamp_interface::sendUnGraspingCommand(agents_tasks& agent){


	if(agent.agents[0]=="RightArm"){


		control_msgs::GripperCommandGoal grippgoal;
		gripperActionClinetRight.waitForServer();
		grippgoal.command.position = 90.0;
		gripperActionClinetRight.sendGoal(grippgoal);
		bool finished_before_timeout = gripperActionClinetRight.waitForResult(ros::Duration(4.0));

		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = gripperActionClinetRight.getState();
			ROS_INFO("Right Gripper Action finished: %s",state.toString().c_str());
			agent.isBusy=false;
			agent.isActionSuccessfullyDone=true;
			agent.emergencyFlag=false;
            PublishRobotAck(agent);
		}
		else
			ROS_INFO("Action did not finish before the time out.");

	}

	else if(agent.agents[0]=="LeftArm"){
         
        control_msgs::GripperCommandGoal grippgoal;
		gripperActionClinetLeft.waitForServer();
		grippgoal.command.position = 90.0;
		gripperActionClinetLeft.sendGoal(grippgoal);
		bool finished_before_timeout = gripperActionClinetLeft.waitForResult(ros::Duration(4.0));

		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = gripperActionClinetLeft.getState();
			ROS_INFO("Left GripperAction finished: %s",state.toString().c_str());
			agent.isBusy=false;
			agent.isActionSuccessfullyDone=true;
			agent.emergencyFlag=false;    
            PublishRobotAck(agent);
		}
		else
			ROS_INFO("Action did not finish before the time out.");




	}



}


void tamp_interface::checkrcommand(agents_tasks& agent){

            agent.isBusy=false;
			agent.isActionSuccessfullyDone=true;
			agent.emergencyFlag=false;    
            PublishRobotAck(agent);
}
void tamp_interface::checklcommand(agents_tasks& agent){

			agent.isBusy=false;
			agent.isActionSuccessfullyDone=true;
			agent.emergencyFlag=false;    
            PublishRobotAck(agent);
}



void tamp_interface::sendHoldingCommand(agents_tasks& agent){



}

void tamp_interface::sendStoppingCommand(agents_tasks& agent){


}

void tamp_interface::sendApproachingCommand(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent){

	cout<<BOLD(FBLU("tamp_interface::sendApproachingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	string input=commandmsg-> data.c_str();
	vector<string> msg, msgAction,msgAgents,msgColleagues;
	string cmndType, reachingPoint;
	boost::split(msg, input, boost::is_any_of(" "));
	boost::split(msgAction, msg[0], boost::is_any_of("_"));

	boost::split(msgAgents, msg[1], boost::is_any_of("+"));



    tamp_msgs::knowledge knowledge_msg;

	knowledge_msg.request.reqType=msgAction[1];

	knowledge_msg.request.Name="";

    std::vector<double> target;
	if(tampKnowledgeClient.call(knowledge_msg)){

         target=knowledge_msg.response.pose;
	
		
	}
	else
	{
		cout<<" The knowledge base does not responded"<<endl;
	}

    cout<<BOLD(FBLU("moving to point: "))<<endl;
    cout<<target[0]<<" "<<target[1]<<" "<<target[2]<<" "<<target[3]<<" "<<
    target[4]<<" "<<target[5]<<" "<<endl;
	
	string arm;
	tamp_msgs::trajquest trajsrv;
	geometry_msgs::Pose tarpos;
	if(msgAgents[0]=="LeftArm"){
       arm="left";
        tarpos.orientation.x=target[3];
    tarpos.orientation.y=target[4];
    tarpos.orientation.z=target[5]; 
	}
	else if(msgAgents[0]=="RightArm"){

     arm="right";
     tarpos.orientation.x=target[3];
    tarpos.orientation.y=target[4];
    tarpos.orientation.z=target[5]; 
	}
	 
    
    
    trajsrv.request.arm = arm;
    
    tarpos.position.x =target[0]; 
    tarpos.position.y =target[1]; 
    tarpos.position.z =target[2];
    
    trajsrv.request.execute=true;
    trajsrv.request.targetpos = tarpos;
    std::vector<string> graspif;
    boost::split(graspif, msgAction[1], boost::is_any_of("-"));
    if(graspif.size()>1){
    	  if(graspif[1]=="grasp"||graspif[1]=="postgrasp"){
	
       knowledge_msg.request.reqType=graspif[0];

    	if(tampKnowledgeClient.call(knowledge_msg)){

		//int vectorSize=knowledge_msg.response.pose.size();
         std::vector<string> objtoremove = knowledge_msg.response.names;
         trajsrv.request.objecttoremve = objtoremove[0];
		//if(vectorSize!=6)
		//{
		//	cout<<"Error in number of knowledge base point size for the joint action"<<endl;
		//	exit(1);
		//
	       }
	      else
	       {
		cout<<" The knowledge base does not responded"<<endl;
	        }

 

    }

    }
  
   



    ptol_ = 0.01;
    rtol_=0.2;
    trajsrv.request.position_tolerance.data = ptol_;
    trajsrv.request.orientation_tolerance.data = rtol_;
    trajsrv.request.withcollision=true;
     
  

geometry_msgs::PoseStamped reachedpoint;
std::size_t j=0;
//for(std::size_t i=0;i<3;i++){
	if(tampMotionPlannerClient.call(trajsrv)){


  	bool results = trajsrv.response.success;
  	double time = trajsrv.response.time;
 // ros::Duration(5).sleep();

   if(trajsrv.response.executedtrajectory){
   	  ROS_INFO("Executed trajrctory.");
   	  tamp_msgs::ackquest ackmsg;
  ackmsg.request.arm =arm; 
  reachedpoint;
  if(tampMotionAckClient.call(ackmsg)){

    reachedpoint = ackmsg.response.eepos;

  }
   }





  	//if(trajsrv.response.executedtrajectory && checkIfReachedToTarget(reachedpoint,target))
  		//break;
  
  }


 // }
  //double dis,drot;
 // if(!){

    
 //   cout<<" didn't reach to target as planned"<<" Eculedan distanse: "<<dis_<<"rotation dis "<<drot_<<endl;
    //ptol_ =0.005;
   // trajsrv.request.position_tolerance.data = ptol_;
 //   if(tampMotionPlannerClient.call(trajsrv)){
//

  //	bool results = trajsrv.response.success;
//  	double time = trajsrv.response.time;
 // }
 // }
  //else {

  //     cout<<"Reached to target as planned"<<" Eculedan distanse: "<<dis_<<"rotation dis "<<drot_<<endl;

// }


  












//if(checkIfReachedToTarget(reachedpoint,target,dis,drot)&& trajsrv.response.executedtrajectory){

  	     checkIfReachedToTarget(reachedpoint,target);
  	    cout<<"Reached to target as planned"<<" Eculedan distanse: "<<dis_<<"rotation dis "<<drot_<<endl;
    	agent.isBusy=false;
		agent.isActionSuccessfullyDone=trajsrv.response.executedtrajectory;
		agent.emergencyFlag=false;    
        PublishRobotAck(agent);

 // }









}

void tamp_interface::sendRestingCommand(agents_tasks& agent){




}



//**************************************Acknowledgement**************************************/
void tamp_interface::controlAckPub(agents_tasks& agent){

    cout<<BOLD(FBLU("tamp_interface::controlAckPub"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	std_msgs::String ackMsg;
	cout<<"Last assigned action: "<<agent.lastAssignedAction<<endl;
	ackMsg.data=agent.lastAssignedAction+" "; // Appraoch_Point2, Grasp, Transport_Cylinder2-GraspingPose1_Point7

	for(int i=0;i<agent.agents.size();i++){
		ackMsg.data+=agent.agents[i];
		if(i<agent.agents.size()-1)
			ackMsg.data=ackMsg.data+"+";
	}
	if(agent.isActionSuccessfullyDone==true)
		ackMsg.data+=" true";
	else
		ackMsg.data+=" false";

	if (agent.emergencyFlag==false)
		robotAckPub.publish(ackMsg);
	else
	{
		cout<<"The agent emergency flag is true, therefore we do not give ack to the planner"<<endl;
		agent.Print();
	}



}

bool  tamp_interface::checkIfReachedToTarget(geometry_msgs::PoseStamped rp, std::vector<double> tar){

     dis_=0;
     drot_=0;
     double dx = rp.pose.position.x - tar[0];
     double dy = rp.pose.position.y - tar[1];
     double dz = rp.pose.position.z - tar[2];
      dis_ = sqrt(dx*dx+dy*dy+dz*dz);
    
     double droll = rp.pose.orientation.x - tar[3];
     double dpitch = rp.pose.orientation.y - tar[4];
     double dyaw = rp.pose.orientation.z - tar[5];
      drot_ = sqrt(droll*droll+dpitch*dpitch+dyaw*dyaw);

     if(dis_<ptol_ && drot_<rtol_){

     	return true;
     }
     else{

     	return false;
     }






}


void tamp_interface::PublishRobotAck(agents_tasks& agent){
	cout<<BOLD(FBLU("tamp_interface::PublishRobotAck"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	std_msgs::String ackMsg;
	cout<<"Last assigned action: "<<agent.lastAssignedAction<<endl;
	ackMsg.data=agent.lastAssignedAction+" "; // Appraoch_Point2, Grasp, Transport_Cylinder2-GraspingPose1_Point7

	for(int i=0;i<agent.agents.size();i++){
		ackMsg.data+=agent.agents[i];
		if(i<agent.agents.size()-1)
			ackMsg.data=ackMsg.data+"+";
	}
	if(agent.isActionSuccessfullyDone==true)
		{    ROS_INFO("Publishing task accomplishment to planner1");


            lastagent_ = agent.agents[0];
           







			ackMsg.data+=" true";
	       std::vector<string> actionvec,graspvec;
	     boost::split(actionvec,agent.lastAssignedAction, boost::is_any_of("_"));
         if(actionvec[0]=="Approach"){
         	ROS_INFO("Publishing task accomplishment to planner2");
               boost::split(graspvec,actionvec[1], boost::is_any_of("-"));
               if(graspvec.size()>1){ 
                 	if(graspvec[1]=="postgrasp"){
         		ROS_INFO("Publishing task accomplishment to planner3");
             
            
              tamp_msgs::knowledge knmsg;
              knmsg.request.reqType=graspvec[0];
               if(tampKnowledgeClient.call(knmsg)){
               	ROS_INFO("Publishing task accomplishment to planner4");
               		string res =knmsg.response.names[0];
               		std::vector<string> vec;
               		boost::split(vec,res, boost::is_any_of("_"));
               		ROS_INFO("vec[1] is *****%s:",vec[1]);
               	 if(vec[1]=="target"){
               	 	ROS_INFO("Publishing task accomplishment to planner5");

               	 	   std_msgs::Bool donemsg;
		                 donemsg.data=true;
		                 ROS_INFO("Publishing task accomplishment to planner");
		                 taskDone.publish(donemsg);

               	 }
               	
              

               }
                std_msgs::String msg;
             msg.data=graspvec[0];
             eliminateObject.publish(msg);

         	}
         }
      


         }





        }
	else
		ackMsg.data+=" false";

	if (agent.emergencyFlag==false)
		robotAckPub.publish(ackMsg);
	else
	{
		cout<<"The agent emergency flag is true, therefore we do not give ack to the planner"<<endl;
		agent.Print();
	}
}