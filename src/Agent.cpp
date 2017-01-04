/*
 * Agent.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: andy
 */


#include "Agent.h"

Agent::Agent(ros::NodeHandle nHandle){
	costmap.init_flag = true;

	costSub = nHandle.subscribe("/map", 0, &Agent::costMapCallback, this);
	locSub = nHandle.subscribe("/odom", 1, &Agent::locationCallback, this);
	marketSub = nHandle.subscribe("/agent0/market", 1, &Agent::marketCallback, this);
	mapUpdatesSub = nHandle.subscribe("/agent0/map", 1, &Agent::mapUpdatesCallback, this);

	markerPub = nHandle.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
	goalPub =  nHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

	marketPub = nHandle.advertise<std_msgs::Float32MultiArray>("/agent0/market", 10);
	locPub = nHandle.advertise<nav_msgs::Odometry>("/agent0/loc", 10);
	mapUpdatesPub = nHandle.advertise<std_msgs::Int16MultiArray>("/agent0/map", 10);

	missionTime = ros::Time::now();
	timeOfLastReport = ros::Time::now();

	maxMissionTime = ros::Time::now() + ros::Duration(50000);
	travelSpeed = 10;
	actTimer = ros::Time::now();
	explorationComplete = false;

	oLoc = Point(50,60);
	float lastPlan = -1;
}

void Agent::publishNavGoalsToMoveBase(){
  	geometry_msgs::PoseStamped goal;
  	goal.header.frame_id = "/map";
	goal.pose.position.y = (gLoc.x - offset.x)/4.8;
	goal.pose.position.x = (gLoc.y - offset.y)/4.8;

	float theta = atan2(float(gLoc.x - cLoc.x), float(gLoc.y - cLoc.y));

	goal.pose.orientation.x = 0.0;
	goal.pose.orientation.y = 0.0;
	goal.pose.orientation.z = sin(theta/2);
	goal.pose.orientation.w = cos(theta/2);

	ROS_INFO("OSU::Publishing goal pose: %.2f, %.2f, %.2f", (gLoc.x-offset.x)/4.8, (gLoc.y-offset.y)/4.8, theta*57.29577);

	goalPub.publish(goal);
}

void Agent::publishRvizMarker(Point loc, float radius, int color, int id){
	visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = id;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = (loc.y - offset.y)/4.8;
    marker.pose.position.y = (loc.x - offset.x)/4.8;
    marker.pose.position.z = 1.5;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;

    // Set the color -- be sure to set alpha to something non-zero!

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    if(color == 0){
    	marker.color.r = 1.0f;
    }
    else if(color == 1){
    	marker.color.g = 1.0f;
    }
    else if(color == 2){
    	marker.color.b = 1.0f;
    }

    marker.lifetime = ros::Duration(2);
    markerPub.publish(marker);
}

//void Agent::marketCallBack( const vector<vector<float> > &market){}

void Agent::locationCallback( const nav_msgs::Odometry& locIn){
	cLoc.x = offset.x + 4.8*(locIn.pose.pose.position.y);
	cLoc.y = offset.y + 4.8*(locIn.pose.pose.position.x);

	market.updatecLoc( cLoc );

	if( costmapInitialized && !locationInitialized){
		plan("marketFrontiers");
	}

	if( costmapInitialized && locationInitialized && marketInitialized && ros::Time::now().toSec() - lastPlan > 5){
		plan("marketFrontiers");
	}

	locationInitialized = true;
	act();
	publishLoc( locIn ); // provide my location to observer

	costmap.buildCellsPlot();
	costmap.addAgentToCostmapPlot( myColor, myPath, cLoc);
	Scalar orange = Scalar(0,165,255);
	costmap.addAgentToCostmapPlot( orange, myPath, gLoc);
	costmap.showCostmapPlot(0);
}

void Agent::publishLoc( const nav_msgs::Odometry& locIn){
	locPub.publish( locIn );
}

void Agent::publishMarket(){
	market.cLocs[myIndex] = cLoc;
	market.gLocs[myIndex] = gLoc;
	marketPub.publish( market.assembleTransmission() );
}

void Agent::marketCallback( const std_msgs::Float32MultiArray& transmission){
	ROS_ERROR("Agent had market callback");
	market.dissasembleTransmission( transmission );
	market.printMarket();
	marketInitialized = true;
}

void Agent::costMapCallback(const nav_msgs::OccupancyGrid& cost_in ){

	ROS_INFO("OSU::Map height/width: %d / %d", cost_in.info.height, cost_in.info.width);

	if( !costmapInitialized ){ // have i initialized the costmap?
		costmap.initCostmap(cost_in.info.height, cost_in.info.width);
		offset.x = cost_in.info.width / 2 + 1;
		offset.y = cost_in.info.height / 2 + 1;
		costmapInitialized = true;
	}
	
	vector<int8_t> occGrid = cost_in.data;
	costmap.updateCells( occGrid );
	publishMapUpdates();

	inference.makeInference( "Geometric", costmap );


	if( locationInitialized ){
		plan("marketFrontiers");
	}

	costmap.buildCellsPlot();
	costmap.addAgentToCostmapPlot( myColor, myPath, cLoc);
	costmap.addAgentToCostmapPlot( myColor, myPath, oLoc);
	costmap.showCostmapPlot(0);
}

void Agent::publishMapUpdates(){

	vector<short int> cm;
	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point p(i,j);
			if( costmap.cells.at<short>(p) == costmap.obsFree || costmap.cells.at<short>(p) == costmap.obsWall){
					cm.push_back(i);
					cm.push_back(j);
					cm.push_back( costmap.cells.at<short>(p) );			
			}
		}
	}

	ROS_ERROR("cm.size: %i", int(cm.size() ));

	std_msgs::Int16MultiArray transmission;
	// set up dimensions
	transmission.layout.dim.push_back(std_msgs::MultiArrayDimension());
	transmission.layout.dim[0].size = cm.size();
	transmission.layout.dim[0].stride = 1;
	transmission.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1

	// copy in the data
	transmission.data.clear();
	transmission.data.insert(transmission.data.end(), cm.begin(), cm.end());
	mapUpdatesPub.publish( transmission );
}

void Agent::mapUpdatesCallback(  const std_msgs::Int16MultiArray& transmission ){
	
	for(size_t i=0; i<transmission.data.size(); i+=3){
		//x,y, val
		Point t(transmission.data[i], transmission.data[i+1]);
		if( costmap.cells.at<short>(t) != costmap.obsFree || costmap.cells.at<short>(t) != costmap.obsWall){
			costmap.cells.at<short>(t) = transmission.data[i+2];
		}
	}
}


void Agent::plan( string planMethod ){
	lastPlan = ros::Time::now().toSec();

	//marketRelaySacrifice();
	//marketReturnInfo();

	// all agents check if they should return, includes relay / sacrifice / normal


	cout << "checkReturnTime: " << checkReturnTime() << endl;
	cout << "checkForExplorationFinished: " << checkForExplorationFinished() << endl;

	if( checkReturnTime() || checkForExplorationFinished() || explorationComplete){
		if(relayFlag || sacrificeFlag){
			if(relayFlag){
				cout << "returning to relay point as relay" << endl;
				gLoc = returnToRelayPt();
				return;
			}
			else{
				gLoc = reportToRelayPt();
				cout << "reporting to relay point as sacrifice" << endl;
				return;
			}
		}
		else{
			gLoc = returnToOperator();
			cout << "returning to operator" << endl;
			return;
		}
	}

	// all agents check if they should report
	if( checkReportTime() && !lastReport()){
		//cout << "reporting to operator " << endl;
		//gLoc = reportToOperator();
		//return;
	}
	
	// don't return or report, so explore
	cout << "exploring" << endl;
	planExplore(planMethod);
}

void Agent::planExplore(string planMethod ){

	if(planMethod.compare("greedyFrontiers") == 0){
		gLoc = costmapPlanning.searchPlanner(costmap, cLoc);
		//gLoc = costmapPlanning.mappingPlanner(costmap, cLoc);

		if(gLoc.x < 0 && gLoc.y < 0){
			cerr << "greedy" << endl;
			if(costmap.cells.at<short>(gLoc) != costmap.infFree || (gLoc.x == cLoc.x && gLoc.y == cLoc.y) ){
				gLoc = costmapPlanning.greedyFrontierPlanner(costmap, cLoc);
			}
		}

	}
	else if(planMethod.compare("marketFrontiers") == 0){

		vector<Point> frntList = costmapCoordination.findFrontiers( costmap );
		costmapCoordination.clusterFrontiers( frntList, costmap);
		costmapCoordination.plotFrontiers( costmap, frntList );
		gLoc = costmapCoordination.marketFrontiers(costmap, cLoc, market);
		if( gLoc.x == -1 && gLoc.y == -1){
			cout << "exploration Complete" << endl;
			explorationComplete = true;
			gLoc = oLoc;
		}
		market.updateTime(ros::Time::now().toSec() );
		market.printMarket();
	}
	else if(planMethod.compare("selectPose") == 0){

		float w[3] = {1, 0, 0}; // explore, search, map
		float e[2] = {0.5, 1}; // dominated, breaches
		float spread = 0.3; // spread rate
		costmap.getRewardMat(w, e, spread);
		//costmap.displayThermalMat( costmap.reward );

		//gLoc = costmapPlanning.explorePlanner(costmap, cLoc);
		//gLoc = costmapPlanning.searchPlanner(costmap, cLoc);
		//gLoc = costmapPlanning.mappingPlanner(costmap, cLoc);

		//clock_t tStart1 = clock();
		graphCoordination.thinGraph.createThinGraph(costmap, 1, 1);
		//printf("Time taken to create thinGraph: %.2fs\n", (double)(clock() - tStart1)/CLOCKS_PER_SEC);
		//cout << "thinGraph.size(): " << graphCoordination.thinGraph.nodeLocations.size() << endl;


		graphCoordination.findPosesEvolution( costmap );		//cout << "out" << endl;
		//cout << "Agent::planExplore::found graphPoses with " << this->graphCoordination.poseGraph.nodeLocations.size() << " nodes" << endl;

		if(graphCoordination.poseGraph.nodeLocations.size() < 1){
			//cout << "PoseGraph.size() == 0" << endl;
			gLoc = costmapPlanning.explorePlanner(costmap, cLoc);
			//gLoc = costmapPlanning.greedyFrontierPlanner(costmap, cLoc);
		}
		else{
			//gLoc = costmapPlanning.explorePlanner(costmap, cLoc);
			graphCoordination.marketPoses( costmap, cLoc, gLoc, market );
		}



		//graphCoordination.displayPoseGraph( costmap );

		if(gLoc.x < 0 || gLoc.y < 0){
			cerr << "greedy" << endl;
			if(costmap.cells.at<short>(gLoc) != costmap.infFree || (gLoc.x == cLoc.x && gLoc.y == cLoc.y) ){
				gLoc = costmapPlanning.greedyFrontierPlanner(costmap, cLoc);
			}
		}
	}
	else if(planMethod.compare("pose") == 0){
		graphCoordination.travelGraph.createPRMGraph(cLoc, costmap, 3, 9);
		//this->graph.displayCoordMap(this->costmap, false);
		//waitKey(10);

		graphCoordination.findPosesEvolution(graphCoordination.travelGraph, costmap, agentLocs);

		if(this->graphCoordination.poseGraph.nodeLocations.size() == 1){ // only one pose, switch to greedy
			gLoc = this->costmapPlanning.greedyFrontierPlanner(costmap, cLoc);
		}
		else{
			cout << "Agent::coordinatedPlan::found graphPoses with " << this->graphCoordination.poseGraph.nodeLocations.size() << " nodes" << endl;

			gLoc = graphCoordination.posePathPlanningTSP(graphCoordination.travelGraph, costmap, agentLocs, myIndex);
		}
	}
	market.updategLoc(gLoc);
}

void Agent::init(int myIndex, float obsThresh, float comThresh, int numAgents){
	this->obsThresh = obsThresh;
	this->comThresh = comThresh;
	
	cLoc= Point(-1,-1);
	gLoc = Point(-1,-1);
	gLocPrior = Point(-1,-1);

	this->myIndex = myIndex;
	this->pickMyColor();

	for(int i=0; i<numAgents; i++){
		agentLocs.push_back( Point(-1,-1) );
	}

	market.init(numAgents, myIndex, false);

	market.updatecLoc( cLoc );
	market.updateTime( ros::Time::now().toSec() );
	market.updategLoc( cLoc );
	market.updateExploreCost( 0 );


	marketInitialized = false;
	locationInitialized = false;
	costmapInitialized = false;
}

void Agent::act(){


	ros::Duration at = ros::Time::now() - actTimer;
	if( at.toSec() > 3 || gLocPrior != gLoc){
		gLocPrior = gLoc;
		actTimer = ros::Time::now();
		cout << "actTimer hit" << endl;

		bool flag = false;

		if( locationInitialized ){
			if( costmapInitialized ){
				if( marketInitialized ){
					flag = true;
				}
				else{
					cout << "waiting on market callback" << endl;
				}
			}
			else{
				cout << "waiting on costmap callback" << endl;
			}
		}
		else{
			cout << "waiting on location callback" << endl;
		}

		if( flag ){
			if(cLoc == gLoc){
				while(true){
					Point g;
					g.x = gLoc.x + rand() % 5 - 2;
					g.y = gLoc.y + rand() % 5 - 2;

					if(costmap.cells.at<short>(g) == costmap.obsFree){
						gLoc = g;
						break;
					}
				}
			}
			//cout << "Agent::act::found graphPoses with " << this->graphPlanning.poseGraph.nodeLocations.size() << " nodes" << endl;
			publishRvizMarker(cLoc, 0.5, 0, 0);
			publishRvizMarker(gLoc, 0.5, 2, 0);
			publishMarket();
			publishNavGoalsToMoveBase();

			cerr << "Agent::act::cLoc / gLoc: " << cLoc << " / " << gLoc << endl;
		}
	}
}


void Agent::showCellsPlot(){
	costmap.buildCellsPlot();

	circle(costmap.displayPlot,cLoc,2, myColor,-1, 8);
	circle(costmap.displayPlot,gLoc,2, Scalar(0,0,255),-1, 8);

	char buffer[50];
	sprintf(buffer,"Agent[%d]::costMap", myIndex);

	namedWindow(buffer, WINDOW_NORMAL);
	imshow(buffer, costmap.displayPlot);
	waitKey(10);
}

void Agent::pickMyColor(){
	Scalar a(0,0,0);
	this->myColor = a;

	if(this->myIndex == 0){
		this->myColor[0] = 255;
	}
	else if(this->myIndex == 1){
		this->myColor[1] = 255;
	}
	else if(this->myIndex == 2){
		this->myColor[2] = 255;
	}
	else if(this->myIndex == 3){
		this->myColor[0] = 255;
		this->myColor[1] = 153;
		this->myColor[2] = 51;
	}
	else if(this->myIndex == 4){
		this->myColor[0] = 255;
		this->myColor[1] = 255;
		this->myColor[2] = 51;
	}
	else if(this->myIndex == 5){
		this->myColor[0] = 255;
		this->myColor[1] = 51;
		this->myColor[2] = 255;
	}
	else if(this->myIndex == 6){
		this->myColor[0] = 51;
		this->myColor[1] = 255;
		this->myColor[2] = 255;
	}
	else if(this->myIndex == 7){
		this->myColor[0] = 153;
		this->myColor[1] = 255;
		this->myColor[2] = 51;
	}
	else if(this->myIndex == 8){
		this->myColor[0] = 255;
		this->myColor[1] = 255;
		this->myColor[2] = 255;
	}
	else if(this->myIndex == 9){
		// white
	}
}

Point Agent::reportToOperator(){

	// simulate communication of operator, find node on graph that is closest to agent that is in coms of operator, return it
	Mat tComs = Mat::zeros(costmap.cells.size(), CV_8UC1);
	graphCoordination.simulateCommunication( oLoc, tComs, costmap);

	vector<Point> comPts;
	for(size_t i=0; i<graphCoordination.thinGraph.nodeLocations.size(); i++){
		if( tComs.at<uchar>(graphCoordination.thinGraph.nodeLocations[i]) == 255 ){
			comPts.push_back( graphCoordination.thinGraph.nodeLocations[i] );
		}
	}

	vector<float> comDists;
	vector<bool> trueDists;
	float minDist = INFINITY;
	int mindex = -1;
	for(size_t i=0; i<comPts.size(); i++){

		comDists.push_back( sqrt(pow(cLoc.x-comPts[i].x,2) + pow(cLoc.y-comPts[i].y,2) ));
		trueDists.push_back( false );

		if(comDists.back() < minDist){
			minDist = comDists.back();
			mindex = i;
		}
	}

	if(mindex >= 0){
		while( true ){ // find closest
			if( trueDists[mindex]){
				return comPts[mindex];
			}
			comDists[mindex] = costmap.aStarDist(cLoc, comPts[mindex]);
			trueDists[mindex] = true;

			minDist = INFINITY;
			mindex= -1;
			for(size_t i=0; i<comDists.size(); i++){
				if(comDists[i] < minDist){
					minDist = comDists[i];
					mindex = i;
				}
			}
		}
	}

	return oLoc;
}

Point Agent::returnToOperator(){
	return oLoc;
}


void Agent::marketRelaySacrifice(){

	for(int a = 0; a<market.nAgents; a++){
		if( market.comCheck(a) ){
			if( market.roles[a] == -1){ // and they're not already a relay or sacrifice

				if( myIndex > a ){
					market.roles[myIndex] = 1; // I'll be a relay, they'll be a sacrifice
					market.roles[a] = 0;
					this->relayFlag = true;

					market.rLocs[myIndex] = market.cLocs[a];
					market.rLocs[a] = market.cLocs[a];

					market.mates[myIndex] = a;
					market.mates[a] = myIndex;
				}
				else{
					market.roles[myIndex] = 0; // I'll be a relay, they'll be a sacrifice
					market.roles[a] = 1;
					this->sacrificeFlag = true;

					market.rLocs[myIndex] = market.cLocs[myIndex];
					market.rLocs[a] = market.cLocs[myIndex];

					market.mates[myIndex] = a;
					market.mates[a] = myIndex;
				}
			}
		}
	}
}

void Agent::marketReturnInfo(){

	bool flag = false;

	if( market.reportRequests[market.myIndex] == 1){
		cout << "got a market request" << endl;
		market.reportRequests[market.myIndex] = 0;
		market.reportTimes[market.myIndex] = reportInterval;
		timeOfLastReport = ros::Time::now();
		timeOfNextReport = timeOfLastReport + ros::Duration(reportInterval);
		reportFlag = false;
		flag = true;
	}

	for( int a = 0; a<market.nAgents; a++){
		if( market.comCheck(a) ){ // am I in contact with them currently?
			cout << "in coms" << endl;
			//cin.ignore();
			if( market.reportCosts[myIndex] < market.reportCosts[a]+1 ){ // am I closer to observer?
				//cout << "myCost is less, I'll report" << endl;
				//cin.ignore();
				if( market.reportTimes[myIndex] <= market.reportTimes[a] ){ // if I have longer until I report
					market.reportTimes[myIndex] = market.reportTimes[a]; // I get their report time
					market.reportRequests[a] = 1; // they reset their report time
					flag = true;
				}
			}
		}
	}
}


Point Agent::returnToRelayPt(){
	return rLoc;
}

Point Agent::reportToRelayPt(){
	// simulate communication of operator, find node on graph that is closest to agent that is in coms of operator, return it
	Mat tComs = Mat::zeros(costmap.cells.size(), CV_8UC1);
	graphCoordination.simulateCommunication( rLoc, tComs, costmap);

	vector<Point> comPts;
	for(size_t i=0; i<graphCoordination.thinGraph.nodeLocations.size(); i++){
		if( tComs.at<uchar>(graphCoordination.thinGraph.nodeLocations[i]) == 255 ){
			comPts.push_back( graphCoordination.thinGraph.nodeLocations[i] );
		}
	}

	vector<float> comDists;
	vector<bool> trueDists;
	float minDist = INFINITY;
	int mindex = -1;
	for(size_t i=0; i<comPts.size(); i++){

		comDists.push_back( sqrt(pow(cLoc.x-comPts[i].x,2) + pow(cLoc.y-comPts[i].y,2) ));
		trueDists.push_back( false );

		if(comDists.back() < minDist){
			minDist = comDists.back();
			mindex = i;
		}
	}

	if(mindex >= 0){
		while( true ){ // find closest
			if( trueDists[mindex]){
				return comPts[mindex];
			}
			comDists[mindex] = costmap.aStarDist(cLoc, comPts[mindex]);
			trueDists[mindex] = true;

			minDist = INFINITY;
			mindex= -1;
			for(size_t i=0; i<comDists.size(); i++){
				if(comDists[i] < minDist){
					minDist = comDists[i];
					mindex = i;
				}
			}
		}
	}

	return rLoc;
}

bool Agent::checkReturnTime(){

	ros::Time ct = ros::Time::now();
	ros::Duration dt =  ct - missionTime;
	missionTime = ct;
	checkTimeReturn -= dt;
	if(checkTimeReturn.toSec() <= 0){

		float dR;
		if(relayFlag){
			dR = getDistanceToReturnToOperatorFromRelayPt() + getDistanceToReturnToRelayPt();
		}
		else if( sacrificeFlag ){
			dR = getDistanceToReportToRelayPt();
		}
		else{
			dR = getDistanceToReturnToOperator();
		}
		float travelTime = dR/travelSpeed;
		float timeLeft = maxMissionTime.toSec() - missionTime.toSec();
		if( travelTime >= timeLeft){
			returnFlag = true;
			return true;
		}
		else{
			checkTimeReturn = ros::Duration(travelTime); // time it takes to return from current location plus slop
			returnFlag = false;
			return false;
		}
	}
	else{
		returnFlag = false;
		return false;
	}
}

bool Agent::checkForExplorationFinished(){

	bool flag = false;
	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point t(i,j);
			if(costmap.cells.at<short>(t) == costmap.infFree || costmap.cells.at<short>(t) == costmap.domFree){
				return false;
			}
			else if(costmap.cells.at<short>(t) == costmap.obsFree){
				flag = true;
				Point t1(i-1,j);
				if( costmap.cells.at<short>(t1) == costmap.unknown){
					return false;
				}
				t1 = Point(i+1,j);
				if( costmap.cells.at<short>(t1) == costmap.unknown){
					return false;
				}
				t1 = Point(i+1,j-1);
				if( costmap.cells.at<short>(t1) == costmap.unknown){
					return false;
				}
				t1 = Point(i,j+1);
				if( costmap.cells.at<short>(t1) == costmap.unknown){
					return false;
				}
			}
		}
	}
	if( flag ){
		return true;
	}
	else{
		return false;
	}
}

bool Agent::lastReport(){

	// if 2 * report interval < return time left then this is the last report, disable reporting
	ros::Duration dt = maxMissionTime - missionTime;
	if( dt.toSec() < 2*reportInterval ){
		return true;
	}
	return false;
}

float Agent::getDistanceToReturnToOperator(){
	return costmap.aStarDist(cLoc, oLoc);
}

float Agent::getDistanceToReportToOperator(){

	// simulate communication of operator, find node on graph that is closest to agent that is in coms of operator, return it
	Mat tComs = Mat::zeros(costmap.cells.size(), CV_8UC1);
	graphCoordination.simulateCommunication( oLoc, tComs, costmap);

	vector<Point> comPts;
	for(size_t i=0; i<graphCoordination.thinGraph.nodeLocations.size(); i++){
		if( tComs.at<uchar>(graphCoordination.thinGraph.nodeLocations[i]) == 255 ){
			comPts.push_back( graphCoordination.thinGraph.nodeLocations[i] );
			//circle(tComs, graphCoordination.thinGraph.nodeLocations[i], 2, Scalar(127), -1, 8);
		}
	}

	//circle(tComs, cLoc, 3, Scalar(0), -1, 8);
	//circle(tComs, cLoc, 1, Scalar(255), -1, 8);

	//namedWindow("tComs", WINDOW_NORMAL);
	//imshow("tComs", tComs);
	//waitKey(0);

	vector<float> comDists;
	vector<bool> trueDists;
	float minDist = INFINITY;
	int mindex = -1;
	for(size_t i=0; i<comPts.size(); i++){

		comDists.push_back( sqrt(pow(cLoc.x-comPts[i].x,2) + pow(cLoc.y-comPts[i].y,2) ));
		trueDists.push_back( false );

		if(comDists.back() < minDist){
			minDist = comDists.back();
			mindex = i;
		}
	}


	if(mindex >= 0){
		while( true ){ // find closest
			//circle(tComs, comPts[mindex], 2, Scalar(127), -1, 8);
			//circle(tComs, comPts[mindex], 1, Scalar(0), -1, 8);
			//cout << "comDist[mindex] = " << comDists[mindex] << endl;

			//namedWindow("tComs", WINDOW_NORMAL);
			//imshow("tComs", tComs);
			//waitKey(0);

			if( trueDists[mindex]){
				return comDists[mindex];
			}
			comDists[mindex] = costmap.aStarDist(cLoc, comPts[mindex]);
			trueDists[mindex] = true;

			minDist = INFINITY;
			mindex= -1;
			for(size_t i=0; i<comDists.size(); i++){
				if(comDists[i] < minDist){
					minDist = comDists[i];
					mindex = i;
				}
			}
		}
	}

	return costmap.aStarDist(cLoc, oLoc);
}

float Agent::getDistanceToReportToRelayPt(){
	// simulate communication of operator, find node on graph that is closest to agent that is in coms of operator, return it
	Mat tComs = Mat::zeros(costmap.cells.size(), CV_8UC1);
	graphCoordination.simulateCommunication( rLoc, tComs, costmap);

	vector<Point> comPts;
	for(size_t i=0; i<graphCoordination.thinGraph.nodeLocations.size(); i++){
		if( tComs.at<uchar>(graphCoordination.thinGraph.nodeLocations[i]) == 255 ){
			comPts.push_back( graphCoordination.thinGraph.nodeLocations[i] );
			//circle(tComs, graphCoordination.thinGraph.nodeLocations[i], 2, Scalar(127), -1, 8);
		}
	}

	//circle(tComs, cLoc, 3, Scalar(0), -1, 8);
	//circle(tComs, cLoc, 1, Scalar(255), -1, 8);

	//namedWindow("tComs", WINDOW_NORMAL);
	//imshow("tComs", tComs);
	//waitKey(0);

	vector<float> comDists;
	vector<bool> trueDists;
	float minDist = INFINITY;
	int mindex = -1;
	for(size_t i=0; i<comPts.size(); i++){

		comDists.push_back( sqrt(pow(cLoc.x-comPts[i].x,2) + pow(cLoc.y-comPts[i].y,2) ));
		trueDists.push_back( false );

		if(comDists.back() < minDist){
			minDist = comDists.back();
			mindex = i;
		}
	}


	if(mindex >= 0){
		while( true ){ // find closest
			//circle(tComs, comPts[mindex], 2, Scalar(127), -1, 8);
			//circle(tComs, comPts[mindex], 1, Scalar(0), -1, 8);
			//cout << "comDist[mindex] = " << comDists[mindex] << endl;

			//namedWindow("tComs", WINDOW_NORMAL);
			//imshow("tComs", tComs);
			//waitKey(0);

			if( trueDists[mindex]){
				return comDists[mindex];
			}
			comDists[mindex] = costmap.aStarDist(cLoc, comPts[mindex]);
			trueDists[mindex] = true;

			minDist = INFINITY;
			mindex= -1;
			for(size_t i=0; i<comDists.size(); i++){
				if(comDists[i] < minDist){
					minDist = comDists[i];
					mindex = i;
				}
			}
		}
	}

	return costmap.aStarDist(cLoc, rLoc);
}

float Agent::getDistanceToReturnToRelayPt(){
	return costmap.aStarDist(cLoc, rLoc);
}

float Agent::getDistanceToReportToOperatorFromRelayPt(){
	// simulate communication of operator, find node on graph that is closest to agent that is in coms of operator, return it
	Mat tComs = Mat::zeros(costmap.cells.size(), CV_8UC1);
	graphCoordination.simulateCommunication( oLoc, tComs, costmap);

	vector<Point> comPts;
	for(size_t i=0; i<graphCoordination.thinGraph.nodeLocations.size(); i++){
		if( tComs.at<uchar>(graphCoordination.thinGraph.nodeLocations[i]) == 255 ){
			comPts.push_back( graphCoordination.thinGraph.nodeLocations[i] );
			//circle(tComs, graphCoordination.thinGraph.nodeLocations[i], 2, Scalar(127), -1, 8);
		}
	}

	//circle(tComs, cLoc, 3, Scalar(0), -1, 8);
	//circle(tComs, cLoc, 1, Scalar(255), -1, 8);

	//namedWindow("tComs", WINDOW_NORMAL);
	//imshow("tComs", tComs);
	//waitKey(0);

	vector<float> comDists;
	vector<bool> trueDists;
	float minDist = INFINITY;
	int mindex = -1;
	for(size_t i=0; i<comPts.size(); i++){

		comDists.push_back( sqrt(pow(rLoc.x-comPts[i].x,2) + pow(rLoc.y-comPts[i].y,2) ));
		trueDists.push_back( false );

		if(comDists.back() < minDist){
			minDist = comDists.back();
			mindex = i;
		}
	}


	if(mindex >= 0){
		while( true ){ // find closest
			//circle(tComs, comPts[mindex], 2, Scalar(127), -1, 8);
			//circle(tComs, comPts[mindex], 1, Scalar(0), -1, 8);
			//cout << "comDist[mindex] = " << comDists[mindex] << endl;

			//namedWindow("tComs", WINDOW_NORMAL);
			//imshow("tComs", tComs);
			//waitKey(0);

			if( trueDists[mindex]){
				return comDists[mindex];
			}
			comDists[mindex] = costmap.aStarDist(rLoc, comPts[mindex]);
			trueDists[mindex] = true;

			minDist = INFINITY;
			mindex= -1;
			for(size_t i=0; i<comDists.size(); i++){
				if(comDists[i] < minDist){
					minDist = comDists[i];
					mindex = i;
				}
			}
		}
	}

	return costmap.aStarDist(rLoc, oLoc);
}

float Agent::getDistanceToReturnToOperatorFromRelayPt(){
	return costmap.aStarDist(oLoc, rLoc);
}

bool Agent::checkReportTime(){

	if( lastReport() ){ // would this be my last time to report?
		return false;
	}

	if(market.times[market.nAgents] <= 1){
		timeOfLastReport = ros::Time::now();
		timeOfNextReport = timeOfLastReport + ros::Duration( reportInterval );
		checkTimeReport = ros::Duration(-1.0);
		market.reportTimes[market.myIndex] = timeOfNextReport.toSec();
		reportFlag = false;
	}

	if(checkTimeReport.toSec() <= 0){ // has enough time gone by? if yes check distance to update time
		// may not have travelled in a straight line away from operator location
		float travelTime = getDistanceToReportToOperator() / travelSpeed;
		market.reportCosts[market.myIndex] = travelTime;

		if( travelTime >= timeOfNextReport.toSec() - ros::Time::now().toSec() ){
			reportFlag = true;
			return true;
		}
		else{
			checkTimeReport = ros::Duration(travelTime);
			reportFlag = false;
			return false;
		}
	}
	else{
		reportFlag = false;
		return false;
	}
}

void Agent::shareCostmap(Costmap &A, Costmap &B){
	for(int i=0; i<A.cells.cols; i++){
		for(int j=0; j<A.cells.rows; j++){
			Point a(i,j);

			// share cells

			if(A.cells.at<short>(a) != B.cells.at<short>(a) ){ // do we think the same thing?
				if(A.cells.at<short>(a) == A.unknown){
					A.cells.at<short>(a) = B.cells.at<short>(a); // if A doesn't know, anything is better
				}
				else if(A.cells.at<short>(a) == A.infFree || A.cells.at<short>(a) == A.infWall){ // A think its inferred
					if(B.cells.at<short>(a) == B.obsFree || B.cells.at<short>(a) == B.obsWall){ // B has observed
						A.cells.at<short>(a) = B.cells.at<short>(a);
					}
				}
				else if(B.cells.at<short>(a) == B.unknown){ // B doesn't know
					B.cells.at<short>(a) = A.cells.at<short>(a); // B doesn't know, anything is better
				}
				else if(B.cells.at<short>(a) == B.infFree || B.cells.at<short>(a) == B.infWall){ // B think its inferred
					if(A.cells.at<short>(a) == A.obsFree || A.cells.at<short>(a) == A.obsWall){ // A has observed
						B.cells.at<short>(a) = A.cells.at<short>(a);
					}
				}
			}


			// share search
			/*
			if(A.searchReward.at<float>(a) < B.searchReward.at<float>(a) ){ // do we think the same thing?
				B.searchReward.at<float>(a) = A.searchReward.at<float>(a);
			}
			else if(A.searchReward.at<float>(a) > B.searchReward.at<float>(a)){
				A.searchReward.at<float>(a) = B.searchReward.at<float>(a);
			}

			// share occ

			float minA = INFINITY;
			float minB = INFINITY;

			if(1-A.occ.at<float>(a) > A.occ.at<float>(a) ){
				minA = 1-A.occ.at<float>(a);
			}
			else{
				minA = A.occ.at<float>(a);
			}

			if(1-B.occ.at<float>(a) > B.occ.at<float>(a) ){
				minB = 1-B.occ.at<float>(a);
			}
			else{
				minB = B.occ.at<float>(a);
			}

			if( minA < minB ){ // do we think the same thing?
				B.occ.at<float>(a) = A.occ.at<float>(a);
			}
			else{
				A.occ.at<float>(a) = B.occ.at<float>(a);
			}
			*/
		}
	}
}



Agent::~Agent() {

}

