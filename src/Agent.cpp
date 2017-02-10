/*
 * Agent.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: andy
 */


#include "Agent.h"

Agent::Agent(ros::NodeHandle nHandle, int myIndex, int numAgents){
	A_angle = -1;
	B_angle = -1;
	costmap.init_flag = true;

	missionTime = ros::Time::now();
	timeOfLastReport = ros::Time::now();

	maxMissionTime = ros::Time::now() + ros::Duration(50000);
	travelSpeed = 10;
	actTimer = ros::Time::now();
	explorationComplete = false;

	oLoc = Point(50,60);
	float lastPlan = -1;

	graphCoordination.init(20, 20);

	planningMethod = "selectPose";
	//planningMethod = "greedyFrontiers";
	//planningMethod = "marketFrontiers";


	//inferenceMethod = "naive";
	inferenceMethod = "geometric";
	//inferenceMethod = "visual";


	// sensor stuff
	costSub = nHandle.subscribe("/map", 0, &Agent::costMapCallback, this);
	locSub = nHandle.subscribe("/odom", 1, &Agent::locationCallback, this);

		// rviz stuff
	markerPub = nHandle.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
	goalPub =  nHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);	
	
	if(myIndex == 0){
		// my coordination stuff
		marketPub = nHandle.advertise<std_msgs::Float32MultiArray>("/agent0/market", 10);
		locPub = nHandle.advertise<nav_msgs::Odometry>("/agent0/loc", 10);
		mapUpdatesPub = nHandle.advertise<std_msgs::Int16MultiArray>("/agent0/map", 10);

		// their coordination stuff
		marketSub_A = nHandle.subscribe("/agent1/market", 1, &Agent::marketCallback, this);
		mapUpdatesSub_A = nHandle.subscribe("/agent1/map", 1, &Agent::mapUpdatesCallback_A, this);

		marketSub_B = nHandle.subscribe("/agent2/market", 1, &Agent::marketCallback, this);
		mapUpdatesSub_B = nHandle.subscribe("/agent2/map", 1, &Agent::mapUpdatesCallback_B, this);
	}
	else if(myIndex == 1){
		// my coordination stuff
		marketPub = nHandle.advertise<std_msgs::Float32MultiArray>("/agent1/market", 10);
		locPub = nHandle.advertise<nav_msgs::Odometry>("/agent1/loc", 10);
		mapUpdatesPub = nHandle.advertise<std_msgs::Int16MultiArray>("/agent1/map", 10);

		// their coordination stuff
		marketSub_A = nHandle.subscribe("/agent0/market", 1, &Agent::marketCallback, this);
		mapUpdatesSub_A = nHandle.subscribe("/agent0/map", 1, &Agent::mapUpdatesCallback_A, this);

		marketSub_B = nHandle.subscribe("/agent2/market", 1, &Agent::marketCallback, this);
		mapUpdatesSub_B = nHandle.subscribe("/agent2/map", 1, &Agent::mapUpdatesCallback_B, this);
	}
	else if(myIndex == 2){
		// my coordination stuff
		marketPub = nHandle.advertise<std_msgs::Float32MultiArray>("/agent2/market", 10);
		locPub = nHandle.advertise<nav_msgs::Odometry>("/agent2/loc", 10);
		mapUpdatesPub = nHandle.advertise<std_msgs::Int16MultiArray>("/agent2/map", 10);

		// their coordination stuff
		marketSub_A = nHandle.subscribe("/agent0/market", 1, &Agent::marketCallback, this);
		mapUpdatesSub_A = nHandle.subscribe("/agent0/map", 1, &Agent::mapUpdatesCallback_A, this);

		marketSub_B = nHandle.subscribe("/agent1/market", 1, &Agent::marketCallback, this);
		mapUpdatesSub_B = nHandle.subscribe("/agent1/map", 1, &Agent::mapUpdatesCallback_B, this);
	}

	//moveBaseRecover stuff
	subResult = nHandle.subscribe("move_base/status", 10, &Agent::actionCallback, this) ;
    subCmdVel = nHandle.subscribe("controller_cmd_vel", 10, &Agent::cmdVelCallback, this) ;
    pubCmdVel = nHandle.advertise<geometry_msgs::Twist>("pioneer/cmd_vel", 10) ;
  
    ros::param::get("move_base/TrajectoryPlannerROS/max_vel_theta",thetaThreshold) ;
    ROS_INFO_STREAM("Recovery behaviour with rotate robot in place at " << thetaThreshold << " rad/s") ;
    timeThreshold = 1.0 ;
    fGoal = false ;
    fTimer = false ;
    fRecovery = false;

    this->init(myIndex, numAgents);
}

void Agent::init(int myIndex, int numAgents){
	this->obsThresh = 50;
	this->comThresh = 50;
	this->myIndex = myIndex;

	cLoc= Point(-1,-1);
	gLoc = Point(-1,-1);
	gLocPrior = Point(-1,-1);

	//this->myIndex = myIndex;
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


void Agent::actionCallback(const actionlib_msgs::GoalStatusArray& msg){
  if (!msg.status_list.empty()){
    if (msg.status_list[0].status == 1)
      fGoal = true ;
    else
      fGoal = false ;
  }
}

void Agent::cmdVelCallback(const geometry_msgs::Twist& msg){
  geometry_msgs::Twist cmd = msg ;
  if (fRecovery){ // recovery mode
    ros::Duration recoveryDuration = ros::Time::now() - initialRecoveryTime ;
    if (recoveryDuration.toSec() < 1.5) // force 1.5 second turnaround
      cmd.angular.z = recoveryTheta ;
    else { // exit recovery mode
      ROS_INFO("Robot exiting recovery mode...") ;
      fRecovery = false ;
      fTimer = false ;
    }
  }
  else if (fGoal){
    if (fabs(msg.linear.x) < 0.001 && fabs(msg.angular.z) < thetaThreshold){
      if (!fTimer){
        initialStoppingTime = ros::Time::now() ;
        fTimer = true ;
      }
      else{
        ros::Duration stoppedTime = ros::Time::now() - initialStoppingTime ;
        if (stoppedTime.toSec() > timeThreshold){
          geometry_msgs::Twist cmd = msg ;
          ROS_INFO("Overriding move_base command velocities to unstick robot in recovery mode...") ;
          fRecovery = true ; // enter recovery mode
          if (msg.angular.z < 0.0)
            recoveryTheta = -thetaThreshold ;
          else
            recoveryTheta = thetaThreshold ;
          cmd.angular.z = recoveryTheta ;
          initialRecoveryTime = ros::Time::now() ;
        }
      }
    }
    else
      fTimer = false ;
  }
  pubCmdVel.publish(cmd) ;
}

double Agent::linearDist(vector<Point2f> &sub_pts, vector<Point2f> &set_pts, vector<Point2f> &sub_matches, vector<Point2f> &set_matches, float tol){
	double distSum = 0;
	for(size_t i=0; i<sub_pts.size(); i++){
		double minDist = INFINITY;
		int mindex = -1;
		for(size_t j=0; j<set_pts.size(); j++){
			double d = pow(set_pts[j].x - sub_pts[i].x,2) + pow(set_pts[j].y - sub_pts[i].y,2);
			if( d < minDist ){
				minDist = d;
				mindex = j;
			}
		}
		if(minDist < tol){
			distSum += minDist;
			sub_matches.push_back( sub_pts[i] ); // only use pts within a set tolerance
			set_matches.push_back( set_pts[mindex] );
		}
		else{
			distSum += 2*tol;
		}
	}
	return distSum;
	/*
	double distSum = 0;
	for(int i=0; i<4; i++){
		while(true){
			double minDist = INFINITY;
			int mindex = -1;
			int rind = rand() % sub_pts.size();
			for(size_t j=0; j<set_pts.size(); j++){
				double d = pow(set_pts[j].x - sub_pts[rind].x,2) + pow(set_pts[j].y - sub_pts[rind].y,2);
				if( d < minDist ){
					minDist = d;
					mindex = j;
				}
			}
			if(minDist < tol){
				distSum += minDist;
				sub_matches.push_back( sub_pts[rind] ); // only use pts within a set tolerance
				set_matches.push_back( set_pts[mindex] );
				break;
			}
		}
	}*/
	return distSum;
}

void Agent::plotMatches( vector<Point2f> &set_pts, vector<Point2f> &sub_matches, vector<Point2f> &set_matches){
	Mat t = Mat::zeros( costmap.cells.size(), CV_8UC3);

	Vec3b red(0,0,255);
	Vec3b blue(0,255,0);
	Vec3b white(255,255,255);

	for(size_t i=0; i<set_pts.size(); i++){
		t.at<Vec3b>(set_pts[i]) = white;
	}

	for(size_t i=0; i<sub_matches.size(); i++){
		t.at<Vec3b>(sub_matches[i]) = red;
		t.at<Vec3b>(set_matches[i]) = blue;
	}

	namedWindow("align map", WINDOW_NORMAL);
	imshow("align map", t);
	waitKey(50);
}

void Agent::getWallPts(Mat &mat, vector<Point2f> &pts){
	for(int i=0; i<mat.cols; i++){
		for(int j=0; j<mat.rows; j++){
			Point p(i,j);
			if(mat.at<short>(p) == costmap.obsWall){
				pts.push_back( p );
			}
		}
	}
}

void Agent::getFreePts(Mat &mat, vector<Point2f> &pts){
	for(int i=0; i<mat.cols; i++){
		for(int j=0; j<mat.rows; j++){
			Point p(i,j);
			if(mat.at<short>(p) == costmap.obsFree){
				pts.push_back( p );
			}
		}
	}
}

double Agent::alignCostmap( Mat &set, Mat &sub, Mat &homography){
	// set is my costmap, remains fixed
	// sub is their costmap, gets aligned
	// homography is the kept cumulative homography matrix from origin to align, updated iteratively


	// get wall pts
	vector<Point2f> sub_wall_pts, sub_free_pts, set_wall_pts, set_free_pts;
	getWallPts(set, set_wall_pts);
	getWallPts(sub, sub_wall_pts);
	getFreePts(set, set_free_pts);
	getFreePts(sub, sub_free_pts);

	Mat lastGood;
	Mat transform;
	vector<float> dists;
	vector<Point2f> sub_wall_raw = sub_wall_pts;
	float dist_tol = 20.0;

	Mat rot_wall_temp = Mat(sub_wall_raw, CV_32FC1);
	Mat rot_wall_matches = Mat::zeros(rot_wall_temp.size(), CV_32FC1);
	perspectiveTransform(rot_wall_temp, rot_wall_matches, homography);
	sub_wall_pts = rot_wall_matches;

	while(true) {
		vector<Point2f> sub_wall_matches, set_wall_matches;
	    double distSum = linearDist(sub_wall_pts, set_wall_pts, set_wall_matches, sub_wall_matches, dist_tol);
	    
	    plotMatches( set_wall_pts, sub_wall_matches, set_wall_matches);
	    waitKey(70);

		float param[3] = {0,0,0};
		float R = fit3DofQUADRATIC(set_wall_matches, sub_wall_matches, param);

		cv::Mat H = cv::Mat(3,3,CV_64FC1);
		H.at<double>(0,0) = cos(param[0]);
		H.at<double>(0,1) = -sin(param[0]);
		H.at<double>(0,2) = param[1];
		H.at<double>(1,0) = sin(param[0]);
		H.at<double>(1,1) = cos(param[0]);
		H.at<double>(1,2) = param[2];
		H.at<double>(2,0) = 0.0;
		H.at<double>(2,1) = 0.0;
		H.at<double>(2,2) = 1.0;

		homography = homography * H;

		Mat rot_wall_temp = Mat(sub_wall_raw, CV_32FC1);
		Mat rot_wall_matches = Mat::zeros(rot_wall_temp.size(), CV_32FC1);
		perspectiveTransform(rot_wall_temp, rot_wall_matches, homography);

		sub_wall_pts = rot_wall_matches;

		//cout << "Param: " << param[0] << " , " << param[1] << " , " << param[2] << endl;
		if(abs(param[0]) + abs(param[1]) + abs(param[2]) < 0.01){
			break;
		}
	}

	return 0;
}

// fits 3DOF (rotation and translation in 2D) with least squares.
float Agent::fit3DofQUADRATIC(const vector<Point2f>& src_, const vector<Point2f>& dst_, float* param){
// http://stackoverflow.com/questions/35765546/convert-from-mat-to-point2f-or-point3f
    const bool debug = false;                   // print more debug info
    assert(dst_.size() == src_.size());
    int N = src_.size();

    // collect inliers
    vector<Point2f> src, dst;
    int ninliers;
	ninliers = N;
	src = src_; // copy constructor
	dst = dst_;

    if (ninliers<2) {
        param[0] = 0.0f; // default return when there is not enough points
        param[1] = 0.0f;
        param[2] = 0.0f;
        return 1000000;
    }

    /* Algorithm: Least-Square Rigid Motion Using SVD by Olga Sorkine
     * http://igl.ethz.ch/projects/ARAP/svd_rot.pdf
     *
     * Subtract centroids, calculate SVD(cov),
     * R = V[1, det(VU')]'U', T = mean_q-R*mean_p
     */

    // Calculate data centroids
    Scalar centroid_src = mean(src);
    Scalar centroid_dst = mean(dst);
    Point2f center_src(centroid_src[0], centroid_src[1]);
    Point2f center_dst(centroid_dst[0], centroid_dst[1]);
    if (debug)
        cerr<<"Centers: "<<center_src<<", "<<center_dst<<endl;

    // subtract centroids from data
    for (int i=0; i<ninliers; i++) {
        src[i] -= center_src;
        dst[i] -= center_dst;
    }

    // compute a covariance matrix
    float Cxx = 0.0, Cxy = 0.0, Cyx = 0.0, Cyy = 0.0;
    for (int i=0; i<ninliers; i++) {
        Cxx += src[i].x*dst[i].x;
        Cxy += src[i].x*dst[i].y;
        Cyx += src[i].y*dst[i].x;
        Cyy += src[i].y*dst[i].y;
    }
    Mat Mcov = (Mat_<float>(2, 2)<<Cxx, Cxy, Cyx, Cyy);
    Mcov /= (ninliers-1);
    if (debug)
        cerr<<"Covariance-like Matrix "<<Mcov<<endl;

    // SVD of covariance
    cv::SVD svd;
    svd = SVD(Mcov, SVD::FULL_UV);
    if (debug) {
        cerr<<"U = "<<svd.u<<endl;
        cerr<<"W = "<<svd.w<<endl;
        cerr<<"V transposed = "<<svd.vt<<endl;
    }

    // rotation (V*Ut)
    Mat V = svd.vt.t();
    Mat Ut = svd.u.t();
    float det_VUt = determinant(V*Ut);
    Mat W = (Mat_<float>(2, 2)<<1.0, 0.0, 0.0, det_VUt);
    float rot[4];
    Mat R_est(2, 2, CV_32F, rot);
    R_est = V*W*Ut;
    if (debug)
        cerr<<"Rotation matrix: "<<R_est<<endl;

    float cos_est = rot[0];
    float sin_est = rot[2];
    float ang = atan2(sin_est, cos_est);

    // translation (mean_dst - R*mean_src)
    Point2f center_srcRot = Point2f(
            cos_est*center_src.x - sin_est*center_src.y,
            sin_est*center_src.x + cos_est*center_src.y);
    Point2f T_est = center_dst - center_srcRot;

    // Final estimate msg
    if (debug)
        cerr<<"Estimate = "<< ang*360/6.26 <<"deg., T = "<<T_est<<endl;

    param[0] = ang; // rad
    param[1] = T_est.x;
    param[2] = T_est.y;
} // fit3DofQUADRATIC()

void Agent::mapUpdatesCallback_A(  const std_msgs::Int16MultiArray& transmission ){

	if(myIndex == 0){
		// agent 0 -> agent1
		A_shift = Point(10,0);
		A_angle = 185;
	}
	else if(myIndex == 1){
		// agent 1 -> agent0
		A_shift = Point(-10,0);
		A_angle = 205;
	}
	else if(myIndex == 2){
		// agent2 -> agent0
		A_shift = Point(-10,-25);
		A_angle = 180; 
	}

	//Mat matA_wall = Mat::zeros( costmap.cells.size(), CV_8UC1 );
	Mat matA_wall = Mat::zeros( 192, 192, CV_8UC1 );
	Mat matA_free = Mat::zeros( matA_wall.size(), CV_8UC1 );
	
	initMap( transmission, A_shift, A_angle, matA_free, matA_wall );
	/*
	namedWindow("A free rot", WINDOW_NORMAL);
	imshow("A free rot", matA_free);
	waitKey(70);

	namedWindow("A wall rot", WINDOW_NORMAL);
	imshow("A wall rot", matA_wall);
	waitKey(70);
	*/
	A_cells = Mat::ones(matA_wall.size(), CV_16SC1)*costmap.unknown;
	vector<Point2f> A_wall_pts, A_free_pts;
	for(int i=0; i<matA_wall.cols; i++){
		for(int j=0; j<matA_wall.rows; j++){
			Point a(i,j);
			if(matA_free.at<uchar>(a) >= 127){
				A_cells.at<short>(a) = costmap.obsFree;
				A_free_pts.push_back(a);
			}
			else if(matA_wall.at<uchar>(a) >= 127){
				A_cells.at<short>(a) = costmap.obsWall;
				A_wall_pts.push_back(a);
			}
		}
	}
	
	if( !costmap.cells.empty() ){

		if( A_homography.empty() ){
			A_homography = Mat::eye(3, 3, CV_64FC1);
			
			if(myIndex == 0){
				// agent0 -> agent1
				A_homography.at<double>(0,0) = 0.9021490578708488;
				A_homography.at<double>(0,1) = -0.4314247333114125;
				A_homography.at<double>(0,2) = 74.07028346057548;
				A_homography.at<double>(1,0) = 0.4314247333114125;
				A_homography.at<double>(1,1) = 0.9021490578708488;
				A_homography.at<double>(1,2) = -32.11890638977299;
				A_homography.at<double>(2,0) = 0;
				A_homography.at<double>(2,1) = 0;
				A_homography.at<double>(2,2) = 1;
			}
			else if(myIndex == 1){
				// agent1 -> agent0
				A_homography.at<double>(0,0) = 0.9959783994237053;
				A_homography.at<double>(0,1) = -0.08959927912193513;
				A_homography.at<double>(0,2) = 11.58855251649272;
				A_homography.at<double>(1,0) = 0.08959927912193513;
				A_homography.at<double>(1,1) = 0.9959783994237053;
				A_homography.at<double>(1,2) = -13.16001256806977;
				A_homography.at<double>(2,0) = 0;
				A_homography.at<double>(2,1) = 0;
				A_homography.at<double>(2,2) = 1;
			}
			else if(myIndex == 2){
				// agent2 -> agent0
				A_homography.at<double>(0,0) = 0.9904669908203648;
				A_homography.at<double>(0,1) = 0.1377500043953933;
				A_homography.at<double>(0,2) = -14.8143625208088;
				A_homography.at<double>(1,0) = -0.1377500043953933;
				A_homography.at<double>(1,1) = 0.9904669908203648;
				A_homography.at<double>(1,2) = 17.87725265552533;
				A_homography.at<double>(2,0) = 0;
				A_homography.at<double>(2,1) = 0;
				A_homography.at<double>(2,2) = 1;
			}
		}

		double dist = alignCostmap( costmap.cells, A_cells, A_homography);

		Mat rot_wall_temp = Mat(A_wall_pts, CV_32FC1);
		Mat rot_wall_matches = Mat::zeros(rot_wall_temp.size(), CV_32FC1);
		perspectiveTransform(rot_wall_temp, rot_wall_matches, A_homography);
		vector<Point2f> rot_wall_vec = rot_wall_matches;

		Mat rot_free_temp = Mat(A_free_pts, CV_32FC1);
		Mat rot_free_matches = Mat::zeros(rot_wall_temp.size(), CV_32FC1);
		perspectiveTransform(rot_free_temp, rot_free_matches, A_homography);
		vector<Point2f> rot_free_vec = rot_free_matches;

		if( dist < INFINITY ){
			for(size_t i=0; i<rot_wall_vec.size(); i++){
				Point p = rot_wall_vec[i];
				if( costmap.cells.at<short>(p) != costmap.obsFree || costmap.cells.at<short>(p) != costmap.obsWall){
					costmap.cells.at<short>(p) = costmap.obsWall;
				}
			}
			for(size_t i=0; i<rot_free_vec.size(); i++){
				Point p = rot_free_vec[i];

				if( costmap.cells.at<short>(p) != costmap.obsFree || costmap.cells.at<short>(p) != costmap.obsWall){
					costmap.cells.at<short>(p) = costmap.obsFree;
				}
			}
		}

		inference.makeInference( inferenceMethod, costmap );
		costmap.buildCellsPlot();
		costmap.addAgentToCostmapPlot( myColor, myPath, cLoc);
		costmap.addAgentToCostmapPlot( myColor, myPath, gLoc);
		costmap.showCostmapPlot(myIndex);
	}

	Mat dispA = Mat::zeros( matA_free.size(),CV_8UC3 );
	for(int i=0; i<A_cells.cols; i++){
		for(int j=0; j<A_cells.rows; j++){
			Point a(i,j);
			if(A_cells.at<short>(a) == costmap.obsFree){
				dispA.at<Vec3b>(a) = costmap.cObsFree;
			}
			else if(A_cells.at<short>(a) == costmap.obsWall){
				dispA.at<Vec3b>(a) = costmap.cObsWall;
			}
			else{ // anything else, should never happen
				dispA.at<Vec3b>(a) = costmap.cUnknown;
			}
		}
	}

	namedWindow("mapUpdates A", WINDOW_NORMAL);
	imshow("mapUpdates A", dispA);
	waitKey(70);
}

void Agent::mapUpdatesCallback_B(  const std_msgs::Int16MultiArray& transmission ){

	if(myIndex == 0){
		// agent0 -> agent2
		B_shift = Point(-10,-35);
		B_angle = 170;
	}
	else if(myIndex == 1){
		//agent 1 -> agent2
		B_shift = Point(-2,-32);
		B_angle = 15;
	}
	else if(myIndex == 2){
		// agent2 -> agent1
		B_shift = Point(12, 32);
		B_angle = -15;
	}

	//Mat matB_wall = Mat::zeros( costmap.cells.size(), CV_8UC1 );
	Mat matB_wall = Mat::zeros( 192, 192, CV_8UC1 );
	Mat matB_free = Mat::zeros( matB_wall.size(), CV_8UC1 );

	/*
	namedWindow("B free", WINDOW_NORMAL);
	imshow("B free", matB_free);
	waitKey(50);

	namedWindow("B wall", WINDOW_NORMAL);
	imshow("B wall", matB_wall);
	waitKey(50);
	*/

	initMap( transmission, B_shift, B_angle, matB_free, matB_wall );
	/*
	namedWindow("B free rot", WINDOW_NORMAL);
	imshow("B free rot", matB_free);
	waitKey(70);

	namedWindow("B wall rot", WINDOW_NORMAL);
	imshow("B wall rot", matB_wall);
	waitKey(70);
	*/
	B_cells = Mat::ones( matB_free.size(), CV_16SC1)*costmap.unknown;
	vector<Point2f> B_wall_pts, B_free_pts;
	for(int i=0; i<matB_wall.cols; i++){
		for(int j=0; j<matB_wall.rows; j++){
			Point a(i,j);
			if(matB_free.at<uchar>(a) >= 127){
				B_cells.at<short>(a) = costmap.obsFree;
				B_free_pts.push_back(a);
			}
			else if(matB_wall.at<uchar>(a) >= 127){
				B_cells.at<short>(a) = costmap.obsWall;
				B_wall_pts.push_back(a);
			}
		}
	}

	if( !costmap.cells.empty() ){
		if( B_homography.empty() ){
			B_homography = Mat::eye(3, 3, CV_64FC1);

			if(myIndex == 0){
				// agent0 -> agent2
				B_homography.at<double>(0,0) = 0.9996901567421376;
				B_homography.at<double>(0,1) = 0.02489192172118513;
				B_homography.at<double>(0,2) = -0.2878059448341174;
				B_homography.at<double>(1,0) = -0.02489192172118513;
				B_homography.at<double>(1,1) = 0.9996901567421376;
				B_homography.at<double>(1,2) = -5.681602015211894;
				B_homography.at<double>(2,0) = 0;
				B_homography.at<double>(2,1) = 0;
				B_homography.at<double>(2,2) = 1;
			}
			else if(myIndex == 1){
				//agent1 -> agent2
				B_homography.at<double>(0,0) = 0.9992674830825189;
				B_homography.at<double>(0,1) = -0.03827665018194062;
				B_homography.at<double>(0,2) = 8.269605631662136;
				B_homography.at<double>(1,0) = 0.03827665018194062;
				B_homography.at<double>(1,1) = 0.9992674830825189;
				B_homography.at<double>(1,2) = -5.072384234159195;
				B_homography.at<double>(2,0) = 0;
				B_homography.at<double>(2,1) = 0;
				B_homography.at<double>(2,2) = 1;
			}
			else if(myIndex == 2){
				//agent2 -> agent1
				B_homography.at<double>(0,0) = 0.9994431693633792;
				B_homography.at<double>(0,1) = 0.03336954485306531;
				B_homography.at<double>(0,2) = -7.734684314522937;
				B_homography.at<double>(1,0) = -0.03336954485306531;
				B_homography.at<double>(1,1) = 0.9994431693633792;
				B_homography.at<double>(1,2) =  0.8812563454476989;
				B_homography.at<double>(2,0) = 0;
				B_homography.at<double>(2,1) = 0;
				B_homography.at<double>(2,2) = 1;
			}

		}
		double dist = alignCostmap( costmap.cells, B_cells, B_homography);

		Mat rot_wall_temp = Mat(B_wall_pts, CV_32FC1);
		Mat rot_wall_matches = Mat::zeros(rot_wall_temp.size(), CV_32FC1);
		perspectiveTransform(rot_wall_temp, rot_wall_matches, B_homography);
		vector<Point2f> rot_wall_vec = rot_wall_matches;

		Mat rot_free_temp = Mat(B_free_pts, CV_32FC1);
		Mat rot_free_matches = Mat::zeros(rot_wall_temp.size(), CV_32FC1);
		perspectiveTransform(rot_free_temp, rot_free_matches, B_homography);
		vector<Point2f> rot_free_vec = rot_free_matches;

		if( dist < INFINITY ){
			for(size_t i=0; i<rot_wall_vec.size(); i++){
				Point p = rot_wall_vec[i];
				if( costmap.cells.at<short>(p) != costmap.obsFree || costmap.cells.at<short>(p) != costmap.obsWall){
					costmap.cells.at<short>(p) = costmap.obsWall;
				}
			}
			for(size_t i=0; i<rot_free_vec.size(); i++){
				Point p = rot_free_vec[i];

				if( costmap.cells.at<short>(p) != costmap.obsFree || costmap.cells.at<short>(p) != costmap.obsWall){
					costmap.cells.at<short>(p) = costmap.obsFree;
				}
			}
		}

		inference.makeInference( inferenceMethod, costmap );
		costmap.buildCellsPlot();
		costmap.addAgentToCostmapPlot( myColor, myPath, cLoc);
		costmap.addAgentToCostmapPlot( myColor, myPath, gLoc);
		costmap.showCostmapPlot(myIndex);
	}

	Mat dispB = Mat::zeros( matB_free.size(),CV_8UC3 );
	for(int i=0; i<B_cells.cols; i++){
		for(int j=0; j<B_cells.rows; j++){
			Point a(i,j);
			if(B_cells.at<short>(a) == costmap.obsFree){
				dispB.at<Vec3b>(a) = costmap.cObsFree;
			}
			else if(B_cells.at<short>(a) == costmap.obsWall){
				dispB.at<Vec3b>(a) = costmap.cObsWall;
			}
			else{ // anything else, should never happen
				dispB.at<Vec3b>(a) = costmap.cUnknown;
			}
		}
	}

	//cout << "B_homography: " << B_homography << endl;

	namedWindow("mapUpdates B", WINDOW_NORMAL);
	imshow("mapUpdates B", dispB);
	waitKey(70);
}

void Agent::initMap( const std_msgs::Int16MultiArray& transmission, Point shift, float angle, Mat &mfree, Mat &mwall ){

	for(size_t i=0; i<transmission.data.size(); i+=3){
		//x,y, val
		Point p(transmission.data[i]+shift.x, transmission.data[i+1]+shift.y);
		if(p.x > 0 && p.y > 0 && p.x < mwall.cols && p.y < mwall.rows){
			if( transmission.data[i+2] == costmap.obsFree ){
				mfree.at<uchar>(p) = 255;
			}
			else if( transmission.data[i+2] == costmap.obsWall ){
				mwall.at<uchar>(p) = 255;
			}
		}
	}

	Point2f src_center(mwall.cols/2.0F, mwall.rows/2.0F);
	Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
	warpAffine(mwall, mwall, rot_mat, mwall.size());
	warpAffine(mfree, mfree, rot_mat, mfree.size());
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
    marker.header.frame_id = "/map";
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

	cLoc.x = offset.x + 9.6*(locIn.pose.pose.position.y);
	cLoc.y = offset.y + 9.6*(locIn.pose.pose.position.x);

	market.updatecLoc( cLoc );

	if( costmapInitialized && !locationInitialized){
		//plan("marketFrontiers");
		 plan( planningMethod );
	}

	if( costmapInitialized && locationInitialized && marketInitialized && ros::Time::now().toSec() - lastPlan > 5){
		//plan("marketFrontiers");
		plan( planningMethod );
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

	if( !A_homography.empty() && !B_homography.empty() && A_angle != -1 && B_angle != -1){

		market.dissasembleTransmission( transmission);

		if(myIndex == 0){
			rotatePoint( A_shift, A_angle, A_homography, market.cLocs[1], costmap.cells);
			rotatePoint( A_shift, A_angle, A_homography, market.gLocs[1], costmap.cells);
			rotatePoint( B_shift, B_angle, B_homography, market.cLocs[2], costmap.cells);
			rotatePoint( B_shift, B_angle, B_homography, market.gLocs[2], costmap.cells);
		}
		else if(myIndex == 1){
			rotatePoint( A_shift, A_angle, A_homography, market.cLocs[0], costmap.cells);
			rotatePoint( A_shift, A_angle, A_homography, market.gLocs[0], costmap.cells);
			rotatePoint( B_shift, B_angle, B_homography, market.cLocs[2], costmap.cells);
			rotatePoint( B_shift, B_angle, B_homography, market.gLocs[2], costmap.cells);
		}
		else if(myIndex == 2){
			rotatePoint( A_shift, A_angle, A_homography, market.cLocs[0], costmap.cells);
			rotatePoint( A_shift, A_angle, A_homography, market.gLocs[0], costmap.cells);
			rotatePoint( B_shift, B_angle, B_homography, market.cLocs[1], costmap.cells);
			rotatePoint( B_shift, B_angle, B_homography, market.gLocs[1], costmap.cells);
		}
		market.printMarket();
		marketInitialized = true;
	}
}

void Agent::rotatePoint( Point shift, float angle, Mat homography, Point &p, Mat &map){

	p.x += shift.x;
	p.y += shift.y;

	Point2f src_center(map.cols/2.0F, map.rows/2.0F);
	Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
	p.x = p.x*rot_mat.at<double>(0,0) - p.y*rot_mat.at<double>(0,1);
	p.y = p.x*rot_mat.at<double>(1,0) + p.y*rot_mat.at<double>(1,1);

	p.x = p.x*homography.at<double>(0,0) - p.y*homography.at<double>(0,1) + homography.at<double>(0,2);
	p.y = p.x*homography.at<double>(1,0) + p.y*homography.at<double>(1,1) + homography.at<double>(1,2);

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

	inference.makeInference( inferenceMethod, costmap );


	if( locationInitialized ){
		plan( planningMethod );
	}

	costmap.buildCellsPlot();
	costmap.addAgentToCostmapPlot( myColor, myPath, cLoc);
	costmap.addAgentToCostmapPlot( myColor, myPath, gLoc);

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


void Agent::plan( string planMethod ){
	lastPlan = ros::Time::now().toSec();

	if( checkForExplorationFinished() ){
		gLoc = cLoc;
		return;
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
		float e[2] = {0, 1}; // dominated, breaches
		float spread = 0.3; // spread rate
		costmap.getRewardMat(w, e, spread);
		costmap.displayThermalMat( costmap.reward, cLoc );

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
			ROS_WARN("PoseGraph.size() == 0");
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

		if( flag || true){
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
		}
	}
}



Agent::~Agent() {

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
