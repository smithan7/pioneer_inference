/*
 * Market.cpp
 *
 *  Created on: Sep 27, 2016
 *      Author: andy
 */

#include "Market.h"

void printVector(vector<int> v);
void printVector(vector<Point> v);
void printVector(vector<float> v);

Market::Market(){}

void Market::init(int nAgents, int myIndex, bool enableRelaySacrifice) {

	for(int i=0; i<nAgents+1; i++){
		Point a(0,0);

		// explore stuff
		cLocs.push_back(a);
		gLocs.push_back(a);
		cRaw.push_back(a);
		gRaw.push_back(a);
		exploreCosts.push_back(0);

		// report stuff
		reportTimes.push_back(1);
		reportCosts.push_back(0);
		reportRequests.push_back(0);

		// relay / sacrifice stuff
		rLocs.push_back(a);
		roles.push_back(-1); // -1 = n/a, 0 = sacrifice, 1 = relay
		mates.push_back(-1); // -1 = n/a, # = index of corresponding relay sacrifice

		// general stuff
		times.push_back(0); // how long since I have heard from each person
	}

	this->myIndex = myIndex;
	this->nAgents = nAgents;
}

bool Market::comCheck(int a){
	if(times[a] - ros::Time::now().toSec() <= 1.5 && a != myIndex){ // am I in contact with them currently?
		return true;
	}
	return false;
}

std_msgs::Float32MultiArray Market::assembleTransmission(){
	// turn the 1d vector into standard market

	std_msgs::Float32MultiArray transmission;
	for(int i=0; i<nAgents+1; i++){ // +1 is to account for observer
		transmission.data.push_back( cRaw[i].x ); // 0
		transmission.data.push_back( cRaw[i].y); // 1
		transmission.data.push_back( gRaw[i].x); // 2
		transmission.data.push_back( gRaw[i].y); // 3
		transmission.data.push_back( exploreCosts[i]); // 4

		transmission.data.push_back( reportTimes[i]); // 5
		transmission.data.push_back( reportCosts[i]); // 6
		transmission.data.push_back( reportRequests[i]); // 7

		transmission.data.push_back( rLocs[i].x); // 8
		transmission.data.push_back( rLocs[i].y); // 9
		transmission.data.push_back( roles[i]); // 10
		transmission.data.push_back( mates[i]); // 11

		transmission.data.push_back( times[i] ); // 12
	}

	return transmission;
}

void Market::dissasembleTransmission(const std_msgs::Float32MultiArray& transmission){

	// turn the 1d vector into a standard market
	int transmissionLength = 13;
	for(int i=0; i<nAgents+1; i++){ // +1 is to account for observer
		if( transmission.data[i*transmissionLength+12] > times[i] && i != myIndex ){ // only update market if the info is new and not me
			cLocs[i].x = transmission.data[i*transmissionLength];
			cLocs[i].y = transmission.data[i*transmissionLength+1];

			cRaw[i].x = transmission.data[i*transmissionLength];
			cRaw[i].y = transmission.data[i*transmissionLength+1];

			gLocs[i].x = transmission.data[i*transmissionLength+2];
			gLocs[i].y = transmission.data[i*transmissionLength+3];

			gRaw[i].x = transmission.data[i*transmissionLength+2];
			gRaw[i].y = transmission.data[i*transmissionLength+3];

			exploreCosts[i] = transmission.data[i*transmissionLength+4];

			reportTimes[i] = transmission.data[i*transmissionLength+5];
			reportCosts[i] = transmission.data[i*transmissionLength+6];
			reportRequests[i] = transmission.data[i*transmissionLength+7];

			rLocs[i].x = transmission.data[i*transmissionLength+8];
			rLocs[i].y = transmission.data[i*transmissionLength+9];
			roles[i] = transmission.data[i*transmissionLength+10];
			mates[i] = transmission.data[i*transmissionLength+11];

			times[i] = transmission.data[i*transmissionLength+12]+1; // add 1 for my hop
		}

		if( i == myIndex){ // if it is me check return and relay / sacrfice
			reportRequests[i] = transmission.data[i*transmissionLength+7]; // check for requests

			if(transmission.data[i*transmissionLength+10]){ // check if someone has made me a relay / sacrifice
				rLocs[i].x = transmission.data[i*transmissionLength+8];
				rLocs[i].y = transmission.data[i*transmissionLength+9];
				roles[i] = transmission.data[i*transmissionLength+10];
				mates[i] = transmission.data[i*transmissionLength+11];
			}
		}
	}
}

void Market::updatecLoc( Point cLoc ){
	cLocs[myIndex] = cLoc;
	times[myIndex] = ros::Time::now().toSec();
}

void Market::updategLoc( Point gLoc ){
	gLocs[myIndex] = gLoc;
}

void Market::updateTime( float t ){
	times[myIndex] = t;
}
void Market::updateExploreCost( float cost ){
	exploreCosts[myIndex] = cost;
}

void Market::printMarket(){

	cout << "cLocs: ";
	printVector(cLocs);

	cout << "gLocs: ";
	printVector( gLocs );

	cout << "exploreCosts: ";
	printVector(exploreCosts);

	cout << "reportTimes: ";
	printVector(reportTimes);

	cout << "reportCosts: ";
	printVector(reportCosts);

	cout << "reportRequests: ";
	printVector(reportRequests);

	cout << "rLocs: ";
	printVector( rLocs );

	cout << "roles: ";
	printVector(roles);

	cout << "mates: ";
	printVector(mates);

	cout << "times: ";
	printVector(times);

}

void printVector(vector<int> v){
	for( size_t i=0; i<v.size(); i++){
		cout << v[i];
		if(i+1 < v.size() ){
			cout << ", ";
		}
	}
	cout << endl;
}

void printVector(vector<Point> v){
	for( size_t i=0; i<v.size(); i++){
		cout << v[i];
		if(i+1 < v.size() ){
			cout << ", ";
		}
	}
	cout << endl;
}

void printVector(vector<float> v){
	for( size_t i=0; i<v.size(); i++){
		cout << v[i];
		if(i+1 < v.size() ){
			cout << ", ";
		}
	}
	cout << endl;
}

Market::~Market() {}