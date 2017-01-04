/*
 * TreeNode.h
 *
 *  Created on: May 18, 2016
 *      Author: andy
 */

#ifndef TREENODE_H_
#define TREENODE_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <math.h>

using namespace cv;
using namespace std;

using namespace std;

class TreeNode{
public:
	TreeNode(int state, vector<vector<float> > &distGraph, vector<Mat> &observations, vector<int> inPath, int depth, float maxLength);
	virtual ~TreeNode();

	vector<TreeNode> children;
	int myState; // link to physical graph representation
	float reward; // my current reward (changes as children are added)
	float cost; // my path cost
	float value; // reward - cost (changes as children are added)
	int nPulls; // number of times I've been pulled
	vector<int> myPath;
	int myDepth;
	bool searchComplete;
	float pathLength;

	// find children
	void getChildren(vector<vector<float> > &distGraph, vector<Mat> &observations);
	void getUniqueChildren(vector<vector<float> > &distGraph, vector<Mat> &observations);

	// select children algorithms
	int UCBChildSelect();
	int eGreedyChildSelect(float epsilon);
	int greedyChildSelect();
	int simAnnealingChildSelect(float& temp, float cooling);
	int randChildSelect();

	// updating value / reward / cost
	void getNodeReward(vector<Mat> &observations);
	void getNodeCost(vector<vector<float> > &distGraph);
	void updateMyReward();
	void updateMyValue();

	// search / exploit tree
	void searchTree(vector<vector<float> > &distGraph, vector<Mat> &observations, float maxDist, bool visitOnce);
	void exploitTree(vector<int>& myPath, vector<float> &values, vector<float> &rewards, vector<float> &costs);
	void policyRollOut(vector<vector<float> > &distGraph, float maxLength, vector<Mat> &observations); // select greedy and search to end of tree

	void deleteTree();
};

#endif /* TREENODE_H_ */
