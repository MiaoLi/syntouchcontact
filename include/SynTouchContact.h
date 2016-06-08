/*
 * SynTouchContact.h
 *
 *  Created on: Jul 21, 2013
 *      Author: miao
 */

#ifndef SYNTOUCHCONTACT_H_
#define SYNTOUCHCONTACT_H_

#include "ros/ros.h"
#include "MathLib/MathLib.h"
#include "Gaussians.h"
#include "syntouchpublisher/biotac_message.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "syntouchcontact/SynTac.h"
#include "geometry_msgs/WrenchStamped.h"
#include <iostream>
#include <fstream>
#include <vector>

class SynTouchContactNode{

private:

	ros::NodeHandle*   nH;

	ros::Subscriber    SynTouchImpedanceF1;
	ros::Subscriber    SynTouchImpedanceF2;
	ros::Subscriber    SynTouchImpedanceF3;

//	/* publish contact force */
//	ros::Publisher    SynPubContactForceF1;
//	ros::Publisher    SynPubContactForceF2;
//	ros::Publisher    SynPubContactForceF3;
//
//	/* publish contact position */
//	ros::Publisher    SynPubContactPointF1;
//	ros::Publisher    SynPubContactPointF2;
//	ros::Publisher    SynPubContactPointF3;
	/*publish contact position and force at the same topic*/

	ros::Publisher  SynPubContactF1;
	ros::Publisher  SynPubContactF2;
	ros::Publisher  SynPubContactF3;


public:

	MathLib::Vector SynImpF1;
	MathLib::Vector SynImpF2;
	MathLib::Vector SynImpF3;

	MathLib::Vector SynImpF1Cal;               // The baseline vector for calibration
	MathLib::Vector SynImpF2Cal;
	MathLib::Vector SynImpF3Cal;

	MathLib::Vector SynImpF1Var;               // The value range of 19 electrodes on finger 1;
	MathLib::Vector SynImpF2Var;
	MathLib::Vector SynImpF3Var;

	MathLib::Matrix SynElectrodePositions;     // The position lookup table for the 19 electordes


	MathLib::Vector SynContactPositionF1;
	MathLib::Vector SynContactPositionF2;
	MathLib::Vector SynContactPositionF3;


	Gaussians*      GMRContactForceF1;
	Gaussians*      GMRContactForceF2;
	Gaussians*      GMRContactForceF3;

    /*  The contact force at the local contact frame,
	with Y along the finger direction, Z outward of the contact area.*/
	MathLib::Vector SynContactForceF1;
	MathLib::Vector SynContactForceF2;
	MathLib::Vector SynContactForceF3;

	vector<MathLib::Vector> LowPassFilterBufferF1;
	vector<MathLib::Vector> LowPassFilterBufferF2;
	vector<MathLib::Vector> LowPassFilterBufferF3;


public:

	SynTouchContactNode();
	~SynTouchContactNode();
	void VectorPrint(MathLib::Vector);
	void SynTouchGetCallbackF1(const syntouchpublisher::biotac_message::ConstPtr&);

	void SynTouchGetCallbackF2(const syntouchpublisher::biotac_message::ConstPtr&);

	void SynTouchGetCallbackF3(const syntouchpublisher::biotac_message::ConstPtr&);

	void LowPassFilter(MathLib::Vector& inputvec, MathLib::Vector& outputvec,int Fidx);

//	void SynPubContactForce(MathLib::Vector, int idx);
//	void SynPubContactPosition(MathLib::Vector,int idx);
    void SynPubContact(MathLib::Vector, MathLib::Vector, int idx);
	MathLib::Vector SynTouchGetContact(MathLib::Vector&, MathLib::IndicesVector);   //contact position

};



#endif /* SYNTOUCHCONTACT_H_ */
