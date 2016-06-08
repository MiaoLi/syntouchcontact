/*
 * SynTouchContact.cpp
 *
 *  Created on: Jul 21, 2013
 *      Author: miao
 */


#include "SynTouchContact.h"

SynTouchContactNode::SynTouchContactNode() {

	int argc = 0;
	char* argv = NULL;
	ros::init(argc, &argv, "SynTouchNode");

	nH = new ros::NodeHandle;

	if(ros::master::check()){
		cout << "read data from syntouch finger 1" <<endl;
		SynTouchImpedanceF1 = nH->subscribe("/finger1", 10, &SynTouchContactNode::SynTouchGetCallbackF1, this);
	}

	if(ros::master::check()){
		cout << "read data from syntouch finger 2" <<endl;
		SynTouchImpedanceF2 = nH->subscribe("/finger2", 10, &SynTouchContactNode::SynTouchGetCallbackF2, this);
	}

	if(ros::master::check()){
		cout << "read data from syntouch finger 3" <<endl;
		SynTouchImpedanceF3 = nH->subscribe("/finger3", 10, &SynTouchContactNode::SynTouchGetCallbackF3, this);
	}


	if(ros::master::check()){
		SynPubContactF1=nH->advertise<syntouchcontact::SynTac>("/contactF1", 10);
	}
	if(ros::master::check()){
		SynPubContactF2=nH->advertise<syntouchcontact::SynTac>("/contactF2", 10);
	}
	if(ros::master::check()){
		SynPubContactF3=nH->advertise<syntouchcontact::SynTac>("/contactF3", 10);
	}

	/* The value range for 19 electordes of finger 1,2,3. In principle, they should be the same for three fingers,
	 * but they are not*/
	SynImpF1Var.Resize(19);
	SynImpF1Var.Zero();
	SynImpF1Var(0)=1400;
	SynImpF1Var(1)=800;
	SynImpF1Var(2)=900;
	SynImpF1Var(3)=1000;
	SynImpF1Var(4)=1100;
	SynImpF1Var(5)=900;
	SynImpF1Var(6)=1200;
	SynImpF1Var(7)=1000;
	SynImpF1Var(8)=900;
	SynImpF1Var(9)=1000;
	SynImpF1Var(10)=1800;
	SynImpF1Var(11)=1200;
	SynImpF1Var(12)=850;
	SynImpF1Var(13)=1500;
	SynImpF1Var(14)=950;
	SynImpF1Var(15)=1500;
	SynImpF1Var(16)=1000;
	SynImpF1Var(17)=800;
	SynImpF1Var(18)=1200;

	SynImpF2Var.Resize(19);
	SynImpF2Var.Zero();
	SynImpF2Var(0)=1400;
	SynImpF2Var(1)=800;
	SynImpF2Var(2)=900;
	SynImpF2Var(3)=1000;
	SynImpF2Var(4)=1100;
	SynImpF2Var(5)=900;
	SynImpF2Var(6)=1200;
	SynImpF2Var(7)=1000;
	SynImpF2Var(8)=900;
	SynImpF2Var(9)=1000;
	SynImpF2Var(10)=1800;
	SynImpF2Var(11)=1200;
	SynImpF2Var(12)=850;
	SynImpF2Var(13)=1500;
	SynImpF2Var(14)=950;
	SynImpF2Var(15)=1500;
	SynImpF2Var(16)=1000;
	SynImpF2Var(17)=800;
	SynImpF2Var(18)=1200;


	SynImpF3Var.Resize(19);
	SynImpF3Var.Zero();
	SynImpF3Var(0)=1400;
	SynImpF3Var(1)=800;
	SynImpF3Var(2)=900;
	SynImpF3Var(3)=1000;
	SynImpF3Var(4)=1100;
	SynImpF3Var(5)=900;
	SynImpF3Var(6)=1200;
	SynImpF3Var(7)=1000;
	SynImpF3Var(8)=900;
	SynImpF3Var(9)=1000;
	SynImpF3Var(10)=1800;
	SynImpF3Var(11)=1200;
	SynImpF3Var(12)=850;
	SynImpF3Var(13)=1500;
	SynImpF3Var(14)=950;
	SynImpF3Var(15)=1500;
	SynImpF3Var(16)=1000;
	SynImpF3Var(17)=800;
	SynImpF3Var(18)=1200;

	/*The positions for the 19 electrodes in the sensor coordinates*/
	SynElectrodePositions.Resize(3,19);
	SynElectrodePositions.Zero();
	SynElectrodePositions.SetColumn(MathLib::Vector3(-7.5,20.42,0),0);
	SynElectrodePositions.SetColumn(MathLib::Vector3(-4.24,16.72,6.18),1);
	SynElectrodePositions.SetColumn(MathLib::Vector3(-4.24,13.22,6.18),2);
	SynElectrodePositions.SetColumn(MathLib::Vector3(-7.5,11.42,0),3);
	SynElectrodePositions.SetColumn(MathLib::Vector3(-4.24,8.92,6.18),4);
	SynElectrodePositions.SetColumn(MathLib::Vector3(-7.5,6.02,0),5);
	SynElectrodePositions.SetColumn(MathLib::Vector3(0,25.62,1.5),6);   //
	SynElectrodePositions.SetColumn(MathLib::Vector3(-4,23.62,4.5),7); //
	SynElectrodePositions.SetColumn(MathLib::Vector3(4,23.62,4.5),8);  //
	SynElectrodePositions.SetColumn(MathLib::Vector3(0,21.62,7.5),9);   //
	SynElectrodePositions.SetColumn(MathLib::Vector3(7.5,20.42,0),10);
	SynElectrodePositions.SetColumn(MathLib::Vector3(4.24,16.72,6.18),11);
	SynElectrodePositions.SetColumn(MathLib::Vector3(4.24,13.22,6.18),12);
	SynElectrodePositions.SetColumn(MathLib::Vector3(7.5,11.42,0),13);
	SynElectrodePositions.SetColumn(MathLib::Vector3(4.24,8.92,6.18),14);
	SynElectrodePositions.SetColumn(MathLib::Vector3(7.5,6.02,0),15);
	SynElectrodePositions.SetColumn(MathLib::Vector3(0,16.62,7.5),16);
	SynElectrodePositions.SetColumn(MathLib::Vector3(0,9.62,7.5),17);
	SynElectrodePositions.SetColumn(MathLib::Vector3(0,5.82,7.5),18);

	GMRContactForceF1 = new Gaussians(21,22,"./data/GMM_Model_4/F1_mu.txt","./data/GMM_Model_4/F1_sigma.txt","./data/GMM_Model_4/F1_prio.txt");
	GMRContactForceF1->InitFastGMR(0,18,19,21);
	GMRContactForceF2 = new Gaussians(21,22,"./data/GMM_Model_4/F2_mu.txt","./data/GMM_Model_4/F2_sigma.txt","./data/GMM_Model_4/F2_prio.txt");
	GMRContactForceF2->InitFastGMR(0,18,19,21);
	GMRContactForceF3 = new Gaussians(21,22,"./data/GMM_Model_4/F3_mu.txt","./data/GMM_Model_4/F3_sigma.txt","./data/GMM_Model_4/F3_prio.txt");
	GMRContactForceF3->InitFastGMR(0,18,19,21);

	SynContactForceF1.Resize(3);
	SynContactForceF1.Zero();
	SynContactForceF2.Resize(3);
	SynContactForceF2.Zero();
	SynContactForceF3.Resize(3);
	SynContactForceF3.Zero();


	LowPassFilterBufferF1.resize(10);
	LowPassFilterBufferF2.resize(10);
	LowPassFilterBufferF3.resize(10);

	for(int i=0;i<10;i++){

		LowPassFilterBufferF1[i]=SynImpF1Cal;
		LowPassFilterBufferF2[i]=SynImpF2Cal;
		LowPassFilterBufferF3[i]=SynImpF3Cal;
	}

}

SynTouchContactNode::~SynTouchContactNode() {

	delete nH;
	delete GMRContactForceF1;
	delete GMRContactForceF2;
	delete GMRContactForceF3;
}
void SynTouchContactNode::SynTouchGetCallbackF1(const syntouchpublisher::biotac_message::ConstPtr& SynPression){

	for(int i=0;i<19;i++){
		SynImpF1(i)=SynPression->E[i];
	}
}
void SynTouchContactNode::SynTouchGetCallbackF2(const syntouchpublisher::biotac_message::ConstPtr& SynPression){

	for(int i=0;i<19;i++){
		SynImpF2(i)=SynPression->E[i];
	}

}

void SynTouchContactNode::SynTouchGetCallbackF3(const syntouchpublisher::biotac_message::ConstPtr& SynPression){

	for(int i=0;i<19;i++){
		SynImpF3(i)=SynPression->E[i];
	}

}
void SynTouchContactNode::LowPassFilter(MathLib::Vector& inputvec, MathLib::Vector& outputvec,int Fidx){
	MathLib::Vector Ftmp;
	Ftmp.Resize(19);
	Ftmp.Zero();
	switch(Fidx){
	case 1:
		for(int i=0;i<9;i++){
			LowPassFilterBufferF1[i]=LowPassFilterBufferF1[i+1];
		}

		LowPassFilterBufferF1[9]=inputvec;

		for(int i=0; i<10; i++){

			Ftmp=Ftmp+LowPassFilterBufferF1[i];

		}
		outputvec=Ftmp/10;
		break;
	case 2:
		for(int i=0;i<9;i++){
			LowPassFilterBufferF2[i]=LowPassFilterBufferF2[i+1];
		}

		LowPassFilterBufferF2[9]=inputvec;
		for(int i=0; i<10; i++){

			Ftmp=Ftmp+LowPassFilterBufferF2[i];

		}
		outputvec=Ftmp/10;
		break;
	case 3:
		for(int i=0;i<9;i++){
			LowPassFilterBufferF3[i]=LowPassFilterBufferF3[i+1];
		}

		LowPassFilterBufferF3[9]=inputvec;
		for(int i=0; i<10; i++){

			Ftmp=Ftmp+LowPassFilterBufferF3[i];

		}
		outputvec=Ftmp/10;
		break;
	default: break;
	}

}

MathLib::Vector SynTouchContactNode::SynTouchGetContact(MathLib::Vector& SynImpCoeff,MathLib::IndicesVector idx){

	// Given the absolute change coefficients on 19 electrodes, computer the contact information;

	if(SynImpCoeff.Max()==0){

		return MathLib::Vector3(0, 0, 0);
	}else{
		//    cout<<"contact position is :";
		//    VectorPrint(SynElectrodePositions.Mult(SynImpCoeff));
		//    cout<<endl;
		//    SynImpCoeff.Sort(&idx);
		//    VectorPrint(SynImpCoeff);
		//    cout<<idx.at(0)+1<<" "<<idx.at(1)+1<<" "<<idx.at(2)+1<<" "<<idx.at(3)+1<<" "<<idx.at(4)+1<<endl;
		//
		return SynElectrodePositions.Mult(SynImpCoeff);
	}
}
void SynTouchContactNode::VectorPrint(MathLib::Vector vec){

	for(int i=0;i<int(vec.Size());i++){
		cout<<vec(i)<<" ";
	}
}


void SynTouchContactNode::SynPubContact(MathLib::Vector pos, MathLib::Vector force, int idx){
	syntouchcontact::SynTac contact;

	contact.x=pos(0);
	contact.y=pos(1);
	contact.z=pos(2);
	contact.fx=force(0);
	contact.fy=force(1);
	contact.fz=force(2);

	switch(idx){
	case 1:
		SynPubContactF1.publish(contact);
		break;
	case 2:
		SynPubContactF2.publish(contact);
		break;
	case 3:
		SynPubContactF3.publish(contact);
		break;
	default:
		break;
	}


}



int main(){

	SynTouchContactNode* SynTac;
	SynTac=new SynTouchContactNode;

	SynTac->SynImpF1.Resize(19);
	SynTac->SynImpF2.Resize(19);
	SynTac->SynImpF3.Resize(19);

	bool bCalibration = true;
	bool bGetForce    = false;
	bool bGetPosition = false;

	MathLib::Vector tmp1;
	MathLib::Vector tmp2;
	MathLib::Vector tmp3;
	tmp1.Resize(19);
	tmp2.Resize(19);
	tmp3.Resize(19);

	MathLib::IndicesVector tmpidx(19);

	ros::Rate r(100);

	for(int i=0; i<100;i++){
		if(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
	}

	while(ros::ok()){

		ros::spinOnce();

		if(bCalibration){

			if(SynTac->SynImpF1(0)==0){
				cout<<"waiting for SynTouch to calibrate F1 or connection lose"<<endl;
			}

			if(SynTac->SynImpF2(0)==0){
				cout<<"waiting for SynTouch to calibrate F2 or connection lose"<<endl;
			}
			if(SynTac->SynImpF3(0)==0){
				cout<<"waiting for SynTouch to calibrate F3 or connection lose"<<endl;
			}

			// This line should be changed to include F3
			if(SynTac->SynImpF1(0)!=0&&SynTac->SynImpF2(0)!=0&&SynTac->SynImpF3(0)!=0){

				for(int i=0;i<100;i++){

					tmp1=tmp1+SynTac->SynImpF1;

					tmp2=tmp2+SynTac->SynImpF2;

					tmp3=tmp3+SynTac->SynImpF3;

					ros::spinOnce();
					r.sleep();
				}
				cout<<"finish the calibration"<<endl;

				SynTac->SynImpF1Cal=tmp1/100;
				SynTac->SynImpF2Cal=tmp2/100;
				SynTac->SynImpF3Cal=tmp3/100;

				bCalibration=false;
				bGetPosition=true;
				bGetForce=true;
			}
		}

		if(bGetForce){
			SynTac->LowPassFilter(SynTac->SynImpF1,SynTac->SynImpF1,1);
			tmp1=SynTac->SynImpF1/100-SynTac->SynImpF1Cal/100;

			SynTac->LowPassFilter(SynTac->SynImpF2,SynTac->SynImpF2,2);
			tmp2=SynTac->SynImpF2/100-SynTac->SynImpF2Cal/100;

			SynTac->LowPassFilter(SynTac->SynImpF3,SynTac->SynImpF3,3);
			tmp3=SynTac->SynImpF3/100-SynTac->SynImpF3Cal/100;

			if(tmp1.Norm()<1){
				SynTac->SynContactForceF1.Zero();
			}else{
				SynTac->GMRContactForceF1->Regression(tmp1,SynTac->SynContactForceF1);

				if(isnan(SynTac->SynContactForceF1(0))){

					cout<<"Contact Force INVALID"<<endl;
					SynTac->SynContactForceF1.Zero();

				}

			}
			if(tmp2.Norm()<1){
				SynTac->SynContactForceF2.Zero();
			}else{
				SynTac->GMRContactForceF2->Regression(tmp2,SynTac->SynContactForceF2);
				if(isnan(SynTac->SynContactForceF2(0))){
					cout<<"Contact Force INVALID"<<endl;
					SynTac->SynContactForceF2.Zero();
				}

			}

			if(tmp3.Norm()<1){
				SynTac->SynContactForceF3.Zero();
			}else{

				SynTac->GMRContactForceF3->Regression(tmp3,SynTac->SynContactForceF3);
				if(isnan(SynTac->SynContactForceF3(0))){
					cout<<"Contact Force INVALID"<<endl;
					SynTac->SynContactForceF3.Zero();
				}

			}

		}

		if(bGetPosition)
		{
			tmp1=(SynTac->SynImpF1-SynTac->SynImpF1Cal).Trunc(-2000,0);

			for(int i=0;i<19;i++){
				tmp1(i)=tmp1(i)/SynTac->SynImpF1Var(i);
				tmp1(i)=1-exp(-2*tmp1(i)*tmp1(i));

			}
			if(tmp1.Max()>0.001){

				tmp1=tmp1/tmp1.Sum();

			}else {
				tmp1.Zero();
			}

			SynTac->SynContactPositionF1=SynTac->SynTouchGetContact(tmp1, tmpidx);
			cout<<"contact position F1: ";

			if(SynTac->SynContactPositionF1.Norm()==0){
				cout<<"NO CONTACT";         //can use for detect if there is a contact;
			}else{
				SynTac->VectorPrint(SynTac->SynContactPositionF1);

			}
			cout<<endl;
			//			SynTac->SynPubContactPosition(SynTac->SynContactPositionF1,1);


			tmp2=(SynTac->SynImpF2-SynTac->SynImpF2Cal).Trunc(-2000,0);
			for(int i=0;i<19;i++){
				tmp2(i)=tmp2(i)/SynTac->SynImpF2Var(i);
				tmp2(i)=1-exp(-2*tmp2(i)*tmp2(i));
			}
			if(tmp2.Max()>0.001){

				tmp2=tmp2/tmp2.Sum();

			}else {
				tmp2.Zero();
			}

			SynTac->SynContactPositionF2=SynTac->SynTouchGetContact(tmp2, tmpidx);
			cout<<"contact position F2: ";
			if(SynTac->SynContactPositionF2.Norm()==0){
				cout<<"NO CONTACT";
			}else{
				SynTac->VectorPrint(SynTac->SynContactPositionF2);

			}
			cout<<endl;
			//			SynTac->SynPubContactPosition(SynTac->SynContactPositionF2,1);


			tmp3=(SynTac->SynImpF3-SynTac->SynImpF3Cal).Trunc(-2000,0);
			for(int i=0;i<19;i++){
				tmp3(i)=tmp3(i)/SynTac->SynImpF3Var(i);
				tmp3(i)=1-exp(-2*tmp3(i)*tmp3(i));
			}
			if(tmp3.Max()>0.001){

				tmp3=tmp3/tmp3.Sum();

			}else {
				tmp3.Zero();
			}

			SynTac->SynContactPositionF3=SynTac->SynTouchGetContact(tmp3, tmpidx);
			cout<<"contact position F3: ";
			if(SynTac->SynContactPositionF3.Norm()==0){
				cout<<"NO CONTACT";
			}else{

				SynTac->VectorPrint(SynTac->SynContactPositionF3);

			}
			cout<<endl;
			//SynTac->SynPubContactPosition(SynTac->SynContactPositionF3,1);
			SynTac->SynPubContact(SynTac->SynContactPositionF1,SynTac->SynContactForceF1,1);
			SynTac->SynPubContact(SynTac->SynContactPositionF2,SynTac->SynContactForceF2,2);
			SynTac->SynPubContact(SynTac->SynContactPositionF3,SynTac->SynContactForceF3,3);

		}

		r.sleep();
	}
	return 0;
}


