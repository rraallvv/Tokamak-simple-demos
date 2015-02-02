#include "StdAfx.h"
#include "jointsim.h"
#include "TokSim.h"
#include "TokSpring.h"

#define FLOORSIZE 100.0f

#define N_BODY 30

#define EPSILON 0.1f


CJointSim::CJointSim(void)
{
	CTokSim::CTokSim();
}

CJointSim::~CJointSim(void)
{
}

void CJointSim::Init()
{

	neGeometry *geom;	
    neV3 boxSize1;		
    neV3 gravity;		
    neV3 pos;			
	gravity.Set(0.0f, -9.81f, 0.0f);

	CTokSim::Init(100,10,gravity);


	//create the floor:
	neAnimatedBody * floor = CreateAnimatedBody();
	geom = floor->AddGeometry();
	boxSize1.Set(FLOORSIZE, 0.5f, FLOORSIZE);
	geom->SetBoxSize(boxSize1[0],boxSize1[1],boxSize1[2]);
	floor->UpdateBoundingInfo();
	pos.Set(0.0f, -3.0f, 0.0f);
	floor->SetPos(pos);

	//create another animated (=fixed) object - a cylinder which will be placed standing on the floor:
	neAnimatedBody * animCyl = CreateAnimatedBody();
	geom = animCyl->AddGeometry();
	geom->SetCylinder(10.0f,5.0f);
	pos.Set(0.0f,0.0f,3.0f);
	animCyl->SetPos(pos);
	animCyl->UpdateBoundingInfo();
	

	//Create two flat boxes next to each other:
	neRigidBody * box1 = CreateRigidBody();
	geom = box1->AddGeometry();
	geom->SetBoxSize(10.0f,1.0f,10.0f);
	box1->UpdateBoundingInfo();
	box1->SetMass(0.05f);
	pos.Set(0.0f,15.0f,0.0f);
	box1->SetPos(pos);
	
	neRigidBody * box2 = CreateRigidBody();
	geom = box2->AddGeometry();
	geom->SetBoxSize(10.0f,1.0f,10.0f);
	box2->UpdateBoundingInfo();
	box2->SetMass(0.05f);
	pos.Set(0.0f,15.0f,10.0f);
	box2->SetPos(pos);
	
	//Create a joint (hinge) between these two boxes:
	neJoint * joint;
	joint = m_Sim->CreateJoint(box1,box2);

	neT3 trans;
	trans.pos.Set(0.0f,15.0f,5.0f);;
	trans.rot[0].Set(0.0f, 0.0f, 1.0f);
	trans.rot[1].Set(1.0f, 0.0f, 0.0f);
	trans.rot[2].Set(0.0f, 1.0f, 0.0f);
	joint->SetJointFrameWorld(trans);

	joint->SetType(neJoint::NE_JOINT_HINGE);
	joint->SetLowerLimit(-NE_PI*0.9f);
	joint->SetUpperLimit(NE_PI*0.9f);
	joint->EnableLimit(true);	
	joint->SetJointLength(10.0f);

	joint->Enable(true);

	box1->CollideConnected(true);
	box2->CollideConnected(true);


	//create a fixed sphere "hanging in the air"
	neAnimatedBody * animSphere = CreateAnimatedBody();
	geom = animSphere->AddGeometry();
	geom->SetSphereDiameter(4.0f);
	animSphere->UpdateBoundingInfo();
	pos.Set(0.0f,30.0f,0.0f);
	animSphere->SetPos(pos);
	

	//Create a box "hanging" on the animated sphere
	neRigidBody * rigidSphere = CreateRigidBody();
	geom = rigidSphere->AddGeometry();
	geom->SetBoxSize(16.0f,1.0f,1.0f);
	rigidSphere->UpdateBoundingInfo();
	pos.Set(10.0f,30.0f,0.0f);
	rigidSphere->SetPos(pos);

	//Create the joint (ball and socket) between the spheres:
	joint = m_Sim->CreateJoint(rigidSphere,animSphere);
	trans.pos.Set(animSphere->GetPos());
	trans.rot.SetIdentity();
	joint->SetJointFrameWorld(trans);
	joint->SetType(neJoint::NE_JOINT_BALLSOCKET);
	joint->SetDampingFactor(20.0f);
	
	joint->Enable(true);


}

void CJointSim::KeyDown(unsigned char key)
{
}

bool CJointSim::CalculateNextFrame(float dtime)
{


	CTokSim::CalculateNextFrame(dtime);


	return true;
}
