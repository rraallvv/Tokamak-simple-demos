#include "StdAfx.h"
#include "JointSimSample1.h"
#include "TokSim.h"
#include "TokSpring.h"

#define FLOORSIZE 100.0f

#define N_BODY 30

#define EPSILON 0.1f


CJointSimSample1::CJointSimSample1(void)
{
	CTokSim::CTokSim();
}

CJointSimSample1::~CJointSimSample1(void)
{
}

void CJointSimSample1::Init()
{

	neGeometry *geom;	
    neV3 boxSize1;		
    neV3 gravity;		
    neV3 pos;			
	gravity.Set(0.0f, -9.81f, 0.0f);

	CTokSim::Init(100,10,gravity);

	//***
	//Copied and slightly modified from Sample1
	

	f32 linkLength = 1.2f;

	neJoint * lastJoint = NULL;

	neRigidBody * lastbox = NULL;

	for (s32 j = 0; j < N_BODY; j++)
	{
		f32 mass = 0.1f;

		neRigidBody * rigidBody = CreateRigidBody();

		rigidBody->CollideConnected(true);
	
		neGeometry * geom = rigidBody->AddGeometry();

		neV3 boxSize; 
		
		boxSize.Set(1.2f, 0.5f, 0.5f);

		geom->SetBoxSize(boxSize[0],boxSize[1],boxSize[2]);

		neV3 tensorSize;
		tensorSize = boxSize;;
		rigidBody->UpdateBoundingInfo();
		
		rigidBody->SetInertiaTensor(neBoxInertiaTensor(tensorSize, mass));
		rigidBody->SetMass(mass);

		neV3 pos;
		pos.Set(-linkLength * (j+1), 0.0f, 0.0f);
		rigidBody->SetPos(pos);

		neJoint * joint = NULL;
		
		neT3 jointFrame;

		jointFrame.SetIdentity();

		if (j != 0)
		{
			joint = m_Sim->CreateJoint(rigidBody, lastbox);

			jointFrame.pos.Set(-linkLength * (0.5f + j), 0.0f, 0.0f);

			joint->SetJointFrameWorld(jointFrame);
		}
		if (j == N_BODY - 1)
		{
			lastJoint = joint;
		}

		if (joint)
		{
			joint->SetType(neJoint::NE_JOINT_BALLSOCKET);

			joint->Enable(true);
		}

		lastbox = rigidBody;
	}
	if (lastJoint)
	{
		lastJoint->SetEpsilon(EPSILON);

		lastJoint->SetIteration(5);
	}
   

	// Create an animated body for the floor
    neAnimatedBody * floor = CreateAnimatedBody();
    // Add geometry to the floor and set it to be a box with size as defined by the FLOORSIZE constant
    geom = floor->AddGeometry();
    boxSize1.Set(FLOORSIZE, 0.5f, FLOORSIZE);
	
    geom->SetBoxSize(boxSize1[0],boxSize1[1],boxSize1[2]);
    floor->UpdateBoundingInfo();
    // Set the position of the box within the simulator
    pos.Set(0.0f, 0.0f, 0.0f);
	floor->SetPos(pos);


	
}

void CJointSimSample1::KeyDown(unsigned char key)
{
	switch (key) 
	{
	case 'i':
		neV3 vel;
		vel.Set (0.0f,20.0f,0.0f);
		m_Rigid[0]->SetVelocity(vel);

	}
}

bool CJointSimSample1::CalculateNextFrame(float dtime)
{


	CTokSim::CalculateNextFrame(dtime);


	return true;
}
