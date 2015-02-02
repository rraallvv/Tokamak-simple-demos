#include "StdAfx.h"
#include "SpringSim2.h"
#include "TokSpring.h"

#define FLOORSIZE 100.0f

#define N_BODY 30



CSpringSim2::CSpringSim2(void)
{
	CTokSim::CTokSim();
}

CSpringSim2::~CSpringSim2(void)
{
}

void CSpringSim2::Init()
{

	neGeometry *geom;	
    neV3 boxSize1;		
    neV3 gravity;		
    neV3 pos;			
	gravity.Set(0.0f, -9.81f, 0.0f);
	


	//Initialize the physics engine:
	CTokSim::Init(200,10,gravity);

	// Create an animated body for the floor
	neAnimatedBody * floor = CreateAnimatedBody();
	geom = floor->AddGeometry();
	boxSize1.Set(FLOORSIZE, 0.5f, FLOORSIZE);
	geom->SetBoxSize(boxSize1[0],boxSize1[1],boxSize1[2]);
	floor->UpdateBoundingInfo();
	pos.Set(0.0f, -20.0f, 0.0f);
	floor->SetPos(pos);



	//Create an animated box "hanging in the air"	
	neAnimatedBody * animBox = CreateAnimatedBody();
	geom = animBox->AddGeometry();
	geom->SetBoxSize(5.0f,0.5f,5.0f);
	animBox->UpdateBoundingInfo();
	pos.Set(0.0f,8.0f,0.0f);
	animBox->SetPos(pos);


	//create a small box:
	neRigidBody * smallBox =CreateRigidBody();
	smallBox->SetMass(0.0125f);
	smallBox->SetInertiaTensor(neBoxInertiaTensor(1.0f,1.0f,1.0f,0.0125f));
	geom = smallBox->AddGeometry();
	geom->SetBoxSize(1.0f,1.0f,1.0f);
	smallBox->UpdateBoundingInfo();
	pos.Set(8.0f,3.0f,0.0f);
	smallBox->SetPos(pos);

	//Create a spring connecting the smallbox with the fixed box:
	CTokSpring * spring = this->CreateSpring(smallBox,animBox);
	//set the default length, spring const, damping factor:
	spring->SetDefaultLength(5.0f);
	spring->SetSpringConst(1.0/40.0f);		
	spring->SetDampingFactor(0.0001f);
	//Set the Connection Position at the small box
	//(relative to the body's center)
	//(the connection position of the animBox remains at 0,0,0)
	neV3 ConnPos;
	ConnPos.Set(0.5f,0.5f,0.0f);
	spring->SetConnPos1(ConnPos);
	//Fix the spring (because we want to set its connectino direction at the two bodies)
	neV3 SpringDir;
	SpringDir.Set(1.0f,1.0f,0.0f);
	SFixationInfo fixInfo;
	fixInfo.bFixed1 = true;
	fixInfo.bFixed2 = true;
	fixInfo.ConnDir1 = SpringDir;
	fixInfo.ConnDir2 = -SpringDir;
	
	//This values set "how strong" the spring keeps the body fixed
	fixInfo.fTangentDamping = 0.5f;
	fixInfo.fTangentForceFactor = 5.0f;
	fixInfo.fTorqueFactor = 5.0f;
	fixInfo.fAngularDamping = 0.5f;
	spring->FixSpring(fixInfo);


}

void CSpringSim2::KeyDown(unsigned char key)
{
 if (key == 'p')
	{
		while (neRigidBody * rigid = CreateRigidBody())
		{
			rigid->SetMass(0.001f);
			neGeometry * geom = rigid->AddGeometry();
			geom->SetSphereDiameter(1.5f);
			neV3 pos;
			pos.Set((rand()/(float)RAND_MAX -0.5f)* 35.0f,20.0f+rand()/(float)RAND_MAX * 100.0f,(rand()/(float)RAND_MAX -0.5f)* 15.0f);
			rigid->SetPos(pos);
		}
	}
	if (key == 'i')
	{
		neV3 imp,pos;
		imp.Set(-1.0f,0.0f,0.0f);
		pos.Set(m_Rigid[0]->GetPos());
		m_Rigid[0]->ApplyImpulse(imp,pos);
	}
	if (key == 'o')
	{
		neV3 imp,pos;
		imp.Set(1.0f,0.0f,0.0f);
		pos.Set(m_Rigid[0]->GetPos());
		m_Rigid[0]->ApplyImpulse(imp,pos);
	}

	if (key == 'k')
	{
		neV3 imp,pos;
		imp.Set(0.0f,5.0f,0.0f);
		pos.Set(m_Rigid[0]->GetPos());
		m_Rigid[0]->ApplyImpulse(imp,pos);
	}
}

