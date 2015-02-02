#include "StdAfx.h"
#include "SpringSim3.h"
#include "TokSpring.h"

#define FLOORSIZE 100.0f

#define N_BODY 30



CSpringSim3::CSpringSim3(void)
{
	CTokSim::CTokSim();
}

CSpringSim3::~CSpringSim3(void)
{
}

void CSpringSim3::Init()
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


	//flat box 1
	CreateRigidBody();
	m_Rigid[m_RigidCount-1]->SetMass(0.125f);
	neV3 size;
	size.Set(20.0f,1.0f,20.0f); 
	m_Rigid[m_RigidCount-1]->SetInertiaTensor(neBoxInertiaTensor(size,m_Rigid[m_RigidCount-1]->GetMass()));
	
	geom = m_Rigid[m_RigidCount-1]->AddGeometry();
	geom->SetBoxSize(size);

	m_Rigid[m_RigidCount-1]->UpdateBoundingInfo();
	pos.Set(0.0f,25.0f,0.0f);
	m_Rigid[m_RigidCount-1]->SetPos(pos);

	
	//flat box 2
	CreateRigidBody();
	m_Rigid[m_RigidCount-1]->SetMass(0.125f);

	size.Set(20.0f,1.0f,20.0f); 
	m_Rigid[m_RigidCount-1]->SetInertiaTensor(neBoxInertiaTensor(size,m_Rigid[m_RigidCount-1]->GetMass()));
	
	geom = m_Rigid[m_RigidCount-1]->AddGeometry();
	geom->SetBoxSize(size);

	m_Rigid[m_RigidCount-1]->UpdateBoundingInfo();
	pos.Set(0.0f,10.0f,0.0f);
	m_Rigid[m_RigidCount-1]->SetPos(pos);

	spring = this->CreateSpring(m_Rigid[m_RigidCount-1],m_Rigid[m_RigidCount-2]);
	spring->SetDefaultLength(15.0f);

	spring->SetSpringConst(1.0f);
		
	pos.Set(0.0f,-0.5f,0.0f);
	spring->SetConnPos2(pos);
	pos[1] = 0.5f;
	spring->SetConnPos1(pos);
	spring->SetDampingFactor(0.01f);
	neV3 up;up.Set(0.0f,1.0f,0.0f);
	SFixationInfo fixInfo;
	fixInfo.bFixed1 = true;
	fixInfo.bFixed2 = true;
	fixInfo.ConnDir1 = up;
	fixInfo.ConnDir2 = -up;
	
	fixInfo.fTangentDamping = 1.5f;
	fixInfo.fTangentForceFactor = 15.0f;
	fixInfo.fTorqueFactor = 500.0f;
	fixInfo.fAngularDamping = 10.0f;
	spring->FixSpring(fixInfo);

}

void CSpringSim3::KeyDown(unsigned char key)
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

