#include "StdAfx.h"
#include "SpringSim4.h"
#include "TokSpring.h"

#define FLOORSIZE 100.0f

#define N_BODY 30



CSpringSim4::CSpringSim4(void)
{
	CTokSim::CTokSim();
}

CSpringSim4::~CSpringSim4(void)
{
}

void CSpringSim4::Init()
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

	//flat box
	neRigidBody * flatBox = CreateRigidBody();
	flatBox->SetMass(0.05f);
	neV3 size;
	size.Set(20.0f,1.0f,20.0f); 
	flatBox->SetInertiaTensor(neBoxInertiaTensor(size,flatBox->GetMass()));
	geom = flatBox->AddGeometry();
	geom->SetBoxSize(size);
	flatBox->UpdateBoundingInfo();
	pos.Set(0.0f,25.0f,0.0f);
	flatBox->SetPos(pos);

	for (int i=0;i<4;i++)
	{
		neRigidBody * smallBox = CreateRigidBody();
		float mass = 0.025f;
		float radius = 2.0f;
		
		smallBox->SetMass(mass);
		//m_Rigid[m_RigidCount-1]->SetInertiaTensor(neSphereInertiaTensor(radius,mass));
		smallBox->SetInertiaTensor(neBoxInertiaTensor(radius,radius,radius,mass));
		geom = smallBox->AddGeometry();
		//geom->SetSphereDiameter(radius);
		geom->SetBoxSize(radius,radius,radius);
		smallBox->UpdateBoundingInfo();
		
		pos.SetZero();
		if (i%2) pos[0]+=5.0f;
		else pos[0]-=5.0f;
		
		if (i/2) pos[2]+=5.0f;
		else pos[2]-=5.0f;

		pos[1] = -5.0f;

		smallBox->SetPos(pos+flatBox->GetPos());

		CTokSpring * spring = this->CreateSpring(smallBox,flatBox);
		spring->SetDefaultLength(4.0f);

		spring->SetSpringConst(2.0f);

		pos[1] = -0.5f;
		spring->SetConnPos2(pos);
		pos.Set(0.0f,radius/2.0f,0.0f);

		spring->SetConnPos1(pos);
        spring->SetDampingFactor(0.01f);
		neV3 up;up.Set(0.0f,1.0f,0.0f);
		SFixationInfo fixInfo;
		fixInfo.bFixed1 = true;
		fixInfo.bFixed2 = true;
		fixInfo.ConnDir1 = up;
		fixInfo.ConnDir2 = -up;
		
		fixInfo.fTangentDamping = 0.5f;
		fixInfo.fTangentForceFactor = 5.0f;
		fixInfo.fTorqueFactor = 5.0f;
		fixInfo.fAngularDamping = 1.0f;
		spring->FixSpring(fixInfo);
	}
	

}

void CSpringSim4::KeyDown(unsigned char key)
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

