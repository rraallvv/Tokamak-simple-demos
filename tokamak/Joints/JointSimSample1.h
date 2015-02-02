#pragma once
#include "toksim.h"
#include "TokSpring.h"





class CJointSimSample1 :
	public CTokSim
{
public:
	CJointSimSample1(void);
	~CJointSimSample1(void);
	bool CalculateNextFrame(float dtime);
	void Init();
	void KeyDown(unsigned char key);
};

