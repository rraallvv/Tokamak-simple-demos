#pragma once
#include "toksim.h"
#include "TokSpring.h"





class CJointSim :
	public CTokSim
{
public:
	CJointSim(void);
	~CJointSim(void);
	bool CalculateNextFrame(float dtime);
	void Init();
	void KeyDown(unsigned char key);
};

