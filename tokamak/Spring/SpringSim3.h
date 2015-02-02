#pragma once
#include "toksim.h"
#include "TokSpring.h"





class CSpringSim3 :
	public CTokSim
{
public:
	CSpringSim3(void);
	~CSpringSim3(void);

	CTokSpring * spring;

	void Init();
	void KeyDown(unsigned char key);
};

