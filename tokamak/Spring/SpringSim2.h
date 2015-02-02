#pragma once
#include "toksim.h"
#include "TokSpring.h"





class CSpringSim2 :
	public CTokSim
{
public:
	CSpringSim2(void);
	~CSpringSim2(void);

	void Init();
	void KeyDown(unsigned char key);
};

