#pragma once
#include "toksim.h"
#include "TokSpring.h"





class CSpringSim :
	public CTokSim
{
public:
	CSpringSim(void);
	~CSpringSim(void);
	
	void Init();
	void KeyDown(unsigned char key);
};

