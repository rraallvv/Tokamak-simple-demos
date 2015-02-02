#pragma once
#include "toksim.h"
#include "TokSpring.h"



class CSpringSim5 :
	public CTokSim
{
public:
	CSpringSim5(void);
	~CSpringSim5(void);

	void Init();
	void KeyDown(unsigned char key);
};

