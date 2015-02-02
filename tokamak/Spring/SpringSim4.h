#pragma once
#include "toksim.h"
#include "TokSpring.h"



class CSpringSim4 :
	public CTokSim
{
public:
	CSpringSim4(void);
	~CSpringSim4(void);

	void Init();
	void KeyDown(unsigned char key);
};

