#ifndef _CSONAR_H
#define _CSONAR_H
#include <string.h>
#include "drv_include.h"

class CSonar
{

public:
	CSonar();
	~CSonar();
	void Init(void);
	float update(void);
	
	volatile float output;
};


#endif

