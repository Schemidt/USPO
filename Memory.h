#ifndef __SHARED__
#define __SHARED__

#include "Uspo.h"

struct pSharedData {
   SOUNDFFT sr;	// ������

};

extern pSharedData *shaData;

char shaInit ( void );
void shaClose( void );

#endif