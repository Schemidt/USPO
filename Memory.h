#ifndef __SHARED__
#define __SHARED__

#include "Uspo.h"

struct pSharedData {
   SOUNDFFT sr;	// Данные

};

extern pSharedData *shaData;

char shaInit ( void );
void shaClose( void );

#endif