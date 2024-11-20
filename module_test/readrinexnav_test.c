
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#ifdef _WIN32
#include "getopt.h"
#else
#include <unistd.h>
#endif
#include "gpssim.h"

#include<stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>


/*! \brief Convert a UTC date into a GPS date
 *  \param[in] t input date in UTC form
 *  \param[out] g output date in GPS form
 */
void date2gps(const datetime_t *t, gpstime_t *g)
{ // convert gpsstime_t to irnsstime_t -> copy gpstime_t structure 

	int doy[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
	int ye;
	int de;
	int lpdays;

	ye = t->y - 1980; // change start time accordingly 

	// Compute the number of leap days since Jan 5/Jan 6, 1980.
	lpdays = ye/4 + 1;
	if ((ye%4)==0 && t->m<=2)
		lpdays--;

	// Compute the number of days elapsed since Jan 5/Jan 6, 1980.
	de = ye*365 + doy[t->m-1] + t->d + lpdays - 6;

	// Convert time to GPS weeks and seconds.
	g->week = de / 7;
	g->sec = (double)(de%7)*SECONDS_IN_DAY + t->hh*SECONDS_IN_HOUR 
		+ t->mm*SECONDS_IN_MINUTE + t->sec;

	return;
}


int replaceExpDesignator(char *str, int len)
{
	int i,n=0;

	for (i=0; i<len; i++)
	{
		if (str[i]=='D')
		{
			n++;
			str[i] = 'E';
		}
	}
	
	return(n);
}


double subGpsTime(gpstime_t g1, gpstime_t g0)
{
	double dt;

	dt = g1.sec - g0.sec;
	dt += (double)(g1.week - g0.week) * SECONDS_IN_WEEK;

	return(dt);
}

//Rinex Nav read for version 2 

int readRinexNavAllv2(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, const char *fname)
{
	FILE *fp;
	int ieph;
	
	int sv;
	char str[MAX_CHAR];
	char tmp[20];

	datetime_t t; // 
	gpstime_t g;
	gpstime_t g0;
	double dt;

	int flags = 0x0;

	if (NULL==(fp=fopen(fname, "rt")))
		return(-1);

	// Clear valid flag
	for (ieph=0; ieph<EPHEM_ARRAY_SIZE; ieph++)
		for (sv=0; sv<MAX_SAT; sv++)
			eph[ieph][sv].vflg = 0;

	// Read header lines
	while (1)
	{
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		if (strncmp(str+60, "END OF HEADER", 13)==0)
			break;
		else if (strncmp(str+60, "ION ALPHA", 9)==0)
		{
			strncpy(tmp, str+2, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha0 = atof(tmp);

			strncpy(tmp, str+14, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha1 = atof(tmp);

			strncpy(tmp, str+26, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha2 = atof(tmp);

			strncpy(tmp, str+38, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha3 = atof(tmp);

			flags |= 0x1;
		}
		else if (strncmp(str+60, "ION BETA", 8)==0)
		{
			strncpy(tmp, str+2, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta0 = atof(tmp);

			strncpy(tmp, str+14, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta1 = atof(tmp);

			strncpy(tmp, str+26, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta2 = atof(tmp);

			strncpy(tmp, str+38, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta3 = atof(tmp);

			flags |= 0x1<<1;
		}
		else if (strncmp(str+60, "DELTA-UTC", 9)==0)
		{
			strncpy(tmp, str+3, 19);
			tmp[19] = 0;
			replaceExpDesignator(tmp, 19);
			ionoutc->A0 = atof(tmp);

			strncpy(tmp, str+22, 19);
			tmp[19] = 0;
			replaceExpDesignator(tmp, 19);
			ionoutc->A1 = atof(tmp);

			strncpy(tmp, str+41, 9);
			tmp[9] = 0;
			ionoutc->tot = atoi(tmp);

			strncpy(tmp, str+50, 9);
			tmp[9] = 0;
			ionoutc->wnt = atoi(tmp);

			if (ionoutc->tot%4096==0)
				flags |= 0x1<<2;
		}
		else if (strncmp(str+60, "LEAP SECONDS", 12)==0)
		{
			strncpy(tmp, str, 6);
			tmp[6] = 0;
			ionoutc->dtls = atoi(tmp);

			flags |= 0x1<<3;
		}
	}

	ionoutc->vflg = FALSE;
	if (flags==0xF) // Read all Iono/UTC lines
		ionoutc->vflg = TRUE;

	// Read ephemeris blocks
	g0.week = -1;
	ieph = 0;

	while (1)
	{
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		// PRN
		strncpy(tmp, str, 2);
		tmp[2] = 0;
		sv = atoi(tmp)-1;

		// EPOCH
		strncpy(tmp, str+3, 2);
		tmp[2] = 0;
		t.y = atoi(tmp) + 2000;

		strncpy(tmp, str+6, 2);
		tmp[2] = 0;
		t.m = atoi(tmp);

		strncpy(tmp, str+9, 2);
		tmp[2] = 0;
		t.d = atoi(tmp);

		strncpy(tmp, str+12, 2);
		tmp[2] = 0;
		t.hh = atoi(tmp);

		strncpy(tmp, str+15, 2);
		tmp[2] = 0;
		t.mm = atoi(tmp);

		strncpy(tmp, str+18, 4);
		tmp[2] = 0;
		t.sec = atof(tmp);

		date2gps(&t, &g);
		
		if (g0.week==-1)
			g0 = g;

		// Check current time of clock
		dt = subGpsTime(g, g0);
		
		if (dt>SECONDS_IN_HOUR)
		{
			g0 = g;
			ieph++; // a new set of ephemerides

			if (ieph>=EPHEM_ARRAY_SIZE)
				break;
		}

		// Date and time
		eph[ieph][sv].t = t;

		// SV CLK
		eph[ieph][sv].toc = g;

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19); // tmp[15]='E';
		eph[ieph][sv].af0 = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].af1 = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].af2 = atof(tmp);

		// BROADCAST ORBIT - 1
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].iode = (int)atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].crs = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].deltan = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].m0 = atof(tmp);

		// BROADCAST ORBIT - 2
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cuc = atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].ecc = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cus = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].sqrta = atof(tmp);

		// BROADCAST ORBIT - 3
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].toe.sec = atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cic = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].omg0 = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cis = atof(tmp);

		// BROADCAST ORBIT - 4
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].inc0 = atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].crc = atof(tmp);
		
		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].aop = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].omgdot = atof(tmp);

		// BROADCAST ORBIT - 5
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].idot = atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].codeL2 = (int)atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].toe.week = (int)atof(tmp);

		// BROADCAST ORBIT - 6
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].svhlth = (int)atof(tmp);
		if ((eph[ieph][sv].svhlth>0) && (eph[ieph][sv].svhlth<32))
			eph[ieph][sv].svhlth += 32; // Set MSB to 1

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].tgd = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].iodc = (int)atof(tmp);

		// BROADCAST ORBIT - 7
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		// Set valid flag
		eph[ieph][sv].vflg = 1;

		// Update the working variables
		eph[ieph][sv].A = eph[ieph][sv].sqrta * eph[ieph][sv].sqrta;
		eph[ieph][sv].n = sqrt(GM_EARTH/(eph[ieph][sv].A*eph[ieph][sv].A*eph[ieph][sv].A)) + eph[ieph][sv].deltan;
		eph[ieph][sv].sq1e2 = sqrt(1.0 - eph[ieph][sv].ecc*eph[ieph][sv].ecc);
		eph[ieph][sv].omgkdot = eph[ieph][sv].omgdot - OMEGA_EARTH;
	}

	fclose(fp);
	
	if (g0.week>=0)
		ieph += 1; // Number of sets of ephemerides

	return(ieph);
}


//testing modified RINEXNAv read function for v3.3 - IRNSS 
int readRinexNavAllv3_3(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, const char *fname)

{ // make changes for rinex v2 header to v3 header 
    FILE *fp;
	int ieph;
	
	int sv;
	char str[MAX_CHAR];
	char tmp[20];

	datetime_t t;
	gpstime_t g;
	gpstime_t g0;
	double dt;

	int flags = 0x0;

	if (NULL==(fp=fopen(fname, "rt")))
		return(-1);
	printf("Reading Nav File \n");
	// Clear valid flag
	for (ieph=0; ieph<EPHEM_ARRAY_SIZE; ieph++)
		for (sv=0; sv<MAX_SAT; sv++)
			eph[ieph][sv].vflg = 0;
 	printf("Valid Flags reset \n");
	// Read header lines
	while (1)
	{
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		if (strncmp(str+60, "END OF HEADER", 13)==0)
			break;
		// GPS correction term - v2 legacy 
		else if (strncmp(str+60, "ION ALPHA", 9)==0)
		{
			strncpy(tmp, str+2, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha0 = atof(tmp);

			strncpy(tmp, str+14, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha1 = atof(tmp);

			strncpy(tmp, str+26, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha2 = atof(tmp);

			strncpy(tmp, str+38, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha3 = atof(tmp);

			flags |= 0x1;
		}
		// GPS COrrection term - v2 legacy 
		else if (strncmp(str+60, "ION BETA", 8)==0)
		{
			strncpy(tmp, str+2, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta0 = atof(tmp);

			strncpy(tmp, str+14, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta1 = atof(tmp);

			strncpy(tmp, str+26, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta2 = atof(tmp);

			strncpy(tmp, str+38, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta3 = atof(tmp);

			flags |= 0x1<<1;
		}
		//GPS Only legacy term - v2 need to cross verify with v3.3 document 
		else if (strncmp(str+60, "DELTA-UTC", 9)==0)
		{
			strncpy(tmp, str+3, 19);
			tmp[19] = 0;
			replaceExpDesignator(tmp, 19);
			ionoutc->A0 = atof(tmp);

			strncpy(tmp, str+22, 19);
			tmp[19] = 0;
			replaceExpDesignator(tmp, 19);
			ionoutc->A1 = atof(tmp);

			strncpy(tmp, str+41, 9);
			tmp[9] = 0;
			ionoutc->tot = atoi(tmp);

			strncpy(tmp, str+50, 9);
			tmp[9] = 0;
			ionoutc->wnt = atoi(tmp);

			if (ionoutc->tot%4096==0)
				flags |= 0x1<<2;
		}
		else if (strncmp(str+60, "LEAP SECONDS", 12)==0)
		{
			strncpy(tmp, str, 6);
			tmp[6] = 0;
			ionoutc->dtls = atoi(tmp);

			flags |= 0x1<<3;
		}
	}

	ionoutc->vflg = FALSE;
	if (flags==0xF) // Read all Iono/UTC lines
		ionoutc->vflg = TRUE;


	
	g0.week = -1;
	ieph = 0;
		
		// this is going to read ephemeris parameters 
		while (1)
	{
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;
		if (strcmp(str,"I")==0) //that means first index is I
		{// PRN
		strncpy(tmp, str+2, 2);
		tmp[2] = 0;
		sv = atoi(tmp)-1;

		// EPOCH
		strncpy(tmp, str+4, 2);
		tmp[2] = 0;
		t.y = atoi(tmp) ;

		strncpy(tmp, str+10, 2);
		tmp[2] = 0;
		t.m = atoi(tmp);

		strncpy(tmp, str+12, 2);
		tmp[2] = 0;
		t.d = atoi(tmp);

		strncpy(tmp, str+15, 2);
		tmp[2] = 0;
		t.hh = atoi(tmp);

		strncpy(tmp, str+19, 2);
		tmp[2] = 0;
		t.mm = atoi(tmp);

		strncpy(tmp, str+22, 4);

		tmp[2] = 0;
		t.sec = atof(tmp);

		date2gps(&t, &g);
		
		if (g0.week==-1)
			g0 = g;

		// Check current time of clock
		dt = subGpsTime(g, g0);
		
		if (dt>SECONDS_IN_HOUR)
		{
			g0 = g;
			ieph++; // a new set of ephemerides

			if (ieph>=EPHEM_ARRAY_SIZE)
				break;
		}

		// Date and time
		eph[ieph][sv].t = t;

		// SV CLK
		eph[ieph][sv].toc = g;

		strncpy(tmp, str+25, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19); // tmp[15]='E';
		eph[ieph][sv].af0 = atof(tmp);

		strncpy(tmp, str+44, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].af1 = atof(tmp);

		strncpy(tmp, str+63, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].af2 = atof(tmp);

		// BROADCAST ORBIT - 1
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+6, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].iode = (int)atof(tmp);

		strncpy(tmp, str+25, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].crs = atof(tmp);

		strncpy(tmp, str+44, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].deltan = atof(tmp);

		strncpy(tmp, str+63, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].m0 = atof(tmp);
        // BROADCAST ORBIT - 2
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+6, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cuc = atof(tmp);

		strncpy(tmp, str+25, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].ecc = atof(tmp);

		strncpy(tmp, str+44, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cus = atof(tmp);

		strncpy(tmp, str+63, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].sqrta = atof(tmp);

		// BROADCAST ORBIT - 3
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+6, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].toe.sec = atof(tmp);

		strncpy(tmp, str+25, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cic = atof(tmp);

		strncpy(tmp, str+44, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].omg0 = atof(tmp);

		strncpy(tmp, str+63, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cis = atof(tmp);
// BROADCAST ORBIT - 5
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].idot = atof(tmp);

//		strncpy(tmp, str+22, 19);
//		tmp[19] = 0;
//		replaceExpDesignator(tmp, 19);
//		eph[ieph][sv].codeL2 = (int)atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].toe.week = (int)atof(tmp);

		// BROADCAST ORBIT - 6
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].svhlth = (int)atof(tmp);
		if ((eph[ieph][sv].svhlth>0) && (eph[ieph][sv].svhlth<32))
			eph[ieph][sv].svhlth += 32; // Set MSB to 1

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].tgd = atof(tmp);

//		strncpy(tmp, str+60, 19);
//		tmp[19] = 0;
//		replaceExpDesignator(tmp, 19);
//		eph[ieph][sv].iodc = (int)atof(tmp);

		// BROADCAST ORBIT - 7
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		// Set valid flag
		eph[ieph][sv].vflg = 1;

		// Update the working variables
		eph[ieph][sv].A = eph[ieph][sv].sqrta * eph[ieph][sv].sqrta;
		eph[ieph][sv].n = sqrt(GM_EARTH/(eph[ieph][sv].A*eph[ieph][sv].A*eph[ieph][sv].A)) + eph[ieph][sv].deltan;
		eph[ieph][sv].sq1e2 = sqrt(1.0 - eph[ieph][sv].ecc*eph[ieph][sv].ecc);
		eph[ieph][sv].omgkdot = eph[ieph][sv].omgdot - OMEGA_EARTH;
		}}

	fclose(fp);
if (g0.week>=0)
		ieph += 1; // Number of sets of ephemerides

	return(ieph);	


}








/*! \brief Replace all 'E' exponential designators to 'D'
 *  \param str String in which all occurrences of 'E' are replaced with *  'D'
 *  \param len Length of input string in bytes
 *  \returns Number of characters replaced
 */



int main ()
{
    char navfile[100] ;
    ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT]; 
    ionoutc_t ionoutc;
    ionoutc.enable = TRUE;
	strcpy(navfile, "brdc0010.22n");
    //navfile = "rinexabovetop.nav" ; // v3.3
    // navfile = 'brdc0010.22n'; //v 2
    int neph = readRinexNavAllv2(eph, &ionoutc, navfile);
    int ieph; 
    for (ieph=0; ieph<EPHEM_ARRAY_SIZE; ieph++)
    {
        for (int sv=0; sv<MAX_SAT; sv++)
            {
             printf ("eph: ieph[%d] sv[%d] \n ", ieph , sv);
             printf ("valid flag = %d \n", eph[ieph][sv].vflg ) ;   
             printf (" UTC TIME Y = %d \n", eph[ieph][sv].t.y ) ; 
             printf ("UTC TIME M = %d \n ", eph[ieph][sv].t.m ) ; 
             printf ("UTC TIME D = %d \n", eph[ieph][sv].t.d ) ; 
             printf ("UTC TIME H = %d \n", eph[ieph][sv].t.hh ) ; 
             printf ("UTC TIME MIN= %d \n", eph[ieph][sv].t.mm ) ; 
             printf ("UTC TIME SEC= %f \n", eph[ieph][sv].t.sec ) ; 
             printf ("TIME OF CLOCK WEEK = %d \n", eph[ieph][sv].toc.week ) ;  
             printf ("TIME OF CLOCK SEC  = %f \n", eph[ieph][sv].toc.sec ) ;
             printf ("TIME OF EPHEMERIS WEEK = %d \n", eph[ieph][sv].toe.week ) ;  
             printf ("TIME OF EPHEMERIS SEC = %f \n", eph[ieph][sv].toe.sec ) ;
             printf ("iodc = %d \n", eph[ieph][sv].iodc ) ;
             printf ("iode = %d \n", eph[ieph][sv].iode) ;
             printf (" deltan = %f \n", eph[ieph][sv].deltan);
             printf (" cuc = %f \n", eph[ieph][sv].cuc);
             printf (" cus = %f \n", eph[ieph][sv].cus);
             printf (" cic = %f \n", eph[ieph][sv].cic);
             printf (" cis = %f \n", eph[ieph][sv].cis);
             printf (" crc = %f \n", eph[ieph][sv].crc);
             printf (" crs = %f \n", eph[ieph][sv].crs);
             printf (" ecc = %f \n", eph[ieph][sv].ecc);
             printf (" sqrta = %f \n", eph[ieph][sv].sqrta);
             printf (" m0 = %f \n", eph[ieph][sv].m0);
             printf (" omg0 = %f \n", eph[ieph][sv].omg0);
             printf (" inc0 = %f \n", eph[ieph][sv].inc0);
             printf (" aop = %f \n", eph[ieph][sv].aop);
             printf (" omgdot = %f \n", eph[ieph][sv].omgdot);
             printf (" idot = %f \n", eph[ieph][sv].idot);
             printf (" af0 = %f \n", eph[ieph][sv].af0);
             printf (" af1 = %f \n", eph[ieph][sv].af1);
             printf (" af2 = %f \n", eph[ieph][sv].af2);
             printf (" tgd = %f \n", eph[ieph][sv].tgd);
             printf (" svhlth = %d \n", eph[ieph][sv].svhlth);
             printf (" code L2 = %d \n", eph[ieph][sv].codeL2);
             printf ("Working variables follow \n"); 
             printf (" n = %f \n", eph[ieph][sv].n);
             printf (" sq1e2 = %f \n", eph[ieph][sv].sq1e2);
             printf (" A = %f \n", eph[ieph][sv].A);
             printf (" omgkdot = %f \n", eph[ieph][sv].omgkdot);
             //printf ("  = %f \n", eph[ieph][sv].);

            }
    }
return(0);}