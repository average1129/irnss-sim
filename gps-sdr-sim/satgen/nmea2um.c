// nmea2um:
//   Convert NMEA GGA data generated by the free GPS NMEA simulation
//   software from LabSat into the ECEF user motion data for gps-sdr-sim. 
//   http://www.labsat.co.uk/index.php/jp/free-gps-nmea-simulator-software

#define _CRT_SECURE_NO_DEPRECATE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_CHAR 256
#define R2D 57.2957795131

void llh2xyz(double *llh, double *xyz)
{
	double n;
	double a;
	double e;
	double e2;
	double clat;
	double slat;
	double clon;
	double slon;
	double d,nph;
	double tmp;

	a = 6378137.0;
	e = 0.0818191908426;
	e2 = e*e;

	clat = cos(llh[0]);
	slat = sin(llh[0]);
	clon = cos(llh[1]);
	slon = sin(llh[1]);
	d = e*slat;

	n = a/sqrt(1.0-d*d);
	nph = n + llh[2];

	tmp = nph*clat;
	xyz[0] = tmp*clon;
	xyz[1] = tmp*slon;
	xyz[2] = ((1.0-e2)*n + llh[2])*slat;

	return;
}

int main(int argc, char *argv[])
{
	FILE *inp,*outp;
	char str[MAX_CHAR];
	char *token;
	double llh[3],xyz[3],t=0.0;
	char tmp[8];

	if (argc!=3)
	{
		printf("Usage: nmea2um <nmea_gga> <user_motion>\n");
		exit(1);
	}

	if ((inp=fopen(argv[1],"rt"))==NULL)
	{
		printf("Failed to open NMEA file.\n");
		exit(1);
	}

	if ((outp=fopen(argv[2],"wt"))==NULL)
	{
		printf("Failed to open user motion file.\n");
		exit(1);
	}

	while (1)
	{
		if (fgets(str, MAX_CHAR, inp)==NULL)
			break;

		token = strtok(str, ",");

		if (strncmp(token+3, "GGA", 3)==0)
		{
			token = strtok(NULL, ","); // Date and time
			
			token = strtok(NULL, ","); // Latitude
			strncpy(tmp, token, 2);
			tmp[2] = 0;
			
			llh[0] = atof(tmp) + atof(token+2)/60.0;

			token = strtok(NULL, ","); // North or south
			if (token[0]=='S')
				llh[0] *= -1.0;

			llh[0] /= R2D; // in radian
			
			token = strtok(NULL, ","); // Longitude
			strncpy(tmp, token, 3);
			tmp[3] = 0;
			
			llh[1] = atof(tmp) + atof(token+3)/60.0;

			token = strtok(NULL, ","); // East or west
			if (token[0]=='W')
				llh[1] *= -1.0;

			llh[1] /= R2D; // in radian

			token = strtok(NULL, ","); // GPS fix
			token = strtok(NULL, ","); // Number of satellites
			token = strtok(NULL, ","); // HDOP

			token = strtok(NULL, ","); // Altitude above meas sea level
			
			llh[2] = atof(token);

			token = strtok(NULL, ","); // in meter

			token = strtok(NULL, ","); // Geoid height above WGS84 ellipsoid
			
			llh[2] += atof(token);

			// Convert geodetic position into ECEF coordinates
			llh2xyz(llh, xyz);

			// Print out
			fprintf(outp, "%5.1f,%12.3f,%12.3f,%12.3f\n", t, xyz[0], xyz[1], xyz[2]);
			
			t += 0.1; // at 10Hz
		}
	}

	fclose(inp);
	fclose(outp);

	return(0);
}