#define _CRT_SECURE_NO_DEPRECATE

#include <stdio.h>
#include <stdint.h>
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
#define CONSTRAINT_LENGTH 7
#define NUM_POLYNOMIALS 2
const uint32_t CRC24Q_POLY = 0x1864CFB;
const uint32_t CRC24Q_INIT = 0x0;

int sinTable512[] = {
	   2,   5,   8,  11,  14,  17,  20,  23,  26,  29,  32,  35,  38,  41,  44,  47,
	  50,  53,  56,  59,  62,  65,  68,  71,  74,  77,  80,  83,  86,  89,  91,  94,
	  97, 100, 103, 105, 108, 111, 114, 116, 119, 122, 125, 127, 130, 132, 135, 138,
	 140, 143, 145, 148, 150, 153, 155, 157, 160, 162, 164, 167, 169, 171, 173, 176,
	 178, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 205, 207,
	 209, 210, 212, 214, 215, 217, 218, 220, 221, 223, 224, 225, 227, 228, 229, 230,
	 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 241, 242, 243, 244, 244, 245,
	 245, 246, 247, 247, 248, 248, 248, 249, 249, 249, 249, 250, 250, 250, 250, 250,
	 250, 250, 250, 250, 250, 249, 249, 249, 249, 248, 248, 248, 247, 247, 246, 245,
	 245, 244, 244, 243, 242, 241, 241, 240, 239, 238, 237, 236, 235, 234, 233, 232,
	 230, 229, 228, 227, 225, 224, 223, 221, 220, 218, 217, 215, 214, 212, 210, 209,
	 207, 205, 204, 202, 200, 198, 196, 194, 192, 190, 188, 186, 184, 182, 180, 178,
	 176, 173, 171, 169, 167, 164, 162, 160, 157, 155, 153, 150, 148, 145, 143, 140,
	 138, 135, 132, 130, 127, 125, 122, 119, 116, 114, 111, 108, 105, 103, 100,  97,
	  94,  91,  89,  86,  83,  80,  77,  74,  71,  68,  65,  62,  59,  56,  53,  50,
	  47,  44,  41,  38,  35,  32,  29,  26,  23,  20,  17,  14,  11,   8,   5,   2,
	  -2,  -5,  -8, -11, -14, -17, -20, -23, -26, -29, -32, -35, -38, -41, -44, -47,
	 -50, -53, -56, -59, -62, -65, -68, -71, -74, -77, -80, -83, -86, -89, -91, -94,
	 -97,-100,-103,-105,-108,-111,-114,-116,-119,-122,-125,-127,-130,-132,-135,-138,
	-140,-143,-145,-148,-150,-153,-155,-157,-160,-162,-164,-167,-169,-171,-173,-176,
	-178,-180,-182,-184,-186,-188,-190,-192,-194,-196,-198,-200,-202,-204,-205,-207,
	-209,-210,-212,-214,-215,-217,-218,-220,-221,-223,-224,-225,-227,-228,-229,-230,
	-232,-233,-234,-235,-236,-237,-238,-239,-240,-241,-241,-242,-243,-244,-244,-245,
	-245,-246,-247,-247,-248,-248,-248,-249,-249,-249,-249,-250,-250,-250,-250,-250,
	-250,-250,-250,-250,-250,-249,-249,-249,-249,-248,-248,-248,-247,-247,-246,-245,
	-245,-244,-244,-243,-242,-241,-241,-240,-239,-238,-237,-236,-235,-234,-233,-232,
	-230,-229,-228,-227,-225,-224,-223,-221,-220,-218,-217,-215,-214,-212,-210,-209,
	-207,-205,-204,-202,-200,-198,-196,-194,-192,-190,-188,-186,-184,-182,-180,-178,
	-176,-173,-171,-169,-167,-164,-162,-160,-157,-155,-153,-150,-148,-145,-143,-140,
	-138,-135,-132,-130,-127,-125,-122,-119,-116,-114,-111,-108,-105,-103,-100, -97,
	 -94, -91, -89, -86, -83, -80, -77, -74, -71, -68, -65, -62, -59, -56, -53, -50,
	 -47, -44, -41, -38, -35, -32, -29, -26, -23, -20, -17, -14, -11,  -8,  -5,  -2
};

int cosTable512[] = {
	 250, 250, 250, 250, 250, 249, 249, 249, 249, 248, 248, 248, 247, 247, 246, 245,
	 245, 244, 244, 243, 242, 241, 241, 240, 239, 238, 237, 236, 235, 234, 233, 232,
	 230, 229, 228, 227, 225, 224, 223, 221, 220, 218, 217, 215, 214, 212, 210, 209,
	 207, 205, 204, 202, 200, 198, 196, 194, 192, 190, 188, 186, 184, 182, 180, 178,
	 176, 173, 171, 169, 167, 164, 162, 160, 157, 155, 153, 150, 148, 145, 143, 140,
	 138, 135, 132, 130, 127, 125, 122, 119, 116, 114, 111, 108, 105, 103, 100,  97,
	  94,  91,  89,  86,  83,  80,  77,  74,  71,  68,  65,  62,  59,  56,  53,  50,
	  47,  44,  41,  38,  35,  32,  29,  26,  23,  20,  17,  14,  11,   8,   5,   2,
	  -2,  -5,  -8, -11, -14, -17, -20, -23, -26, -29, -32, -35, -38, -41, -44, -47,
	 -50, -53, -56, -59, -62, -65, -68, -71, -74, -77, -80, -83, -86, -89, -91, -94,
	 -97,-100,-103,-105,-108,-111,-114,-116,-119,-122,-125,-127,-130,-132,-135,-138,
	-140,-143,-145,-148,-150,-153,-155,-157,-160,-162,-164,-167,-169,-171,-173,-176,
	-178,-180,-182,-184,-186,-188,-190,-192,-194,-196,-198,-200,-202,-204,-205,-207,
	-209,-210,-212,-214,-215,-217,-218,-220,-221,-223,-224,-225,-227,-228,-229,-230,
	-232,-233,-234,-235,-236,-237,-238,-239,-240,-241,-241,-242,-243,-244,-244,-245,
	-245,-246,-247,-247,-248,-248,-248,-249,-249,-249,-249,-250,-250,-250,-250,-250,
	-250,-250,-250,-250,-250,-249,-249,-249,-249,-248,-248,-248,-247,-247,-246,-245,
	-245,-244,-244,-243,-242,-241,-241,-240,-239,-238,-237,-236,-235,-234,-233,-232,
	-230,-229,-228,-227,-225,-224,-223,-221,-220,-218,-217,-215,-214,-212,-210,-209,
	-207,-205,-204,-202,-200,-198,-196,-194,-192,-190,-188,-186,-184,-182,-180,-178,
	-176,-173,-171,-169,-167,-164,-162,-160,-157,-155,-153,-150,-148,-145,-143,-140,
	-138,-135,-132,-130,-127,-125,-122,-119,-116,-114,-111,-108,-105,-103,-100, -97,
	 -94, -91, -89, -86, -83, -80, -77, -74, -71, -68, -65, -62, -59, -56, -53, -50,
	 -47, -44, -41, -38, -35, -32, -29, -26, -23, -20, -17, -14, -11,  -8,  -5,  -2,
	   2,   5,   8,  11,  14,  17,  20,  23,  26,  29,  32,  35,  38,  41,  44,  47,
	  50,  53,  56,  59,  62,  65,  68,  71,  74,  77,  80,  83,  86,  89,  91,  94,
	  97, 100, 103, 105, 108, 111, 114, 116, 119, 122, 125, 127, 130, 132, 135, 138,
	 140, 143, 145, 148, 150, 153, 155, 157, 160, 162, 164, 167, 169, 171, 173, 176,
	 178, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 205, 207,
	 209, 210, 212, 214, 215, 217, 218, 220, 221, 223, 224, 225, 227, 228, 229, 230,
	 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 241, 242, 243, 244, 244, 245,
	 245, 246, 247, 247, 248, 248, 248, 249, 249, 249, 249, 250, 250, 250, 250, 250
};
// Receiver antenna attenuation in dB for boresight angle = 0:5:180 [deg]
double ant_pat_db[37] = {
    0.00, 0.00, 0.22, 0.44, 0.67, 1.11, 1.56, 2.00, 2.44, 2.89, 3.56, 4.22,
    4.89, 5.56, 6.22, 6.89, 7.56, 8.22, 8.89, 9.78, 10.67, 11.56, 12.44, 13.33,
    14.44, 15.56, 16.67, 17.78, 18.89, 20.00, 21.33, 22.67, 24.00, 25.56, 27.33, 29.33,
    31.56};

int allocatedSat[MAX_SAT] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

/*! \brief Subtract two vectors of double
 *  \param[out] y Result of subtraction
 *  \param[in] x1 Minuend of subtraction
 *  \param[in] x2 Subtrahend of subtraction
 */
void subVect(double *y, const double *x1, const double *x2)
{
    y[0] = x1[0] - x2[0];
    y[1] = x1[1] - x2[1];
    y[2] = x1[2] - x2[2];

    return;
}

/*! \brief Compute Norm of Vector
 *  \param[in] x Input vector
 *  \returns Length (Norm) of the input vector
 */
double normVect(const double *x)
{
    return (sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]));
}

/*! \brief Compute dot-product of two vectors
 *  \param[in] x1 First multiplicand
 *  \param[in] x2 Second multiplicand
 *  \returns Dot-product of both multiplicands
 */
double dotProd(const double *x1, const double *x2)
{
    return (x1[0] * x2[0] + x1[1] * x2[1] + x1[2] * x2[2]);
}
/* !\brief generate the C/A code sequence for a given Satellite Vehicle PRN
 *  \param[in] prn PRN number of the Satellite Vehicle
 *  \param[out] ca Caller-allocated integer array of 1023 bytes
 */
//void codegen(int *ca, int prn)
//{
//    int delay[] = {
//        5, 6, 7, 8, 17, 18, 139, 140, 141, 251,
//        252, 254, 255, 256, 257, 258, 469, 470, 471, 472,
//        473, 474, 509, 512, 513, 514, 515, 516, 859, 860,
//        861, 862};
//
//    int g1[CA_SEQ_LEN], g2[CA_SEQ_LEN];
//    int r1[N_DWRD_SBF], r2[N_DWRD_SBF];
//    int c1, c2;
//    int i, j;
//
//    if (prn < 1 || prn > 32)
//        return;
//
//    for (i = 0; i < N_DWRD_SBF; i++)
//        r1[i] = r2[i] = -1;
//
//    for (i = 0; i < CA_SEQ_LEN; i++)
//    {
//        g1[i] = r1[9];
//        g2[i] = r2[9];
//        c1 = r1[2] * r1[9];
//        c2 = r2[1] * r2[2] * r2[5] * r2[7] * r2[8] * r2[9];
//
//        for (j = 9; j > 0; j--)
//        {
//            r1[j] = r1[j - 1];
//            r2[j] = r2[j - 1];
//        }
//        r1[0] = c1;
//        r2[0] = c2;
//    }
//    for (i = 0, j = CA_SEQ_LEN - delay[prn - 1]; i < CA_SEQ_LEN; i++, j++)
//        ca[i] = (1 - g1[i] * g2[j % CA_SEQ_LEN]) / 2;
//
//    return;
//}

void codegen(int *ca, int prn)
{
    // first 10 chips in Octal 2
    // int delay[] = { 0130,   1731,   0713,   1215,  0117 ,  1624, 1753, 1317, 1547, 0233,
    //	1663, 0203, 0455, 1025}; // not entirely sure about how the delay matrix is used to generate prn

    int generator1[10], generator2[10], generator2initial[10][10];
    int generator1op[1023], generator2op[1023];
    int generator1feedback, generator2feedback;
    int i, j;

    // for (int i = 0; i < 10; i++)
    // {
    //    generator2[i] = -1; // or 1 lets see
    //    generator1[i] = 1;
    // }
    // PRN ID 1
    generator2initial[0][0] = 1;
    generator2initial[0][1] = 1;
    generator2initial[0][2] = 1;
    generator2initial[0][3] = -1;
    generator2initial[0][4] = 1;
    generator2initial[0][5] = -1;
    generator2initial[0][6] = -1;
    generator2initial[0][7] = 1;
    generator2initial[0][8] = 1;
    generator2initial[0][9] = 1;

    // PRN ID 2
    generator2initial[1][0] = -1;
    generator2initial[1][1] = -1;
    generator2initial[1][2] = -1;
    generator2initial[1][3] = -1;
    generator2initial[1][4] = 1;
    generator2initial[1][5] = -1;
    generator2initial[1][6] = -1;
    generator2initial[1][7] = 1;
    generator2initial[1][8] = 1;
    generator2initial[1][9] = -1;

    // PRN ID 3
    generator2initial[2][0] = 1;
    generator2initial[2][1] = -1;
    generator2initial[2][2] = -1;
    generator2initial[2][3] = -1;
    generator2initial[2][4] = 1;
    generator2initial[2][5] = 1;
    generator2initial[2][6] = -1;
    generator2initial[2][7] = 1;
    generator2initial[2][8] = -1;
    generator2initial[2][9] = -1;

    // PRN ID 4
    generator2initial[3][0] = -1;
    generator2initial[3][1] = 1;
    generator2initial[3][2] = -1;
    generator2initial[3][3] = 1;
    generator2initial[3][4] = 1;
    generator2initial[3][5] = 1;
    generator2initial[3][6] = -1;
    generator2initial[3][7] = -1;
    generator2initial[3][8] = 1;
    generator2initial[3][9] = -1;

    // PRN ID 5
    generator2initial[4][0] = 1;
    generator2initial[4][-1] = 1;
    generator2initial[4][2] = 1;
    generator2initial[4][3] = -1;
    generator2initial[4][4] = 1;
    generator2initial[4][5] = 1;
    generator2initial[4][6] = -1;
    generator2initial[4][7] = -1;
    generator2initial[4][8] = -1;
    generator2initial[4][9] = -1;

    // PRN ID 6
    generator2initial[5][0] = -1;
    generator2initial[5][1] = -1;
    generator2initial[5][2] = -1;
    generator2initial[5][3] = 1;
    generator2initial[5][4] = 1;
    generator2initial[5][5] = -1;
    generator2initial[5][6] = 1;
    generator2initial[5][7] = -1;
    generator2initial[5][8] = 1;
    generator2initial[5][9] = 1;

    // PRN ID 7
    generator2initial[6][0] = -1;
    generator2initial[6][1] = -1;
    generator2initial[6][2] = -1;
    generator2initial[6][3] = -1;
    generator2initial[6][4] = -1;
    generator2initial[6][5] = 1;
    generator2initial[6][6] = -1;
    generator2initial[6][7] = 1;
    generator2initial[6][8] = -1;
    generator2initial[6][9] = -1;

    // PRN ID 8
    generator2initial[7][0] = -1;
    generator2initial[7][1] = 1;
    generator2initial[7][2] = -1;
    generator2initial[7][3] = -1;
    generator2initial[7][4] = 1;
    generator2initial[7][5] = 1;
    generator2initial[7][6] = -1;
    generator2initial[7][7] = -1;
    generator2initial[7][8] = -1;
    generator2initial[7][9] = -1;

    // PRN ID 9
    generator2initial[8][0] = -1;
    generator2initial[8][1] = -1;
    generator2initial[8][2] = 1;
    generator2initial[8][3] = -1;
    generator2initial[8][4] = -1;
    generator2initial[8][5] = 1;
    generator2initial[8][6] = 1;
    generator2initial[8][7] = -1;
    generator2initial[8][8] = -1;
    generator2initial[8][9] = -1;

    // PRN ID 10
    generator2initial[9][0] = 1;
    generator2initial[9][1] = 1;
    generator2initial[9][2] = -1;
    generator2initial[9][3] = 1;
    generator2initial[9][4] = 1;
    generator2initial[9][5] = -1;
    generator2initial[9][6] = -1;
    generator2initial[9][7] = 1;
    generator2initial[9][8] = -1;
    generator2initial[9][9] = -1;

    if (prn < 1 || prn > 14)
        return;

    for (i = 0; i < 10; i++)
    {
        generator2[i] = generator2initial[prn - 1][i];
        generator1[i] = 1;
    }

    //    printf("Gen2 bits \n");
    //	int o;
    //    for (o = 0; o < 10; o++)
    //    {
    //        printf("%d ", generator2[o]);
    //    }
    //    printf("\n");
    //
    //    printf("Gen1 bits \n");
    //    for (o = 0; o < 10; o++)
    //    {
    //        printf("%d ", generator1[o]);
    //    }
    //    printf("\n");

    for (i = 0; i < CA_SEQ_LEN; i++)
    {

        generator1op[i] = generator1[9]; // generator1[2]*generator1[9];
        generator2op[i] = generator2[9]; // generator2[1]*generator2[2]*generator2[5]*generator2[7]*generator2[8]*generator2[9];
        generator1feedback = generator1[2] * generator1[9];
        generator2feedback = generator2[1] * generator2[2] * generator2[5] * generator2[7] * generator2[8] * generator2[9];

        for (j = 9; j > 0; j--)
        {
            generator1[j] = generator1[j - 1];
            generator2[j] = generator2[j - 1];
        }
        generator1[0] = generator1feedback;
        generator2[0] = generator2feedback;
    }

    //    printf("1024 iterations completed:  1 Full Code length completed \n");
    //    printf("Generator 1 OP \n");
    //
    //    for (i = 0; i < CA_SEQ_LEN; i++)
    //    {
    //        printf("%d", generator1op[i]);
    //    }
    //    printf("\n Generator 2 op \n");
    //
    //    for (i = 0; i < CA_SEQ_LEN; i++)
    //    {
    //        printf("%d", generator2op[i]);
    //    }

    //    printf("\n\n\n");
    for (i = 0; i < CA_SEQ_LEN; i++)
        ca[i] = (1 - generator1op[i] * generator2op[i]) / 2;
    //  ca[i] = (1 - generator1op[i] ^ generator2op[i]) / 2;
    return;
}

void date2gps(const datetime_t *t, gpstime_t *g)
{ // convert gpsstime_t to irnsstime_t -> copy gpstime_t structure

    int doy[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    int ye;
    int de;
    int lpdays;

    ye = t->y - 1999; // change 1980 to 1999

    // Compute the number of leap days since Aug 21/22, 1999.
    lpdays = ye / 4 + 1;
    if ((ye % 4) == 0 && t->m <= 2)
        lpdays--;
    //    printf("date2irnss : calculated number of leap days since Aug 21/22 1999:  %d \n", lpdays);
    // Compute the number of days elapsed since Aug 21/AUg 22, 1999.
    de = ye * 365 + doy[t->m - 1] + t->d + lpdays - 234;

    // Convert time to  weeks and seconds.
    g->week = de / 7;
    g->sec = (double)(de % 7) * SECONDS_IN_DAY + t->hh * SECONDS_IN_HOUR + t->mm * SECONDS_IN_MINUTE + t->sec;

    return;
}

void gps2date(const gpstime_t *g, datetime_t *t)
{
    int c = (int)(7 * g->week + floor(g->sec / 86400.0) + 2451179.5) + 1537;
    int d = (int)((c - 122.1) / 365.25);
    int e = 365 * d + d / 4;
    int f = (int)((c - e) / 30.6001);

    t->d = c - e - (int)(30.6001 * f);
    t->m = f - 1 - 12 * (f / 14);
    t->y = d - 4715 - ((7 + t->m) / 10) + 19;

    t->hh = ((int)(g->sec / 3600.0)) % 24;
    t->mm = ((int)(g->sec / 60.0)) % 60;
    t->sec = g->sec - 60.0 * floor(g->sec / 60.0);
}
/*! \brief Convert Earth-centered Earth-fixed (ECEF) into Lat/Long/Height
 *  \param[in] xyz Input Array of X, Y and Z ECEF coordinates
 *  \param[out] llh Output Array of Latitude, Longitude and Height
 */
void xyz2llh(const double *xyz, double *llh)
{
    double a, eps, e, e2;
    double x, y, z;
    double rho2, dz, zdz, nh, slat, n, dz_new;

    a = WGS84_RADIUS;
    e = WGS84_ECCENTRICITY;

    eps = 1.0e-3;
    e2 = e * e;

    if (normVect(xyz) < eps)
    {
        // Invalid ECEF vector
        llh[0] = 0.0;
        llh[1] = 0.0;
        llh[2] = -a;

        return;
    }

    x = xyz[0];
    y = xyz[1];
    z = xyz[2];

    rho2 = x * x + y * y;
    dz = e2 * z;

    while (1)
    {
        zdz = z + dz;
        nh = sqrt(rho2 + zdz * zdz);
        slat = zdz / nh;
        n = a / sqrt(1.0 - e2 * slat * slat);
        dz_new = n * e2 * slat;

        if (fabs(dz - dz_new) < eps)
            break;

        dz = dz_new;
    }

    llh[0] = atan2(zdz, sqrt(rho2));
    llh[1] = atan2(y, x);
    llh[2] = nh - n;

    return;
}
/*! \brief Convert Lat/Long/Height into Earth-centered Earth-fixed (ECEF)
 *  \param[in] llh Input Array of Latitude, Longitude and Height
 *  \param[out] xyz Output Array of X, Y and Z ECEF coordinates
 */
void llh2xyz(const double *llh, double *xyz)
{
    double n;
    double a;
    double e;
    double e2;
    double clat;
    double slat;
    double clon;
    double slon;
    double d, nph;
    double tmp;

    a = WGS84_RADIUS;
    e = WGS84_ECCENTRICITY;
    e2 = e * e;

    clat = cos(llh[0]);
    slat = sin(llh[0]);
    clon = cos(llh[1]);
    slon = sin(llh[1]);
    d = e * slat;

    n = a / sqrt(1.0 - d * d);
    nph = n + llh[2];

    tmp = nph * clat;
    xyz[0] = tmp * clon;
    xyz[1] = tmp * slon;
    xyz[2] = ((1.0 - e2) * n + llh[2]) * slat;

    return;
}

/*! \brief Compute the intermediate matrix for LLH to ECEF
 *  \param[in] llh Input position in Latitude-Longitude-Height format
 *  \param[out] t Three-by-Three output matrix
 */
void ltcmat(const double *llh, double t[3][3])
{
    double slat, clat;
    double slon, clon;

    slat = sin(llh[0]);
    clat = cos(llh[0]);
    slon = sin(llh[1]);
    clon = cos(llh[1]);

    t[0][0] = -slat * clon;
    t[0][1] = -slat * slon;
    t[0][2] = clat;
    t[1][0] = -slon;
    t[1][1] = clon;
    t[1][2] = 0.0;
    t[2][0] = clat * clon;
    t[2][1] = clat * slon;
    t[2][2] = slat;

    return;
}

/*! \brief Convert Earth-centered Earth-Fixed to ?
 *  \param[in] xyz Input position as vector in ECEF format
 *  \param[in] t Intermediate matrix computed by \ref ltcmat
 *  \param[out] neu Output position as North-East-Up format
 */
void ecef2neu(const double *xyz, double t[3][3], double *neu)
{
    neu[0] = t[0][0] * xyz[0] + t[0][1] * xyz[1] + t[0][2] * xyz[2];
    neu[1] = t[1][0] * xyz[0] + t[1][1] * xyz[1] + t[1][2] * xyz[2];
    neu[2] = t[2][0] * xyz[0] + t[2][1] * xyz[1] + t[2][2] * xyz[2];

    return;
}
/*! \brief Convert North-East-Up to Azimuth + Elevation
 *  \param[in] neu Input position in North-East-Up format
 *  \param[out] azel Output array of azimuth + elevation as double
 */
void neu2azel(double *azel, const double *neu)
{
    double ne;

    azel[0] = atan2(neu[1], neu[0]);
    if (azel[0] < 0.0)
        azel[0] += (2.0 * PI);

    ne = sqrt(neu[0] * neu[0] + neu[1] * neu[1]);
    azel[1] = atan2(neu[2], ne);
//    if (azel[1] < 0.0)
//    {
//        azel[1] = -azel[1];
//    }

    return;
}
// \brief Compute Satellite position, velocity and clock at given time
// 										   *  \param[in] eph Ephemeris data of the satellite
// 											   *  \param[in] g GPS time at which position is to be computed
// 												   *  \param[out] pos Computed
// 												   position(vector) *  \param[out] vel Computed velocity(vector) *  \param[clk] clk Computed clock
// 													   * /
void satpos(ephem_t eph, gpstime_t g, double *pos, double *vel, double *clk)
{
    // Computing Satellite Velocity using the Broadcast Ephemeris
    // http://www.ngs.noaa.gov/gps-toolbox/bc_velo.htm

    double tk;
    double mk;
    double ek;
    double ekold;
    double ekdot;
    double cek, sek;
    double pk;
    double pkdot;
    double c2pk, s2pk;
    double uk;
    double ukdot;
    double cuk, suk;
    double ok;
    double sok, cok;
    double ik;
    double ikdot;
    double sik, cik;
    double rk;
    double rkdot;
    double xpk, ypk;
    double xpkdot, ypkdot;

    double relativistic, OneMinusecosE, tmp;

    tk = g.sec - eph.toe.sec;

    if (tk > SECONDS_IN_HALF_WEEK)
        tk -= SECONDS_IN_WEEK;
    else if (tk < -SECONDS_IN_HALF_WEEK)
        tk += SECONDS_IN_WEEK;

    mk = eph.m0 + eph.n * tk;
    ek = mk;
    ekold = ek + 1.0;

    OneMinusecosE = 0; // Suppress the uninitialized warning.
    while (fabs(ek - ekold) > 1.0E-14)
    {
        ekold = ek;
        OneMinusecosE = 1.0 - eph.ecc * cos(ekold);
        ek = ek + (mk - ekold + eph.ecc * sin(ekold)) / OneMinusecosE;
    }

    sek = sin(ek);
    cek = cos(ek);

    ekdot = eph.n / OneMinusecosE;

    relativistic = -4.442807633E-10 * eph.ecc * eph.sqrta * sek;

    pk = atan2(eph.sq1e2 * sek, cek - eph.ecc) + eph.aop;
    pkdot = eph.sq1e2 * ekdot / OneMinusecosE;

    s2pk = sin(2.0 * pk);
    c2pk = cos(2.0 * pk);

    uk = pk + eph.cus * s2pk + eph.cuc * c2pk;
    suk = sin(uk);
    cuk = cos(uk);
    ukdot = pkdot * (1.0 + 2.0 * (eph.cus * c2pk - eph.cuc * s2pk));

    rk = eph.A * OneMinusecosE + eph.crc * c2pk + eph.crs * s2pk;
    rkdot = eph.A * eph.ecc * sek * ekdot + 2.0 * pkdot * (eph.crs * c2pk - eph.crc * s2pk);

    ik = eph.i0 + eph.idot * tk + eph.cic * c2pk + eph.cis * s2pk;
    sik = sin(ik);
    cik = cos(ik);
    ikdot = eph.idot + 2.0 * pkdot * (eph.cis * c2pk - eph.cic * s2pk);

    xpk = rk * cuk;
    ypk = rk * suk;
    xpkdot = rkdot * cuk - ypk * ukdot;
    ypkdot = rkdot * suk + xpk * ukdot;

    ok = eph.omg0 + tk * eph.omgkdot - OMEGA_EARTH * eph.toe.sec;
    sok = sin(ok);
    cok = cos(ok);

    pos[0] = xpk * cok - ypk * cik * sok;
    pos[1] = xpk * sok + ypk * cik * cok;
    pos[2] = ypk * sik;

    tmp = ypkdot * cik - ypk * sik * ikdot;

    vel[0] = -eph.omgkdot * pos[1] + xpkdot * cok - tmp * sok;
    vel[1] = eph.omgkdot * pos[0] + xpkdot * sok + tmp * cok;
    vel[2] = ypk * cik * ikdot + ypkdot * sik;

    // Satellite clock correction
    tk = g.sec - eph.toc.sec;

    if (tk > SECONDS_IN_HALF_WEEK)
        tk -= SECONDS_IN_WEEK;
    else if (tk < -SECONDS_IN_HALF_WEEK)
        tk += SECONDS_IN_WEEK;

    clk[0] = eph.af0 + tk * (eph.af1 + tk * eph.af2) + relativistic - eph.tgd;
    clk[1] = eph.af1 + 2.0 * tk * eph.af2;

    return;
}

/*! \brief Compute Subframe from Ephemeris
 *  \param[in] eph Ephemeris of given SV
 *  \param[out] sbf Array of 4 subframe
 */
void eph2sbf(const ephem_t eph, const ionoutc_t ionoutc, unsigned long sbf[4][N_DWRD_SBF])
{
    unsigned long wn;
    long af0;
    long af1;
    long af2;
    unsigned long toc;
    unsigned long toe;
    long tgd;
    long deltan;
    unsigned long iodec;
    int sv;
    long cuc;
    long cus;
    long cic;
    long cis;
    long crc;
    long crs;
    long idot;
    long m0;
    unsigned long ecc;
    unsigned long sqrta;
    long reserved = 0;
    long L5Flag = 0;
    long Sflag = 0;

    long omg0;
    long i0;
    long omg;
    long omgdot;

    int svhlth;
    long long spare;
    int ura;
    int a0outc = 0;
    int a1utc = 1;
    int toutc = 2;
    int wnoutc = 3;
    int gnssid = 5;
    int A2 = 6;
    int wnot = 7;
    int a2utc = 8;
    int prnidal = 9;
    int isc = 10;
    int tom;

    /// unsigned long ura = 0UL;
    unsigned long dataId = 1UL;
    //unsigned long sbf4_page25_svId = 63UL;
    //unsigned long sbf5_page25_svId = 51UL;

    unsigned long wna;
    unsigned long toa;

    //signed long alpha0, alpha1, alpha2, alpha3;
    //signed long beta0, beta1, beta2, beta3;
    //signed long A0, A1;
    signed long dtls, dtlsf;
    unsigned long tot, wnt, wnlsf, dn;
    //unsigned long sbf4_page18_svId = 56UL;

    // FIXED: This has to be the "transmission" week number, not for the ephemeris reference time
    // wn = (unsigned long)(eph.toe.week%1024);
    wn = 0UL;
    toe = (unsigned long)(eph.toe.sec / 16.0);
    toc = (unsigned long)(eph.toc.sec / 16.0);
    iodec = (unsigned long)(eph.iodec);
    deltan = (long)(eph.deltan / POW2_M41 / PI);
    cuc = (long)(eph.cuc / POW2_M28);
    cus = (long)(eph.cus / POW2_M28);
    cic = (long)(eph.cic / POW2_M28);
    cis = (long)(eph.cis / POW2_M28);
    crc = (long)(eph.crc / POW2_M4);
    crs = (long)(eph.crs / POW2_M4);
    ecc = (unsigned long)(eph.ecc / POW2_M33);
    sqrta = (unsigned long)(eph.sqrta / POW2_M19);
    m0 = (long)(eph.m0 / POW2_M31 / PI);
    omg0 = (long)(eph.omg0 / POW2_M31 / PI);
    i0 = (long)(eph.i0 / POW2_M31 / PI);
    omg = (long)(eph.omg / POW2_M31 / PI);
    omgdot = (long)(eph.omgdot / POW2_M41 / PI);
    idot = (long)(eph.idot / POW2_M43 / PI);
    af0 = (long)(eph.af0 / POW2_M31);
    af1 = (long)(eph.af1 / POW2_M43);
    af2 = (long)(eph.af2 / POW2_M55);
    tgd = (long)(eph.tgd / POW2_M31);
    svhlth = (unsigned long)(eph.svhlth);
    spare = (unsigned long)(eph.spare);

    // wna = (unsigned long)(eph.toe.week % 256);
    // toa = (unsigned long)(eph.toe.sec / 4096.0);

    //alpha0 = (signed long)round(ionoutc.alpha0 / POW2_M30);
    //alpha1 = (signed long)round(ionoutc.alpha1 / POW2_M27);
    //alpha2 = (signed long)round(ionoutc.alpha2 / POW2_M24);
    //alpha3 = (signed long)round(ionoutc.alpha3 / POW2_M24);
    //beta0 = (signed long)round(ionoutc.beta0 / 2048.0);
    //beta1 = (signed long)round(ionoutc.beta1 / 16384.0);
    //beta2 = (signed long)round(ionoutc.beta2 / 65536.0);
    //beta3 = (signed long)round(ionoutc.beta3 / 65536.0);
    //A0 = (signed long)round(ionoutc.A0 / POW2_M30);
    //A1 = (signed long)round(ionoutc.A1 / POW2_M50);
    //dtls = (signed long)(ionoutc.dtls);
    //tot = (unsigned long)(ionoutc.tot / 4096);
    //wnt = (unsigned long)(ionoutc.wnt % 256);

    // TO DO: Specify scheduled leap seconds in command options
    // 2016/12/31 (Sat) -> WNlsf = 1929, DN = 7 (http://navigationservices.agi.com/GNSSWeb/)
    // Days are counted from 1 to 7 (Sunday is 1).
    wnlsf = 1929 % 256;
    dn = 7;
    dtlsf = 18;

    // int allocatedSat[MAX_SAT];

    sbf[0][0] = 0Ul;
    sbf[0][1] = ((wn & 0x3FFUL) << 19) | ((af0 >> 3) & (0x7FFFFUL));
    sbf[0][2] = ((af0 & 0x7UL) << 26) | ((af1 & 0xFFFFUL) << 10) | ((af2 & 0xFFUL) << 2) | (((ura >> 2) & 0x3UL));
    sbf[0][3] = ((ura & 0X3UL) << 27) | ((toc & 0xFFFFUL) << 11) | ((tgd & 0xFFUL) << 3) | (((deltan >> 19) & 7UL));
    sbf[0][4] = ((deltan & 0x3FFFFUL) << 10) | ((iodec & 0xFFUL) << 2) | (((reserved >> 8) & 3UL));
    sbf[0][5] = ((reserved & 0xFFUL) << 21) | ((L5Flag & 0X1UL) << 20) | ((Sflag & 0x1UL) << 19) | ((cuc & 0x7FFFUL) << 4) | ((cus << 11) & 0XFUL);
    sbf[0][6] = ((cus & 0x7FFUL) << 18) | ((cic & 0x7FFUL) << 3) | (((cis >> 12) & 0x7UL));
    sbf[0][7] = ((cis & 0xFFFUL) << 17) | ((crc & 0x7FFFUL) << 2) | (((crs >> 13) & 0x3UL));
    sbf[0][8] = ((crs & 0x1FFFUL) << 16) | ((idot & 0X3FFFUL) << 2) | (spare & 0x3UL);

    // subframe 2
    sbf[1][0] = 0x1UL << 1;
    sbf[1][1] = ((m0 >> 3) & (0x1FFFFFFFUL));
    sbf[1][2] = ((m0 & 0x7UL) << 26) | ((toe & 0xFFFFUL) << 10) | ((ecc >> 22) & (0x3FFUL));
    sbf[1][3] = ((ecc & 0x3FFFFFUL) << 7) | ((sqrta >> 25) & (0x7FUL));
    sbf[1][4] = ((sqrta & 0x1FFFFFFUL) << 4) | ((omg0 >> 28) & 0xFUL);
    sbf[1][5] = ((omg0 & 0xFFFFFFFUL) << 1) | ((omg >> 31) | (omg & 0x1UL));
    sbf[1][6] = ((omg >> 2) & (0x1FFFFFFFUL));
    sbf[1][7] = ((omg & 0x3ul) << 27) | ((omgdot & 0x3FFFFFFUL) << 5) | ((i0 >> 27) & (i0 & 0x1FUL));
    sbf[1][8] = ((i0 & 0x7FFFFFFUL) << 2) | ((spare & 0x3ul));
    
      // subframe 3
    sbf[2][0] = 0x2UL << 1;
    sbf[2][1] = (0x2AAAAA & 0x2FFFFF)>>7;
    sbf[2][2] = 0x15555555<<3;
    sbf[2][3] = 0xAAAAAAA>>1;
    sbf[2][4] = 0x15555555<<3;
    sbf[2][5] = 0xAAAAAAA>>1;
    sbf[2][6] = 0x15555555<<3;
    sbf[2][7] = 0xAAAAAAA>>1;
    sbf[2][8] = ((0x555555&0x3FFFFF)<<6)|(((int)eph.sv)&0x2F);
    // subframe 4
   	sbf[3][0] = 0x3UL << 1;
    sbf[3][1] = (0x2AAAAA & 0x2FFFFF)>>7;
    sbf[3][2] = 0x15555555<<3;
    sbf[3][3] = 0xAAAAAAA>>1;
    sbf[3][4] = 0x15555555<<3;
    sbf[3][5] = 0xAAAAAAA>>1;
    sbf[3][6] = 0x15555555<<3;
    sbf[3][7] = 0xAAAAAAA>>1;
    sbf[3][8] = ((0x555555&0x3FFFFF)<<6)|(((int)eph.sv)&0x2F);
}

unsigned long countBits(unsigned long v)
{
    unsigned long c;
    const int S[] = {1, 2, 4, 8, 16};
    const unsigned long B[] = {
        0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF, 0x0000FFFF};

    c = v;
    c = ((c >> S[0]) & B[0]) + (c & B[0]);
    c = ((c >> S[1]) & B[1]) + (c & B[1]);
    c = ((c >> S[2]) & B[2]) + (c & B[2]);
    c = ((c >> S[3]) & B[3]) + (c & B[3]);
    c = ((c >> S[4]) & B[4]) + (c & B[4]);

    return (c);
}

int replaceExpDesignator(char *str, int len)
{
    int i, n = 0;

    for (i = 0; i < len; i++)
    {
        if (str[i] == 'D')
        {
            n++;
            str[i] = 'E';
        }
    }

    return (n);
}

double subGpsTime(gpstime_t g1, gpstime_t g0)
{
    double dt;

    dt = g1.sec - g0.sec;
    dt += (double)(g1.week - g0.week) * SECONDS_IN_WEEK;

    return (dt);
}

gpstime_t incGpsTime(gpstime_t g0, double dt)
{
    gpstime_t g1;

    g1.week = g0.week;
    g1.sec = g0.sec + dt;

    g1.sec = round(g1.sec * 1000.0) / 1000.0; // Avoid rounding error

    while (g1.sec >= SECONDS_IN_WEEK)
    {
        g1.sec -= SECONDS_IN_WEEK;
        g1.week++;
    }

    while (g1.sec < 0.0)
    {
        g1.sec += SECONDS_IN_WEEK;
        g1.week--;
    }

    return (g1);
}

int readRinexNavAllv3_3(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, const char *fname)

{ // make changes for rinex v2 header to v3 header
    FILE *fp;
    int ieph;

    int j;

    // int sv1[5] = {1,2,3,4,8};
    int sv;
    char str[MAX_CHAR];
    char tmp[20];

    datetime_t t;
    gpstime_t g;
    gpstime_t g0;
    double dt;

    int flags = 0x0;

    if (NULL == (fp = fopen(fname, "rt")))
        return (-1);
    printf("Reading Nav File \n");
    // Clear valid flag
    for (ieph = 0; ieph < EPHEM_ARRAY_SIZE; ieph++)
    {
        for (sv = 0; sv < MAX_SAT; sv++)
        {
            // sv = sv1[j];
            eph[ieph][sv].vflg = 0;
        }
    }
    printf("Valid Flags reset \n");
    // Read header lines
    while (1)
    {
        if (NULL == fgets(str, MAX_CHAR, fp))
            break;

        if (strncmp(str + 60, "END OF HEADER", 13) == 0)
        {
            printf("END OF HEADER read \n");
            break;
        }
        // GPS correction term - v2 legacy
//        else if (strncmp(str + 60, "ION ALPHA", 9) == 0)
//        {
//            strncpy(tmp, str + 2, 12);
//            tmp[12] = 0;
//            replaceExpDesignator(tmp, 12);
//            ionoutc->alpha0 = atof(tmp);
//
//            strncpy(tmp, str + 14, 12);
//            tmp[12] = 0;
//            replaceExpDesignator(tmp, 12);
//            ionoutc->alpha1 = atof(tmp);
//
//            strncpy(tmp, str + 26, 12);
//            tmp[12] = 0;
//            replaceExpDesignator(tmp, 12);
//            ionoutc->alpha2 = atof(tmp);
//
//            strncpy(tmp, str + 38, 12);
//            tmp[12] = 0;
//            replaceExpDesignator(tmp, 12);
//            ionoutc->alpha3 = atof(tmp);
//
//            flags |= 0x1;
//        }
//        // GPS COrrection term - v2 legacy
//        else if (strncmp(str + 60, "ION BETA", 8) == 0)
//        {
//            strncpy(tmp, str + 2, 12);
//            tmp[12] = 0;
//            replaceExpDesignator(tmp, 12);
//            ionoutc->beta0 = atof(tmp);
//
//            strncpy(tmp, str + 14, 12);
//            tmp[12] = 0;
//            replaceExpDesignator(tmp, 12);
//            ionoutc->beta1 = atof(tmp);
//
//            strncpy(tmp, str + 26, 12);
//            tmp[12] = 0;
//            replaceExpDesignator(tmp, 12);
//            ionoutc->beta2 = atof(tmp);
//
//            strncpy(tmp, str + 38, 12);
//            tmp[12] = 0;
//            replaceExpDesignator(tmp, 12);
//            ionoutc->beta3 = atof(tmp);
//
//            flags |= 0x1 << 1;
//        }
//        // GPS Only legacy term - v2 need to cross verify with v3.3 document
//        else if (strncmp(str + 60, "DELTA-UTC", 9) == 0)
//        {
//            strncpy(tmp, str + 3, 19);
//            tmp[19] = 0;
//            replaceExpDesignator(tmp, 19);
//            ionoutc->A0 = atof(tmp);
//
//            strncpy(tmp, str + 22, 19);
//            tmp[19] = 0;
//            replaceExpDesignator(tmp, 19);
//            ionoutc->A1 = atof(tmp);
//
//            strncpy(tmp, str + 41, 9);
//            tmp[9] = 0;
//            ionoutc->tot = atoi(tmp);
//
//            strncpy(tmp, str + 50, 9);
//            tmp[9] = 0;
//            ionoutc->wnt = atoi(tmp);
//
//            if (ionoutc->tot % 4096 == 0)
//                flags |= 0x1 << 2;
//        }
//        else if (strncmp(str + 60, "LEAP SECONDS", 12) == 0)
//        {
//            strncpy(tmp, str, 6);
//            tmp[6] = 0;
//            ionoutc->dtls = atoi(tmp);
//
//            flags |= 0x1 << 3;
//        }
    }
//
//    ionoutc->vflg = FALSE;
//    if (flags == 0xF) // Read all Iono/UTC lines
//        ionoutc->vflg = TRUE;

    g0.week = -1;
    ieph = 0;

    // this is going to read ephemeris parameters
    while (1)
    {
        if (NULL == fgets(str, MAX_CHAR, fp))
        {
            printf("end of textfile encountered;");
            break;
        }
        int count = 0;
        if (strncmp(str, "I", 1) == 0) // that means first index is I
        {                              // PRN

            //            printf("IRNSS ephemeris being read\n");
            strncpy(tmp, str + 1, 2);
            tmp[2] = 0;
            sv = atoi(tmp) - 1;
            // printf("\nsv = %d",sv);

            // EPOCH
            strncpy(tmp, str + 4, 4);
            tmp[4] = 0;
            t.y = atoi(tmp);

            strncpy(tmp, str + 9, 2);
            tmp[2] = 0;
            t.m = atoi(tmp);

            strncpy(tmp, str + 12, 2);
            tmp[2] = 0;
            t.d = atoi(tmp);

            strncpy(tmp, str + 15, 2);
            tmp[2] = 0;
            t.hh = atoi(tmp);

            strncpy(tmp, str + 18, 2);
            tmp[2] = 0;
            t.mm = atoi(tmp);

            strncpy(tmp, str + 21, 2);
            tmp[2] = 0;
            t.sec = atof(tmp);

            date2gps(&t, &g);

            if (g0.week == -1)
                g0 = g;

            // Check current time of clock
            dt = subGpsTime(g, g0);

            if (dt > SECONDS_IN_HOUR)
            {
                g0 = g;
                ieph++; // a new set of ephemerides

                if (ieph >= EPHEM_ARRAY_SIZE)
                    break;
            }

            // Date and time
            eph[ieph][sv].t = t;

            // SV CLK
            eph[ieph][sv].toc = g;

            strncpy(tmp, str + 24, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18); // tmp[15]='E';
            eph[ieph][sv].af0 = atof(tmp);

            strncpy(tmp, str + 43, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].af1 = atof(tmp);

            strncpy(tmp, str + 62, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].af2 = atof(tmp);

            // BROADCAST ORBIT - 1
            if (NULL == fgets(str, MAX_CHAR, fp))
                break;

            strncpy(tmp, str + 6, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].iodec = (int)atof(tmp);

            strncpy(tmp, str + 24, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].crs = atof(tmp);

            strncpy(tmp, str + 43, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].deltan = atof(tmp);

            strncpy(tmp, str + 62, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].m0 = atof(tmp);

            // BROADCAST ORBIT - 2
            if (NULL == fgets(str, MAX_CHAR, fp))
                break;

            strncpy(tmp, str + 6, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].cuc = atof(tmp);

            strncpy(tmp, str + 24, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].ecc = atof(tmp);

            strncpy(tmp, str + 43, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].cus = atof(tmp);

            strncpy(tmp, str + 62, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].sqrta = atof(tmp);

            // BROADCAST ORBIT - 3
            if (NULL == fgets(str, MAX_CHAR, fp))
                break;

            strncpy(tmp, str + 6, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].toe.sec = atof(tmp);

            strncpy(tmp, str + 24, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].cic = atof(tmp);

            strncpy(tmp, str + 43, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].omg0 = atof(tmp);

            strncpy(tmp, str + 62, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].cis = atof(tmp);

            // BROADCAST ORBIT - 4
            if (NULL == fgets(str, MAX_CHAR, fp))
                break;

            strncpy(tmp, str + 6, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].i0 = atof(tmp);

            strncpy(tmp, str + 24, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].crc = atof(tmp);

            strncpy(tmp, str + 43, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].omg = atof(tmp);

            strncpy(tmp, str + 62, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].omgdot = atof(tmp);

            // BROADCAST ORBIT - 5
            if (NULL == fgets(str, MAX_CHAR, fp))
                break;

            strncpy(tmp, str + 6, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].idot = atof(tmp);

            strncpy(tmp, str + 24, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].spare = (int)atof(tmp);

            strncpy(tmp, str + 43, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].toe.week = (int)atof(tmp) - 1024;

            strncpy(tmp, str + 62, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].spare = atof(tmp);

            // BROADCAST ORBIT - 6
            if (NULL == fgets(str, MAX_CHAR, fp))
                break;
            strncpy(tmp, str + 6, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].ura = atof(tmp);

            strncpy(tmp, str + 24, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].svhlth = (int)atof(tmp);
            if ((eph[ieph][sv].svhlth > 0) && (eph[ieph][sv].svhlth < 32))
                eph[ieph][sv].svhlth += 32; // Set MSB to 1

            strncpy(tmp, str + 43, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].tgd = atof(tmp);

            strncpy(tmp, str + 62, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].spare = atof(tmp);

            // BROADCAST ORBIT - 7
            if (NULL == fgets(str, MAX_CHAR, fp))
                break;

            strncpy(tmp, str + 6, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].tom = atof(tmp);

            strncpy(tmp, str + 24, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].spare = atof(tmp);

            strncpy(tmp, str + 43, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].spare = atof(tmp);

            strncpy(tmp, str + 62, 18);
            tmp[18] = 0;
            replaceExpDesignator(tmp, 18);
            eph[ieph][sv].spare = atof(tmp);

            // Set valid flag
            eph[ieph][sv].vflg = 1;

            // Update the working variables
            eph[ieph][sv].A = eph[ieph][sv].sqrta * eph[ieph][sv].sqrta;
            eph[ieph][sv].n = sqrt(GM_EARTH / (eph[ieph][sv].A * eph[ieph][sv].A * eph[ieph][sv].A)) + eph[ieph][sv].deltan;
            eph[ieph][sv].sq1e2 = sqrt(1.0 - eph[ieph][sv].ecc * eph[ieph][sv].ecc);
            eph[ieph][sv].omgkdot = eph[ieph][sv].omgkdot - OMEGA_EARTH;
        }
    }
    fclose(fp);
    if (g0.week >= 0)
        ieph += 1; // Number of sets of ephemerides

    return (ieph);
}

//double ionosphericDelay(const ionoutc_t *ionoutc, gpstime_t g, double *llh, double *azel)
//{
//    double iono_delay = 0.0;
//    double E, phi_u, lam_u, F;
//
//    if (ionoutc->enable == FALSE)
//        return (0.0); // No ionospheric delay
//
//    E = azel[1] / PI;
//    phi_u = llh[0] / PI;
//    lam_u = llh[1] / PI;
//
//    // Obliquity factor
//    F = 1.0 + 16.0 * pow((0.53 - E), 3.0);
//
//    if (ionoutc->vflg == FALSE)
//        iono_delay = F * 5.0e-9 * SPEED_OF_LIGHT;
//    else
//    {
//        double t, psi, phi_i, lam_i, phi_m, phi_m2, phi_m3;
//        double AMP, PER, X, X2, X4;
//
//        // Earth's central angle between the user position and the earth projection of
//        // ionospheric intersection point (semi-circles)
//        psi = 0.0137 / (E + 0.11) - 0.022;
//
//        // Geodetic latitude of the earth projection of the ionospheric intersection point
//        // (semi-circles)
//        phi_i = phi_u + psi * cos(azel[0]);
//        if (phi_i > 0.416)
//            phi_i = 0.416;
//        else if (phi_i < -0.416)
//            phi_i = -0.416;
//
//        // Geodetic longitude of the earth projection of the ionospheric intersection point
//        // (semi-circles)
//        lam_i = lam_u + psi * sin(azel[0]) / cos(phi_i * PI);
//
//        // Geomagnetic latitude of the earth projection of the ionospheric intersection
//        // point (mean ionospheric height assumed 350 km) (semi-circles)
//        phi_m = phi_i + 0.064 * cos((lam_i - 1.617) * PI);
//        phi_m2 = phi_m * phi_m;
//        phi_m3 = phi_m2 * phi_m;
//
//        AMP = ionoutc->alpha0 + ionoutc->alpha1 * phi_m + ionoutc->alpha2 * phi_m2 + ionoutc->alpha3 * phi_m3;
//        if (AMP < 0.0)
//            AMP = 0.0;
//
//        PER = ionoutc->beta0 + ionoutc->beta1 * phi_m + ionoutc->beta2 * phi_m2 + ionoutc->beta3 * phi_m3;
//        if (PER < 72000.0)
//            PER = 72000.0;
//
//        // Local time (sec)
//        t = SECONDS_IN_DAY / 2.0 * lam_i + g.sec;
//        while (t >= SECONDS_IN_DAY)
//            t -= SECONDS_IN_DAY;
//        while (t < 0)
//            t += SECONDS_IN_DAY;
//
//        // Phase (radians)
//        X = 2.0 * PI * (t - 50400.0) / PER;
//
//        if (fabs(X) < 1.57)
//        {
//            X2 = X * X;
//            X4 = X2 * X2;
//            iono_delay = F * (5.0e-9 + AMP * (1.0 - X2 / 2.0 + X4 / 24.0)) * SPEED_OF_LIGHT;
//        }
//        else
//            iono_delay = F * 5.0e-9 * SPEED_OF_LIGHT;
//    }
//
//    return (iono_delay);
//}

/*! \brief Compute range between a satellite and the receiver
 *  \param[out] rho The computed range
 *  \param[in] eph Ephemeris data of the satellite
 *  \param[in] g GPS time at time of receiving the signal
 *  \param[in] xyz position of the receiver
 */
void computeRange(range_t *rho, ephem_t eph, ionoutc_t *ionoutc, gpstime_t g, double xyz[])
{
    double pos[3], vel[3], clk[2];
    double los[3];
    double tau;
    double range, rate;
    double xrot, yrot;

    double llh[3], neu[3];
    double tmat[3][3];

    // SV position at time of the pseudorange observation.
    satpos(eph, g, pos, vel, clk);

    // Receiver to satellite vector and light-time.
    subVect(los, pos, xyz);
    tau = normVect(los) / SPEED_OF_LIGHT;

    // Extrapolate the satellite position backwards to the transmission time.
    pos[0] -= vel[0] * tau;
    pos[1] -= vel[1] * tau;
    pos[2] -= vel[2] * tau;

    // Earth rotation correction. The change in velocity can be neglected.
    xrot = pos[0] + pos[1] * OMEGA_EARTH * tau;
    yrot = pos[1] - pos[0] * OMEGA_EARTH * tau;
    pos[0] = xrot;
    pos[1] = yrot;

    // New observer to satellite vector and satellite range.
    subVect(los, pos, xyz);
    range = normVect(los);
    rho->d = range;

    // Pseudorange.
    rho->range = range - SPEED_OF_LIGHT * clk[0];

    // Relative velocity of SV and receiver.
    rate = dotProd(vel, los) / range;

    // Pseudorange rate.
    rho->rate = rate; // - SPEED_OF_LIGHT*clk[1];

    // Time of application.
    rho->g = g;

    // Azimuth and elevation angles.
    xyz2llh(xyz, llh);
    ltcmat(llh, tmat);
    ecef2neu(los, tmat, neu);
    neu2azel(rho->azel, neu);

    // Add ionospheric delay
//    rho->iono_delay = ionosphericDelay(ionoutc, g, llh, rho->azel);
    //rho->range += rho->iono_delay;
    if(rho->range <0.0)
    	rho->range = -rho->range;

    return;
}

/*! \brief Compute the code phase for a given channel (satellite)
 *  \param chan Channel on which we operate (is updated)
 *  \param[in] rho1 Current range, after \a dt has expired
 *  \param[in dt delta-t (time difference) in seconds
 */
void computeCodePhase(channel_t *chan, range_t rho1, double dt)
{
    double ms;
    int ims;
    double rhorate;

    // Pseudorange rate.
    rhorate = (rho1.range - chan->rho0.range) / dt;

    // Carrier and code _frequency.
    chan->f_carr = -rhorate / LAMBDA_L1;
    chan->f_code = CODE_FREQ + chan->f_carr * CARR_TO_CODE;

    // Initial code phase and data bit counters.
    ms = ((subGpsTime(chan->rho0.g, chan->g0) + 6.0) - chan->rho0.range / SPEED_OF_LIGHT) * 1000.0;

    ims = (int)ms;
    chan->code_phase = (ms - (double)ims) * CA_SEQ_LEN; // in chip

    int i = 0;
    int j = 0;
    int flag1 = 0;

    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 10; j++)
        {
            if (((unsigned long)chan->iword == chan->sbf[i][0]) || ((unsigned long)chan->iword == chan->sbf[i][9]))
            {
                flag1 = 1;
                break;
            }
        }
        if (flag1 == 1)
            break;
    }

    if (flag1 == 0)
    {
        chan->iword = ims / 580; // 1 word =  29 bits = 580 ms
        ims -= chan->iword * 580;
    }
    else if (flag1 == 1)
    {
        chan->iword = ims / 600; // 1 word = 30 bits = 600 ms
        ims -= chan->iword * 600;
    }

    chan->ibit = ims / 20; // 1 bit = 20 code = 20 ms
    ims -= chan->ibit * 20;

    chan->icode = ims; // 1 code = 1 ms

    chan->codeCA = chan->ca[(int)chan->code_phase] * 2 - 1;

    if (flag1 == 0)
    {
        chan->dataBit = (int)((chan->dwrd[chan->iword] >> (28 - chan->ibit)) & 0x1UL) * 2 - 1;
    }
    else if (flag1 == 1)
    {
        chan->dataBit = (int)((chan->dwrd[chan->iword] >> (29 - chan->ibit)) & 0x1UL) * 2 - 1;
    }
    // Save current pseudorange
    chan->rho0 = rho1;

    return;
}

/*! \brief Read the list of user motions from the input file
 *  \param[out] xyz Output array of ECEF vectors for user motion
 *  \param[[in] filename File name of the text input file
 *  \returns Number of user data motion records read, -1 on error
 */
int readUserMotion(double xyz[USER_MOTION_SIZE][3], const char *filename)
{
    FILE *fp;
    int numd;
    char str[MAX_CHAR];
    double t, x, y, z;

    if (NULL == (fp = fopen(filename, "rt")))
        return (-1);

    for (numd = 0; numd < USER_MOTION_SIZE; numd++)
    {
        if (fgets(str, MAX_CHAR, fp) == NULL)
            break;

        if (EOF == sscanf(str, "%lf,%lf,%lf,%lf", &t, &x, &y, &z)) // Read CSV line
            break;

        xyz[numd][0] = x;
        xyz[numd][1] = y;
        xyz[numd][2] = z;
    }

    fclose(fp);

    return (numd);
}

/*! \brief Read the list of user motions from the input file
 *  \param[out] xyz Output array of LatLonHei coordinates for user motion
 *  \param[[in] filename File name of the text input file with format Lat,Lon,Hei
 *  \returns Number of user data motion records read, -1 on error
 *
 * Added by romalvarezllorens@gmail.com
 */
int readUserMotionLLH(double xyz[USER_MOTION_SIZE][3], const char *filename)
{
    FILE *fp;
    int numd;
    double t, llh[3];
    char str[MAX_CHAR];

    if (NULL == (fp = fopen(filename, "rt")))
        return (-1);

    for (numd = 0; numd < USER_MOTION_SIZE; numd++)
    {
        if (fgets(str, MAX_CHAR, fp) == NULL)
            break;

        if (EOF == sscanf(str, "%lf,%lf,%lf,%lf", &t, &llh[0], &llh[1], &llh[2])) // Read CSV line
            break;

        if (llh[0] > 90.0 || llh[0] < -90.0 || llh[1] > 180.0 || llh[1] < -180.0)
        {
            fprintf(stderr, "ERROR: Invalid file format (time[s], latitude[deg], longitude[deg], height [m].\n");
            numd = 0; // Empty user motion
            break;
        }

        llh[0] /= R2D; // convert to RAD
        llh[1] /= R2D; // convert to RAD

        llh2xyz(llh, xyz[numd]);
    }

    fclose(fp);

    return (numd);
}

int readNmeaGGA(double xyz[USER_MOTION_SIZE][3], const char *filename)
{
    FILE *fp;
    int numd = 0;
    char str[MAX_CHAR];
    char *token;
    double llh[3], pos[3];
    char tmp[8];

    if (NULL == (fp = fopen(filename, "rt")))
        return (-1);

    while (1)
    {
        if (fgets(str, MAX_CHAR, fp) == NULL)
            break;

        token = strtok(str, ",");

        if (strncmp(token + 3, "GGA", 3) == 0)
        {
            token = strtok(NULL, ","); // Date and time

            token = strtok(NULL, ","); // Latitude
            strncpy(tmp, token, 2);
            tmp[2] = 0;

            llh[0] = atof(tmp) + atof(token + 2) / 60.0;

            token = strtok(NULL, ","); // North or south
            if (token[0] == 'S')
                llh[0] *= -1.0;

            llh[0] /= R2D; // in radian

            token = strtok(NULL, ","); // Longitude
            strncpy(tmp, token, 3);
            tmp[3] = 0;

            llh[1] = atof(tmp) + atof(token + 3) / 60.0;

            token = strtok(NULL, ","); // East or west
            if (token[0] == 'W')
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
            llh2xyz(llh, pos);

            xyz[numd][0] = pos[0];
            xyz[numd][1] = pos[1];
            xyz[numd][2] = pos[2];

            // Update the number of track points
            numd++;

            if (numd >= USER_MOTION_SIZE)
                break;
        }
    }

    fclose(fp);

    return (numd);
}
int checkSatVisibility(ephem_t eph, gpstime_t g, double *xyz, double elvMask, double *azel)
{
    double llh[3], neu[3];
    double pos[3], vel[3], clk[3], los[3];
    double tmat[3][3];

    if (eph.vflg != 1)
        return (-1); // Invalid

    xyz2llh(xyz, llh);
    ltcmat(llh, tmat);

    satpos(eph, g, pos, vel, clk);
    subVect(los, pos, xyz);
    ecef2neu(los, tmat, neu);
    neu2azel(azel, neu);
    int k = azel[1] * R2D;
    if(azel[1]*R2D<0)
	{	
		int j = azel[1]*R2D;
		while(j<0)
		{
		   j = j + 2*PI;
		   
	    }
	    k=j;
	}
    // printf("azel = %d\n",azel[1]*R2D);

    if (k > elvMask)
        return (1); // Visible
    // else
    return (0); // Invisible
}

uint32_t calculateCRC24Q(const uint32_t *data, size_t length)
{
    uint32_t crc = CRC24Q_INIT;
    size_t i;
    int j;
    for (i = 0; i < length; i++)
    {
        crc ^= (data[i] << 8);

        for (j = 0; j < 8; j++)
        {
            crc <<= 1;
            if (crc & 0x1000000)
            {
                crc ^= CRC24Q_POLY;
            }
        }
    }
    return crc & 0xFFFFFF; // Mask the CRC to 24 bits
}

// Function to convert a uint32_t decimal number to binary string
char *decimalToBinary(uint32_t n)
{
    int numBits = 24;
    char *binaryStr = (char *)malloc(numBits + 1); // +1 for the null terminator

    if (binaryStr == NULL)
    {
        perror("Memory allocation failed");
        exit(EXIT_FAILURE);
    }
    int i;
    for (i = numBits - 1; i >= 0; i--)
    {
        if (n & (1u << i))
        {
            binaryStr[numBits - i - 1] = '1';
        }
        else
        {
            binaryStr[numBits - i - 1] = '0';
        }
    }

    binaryStr[numBits] = '\0'; // Null-terminate the string
    return binaryStr;
}

// Function to append 'n' bits to the end of a binary string
char *appendBits(char *binaryStr, int n)
{
    int len = strlen(binaryStr);
    char *appendedStr = (char *)malloc(len + n + 1); // +1 for the null terminator

    if (appendedStr == NULL)
    {
        perror("Memory allocation failed");
        exit(EXIT_FAILURE);
    }
    int i;
    strcpy(appendedStr, binaryStr);
    for (i = 0; i < n; i++)
    {
        appendedStr[len + i] = '0'; // Append '0' bits
    }

    appendedStr[len + n] = '\0'; // Null-terminate the appended string
    free(binaryStr);             // Free the original binary string
    return appendedStr;
}

// Function to convert a binary string to an unsigned long long decimal number
unsigned long long binaryToDecimal(const char *binaryStr)
{
    return strtoull(binaryStr, NULL, 2);
}

void intToCharArray(unsigned int num, unsigned char *str, int bufferSize)
{
    snprintf(str, bufferSize, "%d", num);
}

void fecEncoding(const unsigned char *data, size_t dataSize, unsigned char *encodedData)
{
    unsigned int shiftRegisters[CONSTRAINT_LENGTH] = {0};
    size_t i;
    int j, k, k1;
    unsigned int generatorPolynomials[NUM_POLYNOMIALS] = {0x079, 0x05B};

    for (i = 0; i < dataSize; i++)
    {
        for (j = 0; j < NUM_POLYNOMIALS; j++)
        {
            unsigned int outputBit = 0;
            unsigned int polynomial = generatorPolynomials[j];

            // XOR the shift register bits with the generator polynomial
            for (k = 0; k < CONSTRAINT_LENGTH; k++)
            {
                outputBit ^= shiftRegisters[k] & ((polynomial >> k) & 0x1);
            }

            // Shift the shift registers
            for (k1 = CONSTRAINT_LENGTH - 1; k1 > 0; k1--)
            {
                shiftRegisters[k1] = shiftRegisters[k1 - 1];
            }
            shiftRegisters[0] = data[i];

            // Store the encoded bit

            // encodedData[i * NUM_POLYNOMIALS + j] = (unsigned char)outputBit;
            if (outputBit == 0)
            {
                encodedData[i * NUM_POLYNOMIALS + j] = '0';
            }
            else
            {
                encodedData[i * NUM_POLYNOMIALS + j] = '1';
            }
        }
    }
    encodedData[i * NUM_POLYNOMIALS + j - 2] = '\0';
    return;
}

void interleaver(unsigned char interleaver_input[584], unsigned char interleaver_output[584])
{
    int k = 0;
    int i, j;
    unsigned char interleaver[8][73];

    // Populate the interleaver matrix from interleaver_input
    for (i = 0; i < 8; i++)
    {
        for (j = 0; j < 73; j++)
        {
            interleaver[i][j] = interleaver_input[k];
            k++;
        }
    }

    k = 0;
    for (i = 0; i < 73; i++)
    {
        for (j = 0; j < 8; j++)
        {
            interleaver_output[k] = interleaver[j][i];
            k++;
        }
    }
    interleaver_output[k] = '\0';
    return;
}

int generateNavMsg(gpstime_t g, channel_t *chan, int init)
{
    int iwrd, isbf;
    gpstime_t g0;
    unsigned long wn, tow;
    unsigned sbfwrd;
    unsigned long prevwrd;
    //int nib;

    g0.week = g.week;
    g0.sec = (double)(((unsigned long)(g.sec + 0.5)) / 48UL) * 48.0; // Align with the full frame length = 48 sec
    chan->g0 = g0;                                                   // Data bit reference time

    wn = (unsigned long)(g0.week % 1024);
    tow = ((unsigned long)g0.sec) / 12UL;

    //	if (init==1) // Initialize subframe 5
    //	{
    //		prevwrd = 0UL;
    //
    //		for (iwrd=0; iwrd<N_DWRD_SBF; iwrd++)
    //		{
    //			sbfwrd = chan->sbf[4][iwrd];
    //
    //			// Add TOW-count message into HOW
    //			if (iwrd==1)
    //				sbfwrd |= ((tow&0x1FFFFUL)<<13);
    //
    //			// Compute checksum
    //			sbfwrd |= (prevwrd<<30) & 0xC0000000UL; // 2 LSBs of the previous transmitted word
    //			nib = ((iwrd==1)||(iwrd==9))?1:0; // Non-information bearing bits for word 2 and 10
    //			chan->dwrd[iwrd] = computeChecksum(sbfwrd, nib);
    //
    //			prevwrd = chan->dwrd[iwrd];
    //		}
    //	}
    //	else // Save subframe 5
    //	{
    //		for (iwrd=0; iwrd<N_DWRD_SBF; iwrd++)
    //		{
    //			chan->dwrd[iwrd] = chan->dwrd[N_DWRD_SBF*N_SBF+iwrd];
    //
    //			prevwrd = chan->dwrd[iwrd];
    //		}
    //		/*
    //		// Sanity check
    //		if (((chan->dwrd[1])&(0x1FFFFUL<<13)) != ((tow&0x1FFFFUL)<<13))
    //		{
    //			fprintf(stderr, "\nWARNING: Invalid TOW in subframe 5.\n");
    //			return(0);
    //		}
    //		*/
    //	}

    for (isbf = 0; isbf < N_SBF; isbf++)
    {
        tow++;
        uint32_t message[9];

        for (iwrd = 0; iwrd < N_DWRD_SBF; iwrd++)
        {
            sbfwrd = chan->sbf[isbf][iwrd];

            // Add transmission week number to Subframe 1
            if ((isbf == 0) && (iwrd == 1))
                sbfwrd |= (wn & 0x3FFUL) << 19; // Done: 15th Sept : proof in Notebook Nagaphani.

            // Add TOW-count message into HOW
            if (iwrd == 0) // Har sub frame ka jo first word hai usmai iss position pr time of week count aaana chahiye. // Done: 15th Sept
                sbfwrd |= ((tow & 0x1FFFFUL) << 5);

            //			// Compute checksum
            //			sbfwrd |= (prevwrd<<30) & 0xC0000000UL; // 2 LSBs of the previous transmitted word
            //			nib = ((iwrd==1)||(iwrd==9))?1:0; // Non-information bearing bits for word 2 and 10
            //			chan->dwrd[(isbf+1)*N_DWRD_SBF+iwrd] = computeChecksum(sbfwrd, nib);
            //
            //			prevwrd = chan->dwrd[(isbf+1)*N_DWRD_SBF+iwrd];
        }
        int j;
        for (j = 1; j < 9; j++)
        {
            message[j] = chan->sbf[isbf][j]; // Example message in decimal format
        }
        size_t messageLength = sizeof(message) / sizeof(message[0]);
        uint32_t crc = calculateCRC24Q(message, messageLength);
        // printf("CRC-24Q: %u\n", crc);
        //        printBinary(crc, 24);
        char *binaryString = decimalToBinary(crc);
        int appendBitsCount = 6;
        char *appendedBinaryString = appendBits(binaryString, appendBitsCount);
        unsigned long long result = binaryToDecimal(appendedBinaryString);
        chan->sbf[isbf][9] = result;
        // printf("\n");
        // printf("sbf[%d][%d] = %llu\n", isbf, 9, chan->sbf[isbf][9]);
    }

    unsigned char x[4][292] = {"", "", "", ""};
    char y[31] = {0};
    char z[30] = {0};
    int i1 = 0;
    int i2 = 0;
    int j1 = 0;
    int k1 = 0;

    for (i1 = 0; i1 < 4; i1++)
    {
        for (i2 = 0; i2 < 10; i2++)
        {
            if (i2 == 0 || i2 == 9)
            {
                for (j1 = 0; j1 < 30; j1++)
                {
                    if (chan->sbf[i1][i2] % 2 == 0)
                    {
                        y[29 - j1] = '0';
                    }
                    else
                    {
                        y[29 - j1] = '1';
                    }
                    chan->sbf[i1][i2] = chan->sbf[i1][i2] / 2;
                }
                y[30] = '\0';
                strcat(x[i1], y);
            }
            else
            {
                for (k1 = 0; k1 < 29; k1++)
                {
                    if (chan->sbf[i1][i2] % 2 == 0)
                    {
                        z[28 - k1] = '0';
                    }
                    else
                    {
                        z[28 - k1] = '1';
                    }
                    chan->sbf[i1][i2] = chan->sbf[i1][i2] / 2;
                }
                z[29] = '\0';
                strcat(x[i1], z);
            }
        }
    }

    int c;
    int i;
    int j;
    unsigned char *syncWord = "1110101110010000";
    unsigned char masterFrame[4][600] = {"", "", "", ""};
    for (c = 0; c < 4; c++)
    {

        int inputSize = sizeof(x[c]) / sizeof(x[c][0]);

        int encodedSize = inputSize * NUM_POLYNOMIALS;

        unsigned char encodedData[584] = {0};

        unsigned char encodedData1[584] = {0};

        fecEncoding(x[c], inputSize, encodedData);

        // printf("\n String(Encoded Data without interleaving): ");

        //	printf("%s",encodedData);
        //  printf("\n");

        interleaver(encodedData, encodedData1);
        // printf("\n");
        // printf("Interleaved encoded data: \n%s\n",encodedData1);

        strcat(masterFrame[c], syncWord);

        strcat(masterFrame[c], encodedData1);

        // printf("\n");
        // printf("\n");
        // printf("Complete subframe %d:\n", c+1);
        // printf("%s", masterFrame[c]);
        // printf("\n");
    }

    return;
}

int allocateChannel(channel_t *chan, ephem_t *eph, ionoutc_t ionoutc, gpstime_t grx, double *xyz, double elvMask)
{
    int nsat = 0;
    int i, sv;
    double azel[2];

    range_t rho;
    double ref[3] = {0.0};
    double r_ref, r_xyz;
    double phase_ini;

    for (sv = 0; sv < MAX_SAT; sv++)
    {
        if (checkSatVisibility(eph[sv], grx, xyz, 0.0, azel) == 1)
        {
            // printf("\neph[%d] = %d\n",sv,eph[sv]);
            printf("\nSatellite Visible: %d", sv + 1);
            nsat++; // Number of visible satellites

            if (allocatedSat[sv] == -1) // Visible but not allocated
            {
                // Allocated new satellite
                for (i = 0; i < MAX_CHAN; i++)
                {
                    if (chan[i].prn == 0)
                    {
                        // Initialize channel
                        chan[i].prn = sv + 1;
                        chan[i].azel[0] = azel[0];
                        chan[i].azel[1] = azel[1];

                        // C/A code generation
                        codegen(chan[i].ca, chan[i].prn);

                        // Generate subframe
                        eph2sbf(eph[sv], ionoutc, chan[i].sbf);

                        // Generate navigation message
                        generateNavMsg(grx, &chan[i], 1);

                        // Initialize pseudorange
                        computeRange(&rho, eph[sv], &ionoutc, grx, xyz);
                        chan[i].rho0 = rho;

                        // Initialize carrier phase
                        r_xyz = rho.range;

                        computeRange(&rho, eph[sv], &ionoutc, grx, ref);
                        r_ref = rho.range;

                        phase_ini = (2.0 * r_ref - r_xyz) / LAMBDA_L1;
#ifdef FLOAT_CARR_PHASE
                        chan[i].carr_phase = phase_ini - floor(phase_ini);
#else
                        phase_ini -= floor(phase_ini);
                        chan[i].carr_phase = (unsigned int)(512.0 * 65536.0 * phase_ini);
#endif
                        // Done.
                        break;
                    }
                }

                // Set satellite allocation channel
                if (i < MAX_CHAN)
                    allocatedSat[sv] = i;
            }
        }
        else if (allocatedSat[sv] >= 0) // Not visible but allocated
        {
            // Clear channel
            chan[allocatedSat[sv]].prn = 0;

            // Clear satellite allocation flag
            allocatedSat[sv] = -1;
        }
    }
    printf("\nnsat =  %d\n", nsat);

    return (nsat);
}
void usage(void)
{
    fprintf(stderr, "Usage: gps-sdr-sim [options]\n"
                    "Options:\n"
                    "  -e <gps_nav>     RINEX navigation file for GPS ephemerides (required)\n"
                    "  -u <user_motion> User motion file in ECEF x, y, z format (dynamic mode)\n"
                    "  -x <user_motion> User motion file in lat, lon, height format (dynamic mode)\n"
                    "  -g <nmea_gga>    NMEA GGA stream (dynamic mode)\n"
                    "  -c <location>    ECEF X,Y,Z in meters (static mode) e.g. 3967283.154,1022538.181,4872414.484\n"
                    "  -l <location>    Lat, lon, height (static mode) e.g. 35.681298,139.766247,10.0\n"
                    "  -t <date,time>   Scenario start time YYYY/MM/DD,hh:mm:ss\n"
                    "  -T <date,time>   Overwrite TOC and TOE to scenario start time\n"
                    "  -d <duration>    Duration [sec] (dynamic mode max: %.0f, static mode max: %d)\n"
                    "  -o <output>      I/Q sampling data file (default: gpssim.bin)\n"
                    "  -s <frequency>   Sampling frequency [Hz] (default: 2600000)\n"
                    "  -b <iq_bits>     I/Q data format [1/8/16] (default: 16)\n"
                    "  -i               Disable ionospheric delay for spacecraft scenario\n"
                    "  -v               Show details about simulated channels\n",
            ((double)USER_MOTION_SIZE) / 10.0, STATIC_MAX_DURATION);

    return;
}

int main(int argc, char *argv[])
{
    clock_t tstart, tend;

    FILE *fp;
    FILE *fp1;
    int sv;
    int neph, ieph;
    ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT];
    gpstime_t g0;

    double llh[3];

    int i, j;
    channel_t chan[MAX_CHAN];
    double elvmask = 0.0; // in degree

    int ip, qp;
    int iTable;
    short *iq_buff = NULL;
    signed char *iq8_buff = NULL;

    gpstime_t grx;
    double delt;
    int isamp;

    int iumd;
    int numd;
    char umfile[MAX_CHAR];
    double xyz[USER_MOTION_SIZE][3];

    int staticLocationMode = FALSE;
    int nmeaGGA = FALSE;
    int umLLH = FALSE;

    char navfile[MAX_CHAR];
    char outfile[MAX_CHAR];

    double samp_freq;
    int iq_buff_size;
    int data_format;

    int result;

    int gain[MAX_CHAN];
    double path_loss;
    double ant_gain;
    double ant_pat[37];
    int ibs; // boresight angle index

    datetime_t t0, tmin, tmax;
    gpstime_t gmin, gmax;
    double dt;
    int igrx;

    double duration;
    int iduration;
    int verb;

    int timeoverwrite = FALSE; // Overwrite the TOC and TOE in the RINEX file

    ionoutc_t ionoutc;

    // fp1 = fopen("ece.txt", "w");

    strcpy(navfile, "DECE_real_only navic_5.23P");

    //    	////////////////////////////////////////////////////////////
    //	// Read options
    //	////////////////////////////////////////////////////////////
    //
    // Default options
    //  navfile[0] = 0;
    umfile[0] = 0;
    strcpy(outfile, "irnsssim4.bin");
    samp_freq = 2.6e6;
    data_format = SC16;
    g0.week = -1; // Invalid start time
    iduration = USER_MOTION_SIZE;
    duration = (double)iduration / 10.0; // Default duration
    verb = FALSE;
    ionoutc.enable = TRUE;

    //	if (argc<3)
    //	{
    //		usage();
    //		exit(1);
    //	}
    //
    //	while ((result=getopt(argc,argv,"e:u:x:g:c:l:o:s:b:T:t:d:iv"))!=-1)
    //	{
    //		switch (result)
    //		{
    //		case 'e':
    //			strcpy(navfile, optarg);
    //			break;
    //		case 'u':
    //			strcpy(umfile, optarg);
    //			nmeaGGA = FALSE;
    //			umLLH = FALSE;
    //			break;
    //		case 'x':
    //			// Added by romalvarezllorens@gmail.com
    //			strcpy(umfile, optarg);
    //			umLLH = TRUE;
    //			break;
    //		case 'g':
    //			strcpy(umfile, optarg);
    //			nmeaGGA = TRUE;
    //			break;
    //		case 'c':
    //			// Static ECEF coordinates input mode
    //			staticLocationMode = TRUE;
    //			sscanf(optarg,"%lf,%lf,%lf",&xyz[0][0],&xyz[0][1],&xyz[0][2]);
    //			break;
    //		case 'l':
    //			// Static geodetic coordinates input mode
    //			// Added by scateu@gmail.com
    //			staticLocationMode = TRUE;
    //			sscanf(optarg,"%lf,%lf,%lf",&llh[0],&llh[1],&llh[2]);
    //			llh[0] = llh[0] / R2D; // convert to RAD
    //			llh[1] = llh[1] / R2D; // convert to RAD
    //			llh2xyz(llh,xyz[0]); // Convert llh to xyz
    //			break;
    //		case 'o':
    //			strcpy(outfile, optarg);
    //			break;
    //		case 's':
    //			samp_freq = atof(optarg);
    //			if (samp_freq<1.0e6)
    //			{
    //				fprintf(stderr, "ERROR: Invalid sampling frequency.\n");
    //				exit(1);
    //			}
    //			break;
    //		case 'b':
    //			data_format = atoi(optarg);
    //			if (data_format!=SC01 && data_format!=SC08 && data_format!=SC16)
    //			{
    //				fprintf(stderr, "ERROR: Invalid I/Q data format.\n");
    //				exit(1);
    //			}
    //			break;
    //		case 'T':
    //			timeoverwrite = TRUE;
    //			if (strncmp(optarg, "now", 3)==0)
    //			{
    //				time_t timer;
    //				struct tm *gmt;
    //
    //				time(&timer);
    //				gmt = gmtime(&timer);
    //
    //				t0.y = gmt->tm_year+1900;
    //				t0.m = gmt->tm_mon+1;
    //				t0.d = gmt->tm_mday;
    //				t0.hh = gmt->tm_hour;
    //				t0.mm = gmt->tm_min;
    //				t0.sec = (double)gmt->tm_sec;
    //
    //				date2gps(&t0, &g0);
    //
    //				break;
    //			}
    //		case 't':
    //			sscanf(optarg, "%d/%d/%d,%d:%d:%lf", &t0.y, &t0.m, &t0.d, &t0.hh, &t0.mm, &t0.sec);
    //			if (t0.y<=1980 || t0.m<1 || t0.m>12 || t0.d<1 || t0.d>31 ||t0.hh<0 || t0.hh>23 || t0.mm<0 || t0.mm>59 || t0.sec<0.0 || t0.sec>=60.0)
    //			{
    //				fprintf(stderr, "ERROR: Invalid date and time.\n");
    //				exit(1);
    //			}
    //			t0.sec = floor(t0.sec);
    //			date2gps(&t0, &g0);
    //			break;
    //		case 'd':
    //			duration = atof(optarg);
    //			break;
    //		case 'i':
    //			ionoutc.enable = FALSE; // Disable ionospheric correction
    //			break;
    //		case 'v':
    //			verb = TRUE;
    //			break;
    //		case ':':
    //		case '?':
    //			usage();
    //			exit(1);
    //		default:
    //			break;
    //		}
    //	}

    if (navfile[0] == 0)
    {
        fprintf(stderr, "ERROR: GPS ephemeris file is not specified.\n");
        exit(1);
    }

    if (umfile[0] == 0 && !staticLocationMode)
    {
        // Default static location; New Delhi
        staticLocationMode = TRUE;
        //		llh[0] = 35.681298 / R2D;
        //		llh[1] = 139.766247 / R2D;
        llh[0] = 28.644800 / R2D;
        llh[1] = 77.216721 / R2D;
        llh[2] = 10.0;
    }

    if (duration < 0.0 || (duration > ((double)USER_MOTION_SIZE) / 10.0 && !staticLocationMode) || (duration > STATIC_MAX_DURATION && staticLocationMode))
    {
        fprintf(stderr, "ERROR: Invalid duration.\n");
        exit(1);
    }
    iduration = (int)(duration * 10.0 + 0.5);

    // Buffer size
    samp_freq = floor(samp_freq / 10.0);
    iq_buff_size = (int)samp_freq; // samples per 0.1sec
    samp_freq *= 10.0;

    delt = 1.0 / samp_freq;

    ////////////////////////////////////////////////////////////
    // Receiver position
    ////////////////////////////////////////////////////////////

    if (!staticLocationMode)
    {
        // Read user motion file
        if (nmeaGGA == TRUE)
            numd = readNmeaGGA(xyz, umfile);
        else if (umLLH == TRUE)
            numd = readUserMotionLLH(xyz, umfile);
        else
            numd = readUserMotion(xyz, umfile);

        if (numd == -1)
        {
            fprintf(stderr, "ERROR: Failed to open user motion / NMEA GGA file.\n");
            exit(1);
        }
        else if (numd == 0)
        {
            fprintf(stderr, "ERROR: Failed to read user motion / NMEA GGA data.\n");
            exit(1);
        }

        // Set simulation duration
        if (numd > iduration)
            numd = iduration;

        // Set user initial position
        xyz2llh(xyz[0], llh);
    }
    else
    {
        // Static geodetic coordinates input mode: "-l"
        // Added by scateu@gmail.com
        fprintf(stderr, "Using static location mode.\n");

        // Set simulation duration
        numd = iduration;

        // Set user initial position
        llh2xyz(llh, xyz[0]);
    }

    fprintf(stderr, "xyz = %11.1f, %11.1f, %11.1f\n", xyz[0][0], xyz[0][1], xyz[0][2]);
    fprintf(stderr, "llh = %11.6f, %11.6f, %11.1f\n", llh[0] * R2D, llh[1] * R2D, llh[2]);

    ////////////////////////////////////////////////////////////
    // Read ephemeris
    ////////////////////////////////////////////////////////////

    neph = readRinexNavAllv3_3(eph, &ionoutc, navfile);

    if (neph == 0)
    {
        fprintf(stderr, "ERROR: No ephemeris available.\n");
        exit(1);
    }
    else if (neph == -1)
    {
        fprintf(stderr, "ERROR: ephemeris file not found.\n");
        exit(1);
    }

    if ((verb == TRUE) && (ionoutc.vflg == TRUE))
    {
        fprintf(stderr, "  %12.3e %12.3e %12.3e %12.3e\n", ionoutc.alpha0, ionoutc.alpha1, ionoutc.alpha2, ionoutc.alpha3);
        fprintf(stderr, "  %12.3e %12.3e %12.3e %12.3e\n", ionoutc.beta0, ionoutc.beta1, ionoutc.beta2, ionoutc.beta3);
        fprintf(stderr, "   %19.11e %19.11e  %9d %9d\n", ionoutc.A0, ionoutc.A1, ionoutc.tot, ionoutc.wnt);
        fprintf(stderr, "%6d\n", ionoutc.dtls);
    }

    for (sv = 0; sv < MAX_SAT; sv++)
    {
        if (eph[0][sv].vflg == 1)
        {
            gmin = eph[0][sv].toc;
            tmin = eph[0][sv].t;
            break;
        }
    }

    gmax.sec = 0;
    gmax.week = 0;
    tmax.sec = 0;
    tmax.mm = 0;
    tmax.hh = 0;
    tmax.d = 0;
    tmax.m = 0;
    tmax.y = 0;
    for (sv = 0; sv < MAX_SAT; sv++)
    {
        if (eph[neph - 1][sv].vflg == 1)
        {
            gmax = eph[neph - 1][sv].toc;
            tmax = eph[neph - 1][sv].t;
            break;
        }
    }

    if (g0.week >= 0) // Scenario start time has been set.
    {
        if (timeoverwrite == TRUE)
        {
            gpstime_t gtmp;
            datetime_t ttmp;
            double dsec;

            gtmp.week = g0.week;
            gtmp.sec = (double)(((int)(g0.sec)) / 7200) * 7200.0;

            dsec = subGpsTime(gtmp, gmin);

            // Overwrite the UTC reference week number
            ionoutc.wnt = gtmp.week;
            ionoutc.tot = (int)gtmp.sec;

            // Iono/UTC parameters may no longer valid
            // ionoutc.vflg = FALSE;

            // Overwrite the TOC and TOE to the scenario start time
            for (sv = 0; sv < MAX_SAT; sv++)
            {
                for (i = 0; i < neph; i++)
                {
                    if (eph[i][sv].vflg == 1)
                    {
                        gtmp = incGpsTime(eph[i][sv].toc, dsec);
                        gps2date(&gtmp, &ttmp);
                        eph[i][sv].toc = gtmp;
                        eph[i][sv].t = ttmp;

                        gtmp = incGpsTime(eph[i][sv].toe, dsec);
                        eph[i][sv].toe = gtmp;
                    }
                }
            }
        }
        else
        {
            if (subGpsTime(g0, gmin) < 0.0 || subGpsTime(gmax, g0) < 0.0)
            {
                fprintf(stderr, "ERROR: Invalid start time.\n");
                fprintf(stderr, "tmin = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
                        tmin.y, tmin.m, tmin.d, tmin.hh, tmin.mm, tmin.sec,
                        gmin.week, gmin.sec);
                fprintf(stderr, "tmax = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
                        tmax.y, tmax.m, tmax.d, tmax.hh, tmax.mm, tmax.sec,
                        gmax.week, gmax.sec);
                exit(1);
            }
        }
    }
    else
    {
        g0 = gmin;
        t0 = tmin;
    }

    fprintf(stderr, "Start time = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
            t0.y, t0.m, t0.d, t0.hh, t0.mm, t0.sec, g0.week, g0.sec);
    fprintf(stderr, "Duration = %.1f [sec]\n", ((double)numd) / 10.0);

    printf("Start time = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n", t0.y, t0.m, t0.d, t0.hh, t0.mm, t0.sec, g0.week, g0.sec);
    printf("Duration = %.1f [sec]\n", ((double)numd) / 10.0);

    // Select the current set of ephemerides
    ieph = -1;

    for (i = 0; i < neph; i++)
    {
        for (sv = 0; sv < MAX_SAT; sv++)
        {
            if (eph[i][sv].vflg == 1)
            {
                dt = subGpsTime(g0, eph[i][sv].toc);
                if (dt >= -SECONDS_IN_HOUR && dt < SECONDS_IN_HOUR)
                {
                    ieph = i;
                    break;
                }
            }
        }

        if (ieph >= 0) // ieph has been set
            break;
    }

    if (ieph == -1)
    {
        fprintf(stderr, "ERROR: No current set of ephemerides has been found.\n");
        exit(1);
    }

    ////////////////////////////////////////////////////////////
    // Baseband signal buffer and output file
    ////////////////////////////////////////////////////////////

    // Allocate I/Q buffer
    iq_buff = calloc(2 * iq_buff_size, 2);

    if (iq_buff == NULL)
    {
        fprintf(stderr, "ERROR: Failed to allocate 16-bit I/Q buffer.\n");
        exit(1);
    }

    if (data_format == SC08)
    {
        iq8_buff = calloc(2 * iq_buff_size, 1);
        if (iq8_buff == NULL)
        {
            fprintf(stderr, "ERROR: Failed to allocate 8-bit I/Q buffer.\n");
            exit(1);
        }
    }
    else if (data_format == SC01)
    {
        iq8_buff = calloc(iq_buff_size / 4, 1); // byte = {I0, Q0, I1, Q1, I2, Q2, I3, Q3}
        if (iq8_buff == NULL)
        {
            fprintf(stderr, "ERROR: Failed to allocate compressed 1-bit I/Q buffer.\n");
            exit(1);
        }
    }

    // Open output file
    // "-" can be used as name for stdout
    if (strcmp("-", outfile))
    {
        if (NULL == (fp = fopen(outfile, "wb")))
        {
            fprintf(stderr, "ERROR: Failed to open output file.\n");
            exit(1);
        }
    }
    else
    {
        fp = stdout;
    }

    ////////////////////////////////////////////////////////////
    // Initialize channels
    ////////////////////////////////////////////////////////////

    // Clear all channels
    for (i = 0; i < MAX_CHAN; i++)
        chan[i].prn = 0;

    // Clear satellite allocation flag
    for (sv = 0; sv < MAX_SAT; sv++)
        allocatedSat[sv] = -1;

    // Initial reception time
    grx = incGpsTime(g0, 0.0);

    // Allocate visible satellites
    allocateChannel(chan, eph[ieph], ionoutc, grx, xyz[0], elvmask);

    for (i = 0; i < MAX_CHAN; i++)
    {
        if (chan[i].prn > 0)
            //			fprintf(stderr, "%02d %6.1f %5.1f %11.1f %5.1f\n", chan[i].prn,chan[i].azel[0]*R2D, chan[i].azel[1]*R2D, chan[i].rho0.d, chan[i].rho0.iono_delay);
            printf("%02d %6.1f %5.1f %11.1f\n", chan[i].prn, chan[i].azel[0] * R2D, chan[i].azel[1] * R2D, chan[i].rho0.d);
    }

    ////////////////////////////////////////////////////////////
    // Receiver antenna gain pattern
    ////////////////////////////////////////////////////////////

    for (i = 0; i < 37; i++)
        ant_pat[i] = pow(10.0, -ant_pat_db[i] / 20.0);

    ////////////////////////////////////////////////////////////
    // Generate baseband signals
    ////////////////////////////////////////////////////////////

    tstart = clock();
    int flag2 = 1;
    int flag3 = 1;
    // Update receiver time
    grx = incGpsTime(grx, 0.1);

    for (iumd = 1; iumd < numd; iumd++)
    {
        for (i = 0; i < MAX_CHAN; i++)
        {
            if (chan[i].prn > 0)
            {
                //				printf("\ni=%d, prn=%d\n",i,chan[i].prn);
                // Refresh code phase and data bit counters
                range_t rho;
                sv = chan[i].prn - 1;

                // Current pseudorange
                if (!staticLocationMode)
                    computeRange(&rho, eph[ieph][sv], &ionoutc, grx, xyz[iumd]);
                else
                    computeRange(&rho, eph[ieph][sv], &ionoutc, grx, xyz[0]);

                chan[i].azel[0] = rho.azel[0];
                chan[i].azel[1] = rho.azel[1];

                // Update code phase and data bit counters
                computeCodePhase(&chan[i], rho, 0.1);
#ifndef FLOAT_CARR_PHASE
                chan[i].carr_phasestep = (int)round(512.0 * 65536.0 * chan[i].f_carr * delt);
#endif
                // Path loss
                path_loss = 36000000.0 / rho.d;

                // Receiver antenna gain
                ibs = (int)((90.0 - rho.azel[1] * R2D) / 5.0); // covert elevation to boresight
                ant_gain = ant_pat[ibs];

                // Signal gain
                gain[i] = (int)(path_loss * ant_gain * 128.0); // scaled by 2^7
            }
        }

        for (isamp = 0; isamp < iq_buff_size; isamp++)
        {
            int i_acc = 0;
            int q_acc = 0;

            for (i = 0; i < MAX_CHAN; i++)
            {
                if (chan[i].prn > 0)
                {
#ifdef FLOAT_CARR_PHASE
                    iTable = (int)floor(chan[i].carr_phase * 512.0);
#else
                    iTable = (chan[i].carr_phase >> 16) & 0x1ff; // 9-bit index
#endif
                    ip = chan[i].dataBit * chan[i].codeCA * cosTable512[iTable] * gain[i];
                    qp = chan[i].dataBit * chan[i].codeCA * sinTable512[iTable] * gain[i];

                    // Accumulate for all visible satellites
                    i_acc += ip;
                    q_acc += qp;

                    // Update code phase
                    chan[i].code_phase += chan[i].f_code * delt;

                    if (chan[i].code_phase >= CA_SEQ_LEN)
                    {
                        chan[i].code_phase -= CA_SEQ_LEN;

                        chan[i].icode++;

                        if (chan[i].icode >= 20) // 20 C/A codes = 1 navigation data bit
                        {
                            chan[i].icode = 0;
                            chan[i].ibit++;

                            if (chan[i].ibit == 30 && flag3 == 1 && flag2 == 1) // 30 navigation data bits = 1 word // To recognize the first and last word
                            {
                                // To detect the first and last word ( 30 bit ka hai ke nai )
                                int super30 = 1;
                                chan[i].ibit = 0;
                                chan[i].iword++;
                                flag2 = 0; // This is indication that next word is 29 bits ( After the first or last word)
                                flag3 = 1;

                                // What we defined ( naagu )
                                // Next word = 29 bits then flag3 = 1
                                // Current word is 29 bits then flag 2 =0 ;

                                /*
                                if (chan[i].iword>=N_DWRD)
                                    fprintf(stderr, "\nWARNING: Subframe word buffer overflow.\n");
                                */
                            }
                            else if (chan[i].ibit == 29 && flag3 == 1 && flag2 == 0) // 29 navigation data bits = 1 word
                            {
                                // to detect the beechka word.
                                int super30 = 0;
                                chan[i].ibit = 0;
                                chan[i].iword++;
                                flag3 = 1; // next word 29 ka aane vaala hai
                                flag2 = 0; // current word woh 29 ka cha rha hai

                                /*
                                if (chan[i].iword>=N_DWRD)
                                    fprintf(stderr, "\nWARNING: Subframe word buffer overflow.\n");
                                */
                            }
                            else if (chan[i].ibit == 29 && flag3 == 0 && flag2 == 0)
                            {
                                // you are at seond last word and then travesring to last word
                                chan[i].ibit = 0;
                                chan[i].iword++;
                                flag3 = 0;
                                flag2 = 1;
                            }

                            // Set new navigation data bit
                            if (flag2)
                            {
                                chan[i].dataBit = (int)((chan[i].dwrd[chan[i].iword] >> (29 - chan[i].ibit)) & 0x1UL) * 2 - 1;
                            }
                            else
                            {
                                chan[i].dataBit = (int)((chan[i].dwrd[chan[i].iword] >> (28 - chan[i].ibit)) & 0x1UL) * 2 - 1;
                            }
                        }
                    }

                    // Set current code chip
                    chan[i].codeCA = chan[i].ca[(int)chan[i].code_phase] * 2 - 1;

                    // Update carrier phase
#ifdef FLOAT_CARR_PHASE
                    chan[i].carr_phase += chan[i].f_carr * delt;

                    if (chan[i].carr_phase >= 1.0)
                    {

                        chan[i].carr_phase -= 1.0;
                    }
                    else if (chan[i].carr_phase < 0.0)
                    {

                        chan[i].carr_phase += 1.0;
                    }
#else
                    chan[i].carr_phase += chan[i].carr_phasestep;
#endif
                }
            }

            // Scaled by 2^7
            i_acc = (i_acc + 64) >> 7;
            q_acc = (q_acc + 64) >> 7;

            // Store I/Q samples into buffer
            iq_buff[isamp * 2] = (short)i_acc;
            iq_buff[isamp * 2 + 1] = (short)q_acc;
        }

        if (data_format == SC01)
        {
            for (isamp = 0; isamp < 2 * iq_buff_size; isamp++)
            {
                if (isamp % 8 == 0)
                    iq8_buff[isamp / 8] = 0x00;

                iq8_buff[isamp / 8] |= (iq_buff[isamp] > 0 ? 0x01 : 0x00) << (7 - isamp % 8);
            }

            fwrite(iq8_buff, 1, iq_buff_size / 4, fp);
        }
        else if (data_format == SC08)
        {
            for (isamp = 0; isamp < 2 * iq_buff_size; isamp++)
                iq8_buff[isamp] = iq_buff[isamp] >> 4; // 12-bit bladeRF -> 8-bit HackRF

            fwrite(iq8_buff, 1, 2 * iq_buff_size, fp);
        }
        else // data_format==SC16
        {
            fwrite(iq_buff, 2, 2 * iq_buff_size, fp);
        }

        //
        // Update navigation message and channel allocation every 30 seconds
        //

        igrx = (int)(grx.sec * 10.0 + 0.5);

        if (igrx % 240 == 0) // Every 30 seconds
        {
            // Update navigation message
            for (i = 0; i < MAX_CHAN; i++)
            {
                if (chan[i].prn > 0)
                    generateNavMsg(grx, &chan[i], 0);
            }

            // Refresh ephemeris and subframes
            // Quick and dirty fix. Need more elegant way.
            for (sv = 0; sv < MAX_SAT; sv++)
            {
                if (eph[ieph + 1][sv].vflg == 1)
                {
                    dt = subGpsTime(eph[ieph + 1][sv].toc, grx);
                    if (dt < SECONDS_IN_HOUR)
                    {
                        ieph++;

                        for (i = 0; i < MAX_CHAN; i++)
                        {
                            // Generate new subframes if allocated
                            if (chan[i].prn != 0)
                                eph2sbf(eph[ieph][chan[i].prn - 1], ionoutc, chan[i].sbf);
                        }
                    }

                    break;
                }
            }

            // Update channel allocation
            if (!staticLocationMode)
                allocateChannel(chan, eph[ieph], ionoutc, grx, xyz[iumd], elvmask);
            else
                allocateChannel(chan, eph[ieph], ionoutc, grx, xyz[0], elvmask);

            // Show details about simulated channels
            if (verb == TRUE)
            {
                fprintf(stderr, "\n");
                for (i = 0; i < MAX_CHAN; i++)
                {
                    if (chan[i].prn > 0)
                        fprintf(stderr, "%02d %6.1f %5.1f %11.1f %5.1f\n", chan[i].prn, chan[i].azel[0] * R2D, chan[i].azel[1] * R2D, chan[i].rho0.d, chan[i].rho0.iono_delay);
                }
            }
        }

        // Update receiver time
        grx = incGpsTime(grx, 0.1);

        // Update time counter
        fprintf(stderr, "\rTime into run = %4.1f", subGpsTime(grx, g0));
        fflush(stdout);
    }

    tend = clock();

    fprintf(stderr, "\nDone!\n");

    // Free I/Q buffer
    free(iq_buff);

    // Close file
    fclose(fp);
    fclose(fp1);

    // Process time
    fprintf(stderr, "Process time = %.1f [sec]\n", (double)(tend - tstart) / CLOCKS_PER_SEC);

    return (0);
}
