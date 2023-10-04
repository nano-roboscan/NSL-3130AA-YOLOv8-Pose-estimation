/*
 * Copyright (c) 2013, NANOSYSTEMS CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "lens_transform.h"
#include <math.h>

#define LENS_TABLE_SIZE			46

typedef struct lensTransformData_{
    double angle[LENS_TABLE_SIZE];
    double rp[LENS_TABLE_SIZE];

    double xUA[320][240];
    double yUA[320][240];
    double zUA[320][240];
}lensTransformData;

lensTransformData lensData;

LensTransform::LensTransform()
{
}

void LensTransform::initLensDistortionTable(enum LensType nLensType)
{
	width2 = 160;
	height2 = 120;
	lensTableSize = LENS_TABLE_SIZE;
	lensType = nLensType;
	lensKoef = 1.0;
	//loadKoefFile(lensPath);

	if(lensType == WIDE_FIELD)
	{
		lensData.angle[ 0	] = 	0	;
		lensData.angle[ 1	] = 	1.677055033 ;
		lensData.angle[ 2	] = 	3.354110067 ;
		lensData.angle[ 3	] = 	5.0311651	;
		lensData.angle[ 4	] = 	6.708220134 ;
		lensData.angle[ 5	] = 	8.385275167 ;
		lensData.angle[ 6	] = 	10.0623302	;
		lensData.angle[ 7	] = 	11.73938523 ;
		lensData.angle[ 8	] = 	13.41644027 ;
		lensData.angle[ 9	] = 	15.0934953	;
		lensData.angle[ 10	] = 	16.77055033 ;
		lensData.angle[ 11	] = 	18.44760537 ;
		lensData.angle[ 12	] = 	20.1246604	;
		lensData.angle[ 13	] = 	21.80171543 ;
		lensData.angle[ 14	] = 	23.47877047 ;
		lensData.angle[ 15	] = 	25.1558255	;
		lensData.angle[ 16	] = 	26.83288053 ;
		lensData.angle[ 17	] = 	28.50993557 ;
		lensData.angle[ 18	] = 	30.1869906	;
		lensData.angle[ 19	] = 	31.86404563 ;
		lensData.angle[ 20	] = 	33.54110067 ;
		lensData.angle[ 21	] = 	35.2181557	;
		lensData.angle[ 22	] = 	36.89521074 ;
		lensData.angle[ 23	] = 	38.57226577 ;
		lensData.angle[ 24	] = 	40.2493208	;
		lensData.angle[ 25	] = 	41.92637584 ;
		lensData.angle[ 26	] = 	43.60343087 ;
		lensData.angle[ 27	] = 	45.2804859	;
		lensData.angle[ 28	] = 	46.95754094 ;
		lensData.angle[ 29	] = 	48.63459597 ;
		lensData.angle[ 30	] = 	50.311651	;
		lensData.angle[ 31	] = 	51.98870604 ;
		lensData.angle[ 32	] = 	53.66576107 ;
		lensData.angle[ 33	] = 	55.3428161	;
		lensData.angle[ 34	] = 	57.01987114 ;
		lensData.angle[ 35	] = 	58.69692617 ;
		lensData.angle[ 36	] = 	60.3739812	;
		lensData.angle[ 37	] = 	62.05103624 ;
		lensData.angle[ 38	] = 	63.72809127 ;
		lensData.angle[ 39	] = 	65.4051463	;
		lensData.angle[ 40	] = 	67.08220134 ;
		lensData.angle[ 41	] = 	68.75925637 ;
		lensData.angle[ 42	] = 	70.4363114	;
		lensData.angle[ 43	] = 	72.11336644 ;
		lensData.angle[ 44	] = 	73.79042147 ;
		lensData.angle[ 45	] = 	75.4674765	;
		
		
		//size mm
		lensData.rp[	0	] = 	0	;
		lensData.rp[	1	] = 	0.1 ;
		lensData.rp[	2	] = 	0.2 ;
		lensData.rp[	3	] = 	0.3 ;
		lensData.rp[	4	] = 	0.4 ;
		lensData.rp[	5	] = 	0.5 ;
		lensData.rp[	6	] = 	0.6 ;
		lensData.rp[	7	] = 	0.7 ;
		lensData.rp[	8	] = 	0.8 ;
		lensData.rp[	9	] = 	0.9 ;
		lensData.rp[	10	] = 	1	;
		lensData.rp[	11	] = 	1.1 ;
		lensData.rp[	12	] = 	1.2 ;
		lensData.rp[	13	] = 	1.3 ;
		lensData.rp[	14	] = 	1.4 ;
		lensData.rp[	15	] = 	1.5 ;
		lensData.rp[	16	] = 	1.6 ;
		lensData.rp[	17	] = 	1.7 ;
		lensData.rp[	18	] = 	1.8 ;
		lensData.rp[	19	] = 	1.9 ;
		lensData.rp[	20	] = 	2	;
		lensData.rp[	21	] = 	2.1 ;
		lensData.rp[	22	] = 	2.2 ;
		lensData.rp[	23	] = 	2.3 ;
		lensData.rp[	24	] = 	2.4 ;
		lensData.rp[	25	] = 	2.5 ;
		lensData.rp[	26	] = 	2.6 ;
		lensData.rp[	27	] = 	2.7 ;
		lensData.rp[	28	] = 	2.8 ;
		lensData.rp[	29	] = 	2.9 ;
		lensData.rp[	30	] = 	3	;
		lensData.rp[	31	] = 	3.1 ;
		lensData.rp[	32	] = 	3.2 ;
		lensData.rp[	33	] = 	3.3 ;
		lensData.rp[	34	] = 	3.4 ;
		lensData.rp[	35	] = 	3.5 ;
		lensData.rp[	36	] = 	3.6 ;
		lensData.rp[	37	] = 	3.7 ;
		lensData.rp[	38	] = 	3.8 ;
		lensData.rp[	39	] = 	3.9 ;
		lensData.rp[	40	] = 	4	;
		lensData.rp[	41	] = 	4.1 ;
		lensData.rp[	42	] = 	4.2 ;
		lensData.rp[	43	] = 	4.3 ;
		lensData.rp[	44	] = 	4.4 ;
		lensData.rp[	45	] = 	4.5 ;

	}
	else if(lensType == STANDARD_FIELD)
	{
		//===========Standard field ==========
		lensData.angle[ 0	] = 	0	;
		lensData.angle[ 1	] = 	1.391769226 ;
		lensData.angle[ 2	] = 	2.783538452 ;
		lensData.angle[ 3	] = 	4.175307678 ;
		lensData.angle[ 4	] = 	5.567076905 ;
		lensData.angle[ 5	] = 	6.958846131 ;
		lensData.angle[ 6	] = 	8.350615357 ;
		lensData.angle[ 7	] = 	9.742384583 ;
		lensData.angle[ 8	] = 	11.13415381 ;
		lensData.angle[ 9	] = 	12.52592304 ;
		lensData.angle[ 10	] = 	13.91769226 ;
		lensData.angle[ 11	] = 	15.30946149 ;
		lensData.angle[ 12	] = 	16.70123071 ;
		lensData.angle[ 13	] = 	18.09299994 ;
		lensData.angle[ 14	] = 	19.48476917 ;
		lensData.angle[ 15	] = 	20.87653839 ;
		lensData.angle[ 16	] = 	22.26830762 ;
		lensData.angle[ 17	] = 	23.66007684 ;
		lensData.angle[ 18	] = 	25.05184607 ;
		lensData.angle[ 19	] = 	26.4436153	;
		lensData.angle[ 20	] = 	27.83538452 ;
		lensData.angle[ 21	] = 	29.22715375 ;
		lensData.angle[ 22	] = 	30.61892298 ;
		lensData.angle[ 23	] = 	32.0106922	;
		lensData.angle[ 24	] = 	33.40246143 ;
		lensData.angle[ 25	] = 	34.79423065 ;
		lensData.angle[ 26	] = 	36.18599988 ;
		lensData.angle[ 27	] = 	37.57776911 ;
		lensData.angle[ 28	] = 	38.96953833 ;
		lensData.angle[ 29	] = 	40.36130756 ;
		lensData.angle[ 30	] = 	41.75307678 ;
		lensData.angle[ 31	] = 	43.14484601 ;
		lensData.angle[ 32	] = 	44.53661524 ;
		lensData.angle[ 33	] = 	45.92838446 ;
		lensData.angle[ 34	] = 	47.32015369 ;
		lensData.angle[ 35	] = 	48.71192292 ;
		lensData.angle[ 36	] = 	50.10369214 ;
		lensData.angle[ 37	] = 	51.49546137 ;
		lensData.angle[ 38	] = 	52.88723059 ;
		lensData.angle[ 39	] = 	54.27899982 ;
		lensData.angle[ 40	] = 	55.67076905 ;
		lensData.angle[ 41	] = 	57.06253827 ;
		lensData.angle[ 42	] = 	58.4543075	;
		lensData.angle[ 43	] = 	59.84607672 ;
		lensData.angle[ 44	] = 	61.23784595 ;
		lensData.angle[ 45	] = 	62.62961518 ;
		
		//size mm
		lensData.rp[	0	] = 	0	;
		lensData.rp[	1	] = 	0.1 ;
		lensData.rp[	2	] = 	0.2 ;
		lensData.rp[	3	] = 	0.3 ;
		lensData.rp[	4	] = 	0.4 ;
		lensData.rp[	5	] = 	0.5 ;
		lensData.rp[	6	] = 	0.6 ;
		lensData.rp[	7	] = 	0.7 ;
		lensData.rp[	8	] = 	0.8 ;
		lensData.rp[	9	] = 	0.9 ;
		lensData.rp[	10	] = 	1	;
		lensData.rp[	11	] = 	1.1 ;
		lensData.rp[	12	] = 	1.2 ;
		lensData.rp[	13	] = 	1.3 ;
		lensData.rp[	14	] = 	1.4 ;
		lensData.rp[	15	] = 	1.5 ;
		lensData.rp[	16	] = 	1.6 ;
		lensData.rp[	17	] = 	1.7 ;
		lensData.rp[	18	] = 	1.8 ;
		lensData.rp[	19	] = 	1.9 ;
		lensData.rp[	20	] = 	2	;
		lensData.rp[	21	] = 	2.1 ;
		lensData.rp[	22	] = 	2.2 ;
		lensData.rp[	23	] = 	2.3 ;
		lensData.rp[	24	] = 	2.4 ;
		lensData.rp[	25	] = 	2.5 ;
		lensData.rp[	26	] = 	2.6 ;
		lensData.rp[	27	] = 	2.7 ;
		lensData.rp[	28	] = 	2.8 ;
		lensData.rp[	29	] = 	2.9 ;
		lensData.rp[	30	] = 	3	;
		lensData.rp[	31	] = 	3.1 ;
		lensData.rp[	32	] = 	3.2 ;
		lensData.rp[	33	] = 	3.3 ;
		lensData.rp[	34	] = 	3.4 ;
		lensData.rp[	35	] = 	3.5 ;
		lensData.rp[	36	] = 	3.6 ;
		lensData.rp[	37	] = 	3.7 ;
		lensData.rp[	38	] = 	3.8 ;
		lensData.rp[	39	] = 	3.9 ;
		lensData.rp[	40	] = 	4	;
		lensData.rp[	41	] = 	4.1 ;
		lensData.rp[	42	] = 	4.2 ;
		lensData.rp[	43	] = 	4.3 ;
		lensData.rp[	44	] = 	4.4 ;
		lensData.rp[	45	] = 	4.5 ;

	}
	else{
		//==== Narow field ======
		lensData.angle[ 0	] = 	0	;
		lensData.angle[ 1	] = 	0.775	;
		lensData.angle[ 2	] = 	1.55	;
		lensData.angle[ 3	] = 	2.325	;
		lensData.angle[ 4	] = 	3.1 ;
		lensData.angle[ 5	] = 	3.875	;
		lensData.angle[ 6	] = 	4.65	;
		lensData.angle[ 7	] = 	5.425	;
		lensData.angle[ 8	] = 	6.2 ;
		lensData.angle[ 9	] = 	6.975	;
		lensData.angle[ 10	] = 	7.75	;
		lensData.angle[ 11	] = 	8.525	;
		lensData.angle[ 12	] = 	9.3 ;
		lensData.angle[ 13	] = 	10.075	;
		lensData.angle[ 14	] = 	10.85	;
		lensData.angle[ 15	] = 	11.625	;
		lensData.angle[ 16	] = 	12.4	;
		lensData.angle[ 17	] = 	13.175	;
		lensData.angle[ 18	] = 	13.95	;
		lensData.angle[ 19	] = 	14.725	;
		lensData.angle[ 20	] = 	15.5	;
		lensData.angle[ 21	] = 	16.275	;
		lensData.angle[ 22	] = 	17.05	;
		lensData.angle[ 23	] = 	17.825	;
		lensData.angle[ 24	] = 	18.6	;
		lensData.angle[ 25	] = 	19.375	;
		lensData.angle[ 26	] = 	20.15	;
		lensData.angle[ 27	] = 	20.925	;
		lensData.angle[ 28	] = 	21.7	;
		lensData.angle[ 29	] = 	22.475	;
		lensData.angle[ 30	] = 	23.25	;
		lensData.angle[ 31	] = 	24.025	;
		lensData.angle[ 32	] = 	24.8	;
		lensData.angle[ 33	] = 	25.575	;
		lensData.angle[ 34	] = 	26.35	;
		lensData.angle[ 35	] = 	27.125	;
		lensData.angle[ 36	] = 	27.9	;
		lensData.angle[ 37	] = 	28.675	;
		lensData.angle[ 38	] = 	29.45	;
		lensData.angle[ 39	] = 	30.225	;
		lensData.angle[ 40	] = 	31	;
		lensData.angle[ 41	] = 	31.775	;
		lensData.angle[ 42	] = 	32.55	;
		lensData.angle[ 43	] = 	33.325	;
		lensData.angle[ 44	] = 	34.1	;
		lensData.angle[ 45	] = 	34.875	;
				
		//size mm
		lensData.rp[	0	] = 	0	;
		lensData.rp[	1	] = 	0.1 ;
		lensData.rp[	2	] = 	0.2 ;
		lensData.rp[	3	] = 	0.3 ;
		lensData.rp[	4	] = 	0.4 ;
		lensData.rp[	5	] = 	0.5 ;
		lensData.rp[	6	] = 	0.6 ;
		lensData.rp[	7	] = 	0.7 ;
		lensData.rp[	8	] = 	0.8 ;
		lensData.rp[	9	] = 	0.9 ;
		lensData.rp[	10	] = 	1	;
		lensData.rp[	11	] = 	1.1 ;
		lensData.rp[	12	] = 	1.2 ;
		lensData.rp[	13	] = 	1.3 ;
		lensData.rp[	14	] = 	1.4 ;
		lensData.rp[	15	] = 	1.5 ;
		lensData.rp[	16	] = 	1.6 ;
		lensData.rp[	17	] = 	1.7 ;
		lensData.rp[	18	] = 	1.8 ;
		lensData.rp[	19	] = 	1.9 ;
		lensData.rp[	20	] = 	2	;
		lensData.rp[	21	] = 	2.1 ;
		lensData.rp[	22	] = 	2.2 ;
		lensData.rp[	23	] = 	2.3 ;
		lensData.rp[	24	] = 	2.4 ;
		lensData.rp[	25	] = 	2.5 ;
		lensData.rp[	26	] = 	2.6 ;
		lensData.rp[	27	] = 	2.7 ;
		lensData.rp[	28	] = 	2.8 ;
		lensData.rp[	29	] = 	2.9 ;
		lensData.rp[	30	] = 	3	;
		lensData.rp[	31	] = 	3.1 ;
		lensData.rp[	32	] = 	3.2 ;
		lensData.rp[	33	] = 	3.3 ;
		lensData.rp[	34	] = 	3.4 ;
		lensData.rp[	35	] = 	3.5 ;
		lensData.rp[	36	] = 	3.6 ;
		lensData.rp[	37	] = 	3.7 ;
		lensData.rp[	38	] = 	3.8 ;
		lensData.rp[	39	] = 	3.9 ;
		lensData.rp[	40	] = 	4	;
		lensData.rp[	41	] = 	4.1 ;
		lensData.rp[	42	] = 	4.2 ;
		lensData.rp[	43	] = 	4.3 ;
		lensData.rp[	44	] = 	4.4 ;
		lensData.rp[	45	] = 	4.5 ;
	}

	for(int i=0; i<lensTableSize; i++)
		lensData.angle[i] *= lensKoef;

	initTransformTable(0.02, 320, 240, 0, 0);

}



double LensTransform::interpolate(double x_in, double x0, double y0, double x1, double y1)
{
    return ((x_in-x0)*(y1-y0)/(x1-x0) + y0);
}

double LensTransform::getAngle(double x, double y, double sensorPointSizeMM)
{
    double radius = sensorPointSizeMM * sqrt(x*x + y*y);
    double alfaGrad = 0;

    for(int i=1; i<lensTableSize; i++)
    {
        if(radius >= lensData.rp[i-1] && radius <= lensData.rp[i]){

            alfaGrad = interpolate(radius, lensData.rp[i-1], lensData.angle[i-1], lensData.rp[i], lensData.angle[i]);
        }
    }

    return alfaGrad;
}


void LensTransform::transformPixel(int srcX, int srcY, double srcZ, double &destX, double &destY, double &destZ, double sin_angle, double cos_angle)
{        
    if(sin_angle == 0.0 ){
        destY = srcZ * lensData.yUA[srcX][srcY];
        destX = srcZ * lensData.xUA[srcX][srcY];
        destZ = srcZ * lensData.zUA[srcX][srcY];
    }else{

        double y = srcZ * lensData.yUA[srcX][srcY];
        double z = srcZ * lensData.zUA[srcX][srcY];
        destX    = srcZ * lensData.xUA[srcX][srcY];
        destY = z * sin_angle + y * cos_angle;
        destZ = z * cos_angle - y * sin_angle;
    }
}

void LensTransform::transformPixelTable(unsigned int srcX, unsigned int srcY, double srcZ, double &destX, double &destY, double &destZ, double sin_angle, double cos_angle)
{
    if(sin_angle == 0.0 ){
        destY = srcZ * lensData.yUA[srcX][srcY];
        destX = srcZ * lensData.xUA[srcX][srcY];
        destZ = srcZ * lensData.zUA[srcX][srcY];
    }else{

        double y = srcZ * lensData.yUA[srcX][srcY];
        double z = srcZ * lensData.zUA[srcX][srcY];
        destX    = srcZ * lensData.xUA[srcX][srcY];
        destY = z * sin_angle + y * cos_angle;
        destZ = z * cos_angle - y * sin_angle;
    }
}

void LensTransform::transformPixelPolynom(unsigned int srcX, unsigned int srcY, double srcZ, double &destX, double &destY, double &destZ, double sin_angle, double cos_angle)
{
    if(sin_angle == 0.0 ){
		destY = srcZ  * lensData.yUA[srcX][srcY];
        destX = destY * lensData.xUA[srcX][srcY];
        destZ = srcZ  * lensData.zUA[srcX][srcY];
    }else{
        double y = srcZ  * lensData.yUA[srcX][srcY];
        double z = srcZ  * lensData.zUA[srcX][srcY];
        destX    = destY * lensData.xUA[srcX][srcY];
        destY = z * sin_angle + y * cos_angle;
        destZ = z * cos_angle - y * sin_angle;
    }
}

void LensTransform::transformPixelCalib(unsigned int srcX, unsigned int srcY, double srcZ, double &destX, double &destY, double &destZ, double sin_angle, double cos_angle)
{

    if(sin_angle == 0.0 ){
        destY = srcZ * lensData.yUA[srcX][srcY];
        destX = srcZ * lensData.xUA[srcX][srcY];
        destZ = srcZ * lensData.zUA[srcX][srcY];

    }else{

        double y = srcZ * lensData.yUA[srcX][srcY];
        double z = srcZ * lensData.zUA[srcX][srcY];
        destX    = srcZ * lensData.xUA[srcX][srcY];
        destY = z * sin_angle + y * cos_angle;
        destZ = z * cos_angle - y * sin_angle;
    }
}

void LensTransform::initTransformTable(double sensorPointSizeMM, int width, int height, int offsetX, int offsetY)
{
    int x, y, row, col;
    numCols = width;
    numRows = height;

    int r0 = 1 - numRows/2 + offsetY; //lens optical center offset
    int c0 = 1 - numCols/2 + offsetX;

    for(y=0, row = r0; y < numRows; row++, y++){
        for(x=0, col = c0; x < numCols; col++, x++){

            double c = col - 0.5;
            double r = row - 0.5;

            double angleGrad = getAngle(c, r, sensorPointSizeMM);
            double angleRad =  angleGrad * 3.14159265359 / 180.0;

            double rp = sqrt(c * c + r * r);
            double rUA = sin(angleRad);

            lensData.xUA[x][y] = c * rUA / rp;
            lensData.yUA[x][y] = r * rUA / rp;
            lensData.zUA[x][y] = cos(angleRad);
        }
    }

}

void LensTransform::initTransformPolynom(double sensorPointSizeMM, int width, int height)
{
    numRows = height;
    numCols = width;
    int x, y, row, col;
    int r0 = 1 - numRows/2; //lens optical center offset
    int c0 = 1 - numCols/2;

    for(y=0, row = r0; y < numRows; row++, y++){
        for(x=0, col = c0; x < numCols; col++, x++){

            double c = (col - 0.5) * sensorPointSizeMM;
            double r = (row - 0.5) * sensorPointSizeMM;
            double rp = sqrt(r * r + c * c);
            double sin_teta = -0.0114 * rp * rp + 0.287 * rp;            
            double k = c/r;
            double k2 = (c*c)/(r*r);

            lensData.xUA[x][y] = k;

            if(r < 0) lensData.yUA[x][y] = -sin_teta / sqrt(k2+1);
            else lensData.yUA[x][y] = sin_teta / sqrt(k2+1);

            lensData.zUA[x][y] = sqrt(1 - sin_teta * sin_teta);
        }
    }
}


void LensTransform::initTransformNarrow(double sensorPointSizeMM, int width, int height, int offsetX, int offsetY)
{
	int x, y, row, col;
	numCols = width;
	numRows = height;

	int r0 = 1 - numRows/2 + offsetY; //lens optical center offset
	int c0 = 1 - numCols/2 + offsetX;

	for(y=0, row = r0; y < numRows; row++, y++){
		for(x=0, col = c0; x < numCols; col++, x++){

			double c = col - 0.5;
			double r = row - 0.5;

			double angleGrad = getAngle(c, r, sensorPointSizeMM);
			double angleRad =  angleGrad * 3.14159265359 / 180.0;

			double rp = sqrt(c * c + r * r);
			double rUA = sin(angleRad);

			lensData.xUA[x][y] = c * rUA / rp;
			lensData.yUA[x][y] = r * rUA / rp;
			lensData.zUA[x][y] = cos(angleRad);
		}
	}

}




void LensTransform::setLensData(int index, double rp, double angle)
{
    lensData.rp[index] = rp;
    lensData.angle[index] = angle;
}

bool LensTransform::isTransformLoaded()
{
    if(lensTableSize == 0) return false;
    else return true;
}


#if 0
void LensTransform::calibrateLensData(int* data)
{
    int x,y,l;
    int col, row;
    int r0 = 1 - numRows/2;
    int c0 = 1 - numCols/2;

    double sensorPointSizeMM = 0.02;
    double z1 = data[119 * 320 + 159];
    double z2 = data[120 * 320 + 159];
    double z3 = data[119 * 320 + 160];
    double z4 = data[120 * 320 + 160];
    double z = (z1 + z2 + z3 + z4) / 4;

    for(l=0, y=0, row = r0; y < numRows; row++, y++){
        for(x=0, col = c0; x < numCols; col++, x++, l++){

            double c = (col - 0.5) * sensorPointSizeMM;
            double r = (row - 0.5) * sensorPointSizeMM;
            double rap = sqrt(r * r + c * c);
            double dist = data[l];
            double rUA = sqrt(dist * dist  - z * z) / dist;

            lensData.xUA[x][y] = c * rUA / rap;
            lensData.yUA[x][y] = r * rUA / rap;
            lensData.zUA[x][y] = z / dist;
        }
    }

    saveKoefFile();
}


void LensTransform::saveKoefFile(void)
{
#ifdef Q_OS_MAC
    path = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + "/lensData";
#else
    path = QDir::currentPath() + "/lensData";
#endif

    QDir directory(path);

    if(!directory.exists()){ //if do not exist create it
        if(!directory.mkpath(path))
            return;
    }

    path += "/lens.cof";

    QFile file(path);
    file.open( QIODevice::WriteOnly );
    QTextStream out(&file);

    for(int y=0; y< 240; y++)
    {
        for(int x=0; x<320; x++)
        {
            if(isnan(xUAC[x][y]) || isnan(yUAC[x][y]) || isnan(zUAC[x][y])){
                out << 0.0 << "  " << 0.0 << "  " << 0.0 << "\n";
            }else{
                out << xUAC[x][y] << "  " << yUAC[x][y] << "  " << zUAC[x][y] << "\n";
            }
            //qDebug() << lensData.xUA[x][y] << lensData.yUA[x][y] << lensData.zUA[x][y]; //TODO remove
        }

    }
}

void LensTransform::loadKoefFile(void)
{
#ifdef Q_OS_MAC
    path = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + "/lensData/lens.cof";
#else
    path = QDir::currentPath() + "/lensData/lens.cof";
#endif

    QFile file(path);
    if(!file.open( QIODevice::ReadOnly ))
        return;

    QTextStream in(&file);

    for(int y=0; y< 240; y++)
    {
        for(int x=0; x<320; x++)
        {
            in >> xUAC[x][y];
            in >> yUAC[x][y];
            in >> zUAC[x][y];

			qDebug() << lensData.xUA[x][y]<<" " << lensData.yUA[x][y]<<" " << lensData.zUA[x][y]; //TODO remove
        }
    }
}
#endif














