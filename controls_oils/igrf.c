/**
 *@file  frame.c
 *@brief Magnetic Field Estimation using the IGRF Model
*/
#include<avr/pgmspace.h>
#include "common.h"
#include "mathutil.h"
#include "igrf.h"

static float p[50], q[50];
static float cl[15], sl[15];

const static float agh[196] PROGMEM = { 0,-29442,-1501,4797.1,-2445.1,3012.9,-2845.6,1676.7,-641.9,1350.7,-2352.3,-115.3,1225.6,244.9,582,-538.4,907.6,813.7,283.3,120.4,-188.7,-334.9,180.9,70.4,-329.5,-232.6,360.1,47.3,192.4,197,-140.9,-119.3,-157.5,16,4.1,100.2,70,67.7,-20.8,72.7,33.2,-129.9,58.9,-28.9,-66.7,13.2,7.3,-70.9,62.6,81.6,-76.1,-54.1,-6.8,-19.5,51.8,5.7,15,24.4,9.4,3.4,-2.8,-27.4,6.8,-2.2,24.2,8.8,10.1,-16.9,-18.3,-3.2,13.3,-20.6,-14.6,13.4,16.2,11.7,5.7,-15.9,-9.1,-2,2.1,5.4,8.8,-21.6,3.1,10.8,-3.3,11.8,0.7,-6.8,-13.3,-6.9,-0.1,7.8,8.7,1,-9.1,-4,-10.5,8.4,-1.9,-6.3,3.2,0.1,-0.4,0.5,4.6,-0.5,4.4,1.8,-7.9,-0.7,-0.6,2.1,-4.2,2.4,-2.8,-1.8,-1.2,-3.6,-8.7,3.1,-1.5,-0.1,-2.3,2,2,-0.7,-0.8,-1.1,0.6,0.8,-0.7,-0.2,0.2,-2.2,1.7,-1.4,-0.2,-2.5,0.4,-2,3.5,-2.4,-1.9,-0.2,-1.1,0.4,0.4,1.2,1.9,-0.8,-2.2,0.9,0.3,0.1,0.7,0.5,-0.1,-0.3,0.3,-0.4,0.2,0.2,-0.9,-0.9,-0.1,0,0.7,0,-0.9,-0.9,0.4,0.4,0.5,1.6,-0.5,-0.5,1,-1.2,-0.2,-0.1,0.8,0.4,-0.1,-0.1,0.3,0.4,0.1,0.5,0.5,-0.3,-0.4,-0.4,-0.3,-0.8};
	

/*-2.949650e+04, -1.585900e+03, 4.945100e+03, -2.396600e+03, 3.026000e+03,
 -2.707700e+03, 1.668600e+03, -5.754000e+02, 1.339700e+03, -2.326300e+03,
 -1.605000e+02, 1.231700e+03, 2.517000e+02, 6.342000e+02, -5.368000e+02,
 9.126000e+02, 8.090000e+02, 2.864000e+02, 1.666000e+02, -2.112000e+02,
 -3.571000e+02, 1.644000e+02, 8.970000e+01, -3.092000e+02, -2.311000e+02,
 3.572000e+02, 4.470000e+01, 2.003000e+02, 1.889000e+02, -1.412000e+02,
 -1.181000e+02, -1.631000e+02, 1.000000e-01, -7.700000e+00, 1.009000e+02,
 7.280000e+01, 6.860000e+01, -2.080000e+01, 7.600000e+01, 4.420000e+01,
 -1.414000e+02, 6.150000e+01, -2.290000e+01, -6.630000e+01, 1.310000e+01,
 3.100000e+00, -7.790000e+01, 5.490000e+01, 8.040000e+01, -7.500000e+01,
 -5.780000e+01, -4.700000e+00, -2.120000e+01, 4.530000e+01, 6.600000e+00,
 1.400000e+01, 2.490000e+01, 1.040000e+01, 7.000000e+00, 1.600000e+00,
 -2.770000e+01, 4.900000e+00, -3.400000e+00, 2.430000e+01, 8.200000e+00,
 1.090000e+01, -1.450000e+01, -2.000000e+01, -5.700000e+00, 1.190000e+01,
 -1.930000e+01, -1.740000e+01, 1.160000e+01, 1.670000e+01, 1.090000e+01,
 7.100000e+00, -1.410000e+01, -1.080000e+01, -3.700000e+00, 1.700000e+00,
 5.400000e+00, 9.400000e+00, -2.050000e+01, 3.400000e+00, 1.160000e+01,
 -5.300000e+00, 1.280000e+01, 3.100000e+00, -7.200000e+00, -1.240000e+01,
 -7.400000e+00, -8.000000e-01, 8.000000e+00, 8.400000e+00, 2.200000e+00,
 -8.400000e+00, -6.100000e+00, -1.010000e+01, 7.000000e+00, -2.000000e+00,
 -6.300000e+00, 2.800000e+00, 9.000000e-01, -1.000000e-01, -1.100000e+00,
 4.700000e+00, -2.000000e-01, 4.400000e+00, 2.500000e+00, -7.200000e+00,
 -3.000000e-01, -1.000000e+00, 2.200000e+00, -4.000000e+00, 3.100000e+00,
 -2.000000e+00, -1.000000e+00, -2.000000e+00, -2.800000e+00, -8.300000e+00,
 3.000000e+00, -1.500000e+00, 1.000000e-01, -2.100000e+00, 1.700000e+00,
 1.600000e+00, -6.000000e-01, -5.000000e-01, -1.800000e+00, 5.000000e-01,
 9.000000e-01, -8.000000e-01, -4.000000e-01, 4.000000e-01, -2.500000e+00,
 1.800000e+00, -1.300000e+00, 2.000000e-01, -2.100000e+00, 8.000000e-01,
 -1.900000e+00, 3.800000e+00, -1.800000e+00, -2.100000e+00, -2.000000e-01,
 -8.000000e-01, 3.000000e-01, 3.000000e-01, 1.000000e+00, 2.200000e+00,
 -7.000000e-01, -2.500000e+00, 9.000000e-01, 5.000000e-01, -1.000000e-01,
 6.000000e-01, 5.000000e-01, 0.000000e+00, -4.000000e-01, 1.000000e-01,
 -4.000000e-01, 3.000000e-01, 2.000000e-01, -9.000000e-01, -8.000000e-01,
 -2.000000e-01, 0.000000e+00, 8.000000e-01, -2.000000e-01, -9.000000e-01,
 -8.000000e-01, 3.000000e-01, 3.000000e-01, 4.000000e-01, 1.700000e+00,
 -4.000000e-01, -6.000000e-01, 1.100000e+00, -1.200000e+00, -3.000000e-01,
 -1.000000e-01, 8.000000e-01, 5.000000e-01, -2.000000e-01, 1.000000e-01,
 4.000000e-01, 5.000000e-01, 0.000000e+00, 4.000000e-01, 4.000000e-01,
 -2.000000e-01, -3.000000e-01, -5.000000e-01, -3.000000e-01, -8.000000e-01 };
 */
const static float dgh[196] PROGMEM = { 0,10.3,18.1,-26.6,-8.7,-3.3,-27.4,2.1,-14.1,3.4,-5.5,8.2,-0.7,-0.4,-10.1,1.8,-0.7,0.2,-1.3,-9.1,5.3,4.1,2.9,-4.3,-5.2,-0.2,0.5,0.6,-1.3,1.7,-0.1,-1.2,1.4,3.4,3.9,0,-0.3,-0.1,0,-0.7,-2.1,2.1,-0.7,-1.2,0.2,0.3,0.9,1.6,1,0.3,-0.2,0.8,-0.5,0.4,1.3,-0.2,0.1,-0.3,-0.6,-0.6,-0.8,0.1,0.2,-0.2,0.2,0,-0.3,-0.6,0.3,0.5,0.1,-0.2,0.5,0.4,-0.2,0.1,-0.3,-0.4,0.3,0.3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/* 1.140000e+01, 1.670000e+01, -2.880000e+01, -1.130000e+01, -3.900000e+00,
 -2.300000e+01, 2.700000e+00, -1.290000e+01, 1.300000e+00, -3.900000e+00,
 8.600000e+00, -2.900000e+00, -2.900000e+00, -8.100000e+00, -2.100000e+00,
 -1.400000e+00, 2.000000e+00, 4.000000e-01, -8.900000e+00, 3.200000e+00,
 4.400000e+00, 3.600000e+00, -2.300000e+00, -8.000000e-01, -5.000000e-01,
 5.000000e-01, 5.000000e-01, -1.500000e+00, 1.500000e+00, -7.000000e-01,
 9.000000e-01, 1.300000e+00, 3.700000e+00, 1.400000e+00, -6.000000e-01,
 -3.000000e-01, -3.000000e-01, -1.000000e-01, -3.000000e-01, -2.100000e+00,
 1.900000e+00, -4.000000e-01, -1.600000e+00, -5.000000e-01, -2.000000e-01,
 8.000000e-01, 1.800000e+00, 5.000000e-01, 2.000000e-01, -1.000000e-01,
 6.000000e-01, -6.000000e-01, 3.000000e-01, 1.400000e+00, -2.000000e-01,
 3.000000e-01, -1.000000e-01, 1.000000e-01, -8.000000e-01, -8.000000e-01,
 -3.000000e-01, 4.000000e-01, 2.000000e-01, -1.000000e-01, 1.000000e-01,
 0.000000e+00, -5.000000e-01, 2.000000e-01, 3.000000e-01, 5.000000e-01,
 -3.000000e-01, 4.000000e-01, 3.000000e-01, 1.000000e-01, 2.000000e-01,
 -1.000000e-01, -5.000000e-01, 4.000000e-01, 2.000000e-01, 4.000000e-01,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00,
 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 };
*/
void igrf(vector v_lla, float years, uint8_t order, vector v_B_ned)
{
  float lat = v_lla[0], lon = v_lla[1], alt = v_lla[2] / 1000;
  float x = 0.0, y = 0.0, z = 0.0, one, two, three, four;
  float slat = sin(lat), clat = cos(lat), cd, sd;
  float ratio, r, rr = 0.0, t = years - IGRF_YEAR;
  float agh_p, dgh_p;
  uint8_t l = 1, m = 1, n = 0, max, k, fn = 0, fm;
  
  cl[1] = cos(lon);
  sl[1] = sin(lon);
  one = A2 * clat * clat;
  two = B2 * slat * slat;
  three = one + two;
  four = sqrt(three);
  r = sqrt(alt * (alt + 2.0 * four) + (A2 * one + B2 * two)/three);
  
  one = slat;
  cd = (alt + four) / r;
  sd = ((A2 - B2) * slat * clat) / (four * r);
  slat = slat * cd - clat * sd;
  clat = clat * cd + one * sd;
  
  p[1] = 2.0 * slat;
  p[2] = 2.0 * clat;
  p[3] = 4.5 * slat * slat - 1.5;
  p[4] = sqrt(27) * clat * slat;
  q[1] = -1 * clat;
  q[2] = slat;
  q[3] = -3.0 * clat * slat;
  q[4] = sqrt(3) * (slat * slat - clat * clat);
  
  ratio = RE / r;
  
  max = (order * (order + 3)) / 2;
  for(k = 1; k <= max; k++)
  {
    if(n < m)
    {
      m = 0;
      n++;
      rr = pow(ratio, n + 2);
      fn = n;
    }
    fm = m;
    if(k >= 5)
    {
      if(m == n)
      {
        one = sqrt(1 - (0.5 / fm));
        p[k] = (1 + (1.0/fm)) * one * clat * p[k - n - 1];
        q[k] = one * (clat * q[k - n - 1] + (slat * p[k - n - 1] ) / fm);
        sl[m] = sl[m - 1] * cl[1] + cl[m - 1] * sl[1];
        cl[m] = cl[m - 1] * cl[1] - sl[m - 1] * sl[1];
      }
      else
      {
        one = sqrt(fn * fn - fm * fm);
        two = sqrt(pow(fn - 1.0, 2) - fm * fm) / one;
        three = (2.0 * fn - 1.0) / one;
        p[k]  = (fn + 1.0) * ((three * slat * p[k - n]) / fn  - (two * p[k - 2 * n + 1])/(fn - 1.0));
        q[k]  = three * (slat * q[k - n] - clat * p[k - n] / fn) - two * q[k - 2 * n + 1];
      }
    }
    
    agh_p = pgm_read_float(&agh[l]);
    dgh_p = pgm_read_float(&dgh[l]);
    one = (agh_p + dgh_p * t) * rr;
    
    
    if(m == 0)
    {
      x += one * q[k];
      z -= one * p[k];
      l++;
    }
    else
    {
      agh_p = pgm_read_float(&agh[l + 1]);
      dgh_p = pgm_read_float(&dgh[l + 1]);
      two = (agh_p + dgh_p * t) * rr;
      three = one * cl[m] + two * sl[m];
      x += three * q[k];
      z -= three * p[k];
      
      if(clat > 0)
        y += ((one * sl[m] - two * cl[m]) * fm * p[k]) / ((fn + 1.0) * clat);
      else
        y += (one * sl[m] - two * cl[m]) * q[k] * slat;
        
      l += 2;
    }
    m++;
  }
  
  one = x;
  x = x * cd + z * sd;
  z = z * cd - one * sd;
  
  v_B_ned[0] = x;
  v_B_ned[1] = y;
  v_B_ned[2] = z;
     
}
