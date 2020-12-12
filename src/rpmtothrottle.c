/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: rpmtothrottle.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 11-Dec-2020 01:16:32
 */

/* Include Files */
#include "rpmtothrottle.h"
#include "interp1.h"
#include <math.h>
#include <string.h>

/* Function Definitions */

/*
 * #codejen
 * Arguments    : float rpm
 *                const float V_rel_B[3]
 *                float *throttle
 *                float *Tmax
 * Return Type  : void
 */
void rpmtothrottle(float rpm, const float V_rel_B[3], float *throttle, float
                   *Tmax)
{
  float b_y1;
  float mu;
  float lamb;
  float alpha;
  int mid_i;
  float data[360];
  int low_ip1;
  int high_i;
  static const float fv[90] = { 0.0F, 1889.3F, 3007.1F, 3864.5F, 4861.4F,
    5710.6F, 6748.5F, 7530.9F, 8351.3F, 0.0F, 1889.3F, 3007.1F, 3864.5F, 4861.4F,
    5710.6F, 6748.5F, 7530.9F, 8351.3F, 0.0F, 1889.3F, 3007.1F, 3864.5F, 4861.4F,
    5710.6F, 6748.5F, 7530.9F, 8351.3F, 0.0F, 1889.3F, 3007.1F, 3864.5F, 4861.4F,
    5710.6F, 6748.5F, 7530.9F, 8351.3F, 0.0F, 1889.3F, 3007.1F, 3864.5F, 4861.4F,
    5710.6F, 6748.5F, 7530.9F, 8351.3F, 0.0F, 1889.3F, 3007.1F, 3864.5F, 4861.4F,
    5710.6F, 6748.5F, 7530.9F, 8351.3F, 0.0F, 1889.3F, 3007.1F, 3864.5F, 4861.4F,
    5710.6F, 6748.5F, 7530.9F, 8351.3F, 0.0F, 1889.3F, 3007.1F, 3864.5F, 4861.4F,
    5710.6F, 6748.5F, 7530.9F, 8351.3F, 0.0F, 1889.3F, 3007.1F, 3864.5F, 4861.4F,
    5710.6F, 6748.5F, 7530.9F, 8351.3F, 0.0F, 1889.3F, 3007.1F, 3864.5F, 4861.4F,
    5710.6F, 6748.5F, 7530.9F, 8351.3F };

  static const float fv1[36] = { 0.0F, 2662.3F, 3469.8F, 4117.40039F, 5029.6F,
    5893.7F, 6810.1F, 7482.1F, 8644.0F, 0.0F, 2618.3F, 3426.4F, 4094.6F, 5004.8F,
    5875.6F, 7464.2F, 7453.3F, 8510.0F, 0.0F, 2599.9F, 3326.6F, 4037.5F,
    4978.40039F, 5804.90039F, 6591.5F, 7442.90039F, 8555.19922F, 0.0F, 2359.0F,
    3259.5F, 3980.0F, 4918.6F, 5772.5F, 6694.9F, 7412.6F, 8527.60059F };

  static const float fv2[18] = { 0.0F, 1732.0F, 2853.1F, 3832.6F, 4859.5F,
    5753.9F, 6790.5F, 7520.9F, 8678.2F, 0.0F, 1677.8F, 2729.1F, 3747.2F, 4792.0F,
    5674.6F, 6739.69971F, 7370.5F, 8582.5F };

  static const float fv3[36] = { 0.0F, 2210.0F, 3148.90015F, 3938.1F,
    4919.40039F, 5770.1F, 6713.0F, 7436.5F, 8550.10059F, 0.0F, 2059.3F, 3068.4F,
    3799.0F, 4910.69971F, 5739.6F, 6675.9F, 7381.5F, 8565.0F, 0.0F, 1916.5F,
    2981.6F, 3869.9F, 4866.4F, 5733.69971F, 6712.90039F, 7409.2F, 8583.7F, 0.0F,
    1785.9F, 2910.5F, 3829.2998F, 4851.5F, 5741.9F, 6705.1F, 7397.3F, 8588.7F };

  static const float fv4[36] = { 3508.6F, 3923.2F, 4414.6F, 4861.9F, 5396.3F,
    6180.1F, 6950.2F, 7724.19971F, 8812.0F, 3357.59985F, 3809.09985F, 4241.9F,
    4814.30029F, 5397.4F, 6143.6F, 7003.1F, 7747.4F, 8818.4F, 2931.0F,
    3519.90015F, 4115.1F, 4608.5F, 5259.4F, 6093.0F, 6930.5F, 7647.2F,
    8699.19922F, 2091.2002F, 3186.4F, 3863.3F, 4448.5F, 5225.69971F, 6028.3F,
    6898.2F, 7579.3F, 8676.4F };

  static const float fv5[18] = { 0.0F, 1565.5F, 2674.0F, 3688.2998F, 4740.69971F,
    5600.69971F, 6665.59961F, 7337.6F, 8496.0F, 0.0F, 1439.7F, 2569.90015F,
    3631.5F, 4624.7F, 5298.6F, 6528.2F, 7188.0F, 8326.5F };

  static const float fv6[36] = { 0.0F, 2688.8F, 3533.9F, 4170.5F, 5036.2F,
    5893.19971F, 6627.1F, 7522.4F, 8640.4F, 0.0F, 2262.8F, 3231.2F, 3985.5F,
    4906.30029F, 5773.5F, 6761.2F, 7424.2F, 8530.2F, 0.0F, 1851.5F, 2868.7F,
    3869.1F, 4850.3F, 5714.4F, 6716.7F, 7376.90039F, 8514.30078F, 0.0F,
    1699.20007F, 2793.5F, 3775.5F, 4769.6F, 5657.9F, 6671.6F, 7366.30029F,
    8558.60059F };

  static const float fv7[36] = { 5578.0F, 5682.8F, 5954.80029F, 6205.5F, 6542.4F,
    6939.6F, 7534.80029F, 8222.8F, 8984.6F, 5354.19971F, 5486.7F, 5819.2F,
    6087.2F, 6367.6F, 6816.4F, 7461.90039F, 8133.00049F, 9204.5F, 4826.9F,
    5040.7F, 5334.8F, 5697.80029F, 6044.9F, 6584.8F, 7259.4F, 7964.8F, 9036.5F,
    3937.90015F, 4363.1F, 4834.6F, 5250.0F, 5685.59961F, 6376.90039F, 7111.5F,
    7837.4F, 8909.5F };

  static const float fv8[18] = { 0.0F, 1408.3F, 2457.0F, 3616.9F, 4607.5F,
    5396.3F, 6487.5F, 7138.7F, 8302.9F, 0.0F, 1197.1001F, 2441.8F, 3526.5F,
    4497.6F, 5291.90039F, 6060.90039F, 6944.0F, 8037.40039F };

  static const float fv9[36] = { 2520.5F, 3417.09985F, 4123.6F, 4693.3F,
    5321.80029F, 6127.69971F, 6927.9F, 7674.9F, 8750.39941F, 1423.7F, 2650.3F,
    3535.8F, 4114.5F, 5064.8F, 5934.5F, 6794.6F, 7491.4F, 8607.7F, 816.5F,
    2023.79993F, 3039.59985F, 3880.7002F, 4810.59961F, 5667.8F, 6678.80029F,
    7369.2F, 8537.2F, 0.0F, 1624.79993F, 2671.6F, 3703.59985F, 4689.30029F,
    5537.09961F, 6596.9F, 7290.3F, 8451.0F };

  int low_i;
  float data1[90];
  float fcnOutput[360];
  signed char varargin_1[10];
  int j;
  float data2[9];
  float y[9];
  static const float fv10[9] = { 0.0F, 0.2F, 0.3F, 0.4F, 0.5F, 0.599999964F,
    0.7F, 0.799999952F, 0.9F };

  float fv11[4];
  static const float fv12[40] = { 4.4786F, 3.7369F, 2.8859F, 1.9703F, 4.4786F,
    3.7203F, 2.9335F, 1.9468F, 4.4786F, 3.7376F, 3.0585F, 2.0665F, 4.4786F,
    3.872F, 3.3325F, 2.3653F, 4.4786F, 4.0655F, 3.7319F, 2.928F, 4.4786F,
    4.3808F, 4.2796F, 3.7999F, 4.4786F, 4.8671F, 4.8408F, 4.6609F, 4.4786F,
    5.1787F, 5.4706F, 5.4636F, 4.4786F, 5.4028F, 5.923F, 6.4969F, 4.4786F,
    5.4835F, 6.1383F, 6.9107F };

  float fv13[10];
  float varargin_2[10];

  /*  relative incoming velocity */
  /*  propeller radius  [m] */
  /*  wind tunnel speeds */
  /*  throttle_values */
  /*  rad per second */
  b_y1 = rpm * 0.104719698F * 0.1016F;

  /*  tip velocity */
  mu = sqrtf(V_rel_B[0] * V_rel_B[0] + V_rel_B[1] * V_rel_B[1]) / (b_y1 +
    0.0001F);

  /*  advance ratio */
  lamb = V_rel_B[2] / (b_y1 + 0.0001F);

  /*  total external inflow ratio (effect lamb_c ans mu * tan alpha) */
  if (lamb < 0.0F) {
    lamb = 0.0F;
  }

  alpha = 57.2957802F * atanf(1.0F / (mu / (lamb + 0.0001F)));
  lamb = sqrtf(mu * mu + lamb * lamb) * (b_y1 + 0.0001F);
  for (mid_i = 0; mid_i < 360; mid_i++) {
    data[mid_i] = 1.0F;
  }

  for (mid_i = 0; mid_i < 10; mid_i++) {
    for (low_ip1 = 0; low_ip1 < 9; low_ip1++) {
      high_i = low_ip1 + 9 * mid_i;
      data[high_i] = fv[high_i];
    }
  }

  for (mid_i = 0; mid_i < 4; mid_i++) {
    for (low_ip1 = 0; low_ip1 < 9; low_ip1++) {
      high_i = low_ip1 + 9 * mid_i;
      data[high_i + 90] = fv1[high_i];
      data[(low_ip1 + 9 * (mid_i + 4)) + 90] = fv3[high_i];
    }
  }

  for (mid_i = 0; mid_i < 2; mid_i++) {
    for (low_ip1 = 0; low_ip1 < 9; low_ip1++) {
      data[(low_ip1 + 9 * (mid_i + 8)) + 90] = fv2[low_ip1 + 9 * mid_i];
    }
  }

  for (mid_i = 0; mid_i < 4; mid_i++) {
    for (low_ip1 = 0; low_ip1 < 9; low_ip1++) {
      high_i = low_ip1 + 9 * mid_i;
      data[high_i + 180] = fv4[high_i];
      data[(low_ip1 + 9 * (mid_i + 4)) + 180] = fv6[high_i];
    }
  }

  for (mid_i = 0; mid_i < 2; mid_i++) {
    for (low_ip1 = 0; low_ip1 < 9; low_ip1++) {
      data[(low_ip1 + 9 * (mid_i + 8)) + 180] = fv5[low_ip1 + 9 * mid_i];
    }
  }

  for (mid_i = 0; mid_i < 4; mid_i++) {
    for (low_ip1 = 0; low_ip1 < 9; low_ip1++) {
      high_i = low_ip1 + 9 * mid_i;
      data[high_i + 270] = fv7[high_i];
      data[(low_ip1 + 9 * (mid_i + 4)) + 270] = fv9[high_i];
    }
  }

  for (mid_i = 0; mid_i < 2; mid_i++) {
    for (low_ip1 = 0; low_ip1 < 9; low_ip1++) {
      data[(low_ip1 + 9 * (mid_i + 8)) + 270] = fv8[low_ip1 + 9 * mid_i];
    }
  }

  /*  fixing/removing an outlier - perhaps error in experimental data */
  data[105] = (data[104] + data[106]) * 0.5F;

  /*  interpolation */
  for (low_i = 0; low_i < 4; low_i++) {
    for (mid_i = 0; mid_i < 10; mid_i++) {
      for (low_ip1 = 0; low_ip1 < 9; low_ip1++) {
        fcnOutput[(low_i + (mid_i << 2)) + 40 * low_ip1] = data[(low_ip1 + 9 *
          mid_i) + 90 * low_i];
      }
    }
  }

  memset(&data1[0], 0, 90U * sizeof(float));
  if ((lamb <= 15.0F) && (lamb >= 0.0F)) {
    low_i = 1;
    low_ip1 = 2;
    high_i = 4;
    while (high_i > low_ip1) {
      mid_i = (low_i + high_i) >> 1;
      if (lamb >= 5.0F * ((float)mid_i - 1.0F)) {
        low_i = mid_i;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }

    mu = (lamb - 5.0F * ((float)low_i - 1.0F)) / (float)(5 * low_i - 5 * (low_i
      - 1));
    if (mu == 0.0F) {
      for (j = 0; j < 90; j++) {
        data1[j] = fcnOutput[(low_i + (j << 2)) - 1];
      }
    } else if (mu == 1.0F) {
      for (j = 0; j < 90; j++) {
        data1[j] = fcnOutput[low_i + (j << 2)];
      }
    } else {
      for (j = 0; j < 90; j++) {
        low_ip1 = low_i + (j << 2);
        b_y1 = fcnOutput[low_ip1 - 1];
        if (b_y1 == fcnOutput[low_ip1]) {
          data1[j] = b_y1;
        } else {
          data1[j] = (1.0F - mu) * b_y1 + mu * fcnOutput[low_ip1];
        }
      }
    }
  }

  for (mid_i = 0; mid_i < 10; mid_i++) {
    varargin_1[mid_i] = (signed char)(-10 * mid_i + 90);
  }

  for (mid_i = 0; mid_i < 5; mid_i++) {
    low_ip1 = varargin_1[mid_i];
    varargin_1[mid_i] = varargin_1[9 - mid_i];
    varargin_1[9 - mid_i] = (signed char)low_ip1;
  }

  for (j = 0; j < 9; j++) {
    low_ip1 = j * 10 + 10;
    for (low_i = 0; low_i < 5; low_i++) {
      high_i = (low_ip1 + low_i) - 10;
      mu = data1[high_i];
      mid_i = (low_ip1 - low_i) - 1;
      data1[high_i] = data1[mid_i];
      data1[mid_i] = mu;
    }

    data2[j] = 0.0F;
  }

  if ((alpha <= varargin_1[9]) && (alpha >= varargin_1[0])) {
    low_i = 1;
    low_ip1 = 2;
    high_i = 10;
    while (high_i > low_ip1) {
      mid_i = (low_i + high_i) >> 1;
      if (alpha >= varargin_1[mid_i - 1]) {
        low_i = mid_i;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }

    mu = (alpha - (float)varargin_1[low_i - 1]) / (float)(varargin_1[low_i] -
      varargin_1[low_i - 1]);
    if (mu == 0.0F) {
      for (j = 0; j < 9; j++) {
        data2[j] = data1[(low_i + j * 10) - 1];
      }
    } else if (mu == 1.0F) {
      for (j = 0; j < 9; j++) {
        data2[j] = data1[low_i + j * 10];
      }
    } else {
      for (j = 0; j < 9; j++) {
        low_ip1 = low_i + j * 10;
        b_y1 = data1[low_ip1 - 1];
        if (b_y1 == data1[low_ip1]) {
          data2[j] = b_y1;
        } else {
          data2[j] = (1.0F - mu) * b_y1 + mu * data1[low_ip1];
        }
      }
    }
  }

  for (mid_i = 0; mid_i < 9; mid_i++) {
    y[mid_i] = fv10[mid_i];
  }

  if (data2[1] < data2[0]) {
    b_y1 = data2[0];
    data2[0] = data2[8];
    data2[8] = b_y1;
    b_y1 = y[0];
    y[0] = y[8];
    y[8] = b_y1;
    b_y1 = data2[1];
    data2[1] = data2[7];
    data2[7] = b_y1;
    b_y1 = y[1];
    y[1] = y[7];
    y[7] = b_y1;
    b_y1 = data2[2];
    data2[2] = data2[6];
    data2[6] = b_y1;
    b_y1 = y[2];
    y[2] = y[6];
    y[6] = b_y1;
    b_y1 = data2[3];
    data2[3] = data2[5];
    data2[5] = b_y1;
    b_y1 = y[3];
    y[3] = y[5];
    y[5] = b_y1;
  }

  *throttle = 0.0F;
  if ((rpm <= data2[8]) && (rpm >= data2[0])) {
    low_i = 1;
    low_ip1 = 2;
    high_i = 9;
    while (high_i > low_ip1) {
      mid_i = (low_i + high_i) >> 1;
      if (rpm >= data2[mid_i - 1]) {
        low_i = mid_i;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }

    b_y1 = data2[low_i - 1];
    mu = (rpm - b_y1) / (data2[low_i] - b_y1);
    if (mu == 0.0F) {
      *throttle = y[low_i - 1];
    } else if (mu == 1.0F) {
      *throttle = y[low_i];
    } else {
      *throttle = y[low_i - 1];
      if (*throttle != y[low_i]) {
        *throttle = (1.0F - mu) * *throttle + mu * y[low_i];
      }
    }
  }

  /*  color=distinguishable_colors(10); */
  /*  for j=2:4 */
  /*      figure(j) */
  /*       for i=1:10 */
  /*           plot([0,[0.2:0.1:0.9]],data(:,i,j),'.-','DisplayName',['\alpha_R = ',num2str(alpha_range(i))],'color',color(i,:)) */
  /*       hold on */
  /*       end */
  /*      plot([0,[0.2:0.1:0.9]],data(:,1,1),'sq','DisplayName','Hover','color',color(1,:)); */
  /*      legend('off'); legend('show','location','best'); */
  /*      xlabel('Throttle');ylabel('RPM'); */
  /*      saveas(gcf,['throttle_speed',num2str(speeds(j))],'fig') */
  /*      saveas(gcf,['throttle_speed',num2str(speeds(j))],'jpg') */
  /*  end */
  /*  max thrust */
  fv11[0] = 0.0F;
  fv11[1] = 5.0F;
  fv11[2] = 10.0F;
  fv11[3] = 15.0F;
  interp1(fv11, fv12, lamb, fv13);
  for (mid_i = 0; mid_i < 10; mid_i++) {
    varargin_1[mid_i] = (signed char)(-10 * mid_i + 90);
    varargin_2[mid_i] = fv13[mid_i];
  }

  for (mid_i = 0; mid_i < 5; mid_i++) {
    low_ip1 = varargin_1[mid_i];
    varargin_1[mid_i] = varargin_1[9 - mid_i];
    varargin_1[9 - mid_i] = (signed char)low_ip1;
    mu = varargin_2[mid_i];
    varargin_2[mid_i] = varargin_2[9 - mid_i];
    varargin_2[9 - mid_i] = mu;
  }

  *Tmax = 0.0F;
  if ((alpha <= varargin_1[9]) && (alpha >= varargin_1[0])) {
    low_i = 1;
    low_ip1 = 2;
    high_i = 10;
    while (high_i > low_ip1) {
      mid_i = (low_i + high_i) >> 1;
      if (alpha >= varargin_1[mid_i - 1]) {
        low_i = mid_i;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }

    mu = (alpha - (float)varargin_1[low_i - 1]) / (float)(varargin_1[low_i] -
      varargin_1[low_i - 1]);
    if (mu == 0.0F) {
      *Tmax = varargin_2[low_i - 1];
    } else if (mu == 1.0F) {
      *Tmax = varargin_2[low_i];
    } else {
      *Tmax = varargin_2[low_i - 1];
      if (*Tmax != varargin_2[low_i]) {
        *Tmax = (1.0F - mu) * *Tmax + mu * varargin_2[low_i];
      }
    }
  }
}

/*
 * File trailer for rpmtothrottle.c
 *
 * [EOF]
 */
