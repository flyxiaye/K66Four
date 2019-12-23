#ifndef __FILTER_H
#define __FILTER_H

#include "headfile.h"

typedef struct
{
	float in_est;    //Estimator
	float in_obs;    //Observation
	
	float fix_ki;
	float ei_limit;    
	
	float e;
	float ei;

	float out;
}_inte_fix_filter_st;

typedef struct
{
	float in_est_d;   //Estimator
	float in_obs;    //Observation
	
	float fix_kp;
	float e_limit;

	float e;

	float out;
}_fix_inte_filter_st;

typedef struct
{
	float integration_1;
	float integration_2;
	float out_tmp;
	float out;

}_com_fil_t;

typedef struct
{
	float lpf_1;

	float out;
}_lf_t;

typedef struct
{
	float lpf_1;
	float lpf_2;
	float in_old;
	float out;
}_jldf_t;

typedef struct
{
	uint8 cnt;

	int32 lst_pow_sum;
	
	int32 now_out;
	int32 lst_out;
	int32 now_velocity_xdt;
} _steepest_st;

typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;

typedef struct
{
  float a[3];
  float b[3];
}Butter_Parameter;

extern Butter_Parameter Butter_80HZ_Parameter_Acce,Butter_60HZ_Parameter_Acce,Butter_51HZ_Parameter_Acce,
                 Butter_30HZ_Parameter_Acce,Butter_20HZ_Parameter_Acce,Butter_15HZ_Parameter_Acce,
                 Butter_10HZ_Parameter_Acce,Butter_5HZ_Parameter_Acce,Butter_2HZ_Parameter_Acce;

void Butterworth_Parameter_Init(void);
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter);
	
#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))

void inte_fix_filter(float dT,_inte_fix_filter_st *data);
void fix_inte_filter(float dT,_fix_inte_filter_st *data);
void yaw_inte_filter(float dT,_fix_inte_filter_st *data);
void steepest_descend(int32 arr[],uint8 len,_steepest_st *steepest,uint8 step_num,int32 in);

void limit_filter(float T,float hz,_lf_t *data,float in);

void jyoun_limit_deadzone_filter(float T,float hz1,float hz2,_jldf_t *data,float in);//白噪声滤波

void jyoun_filter(float dT,float hz,float ref_value,float exp,float fb,float *out);
//float Moving_Average(uint8 item,uint8 width_num,float in);
void Moving_Average(float moavarray[],//滤波数组 数组长度：len+1
										uint16 len ,//滤波数据长度
										uint16 *fil_cnt,//滤波元素号数标记（静态，用作存储）
										float in,//输入
										float *out //输出
										);



void step_filter(float step,float in,float *out);

void fir_arrange_filter(float *arr,uint16 len,uint8 *fil_cnt,float in,float *arr_out);  //len<=255 len >= 3

void LPF_1(float hz,//截止频率
					float time,//周期
					float in,//输入
					float *out//输出
					);

					
void LPF_1_db(float hz,float time,double in,double *out); //低通滤波，2hz代表0.5秒上升至目标值0.7倍，大约1秒上升到90%

void LPF_I(float raw_a,float raw_b,float time,float in,float *out,float *intera);

float my_deadzone_3(float T,float hz,float x,float ,float zoom,float range_x,float *zoom_adj); //range_x   0 ----- 1  *****


/*============ 坐标转换 ===============
适用坐标系
					x
					|
			y---z
			
对应世界坐标中，x为地磁方向，z为重力方向。

======================================*/
//void vec_3d_transition(_xyz_f_st *ref, _xyz_f_st *in, _xyz_f_st *out);
//
//void mag_horizon_trans(_xyz_f_st *ref, _xyz_f_st *in, _xyz_f_st *out);

/*============== AHRS =================
		自创AHRS姿态解算，非四元数
======================================*/
#define G_KP 1.0f
#define M_KP 1.0f

//mx，my，mz模长需在300 ---- 100 以内
//void Jyoun_AHRS(float T,
//								uint8 acc_fix_en,
//								uint8 mag_fix_en,
//								float ax,float ay,float az,
//								float gx,float gy,float gz,
//								float mx,float my,float mz,
//								float *yaw,float *pit,float *rol,
//								_xyz_f_st *acc_ng_offset
//								);

//extern _xyz_f_st acc_3d_ng,//3d矫正后的机体坐标下运动加速度（去除重力加速度）；
//								acc_ng_offset,
//								acc_3d_hg,//3d校正后世界坐标运动加速度（不含航向），
//								gyro_3d_h;//3d矫正后，世界坐标下的陀螺仪数据（不含航向）

//extern _xyz_f_st vec_g;
//extern _xyz_f_st vec_m;
//extern _xyz_f_st vec_m_h;
//extern uint8 acc_ng_cali;

#define Filter_Num 10

struct _Filter
{
	uint8 count;
	float sum;
	float old[Filter_Num];
};

void Window_Filter(struct _Filter *filt,float *data);



#define M_PIf       3.14159265358979323846f

typedef struct rateLimitFilter_s {
    float state;
} rateLimitFilter_t;

typedef struct pt1Filter_s {
    float state;
    float RC;
    float dT;
} pt1Filter_t;

/* this holds the data required to update samples thru a filter */
typedef struct biquadFilter_s {
    float b0, b1, b2, a1, a2;
    float d1, d2;
} biquadFilter_t;


typedef enum {
    FILTER_PT1 = 0,
    FILTER_BIQUAD,
    FILTER_FIR,
} filterType_e;

typedef enum {
    FILTER_LPF,
    FILTER_NOTCH
} biquadFilterType_e;

typedef struct firFilter_s {
    float *buf;
    const float *coeffs;
    uint8_t bufLength;
    uint8_t coeffsLength;
} firFilter_t;


float pt1FilterApply(pt1Filter_t *filter, float input);
float pt1FilterApply4(pt1Filter_t *filter, float input, uint16_t f_cut, float dt);
void pt1FilterReset(pt1Filter_t *filter, float input);

void rateLimitFilterInit(rateLimitFilter_t *filter);
float rateLimitFilterApply4(rateLimitFilter_t *filter, float input, float rate_limit, float dT);

void biquadFilterInitNotch(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq, uint16_t cutoffHz);
void biquadFilterInitLPF(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq);
void biquadFilterInit(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq, float Q, biquadFilterType_e filterType);
float biquadFilterApply(biquadFilter_t *filter, float sample);
float filterGetNotchQ(uint16_t centerFreq, uint16_t cutoff);

void firFilterInit(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs);
void firFilterInit2(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs, uint8_t coeffsLength);
void firFilterUpdate(firFilter_t *filter, float input);
float firFilterApply(const firFilter_t *filter);
float constrainf(float amt, float low, float high);
float sin_approx(float x);
float cos_approx(float x);

#endif
