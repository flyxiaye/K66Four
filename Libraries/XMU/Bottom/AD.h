#ifndef AD_H
#define AD_H
extern int protect_flag;
void ind_acq(void);
void ind_norm_maxmin(void);
void ind_norm(void);
void get_ind_error(void);
unsigned char IndJudgeCircle(unsigned char type);
unsigned char IndJudgeIntoCircle(unsigned char type);
#endif // !AD_H