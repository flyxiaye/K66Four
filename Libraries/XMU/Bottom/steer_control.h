#ifndef STEER_CONTROL_H
#define STEER_CONTROL_H

void SteerDutyCal(int select_row);
void GiveSteerDuty(void);
void SteerControl(void);
void CarProtect(void);
void Image_Init();
void ErrorCal();
#endif