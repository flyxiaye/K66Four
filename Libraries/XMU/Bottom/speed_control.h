#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

void FlagChange(int * flag);
void GiveDriveDuty();
void SpeedGet(void);
void ExpectSpeedGet();
void LeftPiCalc();
void RightPiCalc();
void SpeedControl();

#endif