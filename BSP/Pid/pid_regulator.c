#include "pid_regulator.h"

void PID_Reset(PID_Regulator_t *pid)
{
	pid->ref = 0.0f;
	pid->fdb = 0.0f;
	pid->componentKp = 0.0f;
	pid->componentKi = 0.0f;
	pid->componentKd = 0.0f;
	pid->output = 0.0f;
}
void PID_Calc(PID_Regulator_t *pid)
{
	pid->err[0] = pid->err[1]; 
	pid->err[1] =  pid->ref - pid->fdb;
	pid->componentKp = pid->kp * pid->err[1];
	pid->componentKi += pid->ki * pid->err[1];
	pid->componentKd = pid->kd * (pid->err[1] - pid->err[0]); 
	VAL_LIMIT(pid->componentKi,-pid->componentKiMax, pid->componentKiMax);
	pid->output = pid->componentKp + pid->componentKi + pid->componentKd;
	VAL_LIMIT(pid->output,-pid->outputMax, pid->outputMax);
}

