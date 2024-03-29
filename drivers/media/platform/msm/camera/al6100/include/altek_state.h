/*
 * this file was generated by NunniFSMGen
 */


#ifndef altek_state_H
#define altek_state_H

#include "altek_statefsm.h"

#ifdef __cplusplus
extern "C"
{
#endif		/* __cplusplus */


int state_open(void *o);
int error_sequence(void *o);
int redundant_sequence(void *o);
int state_enter_hwpt(void *o);
int state_coldboot(void *o);
int state_close(void *o);
int state_leave_hwpt(void *o);
int state_scenario_chg(void *o);
int state_strm_on_off(void *o);
int state_enter_cp(void *o);
int state_set_qp(void *o);
int state_stop(void *o);
int state_leave_cp(void *o);
int state_leave_cp_standy(void *o);


#ifdef __cplusplus
}
#endif		/* __cplusplus */


#endif
