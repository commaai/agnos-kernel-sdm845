/*
 * this file was generated by NunniFSMGen - do not edit!
 */


#ifndef altek_statefsm_H
#define altek_statefsm_H


#ifdef __cplusplus
extern "C"
{
#endif		/* __cplusplus */


struct altek_statefsm;

struct altek_state {
	int (*ispdrv_open)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_enter_hwpt)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_leave_hwpt)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_coldboot)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_scenario_chg)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_strm_on_off)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_set_qp)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_stop)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_enter_cp)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_leave_cp)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_leave_cp_standy)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_close)(struct altek_statefsm *fsm, void *o);
};


struct altek_statefsm {
	int (*ispdrv_open)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_enter_hwpt)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_leave_hwpt)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_coldboot)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_scenario_chg)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_strm_on_off)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_set_qp)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_stop)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_enter_cp)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_leave_cp)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_leave_cp_standy)(struct altek_statefsm *fsm, void *o);
	int (*ispdrv_close)(struct altek_statefsm *fsm, void *o);
	void (*changestate)(struct altek_statefsm *fsm,
		struct altek_state *nextstate);
	struct altek_state *m_state;
};

extern struct altek_statefsm *altek_statefsmcreate(void);


extern void altek_statefsmdelete(struct altek_statefsm *fsm);

/* events */
extern int altek_statefsmispdrv_open(
	struct altek_statefsm *fsm, void *o);
extern int altek_statefsmispdrv_enter_hwpt(
	struct altek_statefsm *fsm, void *o);
extern int altek_statefsmispdrv_leave_hwpt(
	struct altek_statefsm *fsm, void *o);
extern int altek_statefsmispdrv_coldboot(
	struct altek_statefsm *fsm, void *o);
extern int altek_statefsmispdrv_scenario_chg(
	struct altek_statefsm *fsm, void *o);
extern int altek_statefsmispdrv_strm_on_off(
	struct altek_statefsm *fsm, void *o);
extern int altek_statefsmispdrv_set_qp(
	struct altek_statefsm *fsm, void *o);
extern int altek_statefsmispdrv_stop(
	struct altek_statefsm *fsm, void *o);
extern int altek_statefsmispdrv_enter_cp(
	struct altek_statefsm *fsm, void *o);
extern int altek_statefsmispdrv_leave_cp(
	struct altek_statefsm *fsm, void *o);
extern int altek_statefsmispdrv_leave_cp_standy(
	struct altek_statefsm *fsm, void *o);
extern int altek_statefsmispdrv_close(
	struct altek_statefsm *fsm, void *o);


#ifdef __cplusplus
}
#endif		/* __cplusplus */


#endif
