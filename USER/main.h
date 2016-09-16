#ifndef _MAIN_H_
#define _MAIN_H_

struct env_t{
	int env_lock;
	int bat_voltage;
	int bal_angle;
	int bal_kp;
	int bal_kd;
	int vel_kp;
	int vel_ki;
	int enc_filte;
	int mpu_count;
	int mpu_bal_angle;
	int mpu_bal_gypo;
	int mpu_turn_gypo;
	int enc_left;
	int enc_right;
	int moto_left;
	int moto_right;
	int cmd_forward;
	int cmd_back;
	int cmd_left;
	int cmd_right;
};

extern struct env_t* ENV;

#endif
