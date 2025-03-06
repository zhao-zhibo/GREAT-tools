#define _CRT_SECURE_NO_WARNINGS

#pragma once
#include <iostream>

//* ------------  TASKS  ------------

//* TASK 1 : Decode starneto binary log. (Including RAWIMUB & GTIMU_BIN)
int task_xwyd_decode();

//* TASK 2 : Convert imu text file to '.imr'.
int task_txt2imr();

//* TASK 3 : Decode '.imr' to text file.
int task_imr2txt();

//* ---------------------------------

int main()
{
	task_txt2imr();

	return 0;
}

