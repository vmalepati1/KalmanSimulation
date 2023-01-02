/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/trace.h"

#include "timer.h"
#include "led.h"

#include "kalman_filter.h"
#include "state_machine.h"
#include "imu_math_helper.h"

#define DT_SECONDS 0.25

// ----------------------------------------------------------------------------
//
// Standalone STM32F4 led blink sample (trace via DEBUG).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// Then demonstrates how to blink a led with 1 Hz, using a
// continuous loop and SysTick delays.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace-impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 3 / 4)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

void print_matrix(arm_matrix_instance_f32 *A, const char *name) {
	int row;
	int col;

	trace_printf("%s Matrix:\n", name);

	trace_printf("[");

	for (row = 0; row < A->numRows; row++) {
		if (row > 0) {
			trace_printf(" ");
		}

		trace_printf("[");

		for (col = 0; col < A->numCols; col++) {
			if (col < A->numCols - 1) {
				trace_printf("% 15.5f, ", A->pData[row * A->numCols + col]);
			} else {
				trace_printf("% 15.5f", A->pData[row * A->numCols + col]);
			}
		}

		if (row < A->numRows - 1) {
			trace_printf("],\n");
		} else {
			trace_printf("]");
		}
	}

	trace_printf("]\n");
}

State boosterState;
State sustainerState;

void boosterInit() {
	trace_printf("Entered booster state\n");
}

State *boosterExecute() {
	trace_printf("Execute booster state\n");
	return &sustainerState;
}

void boosterFinish() {
	trace_printf("Exit booster state\n");
}

void sustainerInit() {
	trace_printf("Entered sustainer state\n");
}

State *sustainerExecute() {
	trace_printf("Execute sustainer state\n");
	return &sustainerState;
}

void sustainerFinish() {

}

int
main(int argc, char* argv[])
{
	// Basic kalman filter for constant accelerating body
	KalmanFilter kf;
	arm_status status = ARM_MATH_SUCCESS;

	const float accelStd = 0.1;
	const float accelVar = accelStd * accelStd;

	float32_t F_f32[4] = {
		1, DT_SECONDS,
		0, 1
	};

	float32_t G_f32[2] = {
		0.5 * DT_SECONDS * DT_SECONDS,
		DT_SECONDS
	};

	float32_t P_f32[4] = {
			500, 0,
			0, 500
	};

	float32_t Q_f32[4] = {
			1.0, 1.0,
			1.0, 1.0
	};

	float32_t xHat_f32[2] = {
		// Position, velocity
		0, 0
	};

	float32_t stateStdDevs_f32[2] = {
		// Position std, velocity std
		DT_SECONDS * DT_SECONDS / 2.0 * accelVar, DT_SECONDS * accelVar
	};

	status = init_kalman_filter(&kf, 2, 1, &F_f32[0], &G_f32[0], &P_f32[0], &Q_f32[0], &xHat_f32[0], &stateStdDevs_f32[0]);

	print_matrix(&kf.Q, "Q");
	trace_printf("Kalman initialization status: %d\n", status);

	float32_t un_f32[1] = { 9.8 };

	status = predict_kalman_filter(&kf, un_f32);

	print_matrix(&kf.xHat, "xHat");
	print_matrix(&kf.P, "P");
	trace_printf("Step 1 predict status: %d\n", status);

	float32_t zn_f32[1] = { -32.40 };
	float32_t H_f32[2] = { 1, 0 };
	float32_t measurementStdDevs[1] = { 20 };

	status = correct_kalman_filter(&kf, 1, zn_f32, H_f32, measurementStdDevs);

	print_matrix(&kf.xHat, "xHat");
	print_matrix(&kf.P, "P");
	trace_printf("Step 1 correct status: %d\n", status);

	un_f32[0] = 39.72 - 9.8;

	status = predict_kalman_filter(&kf, un_f32);

	print_matrix(&kf.xHat, "xHat");
	print_matrix(&kf.P, "P");
	trace_printf("Step 2 predict status: %d\n", status);

	zn_f32[0] = -11.1;

	status = correct_kalman_filter(&kf, 1, zn_f32, H_f32, measurementStdDevs);

	print_matrix(&kf.xHat, "xHat");
	print_matrix(&kf.P, "P");
	trace_printf("Step 2 correct status: %d\n", status);

	float32_t localToWorld_f32[9] = {0};
	float32_t A_f32[3] = { 0.59, 9.15, -3.47 };

	arm_matrix_instance_f32 localToWorld;

	arm_mat_init_f32(&localToWorld, 3, 3, localToWorld_f32);

	calibrate_imu(A_f32, &localToWorld);

	print_matrix(&localToWorld, "Local to World");

	boosterState.name = "Booster";
	boosterState.initPtr = &boosterInit;
	boosterState.executePtr = &boosterExecute;
	boosterState.finishPtr = &boosterFinish;

	sustainerState.name = "Sustainer";
	sustainerState.initPtr = &sustainerInit;
	sustainerState.executePtr = &sustainerExecute;
	sustainerState.finishPtr = &sustainerFinish;

	StateMachine sm;

	init_state_machine(&sm, &boosterState);

	step_state_machine(&sm);
	step_state_machine(&sm);
	step_state_machine(&sm);

	while (1) { }
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
