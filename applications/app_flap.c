/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "mc_interface.h"
#include "utils.h"
#include "encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 2048);

// Private functions
static void pwm_callback(void);
static void flap(int argc, const char **argv);
static void flap_toggle(void);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile bool cmd_flap_trigger = false;

#define FLAP_OPEN 0
#define FLAP_CLOSE 1

static volatile int flap_direction = FLAP_OPEN;

// Toggle flap
void flap_toggle() {
	if (flap_direction) {
		flap_direction = 0;
    } else {
		flap_direction = 1;
    }
	commands_printf("new flap direction %s\n", flap_direction ? "open" : "close");
}

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
	mc_interface_set_pwm_callback(pwm_callback);

	stop_now = false;
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
			NORMALPRIO, my_thread, NULL);

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"flap",
			"Control the flap",
			0,
			flap);
    commands_printf("Flap Started");

}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	mc_interface_set_pwm_callback(0);
	terminal_unregister_callback(flap);

	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_custom_configure(app_configuration *conf) {
	(void)conf;
}

static THD_FUNCTION(my_thread, arg) {
	(void)arg;

	chRegSetThreadName("App Custom");

	is_running = true;

	// Input pin for flap toggle
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLUP);

	// Aux buttons to send CAN button commands
	palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT_PULLUP);

	// Get motor config
	const volatile mc_configuration *mcconf = mc_interface_get_configuration();

	// Example of using the experiment plot
//	chThdSleepMilliseconds(8000);
//	commands_init_plot("Sample", "Voltage");
//	commands_plot_add_graph("Temp Fet");
//	commands_plot_add_graph("Input Voltage");
//	float samp = 0.0;
//
//	for(;;) {
//		commands_plot_set_graph(0);
//		commands_send_plot_points(samp, mc_interface_temp_fet_filtered());
//		commands_plot_set_graph(1);
//		commands_send_plot_points(samp, GET_INPUT_VOLTAGE());
//		samp++;
//		chThdSleepMilliseconds(10);
//	}

	bool flap_button = false;
	bool pio_button = false;
	bool rxpin_button = false;

	int flap_hold_cnt = 0;
	int pio_hold_cnt = 0;
	int rxpin_hold_cnt = 0;

	int flap_count_ms = 0;

	for(;;) {
		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}

		timeout_reset(); // Reset timeout if everything is OK.

		// Run your logic here. A lot of functionality is available in mc_interface.h.

		pio_button = !palReadPad(HW_ICU_GPIO, HW_ICU_PIN);
		rxpin_button = !palReadPad(HW_UART_RX_PORT, HW_UART_RX_PIN);
		flap_button = !palReadPad(HW_UART_TX_PORT, HW_UART_TX_PIN);

		if (flap_button) {
			flap_hold_cnt++;
		} else if (flap_hold_cnt > 0) {
			flap_hold_cnt--;
		}
		if (pio_button) {
			pio_hold_cnt++;
		} else if (pio_hold_cnt > 0) {
			pio_hold_cnt--;
		}
		if (rxpin_button) {
			rxpin_hold_cnt++;
		} else if (rxpin_hold_cnt > 0) {
			rxpin_hold_cnt--;
		}

		// Send CAN 99,0 with 32 bit value of flap (-1 or 1) for open/close
		if ((flap_hold_cnt > 10) || cmd_flap_trigger) {
			flap_hold_cnt = 0;
			cmd_flap_trigger = false;
			commands_printf("flap button pressed");
			flap_toggle();
			// gear drive flap takes 4-5 seconds to open/close
			flap_count_ms = 8000;
			float duty = utils_map(flap_direction, 0.0, 1.0, -mcconf->l_max_duty, mcconf->l_max_duty);
			comm_can_set_duty(99, duty);
			mc_interface_set_duty(duty);
			chThdSleepMilliseconds(500);
		}

		// Send CAN 99,1
		if (rxpin_hold_cnt > 5) {
			rxpin_hold_cnt = 0;
			commands_printf("rxpin button pressed");
			comm_can_set_current(99, 1);
			chThdSleepMilliseconds(500);
		}

		// Send CAN 99,2
		if (pio_hold_cnt > 5) {
			pio_hold_cnt = 0;
			commands_printf("piopin button pressed");
			comm_can_set_current_brake(99, 1);
			chThdSleepMilliseconds(500);
		}

		// Count down to turn off open/close drive
		const int delay = 10;
		if (flap_count_ms > 0) {
			flap_count_ms -= delay;
		}	
		if (flap_count_ms <= 0) {
			mc_interface_set_duty(0);
		}
		chThdSleepMilliseconds(delay);
	}
}

static void pwm_callback(void) {
	// Called for every control iteration in interrupt context.
}

// Callback function for the terminal command with arguments.
static void flap(int argc, const char **argv) {
	cmd_flap_trigger = true;
}
