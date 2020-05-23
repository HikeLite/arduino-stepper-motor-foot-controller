////////////////////////////////////////////////////////////////////////////////
// A servo motor speed controller for Arduino
// © 2020 ads(0x40)123mail.org, All Rights Reserved
// Version 0.21

#include <math.h>

// Teknic ClearPath Integrated Servo System Motor (Model CPM-SDSK-2321S-RLN)
const unsigned int MOTOR_MAX_SPEED_SPECIFICATION = 3170; // RPM  @ 75 VDC specified maximum motor speed
const unsigned int MOTOR_MAX_SPEED_DESIRED = 1500; // should be less than or equal to motor max speed
const unsigned int MOTOR_MAX_SPEED = min(MOTOR_MAX_SPEED_SPECIFICATION, MOTOR_MAX_SPEED_DESIRED);
const unsigned int MOTOR_STEPS_PER_REVOLUTION = 800;

// Ernie Ball VP Jr 25K Ohm Mono Volume Pedal (Model 6181)
const unsigned int POTENTIOMETER_POLL_FREQUENCY = 16; // Hz: should divide evenly into 1,000,000
const unsigned long POTENTIOMETER_READ_INTERVAL = 1000000 / POTENTIOMETER_POLL_FREQUENCY;

const unsigned int POTENTIOMETER_MIN = 0; // experimentally determined, but must be at least zero
const unsigned int POTENTIOMETER_MAX = 1023; // experimentally determined, but must be 1023 or less

// Pin Assignments (Arduino UNO w/ Proto Screw Shield)
const byte PIN_MOTOR_ENABLE = 9;
const byte PIN_MOTOR_DIRECTION = 10;
const byte PIN_MOTOR_STEP = 11;
const byte PIN_MOTOR_HLFB = 12;

const byte PIN_POTENTIOMETER = A0;

// Turn off debugging if not necessary.
const bool DEBUG = false;

////////////////////////////////////////////////////////////////////////////////
// The setup function runs once when you press reset or power the board.
void setup () {
	// initialize serial communication at 9600 bits per second
	Serial.begin(9600);

	pinMode(PIN_POTENTIOMETER, INPUT); // not strictly necessary as pins default to input mode

	// Signal: Enable +
	// Function: Enable (Logic: High=Enable Low=Disable)
	// Wire: Blue
	pinMode(PIN_MOTOR_ENABLE, OUTPUT);

	// Signal: Input A +
	// Function: Direction (Logic: High=CW Low=CCW)
	// Wire: White
	pinMode(PIN_MOTOR_DIRECTION, OUTPUT);

	// Signal: Input B +
	// Function: Step (Pulse: Digital Step Input)
	// Wire: Black
	pinMode(PIN_MOTOR_STEP, OUTPUT);

	// HLFB +
	// Function: High-Level Feedback
	// Wire: Green
	pinMode(PIN_MOTOR_HLFB, INPUT_PULLUP);

	// initialize digital pin LED_BUILTIN as an output.
	pinMode(LED_BUILTIN, OUTPUT);

	// enable the motor
	digitalWrite(PIN_MOTOR_ENABLE, HIGH);
	delay(1);

	// set the motor direction
	//digitalWrite(PIN_MOTOR_DIRECTION, HIGH);
	delay(1);

	Serial.println("===================================================================");	
	Serial.print("MOTOR_MAX_SPEED: ");
	Serial.print(MOTOR_MAX_SPEED);
	Serial.println(" rpm");
	Serial.print("POTENTIOMETER_POLL_FREQUENCY: ");
	Serial.print(POTENTIOMETER_POLL_FREQUENCY);
	Serial.println(" Hz");
	Serial.print("PIN_MOTOR_ENABLE: ");
	Serial.println(PIN_MOTOR_ENABLE);
	Serial.print("PIN_MOTOR_DIRECTION: ");
	Serial.println(PIN_MOTOR_DIRECTION);
	Serial.print("PIN_MOTOR_STEP: ");
	Serial.println(PIN_MOTOR_STEP);
	Serial.print("PIN_MOTOR_HLFB: ");
	Serial.println(PIN_MOTOR_HLFB);
	Serial.println("===================================================================");

	// Various test functions
	// Note: These test functions may loop forever and never return.
	//test_analog_read_latency();
	//test_potentiometer_noise();
	//test_unsigned_arithmetic();
	//test_overflow_loop();
}


////////////////////////////////////////////////////////////////////////////////
// The loop function runs over and over again forever.
void loop () {
	unsigned long next_potentiometer_read_time = 0;
	unsigned long last_potentiometer_read_time = 0;
	bool overflow_next_potentiometer_read_time = false;
	float potentiometer_value_moving_average = 0.0;
	float desired_rpm = 0.0;
	unsigned short analog_read_loop_counter = 0;
	unsigned long trigger_interval = 0;
	unsigned long next_motor_step_trigger_time = 0;
	unsigned long last_motor_step_trigger_time = 0;
	bool overflow_next_motor_step_trigger_time = false;	
	float request_rpm = 0.0;

	// Loop forever and ever. An iteration of the loop takes about 12 microseconds.
	for (unsigned short loop_counter = 0; ; loop_counter++) {
		// Returns the number of microseconds since the Arduino board began running
		// the current program. This number will overflow (go back to zero), after
		// approximately 70 minutes. On 16 MHz Arduino boards, this function has a
		// resolution of four microseconds (i.e. the value returned is always a
		// multiple of four).
		const unsigned long elapsed_time = micros();

		/*
		// Estimate the frequency of the main loop.  The loop counter rolls over 
		// every 2^16 = 65,536 iterations.
		if (loop_counter == 0) {
			Serial.println(elapsed_time);
		}
		*/

		// Check for overflow / rollover condition of next potentiometer read time.
		if ((!overflow_next_potentiometer_read_time && (elapsed_time >= next_potentiometer_read_time)) || (overflow_next_potentiometer_read_time && (elapsed_time < last_potentiometer_read_time))) {
			// We should be careful how often we do analog reads because
			// an analog read averages 112 microseconds.  This is an issue
			// because in order to spin the motor at 3,170 rpm, we need
			// to send a trigger every 23.7 microseconds
			const unsigned short potentiometer_value = analogRead(PIN_POTENTIOMETER);
	
			if (potentiometer_value == 0) {
				// If the pedal is all the way up, ramp down the motor immediately.
				potentiometer_value_moving_average = 0.0;
			}
			else if (abs((float) potentiometer_value - potentiometer_value_moving_average) > 2.0) {
				// Quick change mode: assume the pedal has moved, so overwrite the average potentiometer value to speed up the change.
				potentiometer_value_moving_average = (float) potentiometer_value;
			}
			else {
				// Assume the change in the potentiometer values is just from noise, so incorporate the new value into the average.
				potentiometer_value_moving_average = exponentially_weighted_moving_average(potentiometer_value_moving_average, potentiometer_value, 16);
			}
	
			desired_rpm = ramp_function(estimate_pedal_fraction(potentiometer_value_moving_average)) * (float) MOTOR_MAX_SPEED;
	
			if (DEBUG) {
				// Printing to the serial port is very slow, so only print debugging data
				// once per n potentiometer reads so we don't slow the system down too much.
				// Disable debug printing when not in debug mode.
				if ((analog_read_loop_counter % 4) == 0) {
					//Serial.println(potentiometer_value_moving_average);
					Serial.print(desired_rpm);
					Serial.print(" / ");
					Serial.println(request_rpm);
				}
			}

			////////////////////////////////////////////////////////////////////////////////
			// Read loop housekeeping
	
			// Calculate the next potentiometer read time.
			last_potentiometer_read_time = elapsed_time;
			next_potentiometer_read_time = next_potentiometer_read_time + POTENTIOMETER_READ_INTERVAL;
			// If (a + b) < a, then we know overflow has occurred in an unsigned variable.
			overflow_next_potentiometer_read_time = (next_potentiometer_read_time < POTENTIOMETER_READ_INTERVAL);
	
			// Increment the analog read loop counter. It doesn't matter if it overflows.
			analog_read_loop_counter++;
		}

		////////////////////////////////////////////////////////////////////////////////
		// Only execute this section every n iterations of the loop, otherwise we will 
		// slow the program down too much. Recommended settings: (64, 64, 256) or (256, 16, 64)
		if ((loop_counter % 64) == 0) {
			if (desired_rpm == 0.0) {
				// If the pedal is all the way up, stop the motor quickly.
				// Abrupt stop. If request rpm is set to zero, the previously-scheduled step won't even be triggered.
				//request_rpm = 0.0;
				// Linear stop
				//request_rpm = max(0.0, request_rpm - 1.0);
				// Smoothed stop. Increase the decay factor to lengthen the slow-down.
				request_rpm = exponentially_weighted_moving_average(request_rpm, desired_rpm, 64);
			}
			else {
				// It would be cool we could ramp up the motor speed with the cosine function 
				// (1 - cos(pi * x)) / 2 but since the desired rpm is often changing, it seems
				// impractical to implement.
				request_rpm = exponentially_weighted_moving_average(request_rpm, desired_rpm, 256);
			}
			
			// Make sure that we don't divide by too small of divisor, otherwise the trigger interval can overflow.
			trigger_interval = (60000000.0 / (max(request_rpm, 1.0) * (float) MOTOR_STEPS_PER_REVOLUTION));
		}
		
		////////////////////////////////////////////////////////////////////////////////
		// Check for overflow / rollover condition of next motor step trigger time
		if ((!overflow_next_motor_step_trigger_time && (elapsed_time >= next_motor_step_trigger_time)) || (overflow_next_motor_step_trigger_time && (elapsed_time < last_motor_step_trigger_time))) {
			if (request_rpm > 1.0) {
				// The pedal is down. Step the motor.
				trigger_motor_step();
				blink_led();
	
				////////////////////////////////////////////////////////////////////////////////
				// Trigger loop housekeeping
	
				// Calculate the next motor step trigger time.
				last_motor_step_trigger_time = elapsed_time;

				if ((elapsed_time - next_motor_step_trigger_time) < (trigger_interval * 8)) {
					next_motor_step_trigger_time = next_motor_step_trigger_time + trigger_interval;					
				}
				else {
					// If the triggers have gotten too far behind schedule, reset the next motor step trigger time based on the elapsed time.
					next_motor_step_trigger_time = elapsed_time + trigger_interval;
				}
				
				// If (a + b) < a, then we know overflow has occurred in an unsigned variable.
				overflow_next_motor_step_trigger_time = (next_motor_step_trigger_time < trigger_interval);
			}
		}
	}
} 


////////////////////////////////////////////////////////////////////////////////
inline float estimate_pedal_fraction (float potentiometer_value_moving_average) {
	const float potentiometer_values[11] = {0.00, 11.59, 25.21, 49.21, 73.39, 98.98, 123.68, 275.14, 520.38, 872.56, 1023.00};

	for (byte i = 10; i >= 0; i--) {		
		if (potentiometer_values[i] == potentiometer_value_moving_average) {
			return (float) i / 10.0;
		}
		else if (potentiometer_values[i] < potentiometer_value_moving_average) {
			return (i + (potentiometer_value_moving_average - potentiometer_values[i]) / (potentiometer_values[i + 1] - potentiometer_values[i])) / 10.0;
		}
	}

	Serial.println("Error: Something went horribly wrong. We fell out of the end of the loop.");
	while (true); // Stop here by looping forever.
}


////////////////////////////////////////////////////////////////////////////////
inline float ramp_function (float pedal_fraction) {
	// Choose which ramp function you desire.
	
	// Linear ramp.  When the pedal is 50% depressed, the motor will run at 50% speed.
	//return pedal_fraction;

	// Somewhat progressive ramp
	//return powf(pedal_fraction, 1.5);
	
	// Moderately progressive ramp
	return pedal_fraction * pedal_fraction;

	// Highly progressive ramp
	//return pedal_fraction * pedal_fraction * pedal_fraction;
}


////////////////////////////////////////////////////////////////////////////////
inline void trigger_motor_step () {
	// Per the manual's "STEP AND DIRECTION TIMING" section:
	// Minimum step pulse width = 1uS

	digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
	digitalWrite(PIN_MOTOR_STEP, HIGH);
	delayMicroseconds(1);
	digitalWrite(PIN_MOTOR_STEP, LOW);
	delayMicroseconds(1);
	digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
}


////////////////////////////////////////////////////////////////////////////////
inline float exponentially_weighted_moving_average (float previous_ewma, float new_value, unsigned short decay_factor) {
	// the decay factor should be a power of 2 for performance purposes
	return previous_ewma + (new_value - previous_ewma) / decay_factor;
}


////////////////////////////////////////////////////////////////////////////////
inline void blink_led () {
	//Serial.println("ENTER blink_led()");
	
	digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
	digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
}


////////////////////////////////////////////////////////////////////////////////
void test_analog_read_latency () {
	Serial.println("ENTER test_analog_read_latency()");	

	// Loop forever.
	while (true) {
		unsigned long time1 = micros();
		unsigned short potentiometer_value = analogRead(PIN_POTENTIOMETER);
		unsigned long time2 = micros();
		Serial.println(time2 - time1);
		Serial.println(potentiometer_value);
	}
}


////////////////////////////////////////////////////////////////////////////////
void test_potentiometer_noise () {
	Serial.println("ENTER test_potentiometer_noise()");	

	// Leave the foot pedal stationary when running this test.
	
	unsigned short min_potentiometer_value = 1023;
	unsigned short max_potentiometer_value = 0;

	// Loop forever.
	while (true) {
		unsigned short potentiometer_value = analogRead(PIN_POTENTIOMETER);
	
		if (potentiometer_value < min_potentiometer_value) {
			min_potentiometer_value = potentiometer_value;
		}
		if (potentiometer_value > max_potentiometer_value) {
			max_potentiometer_value = potentiometer_value;
		}
	
		Serial.println(max_potentiometer_value - min_potentiometer_value);
	}
}


////////////////////////////////////////////////////////////////////////////////
void test_unsigned_arithmetic () {
	Serial.println("ENTER test_unsigned_arithmetic()");

	byte a = 254;
	byte b = 2;
	byte difference = a - b;

	Serial.println(a);
	Serial.println(b);
	Serial.println(difference);
	
	Serial.println(a, BIN);
	Serial.println(b, BIN);
	Serial.println(difference, BIN);

	/*
	byte a = 255;
	byte b = 255;
	byte sum = a + b;
	Serial.println(a, BIN);
	Serial.println(b, BIN);
	Serial.println(sum, BIN);
	
	if (sum < a) {
		Serial.println("OVERFLOWED");
	}
	else {
		Serial.println("DIDN'T OVERFLOW");
	}
	*/
	
	while (true); // Stop here by looping forever.
}


////////////////////////////////////////////////////////////////////////////////
void test_overflow_loop () {
	Serial.println("ENTER test_overflow_loop()");	

	//ULONG_MAX is 4294967295
	const unsigned long INTERVAL = 1000000;

	unsigned long next_potentiometer_read_time = 0;
	unsigned long last_potentiometer_read_time = 0;
	bool overflow_next_potentiometer_read_time = false;
	unsigned long start = micros();

	for (unsigned long i = 0; ; i++) {
	//while (true) {
		unsigned long elapsed_time = micros();

		if (elapsed_time < last_potentiometer_read_time) {
			Serial.print("OVERFLOWED: elapsed_time = ");
			Serial.println(elapsed_time);
		}

		if ((!overflow_next_potentiometer_read_time && (elapsed_time >= next_potentiometer_read_time)) || (overflow_next_potentiometer_read_time && (elapsed_time < last_potentiometer_read_time))) {
			Serial.print(".");
			if (i % 60 == 59) {
				Serial.println("");
			}
			//Serial.println("INSIDE");
			// Read loop housekeeping
			// Calculate the next potentiometer read
			last_potentiometer_read_time = elapsed_time;
			next_potentiometer_read_time = next_potentiometer_read_time + INTERVAL;
			// If (x + y) < x, then we know overflow has occurred in an unsigned variable
			overflow_next_potentiometer_read_time = (next_potentiometer_read_time < INTERVAL);

			if (overflow_next_potentiometer_read_time) {
				Serial.print("OVERFLOWED: next_potentiometer_read_time = ");	
				Serial.println(next_potentiometer_read_time);	
			}

			/*
			if (next_potentiometer_read_time < POTENTIOMETER_READ_INTERVAL) {
				overflow_next_potentiometer_read_time = true;
				last_potentiometer_read_time = elapsed_time;
			}
			else {
				overflow_next_potentiometer_read_time = true;
			}
			*/
		}
	}

	unsigned long stop = micros();

	Serial.print("Elapsed time: ");	
	Serial.print(stop - start);	
	Serial.println(" microseconds");	
	while (true); // Stop here by looping forever.
}
