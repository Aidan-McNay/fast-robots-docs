.. ECE 5160 Lab 5 Write-Up: PID Control

Lab 5: PID Control
==========================================================================

Now that we've assembled our robot, we can introduce a PID control
system to help it navigate. In this lab, we use PID control to navigate
the robot 1 foot away from a wall.

Prelab
--------------------------------------------------------------------------

In this lab, it was especially important to communicate data over
Bluetooth, as it was the only way to gain visibility into the values
used in the algorithm, and adjust PID gains appropriately. To do this,
I defined a function ``log_pid_data`` on the Arduino, which our PID
loop could call; it would log various pieces of data, including:

* The current time
* Whether ToF data was ready
* The measured distance
* The terms generated from PID control (including the integrator terms
  with and without windup protection - useful for the M.Eng. portion)
* The total PID control output
* The PWM sent to the motors

.. code-block:: c++

   void log_pid_data( int time, bool data_ready, int distance, int kp_term,
                      int ki_term, int windup_term, int kd_term, int total_term, int motor_pwm )
   {
     if ( entry_idx < NUM_ENTRIES ) {
       time_entries[entry_idx]       = time;
       data_ready_entries[entry_idx] = data_ready;
       distance_entries[entry_idx]   = distance;
       kp_term_entries[entry_idx]    = kp_term;
       ki_term_entries[entry_idx]    = ki_term;
       windup_term_entries[entry_idx] = windup_term;
       kd_term_entries[entry_idx]    = kd_term;
       total_term_entries[entry_idx] = total_term;
       motor_pwm_entries[entry_idx]  = motor_pwm;
       entry_idx += 1;
     }
   }

From here, I defined a BLE command ``GET_PID_DATA`` to retrieve all of
the recorded data

.. code-block:: c++
   :class: toggle

   // Case statement from within handle_command()

   case GET_PID_DATA:
      Serial.printf("Getting data...\n");
      for ( int i = 0; i < entry_idx; i++ ) {
        tx_estring_value.clear();
        tx_estring_value.append( time_entries[i] );
        tx_estring_value.append( "|" );
        tx_estring_value.append( (int) data_ready_entries[i] );
        tx_estring_value.append( "|" );
        tx_estring_value.append( distance_entries[i] );
        tx_estring_value.append( "|" );
        tx_estring_value.append( kp_term_entries[i] );
        tx_estring_value.append( "|" );
        tx_estring_value.append( ki_term_entries[i] );
        tx_estring_value.append( "|" );
        tx_estring_value.append( windup_term_entries[i] );
        tx_estring_value.append( "|" );
        tx_estring_value.append( kd_term_entries[i] );
        tx_estring_value.append( "|" );
        tx_estring_value.append( total_term_entries[i] );
        tx_estring_value.append( "|" );
        tx_estring_value.append( motor_pwm_entries[i] );
        tx_characteristic_string.writeValue( tx_estring_value.c_str() );
      }
      break;

All that was needed to receive the data was a custom handler to
receive BLE notifications in Python:

.. code-block:: python
   :class: toggle

   def parse_data( data: str ):
     data_components = data.split("|")
     time       = (float(data_components[0]) / 1000)
     ready = bool(int(data_components[1]))
     distance   = int(data_components[2])
     kp_term    = int(data_components[3])
     ki_term    = int(data_components[4])
     windup_term = int(data_components[5])
     kd_term    = int(data_components[6])
     total_term = int(data_components[7])
     motor_pwm  = int(data_components[8])
     return time, ready, distance, kp_term, ki_term, windup_term, kd_term, total_term, motor_pwm
   
   def data_handler(_uid, response):
     global i
     time, ready, distance, kp_term, ki_term, windup_term, kd_term, total_term, motor_pwm = parse_data(response.decode())
     data_time.append(time)
     data_ready.append(ready)
     data_distance.append(distance)
     data_kp_term.append(kp_term)
     data_ki_term.append(ki_term)
     data_windup_term.append(windup_term)
     data_kd_term.append(kd_term)
     data_total_term.append(total_term)
     data_motor_pwm.append(motor_pwm)
     i = i + 1
     print(f"{i * 100 / NUM_SAMPLES:.2f}% done", end = '\r')
   
   ble.start_notify(ble.uuid['RX_STRING'], data_handler)
   ble.send_command(CMD.GET_PID_DATA, "")

After testing with some dummy data, I was successfully able to communicate
data and parse it, providing an invaluable tool for later debugging.

The other thing I did before beginning the lab was to revisit code from
past labs, and package them into object-oriented classes for easier
handling and re-use. As an example, here is the header file for my
``Car`` implementation (which utilizes the ``Wheel`` class to drive
motors):

.. code-block:: c++

   class Car {
    public:
     Car( int MOTOR1_IN1_PIN, int MOTOR1_IN2_PIN, int MOTOR2_IN1_PIN,
          int MOTOR2_IN2_PIN )
         : motor1( MOTOR1_IN1_PIN, MOTOR1_IN2_PIN ),
           motor2( MOTOR2_IN1_PIN, MOTOR2_IN2_PIN ) {};
   
     // Calibration
     void set_motor2_calibration( float factor )
     {
       motor2.set_calibration_factor( factor );
     }
   
     // Motor functions
     void forward( uint8_t pwm );
     void backward( uint8_t pwm );
     void left( uint8_t pwm );
     void right( uint8_t pwm );
     void stop();
     void coast();
   
    private:
     Wheel motor1, motor2;
   };

Now, not only is the code separated and more portable, but our PID
control doesn't have to worry about the exact analog writes, but
can instead use the abstraction and call ``car.forward()``. By
separating and testing these components, I could incrementally build
the system and limit the scope of possibly incorrect code.


Designing the PID Controller
--------------------------------------------------------------------------

To begin with, I began with a simple control system that only used a
proportional term. Unlike lecture, I chose to have my code generate
a positive error when far away, which would translate to a positive
PWM for the motors:

.. code-block:: c++

   void PID::update( int u )
   {
      int error = u - set_point;
      terms.kp_term = error * params.kp;
   }

   int PID::get_control()
   {
     return terms.kp_term;
   }

This was used in the main PID loop, which updated the control when
new values were present:

.. code-block:: c++
   :class: toggle

   void run_pid_step()
   {
     curr_time = millis();
     int curr_distance, curr_kp_term, curr_ki_term, curr_ki_windup,
         curr_kd_term, curr_total_term, curr_motor_pwm;
     
     bool data_ready;
     if( tofs.sensor1.checkForDataReady() ){
       curr_distance = tofs.sensor1.getDistance();
       last_distance = curr_distance;
       tofs.sensor1.clearInterrupt();
       tofs.sensor1.stopRanging();
       tofs.sensor1.startRanging();
       data_ready = true;
     } else {
       curr_distance = last_distance;
       data_ready = false;
     }
   
     pid.update( curr_distance );
     curr_total_term   = pid.get_control();
     curr_kp_term      = pid.terms.kp_term;
     curr_ki_term      = pid.terms.ki_term;
     curr_ki_windup    = pid.terms.ki_windup_term;
     curr_kd_term      = pid.terms.kd_term;
     curr_motor_pwm = pid.scale( curr_total_term );
     if ( curr_motor_pwm > 0 ) {
       car.forward( curr_motor_pwm );
     }
     else {
       car.backward( -1 * curr_motor_pwm );
     }
   
     log_pid_data( curr_time, data_ready, curr_distance, curr_kp_term,
                   curr_ki_term, curr_ki_windup, curr_kd_term, curr_total_term, curr_motor_pwm );
   }

Note that I included a ``pid.scale()`` function; this imposed an upper
limit on the PWM value, moved it outside of the deadband, and also
helped account for the lesser drive strength I observed when reversing:

.. code-block:: c++

   int PID::scale( int pid_output )
   {
     float intermediate_term = pid_output;
   
     // Scale outside deadband
     if ( intermediate_term >= 0 ) {
       intermediate_term += deadband;
     }
     else {
       intermediate_term -= deadband * 1.5;
     }
   
     // Reverse having issues, so scale
     if( intermediate_term < 0 ){
       intermediate_term *= 1.5;
     }
   
     // Max PWM of 180
     if ( intermediate_term > 180 ) {
       intermediate_term = 180;
     } else if ( intermediate_term < -180 ) {
       intermediate_term = -180;
     }
   
     return intermediate_term;
   }

From here, I was able to get a successful result using
:math:`K_p = 0.5`; this still resulted in overshooting, so I
would need to decrease it when adding an integrator (this was
also when I began to scale the reverse PWM).

.. image:: img/lab5/prop-0.05.png
   :align: center
   :width: 90%
   :class: bottompadding

(Note that my robot initially swerves right; this is not due to lack of
calibration, as the remainder is straight, but rather initial slipping
from a fast start)

.. youtube:: HmI5krO30OU
   :align: center
   :width: 70%

Range/Sampling Time
--------------------------------------------------------------------------

Final System
--------------------------------------------------------------------------

(ECE 5160) Integrator Windup
--------------------------------------------------------------------------