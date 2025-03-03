.. ECE 5160 Lab 4 Write-Up: Open-Loop Motor Control

Lab 4: Open-Loop Motor Control
==========================================================================

Now that we can get data around us, we can integrate our electronics into
our robot, including controlling the wheel-driving motors!

Prelab
--------------------------------------------------------------------------

For this lab, we expanded on our hardware setup by introducing two DRV8833
motor drivers to control the motors. These connect to an external battery
to drive the motors, as well as with PWM inputs (and ground) from the
Artemis. Accordingly, I chose analog pins to be able to provide the PWM
output (A0 - A3)

.. image:: img/lab4/motor-wiring.png
   :align: center
   :width: 85%
   :class: bottompadding

.. admonition:: External Battery
   :class: info

   Note that the motor drivers/motors are connected to a separate power
   supply than the other electronics. The relatively high power,
   fast-switching motor supply will cause EMI ripples in the power supply.
   To avoid this interfering with our other sensitive electronics (such as
   out sensors and microcontroller), the power supplies are decoupled.

Motor Drivers - Oscilloscope
--------------------------------------------------------------------------

To incrementally verify our motor driving capability, I first connected
one motor driver to the Artemis. The PWM signals were generated on the
motor driver inputs, and the outputs were connected to the oscilloscope
probes for measuring, with the motor voltage driven by a voltage supply.

.. admonition:: Power Supply
   :class: info

   The `DRV8833 datasheet <https://www.ti.com/lit/ds/symlink/drv8833.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1740659196269>`_
   notes that the motor voltage can range from 2.7V to 10.8V. However,
   we will be driving our motors with a `3.7V battery <https://www.amazon.com/URGENEX-Battery-Rechargeable-Quadcopter-Charger/dp/B08T9FB56F/ref=sr_1_3?keywords=lipo+battery+3.7V+850mah&qid=1639066404&sr=8-3>`_,
   so I chose to set the voltage supply to 3.7V to be more realistic.

.. figure:: img/lab4/oscilloscope-setup.png
   :align: center
   :width: 70%

   Oscilloscope Probe setup for Motor Drivers (only using one initially, then both)

The code snippet below demonstrates the PWM functionality by sweeping
the duty cycle of a forward-driving motor, where the ``IN1`` pin is kept
high, and ``IN2`` switches rapidly (inspired by Nila Narayan's example
from 2024):

.. code-block:: c++
   :caption: PWM test code for a forward-driving motor
   :class: toggle

   #define HIGH_PIN 0 // AIN1 / BIN1
   #define PWM_PIN 1  // AIN2 / BIN2
   
   void setup(){
     pinMode( HIGH_PIN, OUTPUT );
     pinMode(  PWM_PIN, OUTPUT );
   }
   
   void loop(){
     analogWrite( HIGH_PIN, 255 );
     for( int i = 0; i < 255; i = i + 1 ){
       analogWrite( PWM_PIN, i );
     }
   }

.. youtube:: TRONEOA6_nA
   :align: center
   :width: 70%

Once we verified one motor driver, I soldered the second one to
verify it as well; the snippet below similarly sweeps the duty
cycle, but increases the duty cycle of the first forward-driving
motor while decreasing that of the second reverse motor.

.. code-block:: c++
   :caption: PWM test code for complementary motors
   :class: toggle

   #define MOTOR1_IN1 0
   #define MOTOR1_IN2 1
   #define MOTOR2_IN1 2
   #define MOTOR2_IN2 3
   
   void motor1_forward(uint8_t i) {
     analogWrite(MOTOR1_IN1, 255 );
     analogWrite(MOTOR1_IN2, i   );
   }
   
   void motor2_reverse(uint8_t i) {
     analogWrite(MOTOR2_IN1, 255 - i );
     analogWrite(MOTOR2_IN2, 255     );
   }
   
   void setup() {
     pinMode(MOTOR1_IN1, OUTPUT);
     pinMode(MOTOR1_IN2, OUTPUT);
     pinMode(MOTOR2_IN1, OUTPUT);
     pinMode(MOTOR2_IN2, OUTPUT);
   }
   
   void loop() {
     for (int i = 0; i < 255; i = i + 1) {
       motor1_forward( i );
       motor2_reverse( i );
     }
   }

.. youtube:: GNM55a-WYec
   :align: center
   :width: 70%

Motor Drivers - Wheels
--------------------------------------------------------------------------

Once we had verified basic functionality of the drivers, we could connect
them to the wheels. First, I connected one driver to one side of wheels,
and had it run them forward and in reverse repeatedly:

.. code-block:: c++
   :caption: Code to run wheels forward and reverse, with pauses in between
   :class: toggle

   #define MOTOR1_IN1 0
   #define MOTOR1_IN2 1
   
   void motor1_forward(uint8_t i) {
     analogWrite(MOTOR1_IN1, 255 );
     analogWrite(MOTOR1_IN2, i   );
   }
   
   void motor1_reverse(uint8_t i) {
     analogWrite(MOTOR1_IN1, 0   );
     analogWrite(MOTOR1_IN2, i   );
   }
   
   void motor1_stop() {
     analogWrite(MOTOR1_IN1, 255 );
     analogWrite(MOTOR1_IN2, 255 );
   }
   
   void setup(){
     pinMode( MOTOR1_IN1, OUTPUT );
     pinMode( MOTOR1_IN2, OUTPUT );
   }
   
   void loop(){
     motor1_forward(128);
     delay(1000);
     motor1_stop();
     delay(1000);
     motor1_reverse(128);
     delay(1000);
     motor1_stop();
     delay(1000);
   }

.. youtube:: 6YTDzlVLYeQ
   :align: center
   :width: 70%

From there, I soldered on the second motor driver, such that
we can re-use the code to run both motors (with minor changes
to the functions reflecting different orientations of the
motors)

.. code-block:: c++
   :caption: Code to run both wheels forward and reverse, with pauses in between
   :class: toggle

   #define MOTOR1_IN1 0
   #define MOTOR1_IN2 1
   #define MOTOR2_IN1 2
   #define MOTOR2_IN2 3
   
   void motor1_forward( uint8_t i )
   {
     analogWrite( MOTOR1_IN1, 255 );
     analogWrite( MOTOR1_IN2, i );
   }
   
   void motor2_forward( uint8_t i )
   {
     analogWrite( MOTOR2_IN2, 255 );
     analogWrite( MOTOR2_IN1, i );
   }
   
   void motor1_reverse( uint8_t i )
   {
     analogWrite( MOTOR1_IN1, 0 );
     analogWrite( MOTOR1_IN2, i );
   }
   
   void motor2_reverse( uint8_t i )
   {
     analogWrite( MOTOR2_IN2, 0 );
     analogWrite( MOTOR2_IN1, i );
   }
   
   void motor_stop()
   {
     analogWrite( MOTOR1_IN1, 255 );
     analogWrite( MOTOR1_IN2, 255 );
     analogWrite( MOTOR2_IN1, 255 );
     analogWrite( MOTOR2_IN2, 255 );
   }
   
   void setup()
   {
     pinMode( MOTOR1_IN1, OUTPUT );
     pinMode( MOTOR1_IN2, OUTPUT );
     pinMode( MOTOR2_IN1, OUTPUT );
     pinMode( MOTOR2_IN2, OUTPUT );
   }
   
   void loop()
   {
     motor1_forward( 128 );
     motor2_forward( 128 );
     delay( 1000 );
     motor_stop();
     delay( 1000 );
     motor1_reverse( 128 );
     motor2_reverse( 128 );
     delay( 1000 );
     motor_stop();
     delay( 1000 );
   }

This also meant that we could permanently solder the motor drivers
to the 850mAh battery instead of the power supply, demonstrating
that the most power-intensive portion of the circuit can be battery
powered.

.. youtube:: 2E3HwHxjVNc
   :align: center
   :width: 70%

Electronics Installation
--------------------------------------------------------------------------

From here, we could permanently install the electronics in the car, making
the entire system independent!

* I chose to have my motor drivers on a separate side from the rest of
  the electronics, to avoid undue EMI
* My ToF sensors are currently mounted on the front and side, as per
  :doc:`lab3`
* The IMU (next to the Artemis and battery) is as upright as possible,
  for ease of calculating position

.. image:: img/lab4/installation.png
   :align: center
   :width: 85%
   :class: bottompadding image-border