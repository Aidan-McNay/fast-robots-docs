..
  ECE 5160 Lab 1 Write-Up: The Artemis Board and Bluetooth

Lab 1: The Artemis Board and Bluetooth
==========================================================================

This lab focused on getting acquainted with the
`RedBoard Artemis Nano <https://www.sparkfun.com/sparkfun-redboard-artemis-nano.html>`_.
This included using the Arduino IDE to program the board, as well as to
demonstrate communication with the board over Bluetooth.

.. raw:: html

  <hr style="border:2px solid #2980b9">

Lab 1A
--------------------------------------------------------------------------

Blink
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The capabilities of the board were first tested with the example "Blink"
program (blinking the on-board blue LED), to ensure that basic
programming could occur.

.. youtube:: AvO1AA2BEk8
  :align: center
  :width: 70%

Serial
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once basic programmability was established, the "Serial" example
showed that data could be communicated to and from the board using a
serial monitor (with the input data echoed as output).

.. youtube:: BDoguEoJ3Zg
  :align: center
  :width: 70%

Temperature & Microphone
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After being able to receive data from the board, the "Analog Read" example
shoed that data could be received from sensors (in this case,
the temperature sensor). In addition, the "Microphone" example showed that
we could access and manipulate microphone data to collect frequency
information.

.. youtube:: yLlPOWaOOhA
  :align: center
  :width: 70%

C Detector *(ECE 5160)*
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As an added example for ECE 5160, we were responsible for combining the
previous examples to detect when a "C" was played, indicated by
blinking the LED.

For detecting a "C", I modified the microphone example (which
already parsed the microphone data into frequency content) to output a
boolean for whether the detected loudest frequency was our expected
frequency (I used 523Hz, corresponding to ~C5), within a given margin to
account for slight variations and the broad FFT bins of <>

.. code-block:: c++

  #define FREQ_MARGIN 20
  #define FREQ 523
  
  bool is_c_note(void) {
    uint32_t ui32LoudestFrequency;
  
    // ... Microphone data and FFT parsing code, adapted from
    //     the "Microphone" example ...

    return (ui32LoudestFrequency - FREQ_MARGIN) < FREQ &
           (ui32LoudestFrequency + FREQ_MARGIN) > FREQ;
  }

This was used to modify the "Blink" example, such that the ``loop``
function checked whether a C was currently being played, and only
blinked the LED if ``is_c_note`` returned ``true``.

.. youtube:: 8Tiug56DFzs
  :align: center
  :width: 70%

.. raw:: html

  <hr style="border:2px solid #2980b9">

Lab 1B
--------------------------------------------------------------------------

Prelab
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For this section of the lab, we explored communicating with the Artemis
board over Bluetooth (specifically, "Low Energy" Bluetooth, or **BLE**),
making use of the ``ArduinoBLE`` library, and interfacing from our own
machine with the `bleak <https://bleak.readthedocs.io/en/latest/>`_
Python module.

To prepare for communication over Bluetooth, we first needed to be able to
identify our particular Bluetooth device (i.e. the Artemis board) using
it's **MAC Address**, a unique 12-digit hexadecimal number for each
device on a network. The provided ``ble_arduino.ino`` displays this
using the serial monitor when the device first starts up.

Lab Tasks
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Echo
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

2. Send Three Floats
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

3. Get Time Millis
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

4. Notification Handler
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

5. Time Notification Loop
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

6. Send Times in Batch
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

7. Send Temperature in Batch
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

8. Discussion
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

9. Effective Data Rate and Overhead *(ECE 5160)*
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

10. Reliability *(ECE 5160)*
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

.. raw:: html

  <hr style="border:2px solid #2980b9">

Discussion
--------------------------------------------------------------------------