.. ECE 5160 Lab 3 Write-Up: ToF Data

Lab 3: Time-of-Flight Sensing
==========================================================================

Once we go our IMU working, we can begin to include Time-of-Flight (ToF)
data to sense other objects around us.

Prelab
--------------------------------------------------------------------------

This week's prelab included determining how our two ToF sensors (as well
as the IMU) would be wired. This resulted in the following wiring diagram:

.. image:: img/lab3/wiring.png
   :align: center
   :width: 100%
   :class: bottompadding

.. admonition:: Battery Wire Colors
   :class: info

   Note that the wiring colors unconventionally change from the LiPo
   battery to the Artemis; this was a result of our JST connector
   physically connecting black to the ``+`` terminal (and red to 
   ``-``), so the color convention was swapped to have correct voltage
   polarity.

At a high-level, all of our sensor boards communicate over I\ :sup:`2`\ C;
we can use the QWIIC breakout board to connect all of them to the Artemis'
I\ :sup:`2`\ C port, and strip/solder the QWIIC cables to our ToF sensors
as appropriate

Two Time-of-Flight Sensors
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

To detect obstacles in multiple directions, our vehicle will use two
ToF sensors. While their position may change based on future lab results,
I currently plan to have them mounted on the front and side of the car.
In a maze scenario, this will allow us to see obstacles directly in front
of us (most relevant - in the direction of travel) as well as to one side,
at the expense of behind us (likely not needed, as it's where we came from)
and to the other side (can be achieved by rotation).

.. image:: img/lab3/tof-placement.png
   :align: center
   :width: 40%
   :class: bottompadding

This also brings an issue of data communication. Both ToF sensors have a
default I\ :sup:`2`\ C address of ``0x52``. If we attempt to communicate,
both will see thie address as theirs and attempt to respond appropriately,
causing a bus collision. However, their address is programmable; we can
therefore use the ``XSHUT`` pin of one to turn it off, change the address
of the other, and use the independent addresses from then on.

Finally, the ToF sensors are more position-dependent than the IMU; I
accordingly chose to use the long QWIIC cables for these to leave them
the most freedom of position on the robot.

Lab Tasks
--------------------------------------------------------------------------

One Time-of-Flight Sensor
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

The first step was to connect one ToF sensor, to verify it could work
alone.

.. figure:: img/lab3/one-tof.png
   :align: center
   :width: 70%
   :class: image-border

   Hardware setup with one ToF sensor (``XSHUT`` not necessary)

We can use the Arduino example for scanning the I\ :sup:`2`\ C
bus to verify the address.

.. figure:: img/lab3/i2c-scan.png
   :align: center
   :width: 70%
   :class: image-border

   I\ :sup:`2`\ C scanning example

Since we're only using one of two I\ :sup:`2`\ C ports, a device
is only found on one. Here, the address found is ``0x29``; while
this initially seems incorrect, we can notice that this is
``0x52 >> 1``, omitting the last bit. The last I\ :sup:`2`\ C
address bit is used to indicate direction; ``0`` for a write,
``1`` for a read. Because of this, the controlling device only
keeps track of the first 7 bits.

.. figure:: img/lab3/datasheet-address.png
   :align: center
   :width: 70%
   :class: image-border

   An excerpt from the `ToF datasheet <https://cdn.sparkfun.com/assets/8/9/9/a/6/VL53L0X_DS.pdf#page=19>`_, verifying our address expectation

Testing Range
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

For this lab, I chose the "Short" distance mode, as it is the best with
ambient light, and the 1.3 meters of range seemed adequate for a robot
of our size.

.. figure:: img/lab3/distance-modes.png
   :align: center
   :width: 70%
   :class: image-border

   The available distance modes (Source: `Datasheet <https://cdn.sparkfun.com/assets/8/9/9/a/6/VL53L0X_DS.pdf#page=10>`_)

For this experiment, I used a tape measure to get the true position, and
used the IMU over Bluetooth to record and plot measured position, as well
as ranging time.
