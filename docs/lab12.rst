.. ECE 5160 Lab 12 Write-Up: Path Planning and Execution

Lab 12: Path Planning and Execution
==========================================================================

.. image:: img/lab12/kronk.jpg
   :align: center
   :width: 70%
   :class: bottompadding

This lab involves the synthesis of all the previous components in order
to traverse a given path around the lab, involving:

* Localizing where we are in the map (:doc:`lab9` & :doc:`lab11`)
* Moving to the next waypoint in the path, involving turning (:doc:`lab5`) and
  rotating (:doc:`lab6`) using a PID controller and position estimates (:doc:`lab7`)

State Machine
--------------------------------------------------------------------------

To solve this problem, I chose to:

* Localize first
* Send the distance data to Python (to localize)
* Receive the estimated pose and the target (expressed as a turn angle and
  translation distance)
* Use PID to first turn, then translate to the target waypoint
* Repeat for all remaining waypoints

.. admonition:: Localization
   :class: note

   Localizing after each step will significantly reduce our speed;
   however, it will increase the effective number of update steps in our
   Bayes filter, improving our accuracy in pose estimation and moving as
   close as possible to each waypoint

This is implemented as a state machine on the Artemis:

.. image:: img/lab12/state_machine.png
   :align: center
   :width: 100%
   :class: bottompadding