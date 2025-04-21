.. ECE 5160 Lab 10 Write-Up: Localization (Simulation)

Lab 10: Localization (Simulation)
==========================================================================

This lab involved using a Bayes Filter in simulation to demonstrate
accurate localization, even with noisy measurements. This will allow us
to have a concrete start to build off of when localizing with our robot.

Helper Functions
--------------------------------------------------------------------------

In order to implement a Bayes filter, it's useful to have several helper
functions to incrementally develop the entire algorithm. This involves
being able to update our prior belief with predictions based on our
motion model, then refine our belief using our sensor model:

For all :math:`x_t`:

.. math::
   
   \overline{bel}(x_t) = \sum_{x_{t-1}} p(x_t | u_t, x_{t-1}) bel(x_{t-1})

   bel(x_t) = \eta \cdot p(z_t | x_t) \overline{bel}(x_t)

``compute_control``
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

For our motion model, we want to be able to compute the control input
from a current and previous pose (with poses being expressed as the
current :math:`(x,y)` coordinates of the robot, as well as its angle
:math:`\theta`). ``compute_control`` determines the initial rotation,
translation, and final rotation (a.k.a control inputs) needed to achieve
this change (implemented using the given trigonometric functions from
lecture):

.. code-block:: python

  def compute_control(cur_pose, prev_pose):
    """ Given the current and previous odometry poses, this function extracts
    the control information based on the odometry motion model.

    Args:
        cur_pose  ([Pose]): Current Pose
        prev_pose ([Pose]): Previous Pose 

    Returns:
        [delta_rot_1]: Rotation 1  (degrees)
        [delta_trans]: Translation (meters)
        [delta_rot_2]: Rotation 2  (degrees)
    """
    # Unpack inputs
    cur_x, cur_y, cur_t    = cur_pose
    prev_x, prev_y, prev_t = prev_pose

    # Compute control information
    delta_y = cur_y - prev_y
    delta_x = cur_x - prev_x
    
    delta_rot_1 = degrees(atan2(delta_y, delta_x)) - prev_t
    delta_trans = dist((cur_x, cur_y), (prev_x, prev_y))
    delta_rot_2 = cur_t - (prev_t + delta_rot_1)

    # Normalize angles
    delta_rot_1 = loc.mapper.normalize_angle(delta_rot_1)
    delta_rot_2 = loc.mapper.normalize_angle(delta_rot_2)

    return delta_rot_1, delta_trans, delta_rot_2

``odom_motion_model``
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

If we know our current and previous pose, as well as the control inputs
we took between them, we can use ``compute_control`` to get the actual
control inputs needed, and determine the probability of the given
current state (assuming we know the variance in measurements - given,
in this case). ``odom_motion`` model does this, computing

.. math::

   p(x_t | u_t, x_{t-1})

.. code-block:: python

  def odom_motion_model(cur_pose, prev_pose, u):
    """ Odometry Motion Model

    Args:
        cur_pose  ([Pose]): Current Pose
        prev_pose ([Pose]): Previous Pose
        (rot1, trans, rot2) (float, float, float): A tuple with control data in the format 
                                                   format (rot1, trans, rot2) with units (degrees, meters, degrees)


    Returns:
        prob [float]: Probability p(x'|x, u)
    """
    actual_u = compute_control(cur_pose, prev_pose)

    prob_rot_1 = loc.gaussian(actual_u[0] - u[0], 0, loc.odom_rot_sigma)
    prob_trans = loc.gaussian(actual_u[1] - u[1], 0, loc.odom_trans_sigma)
    prob_rot_2 = loc.gaussian(actual_u[2] - u[2], 0, loc.odom_rot_sigma)
    prob  = prob_rot_1 * prob_trans * prob_rot_2

    return prob

``prediction_step``
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Finally, we can combine these functions in ``prediction_step``, which
implements the overall prediction step by iterating overall all locations
in ``loc.bel_bar``, and updating them based on the sum of the previous
beliefs of locations (in ``loc.bel``) times the probability that we could
get to :math:`x_t` from that location (from ``odom_motion_model``):

For all :math:`x_t`:

.. math::
   
   \overline{bel}(x_t) = \sum_{x_{t-1}} p(x_t | u_t, x_{t-1}) bel(x_{t-1})

.. code-block:: python

  def prediction_step(cur_odom, prev_odom):
    """ Prediction step of the Bayes Filter.
    Update the probabilities in loc.bel_bar based on loc.bel from the previous time step and the odometry motion model.

    Args:
        cur_odom  ([Pose]): Current Pose
        prev_odom ([Pose]): Previous Pose
    """
    u = compute_control(cur_odom, prev_odom)
    for ( x_idx, y_idx, a_idx ) in np.ndindex( loc.bel_bar.shape ):
        x_t = loc.mapper.from_map( x_idx, y_idx, a_idx )
        new_bel_bar = 0

        for ( x_idx_t_1, y_idx_t_1, a_idx_t_1 ), bel in np.ndenumerate( loc.bel ):
            if bel > 0.001:
                x_t_1 = loc.mapper.from_map( x_idx_t_1, y_idx_t_1, a_idx_t_1 )
                new_bel_bar += (
                    odom_motion_model( x_t, x_t_1, u ) *
                    bel
                )
        loc.bel_bar[x_idx][y_idx][a_idx] = new_bel_bar  

.. admonition:: Optimization
   :class: note

   Note that we only calculate the probability if ``bel > 0.001``; this
   helps to avoid the (relatively) lengthy probability calculation for
   terms that won't meaningfully contribute to the overall sum, increasing
   the speed of our calculation

``sensor_model``
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Similar to our motion model, we also need our sensor model to provide
:math:`p(z_t | x_t)`. This is done in ``sensor_model``, where we compare
a ground-truth observation to our current views (given with
``loc.obs_range_data``) to compute the probability of our views for a
given location. Note that we can do this with a single call to
``loc.gaussian`` for all elements, as the numpy arrays can be operated on
collectively

.. admonition:: Array Size

   Since our true views from ``mapper.get_views`` are a 1-D array of length
   18, but ``loc.obs_range_data`` is a 2D :math:`(18, 1)` array, we have
   to first flatten the latter to compare them correctly

.. code-block:: python

  def sensor_model(obs):
    """ This is the equivalent of p(z|x).


    Args:
        obs ([ndarray]): A 1D array consisting of the true observations for a specific robot pose in the map 

    Returns:
        [ndarray]: Returns a 1D array of size 18 (=loc.OBS_PER_CELL) with the likelihoods of each individual sensor measurement
    """

    prob_array = loc.gaussian(obs, loc.obs_range_data.flatten(), loc.sensor_sigma)
    return prob_array

``update_step``
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Finally, we can use our sensor model to implement our update step. Here,
we've previously calculated the true views for each location, use
``sensor_model`` to determine the likelihood that we're in that location,
and update our current belief (in ``loc.bel``) with that probability times
our prior belief from our motion model (for all locations). Lastly, we
normalize our belief to sum to 1, ensuring no drift of the overall belief sum.

For all :math:`x_t`:

.. math::

   bel(x_t) = \eta \cdot p(z_t | x_t) \overline{bel}(x_t)

.. admonition:: Optimization

   To avoid calling ``flatten`` each time with ``sensor_model``, I instead
   chose to "inline" the function to only call ``flatten`` once, speeding
   up runtime

.. code-block:: python

  def update_step():
    """ Update step of the Bayes Filter.
    Update the probabilities in loc.bel based on loc.bel_bar and the sensor model.
    """
    range_data = loc.obs_range_data.flatten()
    for ( x_idx, y_idx, a_idx ), bel_bar in np.ndenumerate( loc.bel_bar ):
        true_measurements = loc.mapper.get_views( x_idx, y_idx, a_idx )
        
        # Equivalent to `sensor_model( true_measurements )`
        prob_array = loc.gaussian(true_measurements, range_data, loc.sensor_sigma)
        loc.bel[x_idx][y_idx][a_idx] = np.prod(prob_array) * bel_bar

    # Normalize -> equivalent to multiplying by eta
    loc.bel = loc.bel / loc.bel.sum()

This can now be used in our top-level loop (given) to implement the Bayes Filter,
where we iteratively take a step along our trajectory (based on our belief),
predict where we are, then gather sensor measurements and update our belief with
them:

.. code-block:: python

  for t in range(0, traj.total_time_steps):

    # Take a step    
    prev_odom, current_odom, prev_gt, current_gt = traj.execute_time_step(t)
        
    # Prediction Step
    prediction_step(current_odom, prev_odom)
    loc.print_prediction_stats(plot_data=True)
    
    # Get Observation Data by executing a 360 degree rotation motion
    loc.get_observation_data()
    
    # Update Step
    update_step()
    loc.print_update_stats(plot_data=True)

Running the Bayes Filter
--------------------------------------------------------------------------

With our filter implemented, we can run it on our simulation to see how
well we can predict the ground truth, even with noise (I performed two
trials, shown on the left and right). Here, the ground truth is plotted
in green, our raw odometry model in red, and our Bayes filter in blue.
*You can click on pictures if they aren't big enough*

.. grid:: 2
    :gutter: 2
    :margin: 0
    :padding: 0

    .. grid-item::

        .. youtube:: pk-QN6EU02I
            :align: center
            :width: 100%

    .. grid-item::

        .. youtube:: bo73dovExQA
            :align: center
            :width: 100%

.. grid:: 2
    :gutter: 2
    :margin: 0
    :padding: 0

    .. grid-item::

        .. image:: img/lab10/map-1.png
            :align: center
            :width: 100%
            :class: bottompadding

    .. grid-item::

        .. image:: img/lab10/map-2.png
            :align: center
            :width: 100%
            :class: bottompadding

We can immediately see that our Bayes filter gives a much
better estimate of our location, further reinforced with the
outputs of our simulator (adjusted to have the ground truth
angle be normalized, and knowing that a probability of 1.0 is
an artifact of rounding):

Run 1 Data
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

:download:`Raw Output <files/lab10/output-1.txt>`

.. list-table::
    :header-rows: 1
    :stub-columns: 1

    * - Step
      - Ground Truth State
      - Belief State
      - Belief Probability
      - Error
    * - 0
      - :math:`(0.287, -0.087, 320.561)`
      - :math:`(0.305, 0.000, -50.000)`
      - :math:`0.9999999`
      - :math:`(-0.018, -0.087, 10.561)`
    * - 1
      - :math:`(0.515, -0.523, 657.643)`
      - :math:`(0.305, -0.610, -70.000)`
      - :math:`1.0`
      - :math:`(0.210, 0.087, 7.643)`
    * - 2
      - :math:`(0.515, -0.523, 995.105)`
      - :math:`(0.305, -0.610, -90.000)`
      - :math:`1.0`
      - :math:`(0.210, 0.087, 5.105)`
    * - 3
      - :math:`(0.551, -0.921, 1355.105)`
      - :math:`(0.610, -0.914, -90.000)`
      - :math:`1.0`
      - :math:`(-0.059, -0.007, 5.105)`
    * - 4
      - :math:`(0.814, -1.053, 1802.003)`
      - :math:`(0.914, -0.914, 10.000)`
      - :math:`1.0`
      - :math:`(-0.100, -0.139, -7.997)`
    * - 5
      - :math:`(1.592, -0.867, 2210.970)`
      - :math:`(1.524, -0.914, 50.000)`
      - :math:`0.9999999`
      - :math:`(0.068, 0.047, 0.970)`
    * - 6
      - :math:`(1.660, -0.480, 2600.101)`
      - :math:`(1.524, -0.610, 70.000)`
      - :math:`1.0`
      - :math:`(0.136, 0.130, 10.101)`
    * - 7
      - :math:`(1.722, -0.125, 2965.832)`
      - :math:`(1.829, -0.305, 90.000)`
      - :math:`1.0`
      - :math:`(-0.107, 0.180, -4.168)`
    * - 8
      - :math:`(1.709, 0.366, 3348.373)`
      - :math:`(1.829, 0.000, 90.000)`
      - :math:`0.8496144`
      - :math:`(-0.119, 0.366, 18.373)`
    * - 9
      - :math:`(1.703, 0.685, 3747.520)`
      - :math:`(1.524, 0.610, 150.000)`
      - :math:`1.0`
      - :math:`(0.179, 0.076, -2.480)`
    * - 10
      - :math:`(1.281, 0.954, 4118.791)`
      - :math:`(1.524, 0.914, 150.000)`
      - :math:`1.0`
      - :math:`(-0.243, 0.039, 8.791)`
    * - 11
      - :math:`(0.402, 0.847, 4574.553)`
      - :math:`(0.610, 0.914, -110.000)`
      - :math:`1.0`
      - :math:`(-0.207, -0.068, 4.553)`
    * - 12
      - :math:`(0.229, 0.220, 4979.635)`
      - :math:`(0.305, 0.305, -70.000)`
      - :math:`1.0`
      - :math:`(-0.075, -0.085, 9.635)`
    * - 13
      - :math:`(-0.013, -0.090, 5272.037)`
      - :math:`(0.000, -0.305, -130.000)`
      - :math:`1.0`
      - :math:`(-0.013, 0.215, 2.037)`
    * - 14
      - :math:`(-0.372, -0.249, 5609.496)`
      - :math:`(-0.305, -0.305, -150.000)`
      - :math:`1.0`
      - :math:`(-0.067, 0.056, -0.504)`
    * - 15
      - :math:`(-0.766, -0.255, 5946.383)`
      - :math:`(-0.610, -0.305, -170.000)`
      - :math:`0.9039539`
      - :math:`(-0.156, 0.050, -3.617)`