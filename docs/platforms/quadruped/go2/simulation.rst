.. _doc-platforms-quadruped-go2-simulation--unitree-go2-simulation:

========================
Unitree Go2 — Simulation
========================

.. _unitree-go2--simulation:

.. _doc-platforms-quadruped-go2-simulation--unitree-go2-simulation-1:

Unitree Go2 — Simulation
------------------------

Run the full Go2 navigation stack without any hardware — replay recorded sessions or simulate in MuJoCo. Same code, no robot.

See :doc:`Setup </platforms/quadruped/go2/setup>` for installing DimOS first.

.. _try-it--no-hardware-needed:

.. _doc-platforms-quadruped-go2-simulation--try-it-no-hardware-needed:

Try It — No Hardware Needed
---------------------------

.. code-block:: bash

   # Replay a recorded Go2 navigation session
   # First run downloads ~2.4 GB of LiDAR/video data from LFS
   dimos --replay run unitree-go2

Opens the command center at `localhost:7779 <http://localhost:7779>`__ with Rerun 3D visualization — watch the Go2 map and navigate an office in real time.

.. _doc-platforms-quadruped-go2-simulation--mujoco-simulation:

MuJoCo Simulation
-----------------

.. code-block:: bash

   uv pip install 'dimos[base,unitree,sim]'
   dimos --simulation run unitree-go2

Full navigation stack in MuJoCo — same code, simulated robot.
