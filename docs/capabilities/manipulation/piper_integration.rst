.. _doc-capabilities-manipulation-piper-integration--piper-integration:

=================
Piper Integration
=================

.. _doc-capabilities-manipulation-piper_integration--optional-slcan-setup:

Optional SLCAN setup
--------------------

Use this separate path only with a serial-CAN adapter, such as ``/dev/ttyACM0``;

.. code-block:: bash

   sudo slcand -o -c -s8 /dev/ttyACM0 can0
   sudo ip link set can0 up

This is a separate prerequisite for serial-CAN adapters. It is not needed when the Piper adapter already exposes a native SocketCAN interface.

.. _doc-capabilities-manipulation-piper_integration--bring-up-a-native-piper-can-interface:

Bring up a native Piper CAN interface
-------------------------------------

Piper uses SocketCAN at 1,000,000 bit/s. For the default vendor setup, use the DimOS CLI to configure an existing CAN interface and bring it up:

.. code-block:: bash

   dimos piper can-activate can0

For a non-default bitrate, pass ``--bitrate`` explicitly:

.. code-block:: bash

   dimos piper can-activate can0 --bitrate 500000

The command asks for confirmation before requesting sudo. Verify the interface before starting a blueprint:

.. code-block:: bash

   ip link show can0

.. _doc-capabilities-manipulation-piper_integration--run-a-piper-blueprint:

Run a Piper blueprint
---------------------

Use the coordinator for the basic manipulation composition:

.. code-block:: bash

   dimos --can-port can0 run coordinator-piper

For keyboard Cartesian teleoperation, use:

.. code-block:: bash

   dimos --can-port can0 run keyboard-teleop-piper

The Quest teleoperation composition is available as:

.. code-block:: bash

   dimos --can-port can0 run teleop-quest-piper

Omitting the ``--can-port`` argument makes the control coordinator fall back
to a fake hardware adapter. This is useful for testing.
