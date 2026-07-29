.. _doc-capabilities-manipulation-agentic--agentic-xarm-simulation:

=======================
Agentic xArm Simulation
=======================

``xarm-perception-sim-agent`` runs the xArm perception, planning, MuJoCo simulation, MCP server, and built-in agent together. It is **simulation-only**; This guide uses this blueprint to provide a walk-through of dimos's agentic manipulation stack.

See the :doc:`manipulation capability overview <index>` for the underlying
planning and perception stack.

.. _doc-capabilities-manipulation-agentic--prerequisites:

Prerequisites
-------------

Install the manipulation dependencies:

.. code-block:: bash

   uv sync --extra manipulation --inexact

The built-in agent requires an ``OPENAI_API_KEY``.

.. _doc-capabilities-manipulation-agentic--start-and-stop:

Start and stop
--------------

Run in the foreground:

.. code-block:: bash

   uv run dimos run xarm-perception-sim-agent

Or run it as a daemon:

.. code-block:: bash

   uv run dimos run xarm-perception-sim-agent --daemon

Inspect and control the run from another terminal:

.. code-block:: bash

   uv run dimos status
   uv run dimos log
   uv run dimos stop

Use ``dimos log -f`` to follow the log while the run is active.

.. _doc-capabilities-manipulation-agentic--daily-interaction:

Daily interaction
-----------------

For normal interactive use, start the human-friendly terminal client:

.. code-block:: bash

   uv run dimos humancli

It connects to the running agent so you can send prompts and read responses in one session.

.. _doc-capabilities-manipulation-agentic--try-these-prompts:

Try these prompts
~~~~~~~~~~~~~~~~~

Start with a non-motion state check:

.. code-block:: text

   Report the current robot state without moving.

Scan the scene for objects. This moves the arm to its observation pose:

.. code-block:: text

   Scan for objects.

Try basic motion commands:

.. code-block:: text

   Move 10 cm to the left.

.. code-block:: text

   Move 10 cm above the detected object's pose.

.. _doc-capabilities-manipulation-agentic--debugging-and-testing-interfaces:

Debugging and testing interfaces
--------------------------------

Use ``agent-send`` for one-shot LCM input when testing or diagnosing the agent:

.. code-block:: bash

   uv run dimos agent-send "Report the current robot state and visible objects; do not move the arm or gripper."

The blueprint also includes an MCP server. Use these commands for direct server inspection and tool-level testing:

.. code-block:: bash

   uv run dimos mcp status
   uv run dimos mcp list-tools

For example:

.. code-block:: bash

   uv run dimos mcp call get_robot_state
   uv run dimos mcp call look
   uv run dimos mcp call scan_objects
