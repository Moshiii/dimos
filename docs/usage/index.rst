.. _doc-usage-index--concepts:

========
Concepts
========

.. _doc-usage-index--usage:

=====
Usage
=====

Learn how DimOS modules communicate, compose, and run.

.. _doc-usage-index--table-of-contents:

Table of Contents
-----------------

- :doc:`Modules </usage/modules>`: The primary units of deployment in DimOS, modules run in parallel and are python classes.
- :doc:`Streams </usage/sensor_streams/index>`: How modules communicate, a Pub / Sub system.
- :doc:`Blueprints </usage/blueprints>`: a way to group modules together and define their connections to each other.
- :ref:`RPC <doc-usage-blueprints--calling-the-methods-of-other-modules>`: how one module can call a method on another module (arguments get serialized to JSON-like binary data).
- :ref:`Skills <doc-usage-blueprints--defining-skills>`: An RPC function, except it can be called by an AI agent (a tool for an AI).
- Agents: AI that has an objective, access to stream data, and is capable of calling skills as tools.

.. toctree::
   :maxdepth: 2
   :hidden:

   modules
   blueprints
   configuration
   cli
   lcm
   transforms
   visualization
   python-api
   camera_calibration
   native_modules
   tool_streams
   transports/index
   data_streams/index
   sensor_streams/index
