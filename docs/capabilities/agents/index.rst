.. _doc-capabilities-agents-index--agents:

======
Agents
======

LLM agents run as native DimOS modules. They subscribe to camera, LiDAR, odometry, and spatial memory streams and they control the robot through skills.

.. _doc-capabilities-agents-index--architecture:

Architecture
------------

.. code-block:: text

   Human Input ──→ Agent ──→ Skill Calls ──→ Robot
     (text/voice)     │         (RPC)
                      │
             subscribes to streams:
             color_image, odom, spatial_memory

**McpClient** (``dimos/agents/mcp/mcp_client.py``) is a :class:`Module <dimos.core.module.Module>` with:

- ``human_input: In[str]``: receives text from ``humancli``, :class:`WebInput <dimos.agents.web_human_input.WebInput>`, or ``agent-send``
- ``agent: Out[BaseMessage]``: publishes agent responses (text, tool calls, images)
- ``agent_idle: Out[bool]``: signals when the agent is waiting for input

The agent uses LangGraph with a configurable LLM. The default is ``gpt-4o`` and you need to provide an ``OPENAI_API_KEY`` environment variable. On startup, it discovers all :func:`@skill <dimos.agents.annotation.skill>`-annotated methods across deployed modules via RPC and exposes them as LangChain tools.

.. _doc-capabilities-agents-index--skills:

Skills
------

Skills are methods decorated with :func:`@skill <dimos.agents.annotation.skill>` on any :class:`Module <dimos.core.module.Module>`. The agent discovers them automatically at startup.

.. code-block:: python

   from dimos.agents.annotation import skill
   from dimos.core.module import Module

   class MySkillContainer(Module):
       @skill
       def wave_hello(self) -> str:
           """Wave at the nearest person."""
           # ... robot control logic ...
           return "Waving!"

**Rules:**

- Parameters must be JSON-serializable primitives (``str``, ``int``, ``float``, ``bool``, ``list``, ``dict``).
- Docstrings become the tool description the LLM sees. Write them clearly so the agent has sufficient context.
- The function must return a string or image which will be used by the agent to decide what to do next.

.. _doc-capabilities-agents-index--built-in-skills:

Built-in Skills
~~~~~~~~~~~~~~~

+-------------------------------------------+--------------------------------------------------------------------------------------------------------------+-------------------------------------------------+
| Skill                                     | Module                                                                                                       | Description                                     |
+===========================================+==============================================================================================================+=================================================+
| ``relative_move(forward, left, degrees)`` | :class:`UnitreeSkillContainer <dimos.robot.unitree.unitree_skill_container.UnitreeSkillContainer>`           | Move robot relative to current position         |
+-------------------------------------------+--------------------------------------------------------------------------------------------------------------+-------------------------------------------------+
| ``execute_sport_command(command_name)``   | :class:`UnitreeSkillContainer <dimos.robot.unitree.unitree_skill_container.UnitreeSkillContainer>`           | Unitree sport commands (sit, stand, flip, etc.) |
+-------------------------------------------+--------------------------------------------------------------------------------------------------------------+-------------------------------------------------+
| ``wait(seconds)``                         | :class:`UnitreeSkillContainer <dimos.robot.unitree.unitree_skill_container.UnitreeSkillContainer>`           | Pause execution                                 |
+-------------------------------------------+--------------------------------------------------------------------------------------------------------------+-------------------------------------------------+
| ``observe()``                             | :class:`GO2Connection <dimos.robot.unitree.go2.connection.GO2Connection>`                                    | Capture and return current camera frame         |
+-------------------------------------------+--------------------------------------------------------------------------------------------------------------+-------------------------------------------------+
| ``navigate_with_text(query)``             | :class:`NavigationSkillContainer <dimos.agents.skills.navigation.NavigationSkillContainer>`                  | Navigate to a location by description           |
+-------------------------------------------+--------------------------------------------------------------------------------------------------------------+-------------------------------------------------+
| ``tag_location(name)``                    | :class:`NavigationSkillContainer <dimos.agents.skills.navigation.NavigationSkillContainer>`                  | Tag current position for later recall           |
+-------------------------------------------+--------------------------------------------------------------------------------------------------------------+-------------------------------------------------+
| ``stop_navigation()``                     | :class:`NavigationSkillContainer <dimos.agents.skills.navigation.NavigationSkillContainer>`                  | Cancel current navigation goal                  |
+-------------------------------------------+--------------------------------------------------------------------------------------------------------------+-------------------------------------------------+
| ``follow_person(query)``                  | ``PersonFollowSkill``                                                                                        | Visual servoing to follow a described person    |
+-------------------------------------------+--------------------------------------------------------------------------------------------------------------+-------------------------------------------------+
| ``stop_following()``                      | ``PersonFollowSkill``                                                                                        | Stop person following                           |
+-------------------------------------------+--------------------------------------------------------------------------------------------------------------+-------------------------------------------------+
| ``speak(text)``                           | :class:`SpeakSkill <dimos.agents.skills.speak_skill.SpeakSkill>`                                             | Text-to-speech through robot speakers           |
+-------------------------------------------+--------------------------------------------------------------------------------------------------------------+-------------------------------------------------+
| ``where_am_i()``                          | :class:`GoogleMapsSkillContainer <dimos.agents.skills.google_maps_skill_container.GoogleMapsSkillContainer>` | Current street/area from GPS                    |
+-------------------------------------------+--------------------------------------------------------------------------------------------------------------+-------------------------------------------------+
| ``get_gps_position_for_queries(queries)`` | :class:`GoogleMapsSkillContainer <dimos.agents.skills.google_maps_skill_container.GoogleMapsSkillContainer>` | Look up GPS coordinates                         |
+-------------------------------------------+--------------------------------------------------------------------------------------------------------------+-------------------------------------------------+
| ``set_gps_travel_points(points)``         | ``GPSNavSkill``                                                                                              | Navigate via GPS waypoints                      |
+-------------------------------------------+--------------------------------------------------------------------------------------------------------------+-------------------------------------------------+
| ``map_query(query)``                      | :class:`OsmSkill <dimos.agents.skills.osm.OsmSkill>`                                                         | Search OpenStreetMap with VLM                   |
+-------------------------------------------+--------------------------------------------------------------------------------------------------------------+-------------------------------------------------+

.. _doc-capabilities-agents-index--mcp:

MCP
---

All agentic blueprints use two modules: :class:`McpServer <dimos.agents.mcp.mcp_server.McpServer>` and :class:`McpClient <dimos.agents.mcp.mcp_client.McpClient>`.

- :class:`McpServer <dimos.agents.mcp.mcp_server.McpServer>` exposes the methods annotated with :func:`@skill <dimos.agents.annotation.skill>` as MCP tools. Any external client can connect to the server to use the MCP tools.
- :class:`McpClient <dimos.agents.mcp.mcp_client.McpClient>` has a LangGraph LLM which calls MCP tools from :class:`McpServer <dimos.agents.mcp.mcp_server.McpServer>`.

CLI access:

.. code-block:: bash

   dimos mcp list-tools                                # List available skills
   dimos mcp call relative_move --arg forward=0.5      # Call a skill
   dimos mcp status                                    # Server status

.. _doc-capabilities-agents-index--input-methods:

Input Methods
-------------

+-----------------------------------------------------------+-----------------------------------------------------------+
| Method                                                    | How it works                                              |
+===========================================================+===========================================================+
| ``humancli``                                              | Standalone terminal — type messages, see responses        |
+-----------------------------------------------------------+-----------------------------------------------------------+
| ``dimos agent-send "text"``                               | One-shot CLI command via LCM                              |
+-----------------------------------------------------------+-----------------------------------------------------------+
| :class:`WebInput <dimos.agents.web_human_input.WebInput>` | Web interface at localhost:7779 with optional Whisper STT |
+-----------------------------------------------------------+-----------------------------------------------------------+

.. _doc-capabilities-agents-index--models:

Models
------

+---------------------+--------------------------+----------------------------------------------+
| Config              | Model                    | Notes                                        |
+=====================+==========================+==============================================+
| Default             | ``gpt-4o``               | Best quality, requires ``OPENAI_API_KEY``    |
+---------------------+--------------------------+----------------------------------------------+
| ``ollama:llama3.1`` | Local Ollama             | Requires ``ollama serve`` running            |
+---------------------+--------------------------+----------------------------------------------+
| Custom              | Any LangChain-compatible | Set via ``McpClient.blueprint(model="...")`` |
+---------------------+--------------------------+----------------------------------------------+
