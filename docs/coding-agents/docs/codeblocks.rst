.. _documentation-code-blocks:

=============
Code Examples
=============

Documentation examples must be accurate, minimal, and safe to copy. Use a
``literalinclude`` directive when the example already exists in the
repository. This keeps the documentation synchronized with code that tests
and static analysis can exercise.

.. code-block:: rst

   .. literalinclude:: /code/index.py
      :language: python
      :pyobject: RobotConnection

Use ``code-block`` for short commands, configuration fragments, or examples
that cannot reasonably live in a source file:

.. code-block:: rst

   .. code-block:: bash

      dimos --replay run unitree-go2

Choose the correct lexer (for example ``python``, ``bash``, ``toml``, or
``json``). Use ``text`` for pseudocode, console output, and formats without a
lexer. Do not label pseudocode as an executable language.

Validate Python examples with an automated test whenever practical. Sphinx
validates directive syntax and source paths during the strict documentation
build:

.. code-block:: bash

   uv run make -C docs html
