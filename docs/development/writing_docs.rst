.. _writing-docs:

============
Writing Docs
============

DimOS uses Sphinx with reStructuredText sources in ``docs/``.

Place user-facing guides in ``docs/usage`` or ``docs/capabilities`` and
contributor-only guides in ``docs/development``. Add every new page to the
nearest ``toctree`` so readers can discover it.

Use ``:doc:``, ``:ref:``, and Python-domain roles for internal links. Prefer
``literalinclude`` for source examples, and keep images beside the relevant
section in an ``assets`` directory. See
:doc:`/coding-agents/docs/index` for the complete authoring conventions.

Run the same strict checks as CI:

.. code-block:: bash

   uv sync --only-group docs
   uv run make -C docs html
   uv run make -C docs spelling

The HTML and spelling targets fail on warnings.
