.. _documentation-links:

==========================
Links and Cross-References
==========================

Use Sphinx cross-references for content inside the documentation tree. The
strict build reports missing targets.

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - Target
     - Syntax
   * - Another page
     - ``:doc:\`/usage/configuration\```
   * - Named section
     - ``:ref:\`configuration-precedence\```
   * - Python class
     - ``:class:\`dimos.core.stream.In\```
   * - Python function
     - ``:func:\`dimos.core.coordination.blueprints.autoconnect\```
   * - External site
     - A standard named hyperlink, such as the Sphinx website

Give important sections explicit, stable labels immediately before the
heading:

.. code-block:: rst

   .. _configuration-precedence:

   Configuration precedence
   ========================

Use GitHub links for files that are not part of the Sphinx documentation,
such as a package-specific ``README.md``. Prefer API roles for Python symbols
because they follow the generated API documentation.
