.. _doc-development-large-file-management--data-loading:

============
Data Loading
============

.. _doc-development-large_file_management--data-loading:

============
Data Loading
============

The `get_data <https://github.com/dimensionalOS/dimos/blob/main/dimos/utils/data.py>`__ function provides access to test data and model files, handling Git LFS downloads automatically.

.. _doc-development-large_file_management--basic-usage:

Basic Usage
-----------

.. code-block:: python

   from dimos.utils.data import get_data

   # Get path to a data file/directory
   data_path = get_data("cafe.jpg")
   print(f"Path: {data_path}")
   print(f"Exists: {data_path.exists()}")

.. code-block:: text

   Path: /home/lesh/coding/dimos/data/cafe.jpg
   Exists: True

.. _doc-development-large_file_management--how-it-works:

How It Works
------------

.. raw:: html

   <details>
   <summary>Pikchr</summary>

.. code-block:: text

   color = white
   fill = none

   A: box "get_data(name)" rad 5px fit wid 170% ht 170%
   arrow right 0.4in
   B: box "Check" "data/{name}" rad 5px fit wid 170% ht 170%

   # Branch: exists
   arrow from B.e right 0.3in then up 0.4in then right 0.3in
   C: box "Return path" rad 5px fit wid 170% ht 170%

   # Branch: missing
   arrow from B.e right 0.3in then down 0.4in then right 0.3in
   D: box "Pull LFS" rad 5px fit wid 170% ht 170%
   arrow right 0.3in
   E: box "Decompress" rad 5px fit wid 170% ht 170%
   arrow right 0.3in
   F: box "Return path" rad 5px fit wid 170% ht 170%

.. raw:: html

   </details>

|output|

1. Checks if ``data/{name}`` already exists locally
2. If missing, pulls the ``.tar.gz`` archive from Git LFS
3. Decompresses the archive to ``data/``
4. Returns the ``Path`` to the extracted file/directory

.. _doc-development-large_file_management--common-patterns:

Common Patterns
---------------

.. _doc-development-large_file_management--loading-images:

Loading Images
~~~~~~~~~~~~~~

.. code-block:: python

   from dimos.utils.data import get_data
   from dimos.msgs.sensor_msgs.Image import Image

   image = Image.from_file(get_data("cafe.jpg"))
   print(f"Image shape: {image.data.shape}")

.. code-block:: text

   Image shape: (771, 1024, 3)

.. _doc-development-large_file_management--loading-model-checkpoints:

Loading Model Checkpoints
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from dimos.utils.data import get_data

   model_dir = get_data("models_yolo")
   checkpoint = model_dir / "yolo11n.pt"
   print(f"Checkpoint: {checkpoint.name} ({checkpoint.stat().st_size // 1024}KB)")

.. code-block:: text

   Checkpoint: yolo11n.pt (5482KB)

.. _doc-development-large_file_management--loading-recorded-data-for-replay:

Loading Recorded Data for Replay
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from dimos.utils.data import get_data
   from dimos.utils.testing.replay import TimedSensorReplay

   data_dir = get_data("unitree_office_walk")
   replay = TimedSensorReplay(data_dir / "lidar")
   print(f"Replay {replay} loaded from: {data_dir.name}")
   print(replay.find_closest_seek(1))

.. code-block:: text

   Replay <dimos.utils.testing.replay.TimedSensorReplay object at 0x7fdc24c708f0> loaded from: unitree_office_walk
   {'type': 'msg', 'topic': 'rt/utlidar/voxel_map_compressed', 'data': {'stamp': 1751591000.0, 'frame_id': 'odom', 'resolution': 0.05, 'src_size': 77824, 'origin': [-3.625, -3.275, -0.575], 'width': [128, 128, 38], 'data': {'points': array([[ 2.725, -1.025, -0.575],
          [ 2.525, -0.275, -0.575],
          [ 2.575, -0.275, -0.575],
          ...,
          [ 2.675, -0.525,  0.775],
          [ 2.375,  1.175,  0.775],
          [ 2.325,  1.225,  0.775]], shape=(22730, 3))}}}

.. _doc-development-large_file_management--loading-point-clouds:

Loading Point Clouds
~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from dimos.utils.data import get_data
   from dimos.mapping.pointclouds.util import read_pointcloud

   pointcloud = read_pointcloud(get_data("apartment") / "sum.ply")
   print(f"Loaded pointcloud with {len(pointcloud.points)} points")

.. code-block:: text

   Loaded pointcloud with 63672 points

.. _doc-development-large_file_management--data-directory-structure:

Data Directory Structure
------------------------

Data files live in ``data/`` at the repo root. Large files are stored in ``data/.lfs/`` as ``.tar.gz`` archives tracked by Git LFS.

.. raw:: html

   <details>
   <summary>Diagram</summary>

.. code-block:: text

   data/
     cafe.jpg
     apartment/
       sum.ply
     .lfs/
       cafe.jpg.tar.gz
       apartment.tar.gz

.. raw:: html

   </details>

.. code-block:: text

   data/
    ├──cafe.jpg
    ├──apartment/
    │   └──sum.ply
    └──.lfs/
        ├──cafe.jpg.tar.gz
        └──apartment.tar.gz

.. _doc-development-large_file_management--adding-new-data:

Adding New Data
---------------

.. _small-files--1mb:

.. _doc-development-large_file_management--small-files-1mb:

Small Files (< 1MB)
~~~~~~~~~~~~~~~~~~~

Commit directly to ``data/``:

.. code-block:: sh

   cp my_image.jpg data/

   # 2. Compress and upload to LFS
   ./bin/lfs_push

   git add data/.lfs/my_image.jpg.tar.gz

   git commit -m "Add test image"

.. _doc-development-large_file_management--large-files-or-directories:

Large Files or Directories
~~~~~~~~~~~~~~~~~~~~~~~~~~

Use the LFS workflow:

.. code-block:: sh

   # 1. Copy data to data/
   cp -r my_dataset/ data/

   # 2. Compress and upload to LFS
   ./bin/lfs_push

   git add data/.lfs/my_dataset.tar.gz

   # 3. Commit the .tar.gz reference
   git commit -m "Add my_dataset test data"

The `lfs_push <https://github.com/dimensionalOS/dimos/blob/main/bin/lfs_push>`__ script:

1. Compresses ``data/my_dataset/`` → ``data/.lfs/my_dataset.tar.gz``
2. Uploads to Git LFS
3. Stages the compressed file

A pre-commit hook (`bin/hooks/lfs_check <https://github.com/dimensionalOS/dimos/blob/main/bin/hooks/lfs_check#L26>`__) blocks commits if you have uncompressed directories in ``data/`` without a corresponding ``.tar.gz`` in ``data/.lfs/``.

.. _doc-development-large_file_management--location-resolution:

Location Resolution
-------------------

When running from:

- **Git repo**: Uses ``{repo}/data/``
- **Installed package**: Clones repo to user data dir:

  - Linux: ``~/.local/share/dimos/repo/data/``
  - macOS: ``~/Library/Application Support/dimos/repo/data/``
  - Fallback: ``/tmp/dimos/repo/data/``

.. _doc-development-large_file_management--docs-media-assets:

Docs media assets
-----------------

Binary media displayed by the docs (screenshots, plots, and GIFs) is
LFS-tracked under ``docs/**/assets/``. The Sphinx build does not fetch Git LFS
objects.

So content behind those LFS pointers is hosted as plain git blobs in `dimensionalOS/dimos-docs-assets <https://github.com/dimensionalOS/dimos-docs-assets>`__ (mirroring the ``docs`` tree, minus the ``docs/`` prefix), and docs pages reference that copy with an absolute URL:

.. code-block:: rst

   .. image:: https://raw.githubusercontent.com/dimensionalOS/dimos-docs-assets/main/capabilities/navigation/assets/coverage.png

Small text SVGs emitted by pikchr and ``to_svg`` are exempt: they are committed here as plain text and referenced relatively.

.. |output| image:: assets/get_data_flow.svg
