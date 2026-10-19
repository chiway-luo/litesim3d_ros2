^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package drone_3d_models
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Prepared for the first planned 0.4.0 release. Entries consolidate development
history; no earlier tagged releases or publication dates are implied. See the
`repository development history <../../../CHANGELOG.rst>`_ for chronology and evidence.

Forthcoming
-----------
* Split the quadrotor and flight-course assets out of the original shared litesim_models package.
* Install the quadrotor model, aerial course, drone_arena world and project-authored mesh/material/texture fixtures as a resource-only package.
* Keep model, mesh and texture references self-contained, with an independent copy of shared texture fixtures and no reference to the car model directory.
* Provide reproducible generation of the shipped flight-course assets.
* Set the maintainer contact to chiwayfpv@gmail.com and provide the repository Apache-2.0 license; retain existing third-party notices.

References: `Source 1 <README.md>`_, `Source 2 <../../../scripts/fixtures/generate_example_worlds.py>`_.
