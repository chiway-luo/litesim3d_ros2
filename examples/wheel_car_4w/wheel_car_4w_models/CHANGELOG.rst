^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wheel_car_4w_models
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Prepared for the first planned 0.4.0 release. Entries consolidate development
history; no earlier tagged releases or publication dates are implied.

Forthcoming
-----------
* Split car assets out of the original litesim_models package into a self-contained, resource-only package.
* Install the four-wheel robot, furnished office, warehouse, basic obstacles and project-authored mesh/material/texture fixtures.
* Keep visual and collision geometry distinct and preserve doorways and under-table/bridge clearance in the SDF scenes.
* Keep all car scene texture references inside this package; the drone example now uses its own drone_3d_models resources.
* Provide reproducible scene generation while shipping the generated assets so users do not need to run the generator.
* Set the maintainer contact to chiwayfpv@gmail.com and provide the repository Apache-2.0 license; retain existing third-party notices.

References: `Source 1 <README.md>`_, `Source 2 <../../../scripts/fixtures/generate_example_worlds.py>`_.
