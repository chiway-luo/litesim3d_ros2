^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package litesim3d_assets
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Prepared for the first planned 0.4.0 release. Entries consolidate development
history; no earlier tagged releases or publication dates are implied. See the
`repository development history <../CHANGELOG.rst>`_ for chronology and evidence.

Forthcoming
-----------
* Own importer test registration and fixed local fixtures; remove dependencies on example test sources and declare tinyxml2/pkg-config directly.
* Introduce validated SDF scene import through libsdformat12, model/package resource resolution and conversion for the simulation core and renderer.
* Preserve visual and collision geometry separately, mesh/material references, transforms and supported OBJ/DAE/PBR resources; reject missing assets and unsupported scene behavior.
* Add optional pre-load collision complexity checks without trimming geometry, camera binding/optical-frame validation and rejection of camera-enabled pure CPU configurations.
* Extend imported runtime configuration for multicopters and verify the installed office and flight-course resources.
* Update resource fixtures and installed-asset checks for the separate wheel_car_4w_models and drone_3d_models packages.
* Set the maintainer contact to chiwayfpv@gmail.com and provide the repository Apache-2.0 license; retain existing third-party notices.

References: `Source 1 <../docs/testing/SDF实现验证.md>`_, `Source 2 <../docs/testing/无人机与办公场景验证.md>`_, `Source 3 <test/test_sdf_import.py>`_.
