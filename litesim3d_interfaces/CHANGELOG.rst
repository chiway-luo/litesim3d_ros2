^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package litesim3d_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Prepared for the first planned 0.4.0 release. Entries consolidate development
history; no earlier tagged releases or publication dates are implied. See the
`repository development history <../CHANGELOG.rst>`_ for chronology and evidence.

Forthcoming
-----------
* Introduce Contact messages and Step and SetPose services for collision reporting, bounded idempotent stepping and initialization before a run starts.
* Add the Reset service for coordinated session replacement with a new run identity, configuration/seed selection and explicit failure reporting.
* Add CameraFrame messages and RenderCameras service definitions for simulation-time rendering requests and RGB/depth image results.
* Set the maintainer contact to chiwayfpv@gmail.com and provide the repository Apache-2.0 license; retain existing third-party notices.

References: `Source 1 <../docs/api/消息与服务.md>`_, `Source 2 <../docs/testing/SDF实现验证.md>`_.
