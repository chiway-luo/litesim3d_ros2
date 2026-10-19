^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package litesim3d_gui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Prepared for the first planned 0.4.0 release. Entries consolidate development
history; no earlier tagged releases or publication dates are implied. See the
`repository development history <../CHANGELOG.rst>`_ for chronology and evidence.

Forthcoming
-----------
* Use the resolvable nlohmann-json-dev key and keep the optional Ogre license within this package.
* Introduce a Qt5/Ogre2 scene viewer with orbit, pan, zoom and resize controls that affect only the observation camera, plus measured real-time-factor, FPS and collision-budget status.
* Render supported SDF meshes, per-material OBJ/DAE textures and PBR resources; restore the full static scene when the viewer reconnects.
* Add RGB, depth and RGB-D rendering, Image/CameraInfo publication, independent camera rates and read-only image previews.
* Preserve metric axial depth, optical frames, matching RGB-D timestamps/intrinsics and NaN values beyond the depth range.
* Keep camera rendering independent of scene-viewer and image-preview lifetime, and propagate failure of the required renderer.
* Add project-local macOS Metal backend selection and installed relative-library lookup fixes while preserving the bundled OGRE license notice.
* Set the maintainer contact to chiwayfpv@gmail.com and provide the repository Apache-2.0 license; retain existing third-party notices.

References: `Source 1 <../docs/testing/SDF实现验证.md>`_, `Source 2 <../docs/testing/无人机与办公场景验证.md>`_.
