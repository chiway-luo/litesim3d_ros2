# Render helper fixtures

`textured_crate` contains byte-for-byte copies of the OBJ, COLLADA, MTL and
referenced checker texture from
`litesim3d_assets/test/fixtures/assets/models/textured_crate`.

Keep these test resources inside the GUI package: standalone release validation
stages one package at a time as `packages/current-source`, without sibling source
packages. The relative mesh/material/texture layout is intentional and is tested
after copying it beneath another source root. The remaining SDF model files and
unreferenced texture maps are not needed by these mesh-decoding units.
