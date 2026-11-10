# Scenario-asset CLI fixtures

These fixtures are synthetic and versioned with the tests. They exercise asset
selection, dry runs, local downloads, content checks, checksum verification and
failure diagnostics. They have no physical asteroid provenance.

`sources/triangle.tab` is a 32-byte Wavefront OBJ with three vertices and one
face. Its `.tab` suffix tests content-based validation and installation to an
`.obj` destination. The manifests pin its SHA-256 checksum.
`sources/not_obj.tab` contains invalid record names for the rejection test.

The two manifests use `fixture://` URLs. Test setup copies them to a temporary
data root and replaces those URLs with `file://` URIs pointing to these tracked
payloads. All downloaded files and deliberately modified destinations stay in
that temporary root. Production manifests and asset directories are not read.

The `fixture_large_albedo` entry declares 2.5 GB to exercise the CLI's size gate;
its source remains the same 32-byte payload. It is planning metadata for this
test case, not an albedo image or a large checked-in file.
