# MAVLink C headers (vendored)

Generated MAVLink 2 C library, from https://github.com/mavlink/c_library_v2 at commit
28eae47457249ab6c8c1cadd104fcbe25eacd7a3 (2026-10-07). Only the top-level headers and the
`common`, `standard` and `minimal` dialects are kept; their `testsuite.h` files are left out.

The generated MAVLink libraries are MIT licensed (https://mavlink.io/en/#license).
Vendored so the library builds without network access (spec 07 §2, spec 14 §1).
To use another copy, set MRS_MAVLINK_DIR to a folder that holds common/mavlink.h.
