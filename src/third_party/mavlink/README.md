# Vendored MAVLink C library

Upstream: https://github.com/mavlink/c_library_v2 at commit `56f6435ee725ed15d2e6d2a4c97ab6b41886ffc9`
(see `VERSION`). Root helpers plus the `common`, `standard` and `minimal`
dialects.

The generated MAVLink C library is distributed under the MIT licence
(https://mavlink.io/en/#license).

Do not edit these files. Update them with:

    tools/mavlink/update_vendor.sh <commit-sha>

Include them only through `src/telemetry/mavlink/Mavlink.h`.
