# RSI files you supply

This directory holds the files that come from RSI and are not part of this
repository. Place them here before you build or run. Everything in this
directory except this README is ignored by git.

| File | Needed for | Used how |
|---|---|---|
| `rmp_*_amd64.deb` | `./run.sh build` | Installed into the image. It provides RapidCode (`/rsi` libraries, `RSI.cmake`, and the public headers). Put exactly one `.deb` here. |
| `rsi.lic` | `./run.sh up` | Your RapidCode license. The container reads it at `/rsi/rsi.lic` through a bind mount, so you can replace it without a rebuild. |
| `EtherCAT.xml` | `./run.sh up hardware` | The EtherCAT Network Information (ENI) file for your robot's network topology. The container reads it at `/rsi/EtherCAT.xml` through the same bind mount. Phantom mode does not need it. |

Get the `.deb` and the license from RSI. Generate the ENI with RSI's
network configuration tool for your EtherCAT topology.

`./run.sh` checks for these files and names any that are missing.
