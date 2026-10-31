# landfall_bringup

XML launch files, parameter files (including per-distro Nav2 overlays), the RViz configuration and
simulation assets for Landfall.

Status: the simulation assets used to record and evaluate maps (since M2, see
`docs/recording.md`):

| Path | Content |
|---|---|
| `sim/urdf/` | TurtleBot 4 description derived from `nav2_minimal_tb4_description`: 640 x 480 camera, Gazebo ground-truth pose |
| `sim/scripts/start_depot_sim.sh` | depot simulation with GPU headless rendering, Nav2 and the ground-truth bridge |
| `sim/scripts/record_depot.sh` | the M2 recording drive: recorder, bag, route |
| `sim/routes/`, `sim/params/` | the route, and the AMCL, recorder and builder settings of the simulation |

They run in the simulation image (`docker/Dockerfile.sim`, through `scripts/sim.sh`), so the
simulator packages are not dependencies of this package. Launch and parameter files for robots
arrive in M6 (`docs/TODO.md` T-001).
