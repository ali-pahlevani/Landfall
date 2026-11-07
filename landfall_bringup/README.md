# landfall_bringup

XML launch files and parameters for running Landfall with Nav2, and the simulation assets.

| Path | Content |
|---|---|
| `launch/localization.launch.xml` | map_server, AMCL and the relocalizer under one lifecycle manager, in that order (in place of nav2_bringup's localization launch) |
| `launch/navigation.launch.xml` | nav2_bringup's navigation with Landfall's behavior tree plugin and tree |
| `launch/bringup.launch.xml` | both |
| `launch/relocalizer.launch.xml` | the relocalizer, with or without a lifecycle manager of its own |
| `params/relocalizer.yaml` | a starting configuration of the relocalizer |
| `params/nav2_<distro>.yaml` | what Landfall adds to Nav2's parameters on Humble, Jazzy and Lyrical |
| `sim/urdf/` | TurtleBot 4 description derived from `nav2_minimal_tb4_description`: 640 x 480 camera, Gazebo ground-truth pose |
| `sim/scripts/start_depot_sim.sh` | depot simulation with GPU headless rendering, Nav2 and the ground-truth bridge |
| `sim/scripts/record_depot.sh` | the M2 recording drive: recorder, bag, route |
| `sim/scripts/navigate_depot.sh`, `sim/launch/` | the M6 navigation run: startup relocalization, navigation, a kidnap |
| `sim/routes/`, `sim/params/` | the route, and the simulation's AMCL, localization, recorder, relocalizer and builder settings |

See `docs/nav2_integration.md` for the launch files and `docs/recording.md` for the simulation.
`navigation.launch.xml` needs nav2_bringup, and the simulation runs in the simulation image
(`docker/Dockerfile.sim`, through `scripts/sim.sh`): neither nav2_bringup nor the simulator
packages are dependencies of this package, because they pull in the simulators.
