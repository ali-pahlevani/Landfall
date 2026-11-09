# Contributing

Issues and pull requests are welcome. Landfall publishes poses that a robot then drives on, so
changes are judged first by one question: can this make the relocalizer publish a wrong pose? A
miss (no pose) is always acceptable; a wrong pose is not.

## Building and testing

Everything builds and tests in Docker, the same way CI does:

```bash
scripts/docker_ci.sh jazzy        # or humble, lyrical
LANDFALL_VARIANT=cxx20 LANDFALL_CMAKE_ARGS="-DCMAKE_CXX_STANDARD=20" scripts/docker_ci.sh jazzy
LANDFALL_VARIANT=asan LANDFALL_SANITIZE=1 LANDFALL_PACKAGES=landfall_core scripts/docker_ci.sh jazzy
```

Build output stays in `.docker_ws/<distro>/` (git-ignored). [`docker/README.md`](docker/README.md)
describes the images. The learned backends' tests need the models
([`docs/models.md`](docs/models.md)); without them they skip. The simulation (`scripts/sim.sh`, an
NVIDIA GPU recommended) is not part of CI: see [`docs/recording.md`](docs/recording.md) and
[`docs/benchmark.md`](docs/benchmark.md).

## Checks

CI fails on any of these, so run them before a pull request:

| Check | Command |
|---|---|
| clang-format 18 | `scripts/format.sh` (rewrites), `scripts/format.sh --check` |
| clang-tidy 18 | `scripts/tidy.sh`, after `scripts/docker_ci.sh jazzy` |
| flake8 (`landfall_tools`) | `flake8 --max-line-length 99 landfall_tools` |
| Open items in the code are registered | `scripts/check_todos.sh` |
| Repository policies | `scripts/check_policies.sh` |
| `docs/parameters.md` is current | `scripts/update_parameters_doc.sh` after changing a parameter (a test fails while it is stale) |

[`.pre-commit-config.yaml`](.pre-commit-config.yaml) runs the formatting, flake8 and policy checks
before each commit (`pre-commit install`).

## Rules

- **Every distro.** Code builds on Humble, Jazzy and Lyrical (C++17, C++20 on Lyrical).
  Differences go behind the compatibility headers and CMake helpers
  ([`docs/architecture.md`](docs/architecture.md#compatibility)). Link packages with
  `landfall_link_package`, never `ament_target_dependencies`.
- **No silent stubs.** Unfinished work in the code is marked with the ID of its entry in
  [`docs/TODO.md`](docs/TODO.md), which says how to mark it, why the item is open and when it's
  due.
- **Permissive dependencies.** Anything linked must have a permissive license
  ([`docs/PLAN.md`](docs/PLAN.md) §8) and is listed in
  [`THIRD_PARTY_LICENSES.md`](THIRD_PARTY_LICENSES.md). Python is for `landfall_tools` and launch
  tests only.
- **Evidence for anything that decides.** A change to what the relocalizer accepts (a check, a
  threshold, a backend) comes with numbers: the offline evaluation (`landfall query`,
  [`docs/relocalization.md`](docs/relocalization.md#offline-evaluation)) or the kidnap benchmark,
  including the wrong poses. Tests use fixed seeds.
- **Stable interfaces.** From v1.0.0 on, the messages and services, the parameters and the
  database schema change only compatibly within a major version (PLAN §11.1). A database schema
  change ships with its migration (`landfall migrate`, [`docs/database.md`](docs/database.md)).

## License

Contributions are licensed under the Apache License 2.0, as the rest of the project (section 5 of
[`LICENSE`](LICENSE)).
