# OpenArm robot module for [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/)

This package contains a software representation, or a robot module, of the
[OpenArm](https://docs.openarm.dev/1.0/) bimanual robot platform. This robot
module is used by the [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/)
framework to realize control of the OpenArm robot in simulation or in
experiments with the real platform.

> Both OpenArm hardware/URDF versions are supported, as separate module
> names: `OpenArmV1` and `OpenArmV2` (`OpenArm` is an alias of `OpenArmV1`).

## Required dependencies

- [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/)
- [openarm_description](https://github.com/enactic/openarm_description)
- [xacro](https://github.com/ros/xacro) (`ros-<distro>-xacro`): the URDFs are
  generated from the xacro files of openarm_description at build time

## Build instructions

```
$ source /opt/ros/<distro>/setup.bash
$ source <path-to-openarm-ws>/install/setup.bash
$ cd mc_openarm
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
```

## Use

```yaml
Robots:
  openarm:
    module: OpenArmV2 # or OpenArm (alias of OpenArmV1)
```

### On a mobile base

Every module name also exists with the `FloatingBase` suffix, for example
`OpenArmV2FloatingBase`. It is the same URDF and the same robot name
(`openarm`) with a free root at `openarm_body_link0`, the pedestal's foot. The
joints, their order and the self-collision sets do not change.

All modules have a planar surface `Base` under the pedestal's foot (the origin
of `openarm_body_link0`). A controller places the robot on the mobile base's
mount frame and adds a 6-D contact between this surface and one of the base's.
The surfaces are installed with this module (`share/mc_openarm/rsdf/openarm`),
since openarm_description has none.

## Tests

`ctest` in the build folder loads every version, fixed and floating. It checks
that the self-collision pairs name existing convexes, that `Base` loads, and
that the floating variant matches the fixed one: the same joints, and the same
pose of every body relative to the pedestal's foot.
