# OMPL Reeds-Shepp Core

This directory contains a small, dependency-free extraction of OMPL's
`ReedsSheppStateSpace` path solver.

Source:

- https://github.com/ompl/ompl/blob/main/src/ompl/base/spaces/ReedsSheppStateSpace.h
- https://github.com/ompl/ompl/blob/main/src/ompl/base/spaces/src/ReedsSheppStateSpace.cpp

Original author: Mark Moll.

License: BSD 3-Clause, retained in `OmplReedsShepp.hpp` and
`OmplReedsShepp.cpp`.

Local changes:

- Removed OMPL, Boost, and state-space dependencies.
- Kept the Reeds-Shepp path type table and shortest-path formula set.
- Exposed a tiny standard-library API for normalized SE(2) poses.
