# Third-party files

Most dependencies are downloaded at build time and are not part of this repository: Eigen,
GoogleTest, Google Benchmark and pybind11, pinned in `cmake/StapesDependencies.cmake`, and NumPy,
Matplotlib and pytest, pinned in `python/requirements.txt`.

The files below are kept in the repository so that the viewer does not depend on a
network. They remain under their own licences, which are not changed by this project's licence.

| Files | Project | Version | Licence |
|---|---|---|---|
| `viewer/vendor/three.module.min.js`, `three.core.min.js`, `OrbitControls.js` | [three.js](https://threejs.org) | 0.180.0 | MIT, in `viewer/vendor/LICENSE-three.txt` |
