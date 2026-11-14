# Third-Party Notices

The CommonRoad Environment Model is licensed under the BSD 3-Clause License (see [LICENSE.txt](LICENSE.txt)).
It contains or links against third-party software and data, which are subject to their own licenses as listed below.

## 1. Code included in this repository

| File(s) | Origin | License |
|---|---|---|
| `include/commonroad_cpp/roadNetwork/lanelet/graph.h`, `include/commonroad_cpp/roadNetwork/lanelet/dijkstra.h` | [MartenBE/varia](https://gitlab.com/MartenBE/varia) | MIT, Copyright (C) 2020 Martijn Saelens |
| `include/commonroad_cpp/auxiliaryDefs/timer.h`, `src/commonroad_cpp/auxiliaryDefs/timer.cpp` | [AutoPas](https://github.com/AutoPas/AutoPas) | BSD-2-Clause, Copyright (c) 2018, AutoPas development team |
| `cmake/CodeCoverage.cmake` | [cmake-modules](https://github.com/bilke/cmake-modules) | BSD-3-Clause, Copyright (c) 2012 - 2017, Lars Bilke (license text in the file header) |

### MIT License (graph.h, dijkstra.h)

```
Copyright (C) 2020 Martijn Saelens

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

### BSD 2-Clause License (timer.h, timer.cpp)

```
Copyright (c) 2018, AutoPas development team
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

## 2. Dependencies

The following libraries are not part of this repository.
They are either taken from the system or downloaded at build time via CMake `FetchContent`.
Binary distributions (e.g., the Python wheels) link the libraries marked "bundled" statically.

| Library | Usage | License | Copyright |
|---|---|---|---|
| [Boost](https://www.boost.org/) | bundled | BSL-1.0 | Boost contributors |
| [CommonRoad CLCS](https://github.com/CommonRoad/commonroad-clcs) | bundled | BSD-3-Clause | Copyright (c) 2019, Technical University of Munich, Professorship Cyber-Physical Systems |
| [Eigen](https://eigen.tuxfamily.org/) | bundled (header-only) | MPL-2.0 | Eigen contributors; source code available at https://gitlab.com/libeigen/eigen |
| [nanobind](https://github.com/wjakob/nanobind) | bundled (Python bindings) | BSD-3-Clause | Copyright (c) 2022 Wenzel Jakob |
| [Protocol Buffers](https://github.com/protocolbuffers/protobuf) | bundled | BSD-3-Clause | Copyright 2008 Google Inc. |
| [pugixml](https://github.com/zeux/pugixml) | bundled | MIT | Copyright (c) 2006-2025 Arseny Kapoulkine |
| [range-v3](https://github.com/ericniebler/range-v3) | bundled (header-only) | BSL-1.0 | Copyright Eric Niebler and contributors |
| [spdlog](https://github.com/gabime/spdlog) | bundled | MIT | Copyright (c) 2016 Gabi Melman |
| [robin-map](https://github.com/Tessil/robin-map) | bundled (header-only) | MIT | Copyright (c) 2017 Thibaut Goetghebuer-Planchon |
| [yaml-cpp](https://github.com/jbeder/yaml-cpp) | bundled | MIT | Copyright (c) 2008-2015 Jesse Beder |
| [GoogleTest](https://github.com/google/googletest) | tests only, not distributed | BSD-3-Clause | Copyright 2008, Google Inc. |
| [commonroad-io](https://github.com/CommonRoad/commonroad-io) | Python runtime dependency, not bundled | BSD-3-Clause | Copyright 2021 Technical University of Munich, Professorship of Cyber-Physical Systems |

### MIT License (pugixml, spdlog, robin-map, yaml-cpp)

The copyright notices for these libraries are listed in the table above.

```
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

### BSD 3-Clause License (CommonRoad CLCS, nanobind, Protocol Buffers)

The copyright notices for these libraries are listed in the table above.

```
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

### Boost Software License 1.0 (Boost, range-v3)

Binary distributions are exempt from the notice requirement of the BSL-1.0.
See https://www.boost.org/LICENSE_1_0.txt for the full text.

### Mozilla Public License 2.0 (Eigen)

Eigen is used unmodified. Its source code is available at https://gitlab.com/libeigen/eigen.
See https://www.mozilla.org/en-US/MPL/2.0/ for the full text.

## 3. Test data

The CommonRoad scenarios in `tests/scenarios` are partially derived from third-party data sources.
See [tests/scenarios/README.md](tests/scenarios/README.md) for details and attributions.
