# tictoc_profiler

**Maintainer**: Daniel Maturana (dimatura@cmu.edu)

A simple wall-timer profiler.

Can output detailed runtime statistics (in csv-like format), a human-readable summary
table, or ROS messages.

Note that this is best used to time sections with ~5 ms or higher duration, and that are not in a tight loop.
Otherwise, the cost and uncertainty of the profiling itself may be significant relative to those of the timed code.

See `src/example.cpp` for an example.

# Update 08/2018

I have updated this library to use C++11 features, which allowed me to remove all Boost dependencies.
The API has not been modified, but a few new features have been added.


### License ###
[This software is BSD licensed.](http://opensource.org/licenses/BSD-3-Clause)

Copyright (c) 2015-2018, Carnegie Mellon University
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

This software additionally uses the [chobo flat map
implementation](https://github.com/Chobolabs/chobo-shl), which is MIT licensed
(see the header in `flat_map.hpp`). If this is a problem, it is trivially
removable by changing `chobo::flat_map` to `std::map` in `profiler.hpp`.
