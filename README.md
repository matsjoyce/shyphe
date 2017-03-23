shyphe - Stiff HIgh velocity PHysics Engine
-------------------------------------------

A rigid-body physics engine designed for high velocity space simulation.

Dependancies
------------

Runtime:

 - Python 3 (tested using 3.6)
 - Boost.Python (tested using 1.63)

For building:

 - CMake (tested using 3.7.2)

For tests:

 - pytest (tested using 3.0.7)

Building
--------

```bash
git clone https://github.com/matsjoyce/shyphe.git
cd shyphe
mkdir build
cd build
cmake ..
make
```

Testing
-------

Run `pytest`. For coverage reporting use `pytest --coverage`, and `pytest --flake8` for python style checking.

Used by
-------

 - LunyFringe

Want your project to be listed here? [Create a GitHub issue.](https://github.com/matsjoyce/shyphe/issues/new)

Notes
-----

Extra reading: http://www.kuffner.org/james/software/dynamics/mirtich/mirtichThesis.pdf
