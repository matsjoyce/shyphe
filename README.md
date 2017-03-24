shyphe - Stiff HIgh velocity PHysics Engine
-------------------------------------------

A 2D rigid-body physics engine designed for high velocity space simulation. This physics engine uses [continuous collision detection](https://en.wikipedia.org/wiki/Collision_detection#A_posteriori_.28discrete.29_versus_a_priori_.28continuous.29), so tunneling does not occur.

FAQs
----

 - **What can I do with shyphe?**

     Shyphe is designed to be used for space simulation games without gravity.

 - **Is shyphe 3D?**

     No, shyphe is a 2D physics engine. For a 3D engine look at something like [Bullet](http://bulletphysics.org/wordpress/).

 - **Can I make springs and joins and stuff?**

     No, shyphe is an impulse based physics engine. For a constraint-based physics engine see [Chipmunk] or [Box2D]

 - **What is shyphe bad at?**

     Shyphe currently cannot do:

      - Friction
      - Gravity
      - Continuous contact (e.g. pile of blocks with gravity)

     Some of these things are on our TODO list, however.

Dependencies
------------

Runtime (python module):

 - [Python 3](https://github.com/python/cpython) (tested using 3.6)
 - [Boost.Python](https://github.com/boostorg/python) (tested using 1.63)

For building:

 - CMake (tested using 3.7.2)

For tests:

 - [pytest](http://docs.pytest.org/en/latest/index.html) (tested using 3.0.7)

For examples:

 - [pygame-go](https://github.com/matsjoyce/pygame-go) (tested using 1.0)

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
