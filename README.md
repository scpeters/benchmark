# Physics benchmarks using the Gazebo Simulator

Documentation for physics benchmarks in the [gazebo simulator](http://gazebosim.org).
This folder contains documentation for several multibody benchmark problems
in iPython notebook format.
These documents can be viewed online in the following locations
or on your own machine following the instructions given below:

* [Boxes: free-floating rigid bodies](http://nbviewer.ipython.org/github/scpeters/benchmark/blob/master/boxes_description.ipynb)

These benchmarks can be run for the physics engines supported by gazebo:

* [Open Dynamics Engine (ODE)](http://ode.org), (source code on [bitbucket](https://bitbucket.org/odedevs/ode))
* [Bullet](http://bulletphysics.org), (source code on [github](https://github.com/bulletphysics/bullet3))
* [Simbody](https://simtk.org/home/simbody), (source code on [github](https://github.com/simbody/simbody))
* [Dynamic Animation and Robotics Toolkit (DART)](http://dartsim.github.io), (source code on [github](https://github.com/dartsim/dart))

The benchmarks tests are hosted in the
[scpeters/benchmark](https://github.com/scpeters/benchmark)
repository on github.

# Instructions

To run the tests,
[build or install gazebo 6 or later
with the optional physics engines](http://gazebosim.org/tutorials/?tut=install#OptionalPhysicsEngines),
and then clone and build the benchmarks:

~~~
git clone https://github.com/scpeters/benchmark
cd benchmark
mkdir build
cd build
cmake ..
make
make test
~~~

Once the tests are completed, they will create csv files in the `test_results` folder.

To load and visualize the test results, you should make sure ipython notebook, matplotlib, and numpy are installed on your machine:
~~~
# Ubuntu Precise: do this step first
# Trusty uses ipython 1.2.1, use ppa on Precise to get the newer version
sudo apt-add-repository ppa:jtaylor/ipython

# Ubuntu Precise and Trusty
sudo apt-get install ipython-notebook python-matplotlib python-numpy

# Mac
brew install matplotlib
# do the following in specific terminals when using ipython notebook
export PYTHONPATH=/usr/local/lib/python2.7/site-packages
~~~

With the proper dependencies installed run ipython notebook from the test/accuracy/doc folder:

~~~
cd test/accuracy/doc
ipython notebook
~~~
