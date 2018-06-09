
You'll need:

* swig (Debian/Ubuntu package)
* https://github.com/pybox2d/pybox2d.git

To run 'main.py', you'll need to put the PyBox2d framework module onto the path, for example:

    export PYTHONPATH=/home/jimm/apps/pybox2d/examples

You'll also need to modify some of the relative imports inside PyBox2D. Since we're using the 'framework' module outside of the main directory, we need to modify the source to remove the relative imports, e.g. in framework.py:

    from .settings import fwSettings

needs to be changed to:

    from settings import fwSettings

In future we should find a better way of doing this import, but this works for the time being.


