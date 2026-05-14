# baxter_interface migration

## Organization of executable logic

ROS 1

In ROS 1 it was a standalone script in scripts/. In ROS 2 the pattern is to put executable logic as a module and register it as a console_scripts entry point in setup.py:

```python
  entry_points={
      'console_scripts': [
          'analog_io_rampup = baxter_examples.analog_io_rampup:main',
      ],
  },

```
