Testing
=======

The `catkin_tools` test harness includes the following types of tests,
organized into different directories:

* **unit** -- API tests for the `catkin_tools` python interface
* **system** -- Tests which not only test integrated parts of `catkin_tools`
  but the interaction with other, external projects like catkin_pkg and catkin.

## Running Tests

To run all tests and view the output, run the following in this directory:

```
nosetests -s
```
