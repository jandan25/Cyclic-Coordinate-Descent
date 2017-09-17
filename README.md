# Cyclic-Coordinate-Descent
Realization Cyclic Coordinate Descent method of inverse kinematics.

[Cyclic Coordinate Descent](http://www.ryanjuckett.com/programming/cyclic-coordinate-descent-in-2d/) is the position method already calculated in the forward and backward modes. This method, unlike the transformation of rotations, reverses the task of finding the position of a node in the task of finding a point on a line; therefore, you can save time and reduce the number of calculations.

Calculations using delimiters are used. They are used for greater similarity to real physical bodies. The main idea of ​​using limiters is to reposition and reorient the nodes within the constraints.

In the modeling process, an algorithm is used to simulate the motion of solid objects using the inverse kinematics method. The development process used the [Windows Presentation Foundation](https://msdn.microsoft.com/en-us/library/bb200104.aspx).

All materials used in this project are taken from [www.ryanjuckett.com](http://www.ryanjuckett.com/index.php?section=home).

```
Model parameters:
```
* Simulation - real-time simulation mode;
* Pause "Step by step" - simulation mode by iteration;
* Add, remove - allows you to add 1 more bone, remove 1 bone;
* Table of parameters of bones:
  * Length - the length of the bone;
  * Angle is the angle relative to the previous bone;
  * Constraints - constraints imposed on a node (in degrees);
* Position of the point - coordinates of the target for the final effector;
* Distance from the goal - the distance from the goal at which it will be considered that the final effector has reached the goal;
* Result is the current status of the model.
