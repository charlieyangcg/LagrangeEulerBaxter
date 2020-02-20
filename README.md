[![MIT licensed](https://img.shields.io/badge/license-MIT-brightgreen.svg)](LICENSE)

# LagrangeEulerBaxter
Uses symbolic method with Baxter kinematic and dynamic information to generate symbolic matrices representing closed form dynamics of the Baxter manuipulator.

## **Development of a dynamics model for the Baxter robot** ##
  
  ### Abstract
  
  The dynamics model of a robot is important to find the relation between the joint actuator torques and the resulting motion. There are two common methods to do this: The Lagrange formulation, which gives a closed form of the dynamics equations, and the Newton-Euler method, which uses a recursive form. Presented in this paper is a formulation of the Lagrange-Euler (L-E) equations representing the dynamics of the Baxter manipulator. These equations are then verified against torque trajectories recorded from the Baxter robot. Experimental studies show that torques generated using the L-E method closely match recorded actuator torques. All of Baxter's kinematic and dynamics parameters are presented here for easy future reference, and the full symbolic dynamics are made available online for closed loop analysis by the community.
  
  ### Published in:
  
  2016 IEEE International Conference on Mechatronics and Automation
  
  ### Date of Conference
  
  7-10 Aug. 2016
  
  ### Conference Location
  
  Harbin, China
  
  ### Publisher
  
  IEEE
  
  ### DOI
  
  10.1109/ICMA.2016.7558740 [Learn more](https://ieeexplore.ieee.org/document/7558740)
  

## **Dependency**

Peter Corke's robotics toolbox for MATLAB&reg;, at least version 9.10

Be careful about its license when using this toolbox. [Learn more](http://petercorke.com/wordpress/toolboxes/robotics-toolbox)

## **License**
The entire codebase is under [MIT license](LICENSE)
