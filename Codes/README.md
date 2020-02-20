# Baxter Simulator

## Overview
Uses symbolic method with Baxter kinematic and dynamic information to generate symbolic matrices representing closed form dynamics of the Baxter manuipulator.

## Running
Run LagrangeEulerBaxterV4.m to generate symbolic dynamics of Baxter robot. The function roundSymbolic() must be in the local folder or path. 

## Attention
This is very memory intensive due to the large number of symbolic coefficents generated in the matrices. To aid in this, variables are saved to a .mat file after generation, and then cleared from RAM.
