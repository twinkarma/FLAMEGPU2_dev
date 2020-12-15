# Pedestrian RVO model
Pedestrian simulation model for the RateSetter+ project. The pedestrian agents performs collision avoidance using the ORCA algorithm. Each pedestrian can have sequence of goals (seek target, flee target or idle).

While the model is 2D, it is designed based on SteerSuite's convention which means that pedestrian agents move around 
on X and Z plane and the Y axis is largely ignored. Where `float2` is used in model code, the convention is to normally
re-map z to y axis (e.g. `float2 f2 = make_float2(f3.x, f3.z)`). The reverse mapping is also true (e.g. `float3 f3 = make_float3(f2.x, 0, f2.y)`).

## Running the model

The model specifically takes the following input parameters:

* `-m   --model` The the model test case in the form of SteerSuite xml format.
* `-o   --output` The recording output file path. Recording file is in the format of SteerSuite recording file.

For example:

```
pedestrian_rvo -m modelspec.xml -o output.xml 
```

