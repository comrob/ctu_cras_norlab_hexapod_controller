# Parameter file
The structure of the robot is parsed from the `json` configuration file. The parameter file prescribes the individual legs with the respective parameters:
* `name` - leg name
* `base_T` - base transformation of the leg
* `dh_params` - [Denavit-Hartenberg parameters](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters) of the leg
* `theta_default` - leg joint angles in the default (standing) position of the robot
* `joint_dirs` - compensation of difference between the DH notation and the physical  mounting of the servomotors

## Parameter: name
Name to identify the leg in the robot model. 
    
    (type: string)
    
    (example (left-front leg):
      "name":"L1",
    )


## Parameter: base_T
Rigid body transformation matrix between the base frame of the robot and the coordinate frame of the leg.
    
    (type: 4x4 matrix of float)
    
    (example:
      "base_T": [
        [0.86602,-0.5,0,0.313],
        [0.5,0.86602,0,0.181],
        [0,0,1,0],
        [0,0,0,1]
      ],
    )

## Parameter: dh_params
The [Denavit-Hartenberg parameters](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters) of the leg written in format: alpha [rad], phi [rad], d [m], a [m] 
    
    (type: array(n x 4) of float)
    
    (example(3 DoF leg in yaw, pitch, pitch configuration):
      "dh_params":[
        [1.5707,0,0.06,0.0],
        [0,0,0,0.325],
        [0,0,0,0.325]
      ],
    )

## Parameter: theta_default
The leg joint angles in the default (standing) position [rad]. 
    
    (type: vector(n) of float)
    
    (example:
      "theta_default":[0,0.0,-1.5],
    )
    (default: 0)

## Parameter joint_dirs
The optional parameter of joint directions applied on the output signals to compensate the difference between the DH description of the leg and the actual positive direction of the real servomotors given their mounting on the robot.
    
    (type: vector(n) of float)
    
    (example:
      "joint_dirs":[1,-1,1]
    )

