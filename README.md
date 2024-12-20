## Controls

### Driver Controller
![flysky](docs/driver-controls.png)

### Operator Controller
![operator](docs/operator-controls.png)

## CAN Bus

| Subsystem | Type     | Talon      | ID  | Comp PDP | Proto PDP | Motor  | Breaker |
| --------- | -------- | ---------- | --- | -------- | --------- | ------ | ------- |
| Drive     | SRX      | azimuth    | 0   |  16      | 16        | 9015   |         |
| Drive     | SRX      | azimuth    | 1   |  3       | 2         | 9015   |         |
| Drive     | SRX      | azimuth    | 2   |  17      | 17        | 9015   |         |
| Drive     | SRX      | azimuth    | 3   |  2       | 4         | 9015   |         |
| Drive     | FX       | drive      | 10  |  18      | 18        | kraken |         |
| Drive     | FX       | drive      | 11  |  1       | 1         | kraken |         |
| Drive     | FX       | drive      | 12  |  19      | 19        | kraken |         |
| Drive     | FX       | drive      | 13  |  0       | 0         | kraken |         |
| Intake    | FX       | intake     | 20  |  5       | 3         | falcon |         |
| Magazine  | FX       | magazine   | 25  |  10      | 13        | falcon |         |
| Exiter    | FX       | leftExit   | 40  |  12      | 12        | falcon |         |
| Exiter    | FX       | rightExit  | 41  |  13      | 10        | falcon |         |
| -         | rio      | -          | -   | 20       |           |        |         |
| coder/sw  | vrm      | top        | -   | 21       |           |        |         |
| radio     | vrm      | bottom     | -   | 22       |           |        |         |

* intake beam break: to fwd limit
* magazine y-axis beam break: to magazine rev lim

## Roborio
| Subsystem | Interface | Device | 
| --------- | --------- | ------ |
| Drive     | USB       | NAVX   |

## DIO
| Subsystem | name           | ID  |
| --------- | -------------- | --- |
| Auto      | autoSwitch     | 0   |
| Auto      | autoSwitch     | 1   |
| Auto      | autoSwitch     | 2   |
| Auto      | autoSwitch     | 3   |
| Auto      | autoSwitch     | 4   |
| Auto      | autoSwitch     | 5   |
|           |                | 6   |
|           |                | 7   |
|           |                | 8   |
|           |                | 9   | 

## PWM
| Subsystem | name         | ID  |
| --------- | ------------ | --- |
|           |              | 0   |
|           |              | 1   |
|           |              | 2   |
|           |              | 3   |
| Lights    |  lights      | 4   |
|           |              | 5   |
|           |              | 6   |
|           |              | 7   |
|           |              | 8   |
|           |              | 9 |

## MXP
| Subsystem | name   | ID |
| --------- | ------ | -- |
|           |        |    |

