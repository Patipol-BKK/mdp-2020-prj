# mdp-2020-prj

## Algo
Executions for running the algo is handled on the PC by a nodejs server. To start the server, navigate to the python directory and execute:
```
node app.js
```

Inputs for obstacles and starting position can be sent to the PC through HTTP GET formatted as follows:
```
127.0.0.1/set-obstacles?str=[(x1,y1,orient1),(x2,y2,orient2), ...]&&init=(x,y,orient)
```
-> can be tested by sending from any devices that are connected to the RPi wifi on a web browser

(change `127.0.0.1` to the IP address of the pc, `[(x1, y1, orient1),(x2,y2,orient2), ...]` to the list cells in which the obstacles are occupied, and `(x,y,orient)` to the starting position of the robot with the coordinates at the central of the rear axle)

The server will run `movement.py` with the given parameters
