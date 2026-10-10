PythonFinalizationError
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from dataclasses import dataclass
import numpy as np
import math

@dataclass
class State:
    xpos: float
    ypos: float
    xvel: float
    time: float
    lateralvelocity: float = 0

# time increments of .01 sec
timestep = 0.01

# Throttle: this function determines car position while accelerating 
def step (state:State) -> State:
    maxtorque = 180 # in NMV
    carmass = 300 # in kg
    gearratio = 3
    wheelradius = 0.216 # in M
    
    # conditional to calculate driverinput at current state.time
    if state.time <= 7:
        driverinput = state.time / 7
    elif state.time <= 22:
        driverinput = 1
    else:
        driverinput = 0

    # physics equations to get new state info
    commandtorque = driverinput * maxtorque
    wheelforce = (commandtorque * gearratio) / wheelradius
    acceleration = wheelforce / carmass
   
    newtime = state.time + timestep
    newxpos = timestep * state.xvel + state.xpos
    newvel = acceleration * timestep + state.xvel

    # creating new state to return
    newstate = State(
            xpos = newxpos,
            ypos = 0,
            xvel = newvel,
            time = newtime
            )

    return newstate

# traction: This will update the state's lateral velocity
def traction(state:State) -> State:
    forwardspeed = 15 # in m/s
    cornerstiffness = 36000 # in N/rad
    carmass = 300 # in kg
    degreechange = 5 # change from 0 - 5
    radianchange = (math.pi / 180.0) * degreechange # change degrees to radians using math library value for pi
    
    # conditional to  calculate steerangle at current state.time
    if state.time <= 3:
        steerangle = (radianchange / 3) * state.time
    elif state.time <= 7:
        steerangle = radianchange 
    else:
        steerangle = 0
    
    # physics equations to update lateral velocity
    slipangle = steerangle - (lateralvelocity/forwardspeed)
    lateralforce = cornerstiffness * slipangle
    lateralacceleration = lateralforce/mass
    newlatvelocity = state.lateralvelocity + (lateralacceleration * timestep)

    newtime = state.time + timestep
    newvel = forwardspeed # not updating for simplicity, but irl this would likely lower by some portion of lateral velocity
    newxpos = timestep * state.xvel + state.xpos 

    newstate = State(
            xpos = newxpos,
            ypos = 0, # turning may actually cause this to update if we consider a topdown view?
            xvel = newvel,
            time = newtime,
            lateralvelocity = newlatvelocity
            )

    
    return newstate

    
# Drag: This function simulates a car speeding up for 10 seconds then slowing down by the drag force
def drag(state:State) -> State:
    airdensity = 1.2 # in kg/m^3
    area = 1.2 # in m^2
    dragcoef = 0.7
    carmass = 300 

    # accelerate from 0 at 5 m/s^2 for 10 sec
    if state.time <= 10:
        acceleration = 5
        # Calculate drag, total acceleration, update velocity
        drag = .5 * area * dragcoef * airdensity * state.xvel * state.xvel
        netacceleration = acceleration - (drag / carmass)
        velocity = state.xvel + (netacceleration * timestep)

    # stop accelerating, let drag slow the car to 0 (stop at 0.1 m/s) 
    elif state.xvel > 0.1:
        acceleration = 0 
        #calculate drag, total acceleration, update velocity
        drag = .5 * area * dragcoef * airdensity * state.xvel * state.xvel
        netacceleration = acceleration - (drag / carmass)
        velocity = state.xvel + (netacceleration * timestep)
              
    else: #car is now stopped
        velocity = 0

    newxpos = state.xpos + (state.xvel * timestep)
    newtime = state.time + timestep

    newstate = state(
            xpos = newxpos,
            ypos = 0,
            xvel = velocity,
            time = newtime
            )

    return newstate

# Motor simulates an engine's max speed so we're not accelerating towards infinity
def Motor(state:State) -> State:
    carmass = 300 
    maxvelocity = 27 
    maxpropforce = 2000 # in N
    
    # conditional to calculate driverinput at current state.time
    if state.time <= 3:
        driverinput = state.time / 3
    elif state.time <= 23:
        driverinput = 1
    else:
        driverinput = 0

    # physics equations to get new state info
    propforce = maxpropforce * driverinput * (1 - (state.xvel / maxvelocity))
    acceleration = propforce / mass

    newtime = state.time + timestep
    newxpos = timestep * state.xvel + state.xpos
    newvel = acceleration * timestep + state.xvel

    # creating new state to return
    newstate = State(
            xpos = newxpos,
            ypos = 0,
            xvel = newvel,
            time = newtime
            )

    return newstate

def brake(state:State) -> State:
    maxbreak = 1850 # in N
    carmass = 300
    if state.time <= 2:
        state.xvel = 25 #sets velocity to 25 m/s for first 2 sec
        driverinput = 0
    else:
        driverinput = 1

    brakeforce = driverinput*maxbreak # will be 0 for first 2 sec
    acceleration = brakeforce / mass # also 0 for first 2 sec
    
    newvelocity = state.xvel + (acceleration + timestep) 
    newtime = state.time + timestep
    newxpos = timestep * state.xvel + state.xpos
    
    if newvelocity < 0:
        newvelocity = 0 # ensures velocity doesn't drop below 0
    # instructions mention stopping sim when we reach 0, since this function just returns a current state
    # the code calling this should have some sort of check to stop once it registers state.xvel at 0?
    newstate = State(
            xpos = newxpos,
            ypos = 0,
            xvel = newvel,
            time = newtime
            )


    return newstate

def animate (i):
    global s0
    s0 = step(s0)
    ax.clear()
    ax.scatter([s0.xpos], [s0.ypos], s = 200, c = 'pink', marker = 's')
    ax.set_xlim(0,300)
    ax.set_ylim(0,10)
    return ax,

s0 = State(
        xpos = 0,
        ypos = 0,
        xvel = 2,
        time = 0
)


fig = plt.figure(figsize=(3,3), dpi=150)
ax = fig.add_subplot(111)
ax.grid()
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
# these lines are so the animation doesnt zoom in or out
plt.pause(3)
ani = animation.FuncAnimation(fig, animate, interval=0)
plt.show()
