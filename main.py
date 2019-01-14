import krpc
import time
import math
import matplotlib.pyplot as plt


conn = krpc.connect(name='Drone Controller')
vessel = conn.space_center.active_vessel
resources=vessel.resources


orbit_info=vessel.orbit
apoapsis=conn.add_stream(getattr,orbit_info,"apoapsis_altitude")
periapsis=conn.add_stream(getattr,orbit_info,"periapsis")

flight_info=vessel.flight()
height=conn.add_stream(getattr,flight_info,"mean_altitude")
speed=conn.add_stream(getattr,flight_info,"velocity")
pitch=conn.add_stream(getattr,flight_info,"pitch")
prograde=conn.add_stream(getattr,flight_info,"prograde")

# Set up streams for telemetry
ut = conn.add_stream(getattr, conn.space_center, 'ut')
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')

#Reference Frames
obt_frame = vessel.orbit.body.non_rotating_reference_frame
srf_frame = vessel.orbit.body.reference_frame


heading=270
print(vessel.name)

#Pre set controls
vessel.auto_pilot.target_pitch_and_heading(90, heading)
vessel.auto_pilot.target_roll=90
vessel.auto_pilot.engage()
#vessel.auto_pilot.sas=True
vessel.control.throttle = 1
time.sleep(1)

#Launch
print("3")
print("2")
print("1")
print('Launch!')
vessel.control.activate_next_stage()

#Launch variables
target_altitude=40000

launchpad=(vessel.flight().longitude,vessel.flight().latitude)
current_pitch=90
min_pitch=30

adjust_size=0.1
next_adjust=1
boosters=False
reached_target_pitch=False
reached_target_min_pitch=False
while apoapsis()<target_altitude-(target_altitude*0.1):
    speed = vessel.flight(srf_frame).speed      
    if speed>next_adjust:
        #print("Adjusting pitch by: "+str(adjust_size))
        current_pitch-=adjust_size
        if adjust_size!=0:
            vessel.auto_pilot.target_pitch_and_heading(float(current_pitch),heading)
        next_adjust+=1
    if pitch()<=60 and not reached_target_pitch:
        print("Pitched reached 60")
        adjust_size=0.05
        reached_target_pitch=True
    if pitch()<=min_pitch and not reached_target_min_pitch:
        print("Pitch following apoapsis")
        adjust_size=0
        vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
        vessel.auto_pilot.target_direction = (0, 1, 0)
        reached_target_min_pitch=True
    if resources.amount('SolidFuel')<0.1 and boosters:
        boosters=False
        print("Separate Boosters")
        vessel.control.activate_next_stage()
        time.sleep(0.25)
        vessel.control.activate_next_stage()

print("10% off target")
vessel.control.throttle = 0.5
while apoapsis()<target_altitude-(target_altitude*0.05):
    pass
print("5% off target")
vessel.control.throttle = 0.5
while apoapsis()<target_altitude:
    pass
vessel.control.throttle = 0
print("On target")
time.sleep(0.5)


#Print Separating Main Stage
vessel.control.activate_next_stage()
time.sleep(0.25)
vessel.control.activate_next_stage()
time.sleep(0.25)
vessel_list=conn.space_center.vessels
conn.space_center.active_vessel=vessel_list[1]
vessel=conn.space_center.active_vessel

for v in vessel_list:
    print(v.name)
    print(v.type)
    print()



print("Controlling: '" +str(vessel.name)+"'")


#Killing horizontal Speed

#Orient Vessel
vessel.auto_pilot.disengage()
vessel.auto_pilot.target_pitch_and_heading(0.0,90)
vessel.auto_pilot.engage()
vessel.control.rcs=True

while vessel.auto_pilot.heading_error>10:
    print(vessel.auto_pilot.heading_error)
    time.sleep(1)
    None
while vessel.auto_pilot.pitch_error>10:
    print(vessel.auto_pilot.pitch_error)
    time.sleep(1)
    None
#Burn
vessel.control.throttle = 1.0
while vessel.flight(srf_frame).horizontal_speed>vessel.available_thrust/vessel.mass*0.25:
    None
vessel.control.throttle = 0.0


#Horizontal Velocity To Landing Pad

altitude=vessel.flight().surface_altitude
velocity=vessel.flight(srf_frame).vertical_speed
gravity=9.81


#Coords after horizontal burn
current=(vessel.flight(srf_frame).longitude,vessel.flight(srf_frame).latitude)
print("Current coords are: "+str(current))
x=current[0]-launchpad[0]
y=current[1]-launchpad[1]
z=math.sqrt(x**2+y**2)
#Add on 100m buffer
time_to_impact=math.sqrt((velocity**2)+2*gravity*(altitude+100))/gravity+(velocity/gravity)
requried_velocity=z/time_to_impact
acceleration=vessel.available_thrust/vessel.mass
requried_velocity=(-time_to_impact+math.sqrt(
    (time_to_impact**2)-
    4*
    (1/(2*acceleration))*
    (-(velocity**2))
    ))/2*(1/acceleration)

heading=math.atan(y/x)
print("Heading is: "+str(heading))
print("Velocity required is: "+str(requried_velocity))
print("Distance to impact is: "+str(z))
print("Time to impact is: "+str(time_to_impact))
#Speed burn
vessel.auto_pilot.target_pitch_and_heading(0.0,heading)
print("Adjusting")
while vessel.auto_pilot.pitch_error>10 and vessel.auto_pilot.heading_error>10:None
print("Burning")
vessel.throttle=1.0
while vessel.flight(srf_frame).horizontal_speed<requried_velocity*0.9:None
vessel.throttle=0.5
while vessel.flight(srf_frame).horizontal_speed<requried_velocity:None
vessel.throttle=0.0


#Parameters
mass=vessel.mass
thrust=vessel.available_thrust
gravity=9.81
specific_impulse=vessel.specific_impulse
fuel_flow=thrust/specific_impulse

burn_time=(mass-math.exp(vessel.flight(srf_frame).horizontal_speed/(gravity*specific_impulse)))/(fuel_flow)
stop_distance=((specific_impulse*burn_time)-((specific_impulse*mass)/(fuel_flow)))*(math.log(mass-(fuel_flow*burn_time))-1)

#Wait till need to start burn
while z>stop_distance:
    current=(vessel.flight(srf_frame).longitude,vessel.flight(srf_frame).latitude)
    x=current[0]-launchpad[0]
    y=current[1]-launchpad[1]
    z=math.sqrt(x**2+y**2)
    burn_time=(mass-math.exp(vessel.flight(srf_frame).horizontal_speed/(gravity*specific_impulse)))/(fuel_flow)
    stop_distance=((specific_impulse*burn_time)-((specific_impulse*mass)/(fuel_flow)))*(math.log(mass-(fuel_flow*burn_time))-1)
	
#Start burn
vessel.thrust=1.0
time.sleep(burn_time)
vessel.thrust=0.0


#Start Suicide Burn
vessel.auto_pilot.target_pitch_and_heading(90.0,90)


max_velocity=math.sqrt(velocity**2+2*gravity*altitude)
time_to_impact=math.sqrt((velocity**2)+2*gravity*altitude)/gravity+(velocity/gravity)
print("Impacting in: "+str(time_to_impact))
burn_time=(max_velocity/vessel.available_thrust)*1.1
spare_time=time_to_impact-burn_time
print("Approximately "+str(spare_time)+" till action required")


for v in vessel_list:
    print(v.name)
    print(v.type)
    print()


#Vessel Suicide Burn
interval=time.time()
while burn_time<time_to_impact:
    altitude=vessel.flight().surface_altitude
    velocity=vessel.flight(srf_frame).vertical_speed
    gravity=9.81

    time_to_impact=math.sqrt((velocity**2)+2*gravity*altitude)/gravity+(velocity/gravity)
    
    

    max_velocity=math.sqrt(velocity**2+2*gravity*altitude)
    specific_impulse=vessel.specific_impulse

    try:
        burn_time=((vessel.mass-math.exp(max_velocity/(gravity*specific_impulse)))/
        (vessel.available_thrust/vessel.specific_impulse))
        
    except OverflowError:
        #print("Burn Time: Infinity")
        None
    time.sleep(0.01)
    print()
    if time.time()-interval>5:
        interval=time.time()
        print("Specific Impulse: "+str(specific_impulse))
        print("Current Altitude: "+str(altitude))
        print("Current Velocity: "+str(velocity))
        print("Max Velcity:      "+str(max_velocity))
        print("Impacting in:     "+str(time_to_impact))       
        print("Burn Time:        "+str(burn_time))
print("Burning")
vessel.control.throttle=1.0
time.sleep(burn_time)
vessel.control.throttle=0.0



exit(0)
















for v in vessel_list:
    print(v.name)
    print(v.type)
    print()
vessel_list[1].control.sas = True
vessel_list[1].control.SASMode="retrograde"
vessel_list[1].throttle=1.0



#Circulise burn
print('Planning circularization burn')
mu = vessel.orbit.body.gravitational_parameter
r = vessel.orbit.apoapsis
a1 = vessel.orbit.semi_major_axis
a2 = r
v1 = math.sqrt(mu*((2./r)-(1./a1)))
v2 = math.sqrt(mu*((2./r)-(1./a2)))
delta_v = v2 - v1
node = vessel.control.add_node(
    ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

# Calculate burn time (using rocket equation)
F = vessel.available_thrust
Isp = vessel.specific_impulse * 9.82
m0 = vessel.mass
m1 = m0 / math.exp(delta_v/Isp)
flow_rate = F / Isp
burn_time = (m0 - m1) / flow_rate

print('Orientating ship for circularization burn')
vessel.auto_pilot.reference_frame = node.reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)
vessel.auto_pilot.wait()

# Wait until burn
print('Waiting until circularization burn')
burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time/2.)
lead_time = 5
conn.space_center.warp_to(burn_ut - lead_time)

# Execute burn
print('Ready to execute burn')
time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
while time_to_apoapsis() - (burn_time/2.) > 0:
    pass
print('Executing burn')
vessel.control.throttle = 1.0
time.sleep(burn_time)
vessel.control.throttle = 0.1
node.remove()
vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)
while vessel.orbit.periapsis_altitude<vessel.orbit.apoapsis_altitude:
    pass
vessel.control.throttle = 0
print('Launch complete')


exit(0)


#Trigger boosters
fuel_amount = conn.get_call(vessel.resources.amount, 'SolidFuel')
expr = conn.krpc.Expression.less_than(
    conn.krpc.Expression.call(fuel_amount),
    conn.krpc.Expression.constant_float(0.1))
event = conn.krpc.add_event(expr)
with event.condition:
    event.wait()
print('Booster separation')
vessel.control.activate_next_stage()


#Gravity Turn
mean_altitude = conn.get_call(getattr, vessel.flight(), 'mean_altitude')
expr = conn.krpc.Expression.greater_than(
    conn.krpc.Expression.call(mean_altitude),
    conn.krpc.Expression.constant_double(10000))
event = conn.krpc.add_event(expr)
with event.condition:
    event.wait()

print('Gravity turn')
vessel.auto_pilot.target_pitch_and_heading(60, 90)

#Target altitude
apoapsis_altitude = conn.get_call(getattr, vessel.orbit, 'apoapsis_altitude')
expr = conn.krpc.Expression.greater_than(
    conn.krpc.Expression.call(apoapsis_altitude),
    conn.krpc.Expression.constant_double(100000))
event = conn.krpc.add_event(expr)
with event.condition:
    event.wait()

#Freefall
print('Launch stage separation')
vessel.control.throttle = 0
time.sleep(1)
vessel.control.activate_next_stage()
vessel.auto_pilot.disengage()

#Parachute
srf_altitude = conn.get_call(getattr, vessel.flight(), 'surface_altitude')
expr = conn.krpc.Expression.less_than(
    conn.krpc.Expression.call(srf_altitude),
    conn.krpc.Expression.constant_double(1000))
event = conn.krpc.add_event(expr)
with event.condition:
    event.wait()

vessel.control.activate_next_stage()

#Print altitude
while vessel.flight(vessel.orbit.body.reference_frame).vertical_speed < -0.1:
    print('Altitude = %.1f meters' % vessel.flight().surface_altitude)
    time.sleep(1)
print('Landed!')
