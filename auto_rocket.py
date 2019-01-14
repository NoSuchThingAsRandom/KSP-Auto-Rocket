from launch import launch
from orbit import orbit
from land import land
import krpc


conn = krpc.connect(name='Drone Controller')
vessel = conn.space_center.active_vessel
srf_frame = vessel.orbit.body.reference_frame
launchpad=(vessel.flight(srf_frame).longitude,vessel.flight(srf_frame).latitude)
print(launchpad)
print("Start")
launch("Launcher2",50000,75000,270)
land("Launcher2",launchpad)
#orbit("Launcher2")