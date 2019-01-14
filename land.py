import time
import math
import krpc
class land:
    def __init__(self,name,launchpad):
        
        self.launchpad=launchpad
        self.vessel_name=name
        self.connect()
        self.streams()
        print("Landing Started!")
        self.kill_horizontal_velocity()
        self.landing_pad_burn()
        self.cancel_landing_pad()
        self.suicide_burn()

    def connect(self):
        self.conn = krpc.connect(name=self.vessel_name)
        for v in self.conn.space_center.vessels:
            if v.name==self.vessel_name:
                self.vessel=v
                self.conn.space_center.active_vessel=v

    def streams(self):
        #Reference Frames
        self.obt_frame = self.vessel.orbit.body.non_rotating_reference_frame
        self.srf_frame = self.vessel.orbit.body.reference_frame
        self.gravity=9.81

        self.resources=self.vessel.resources
        self.mass=self.conn.add_stream(getattr,self.vessel,"mass")
        self.specific_impulse=self.conn.add_stream(getattr,self.vessel,"specific_impulse")
        self.max_thrust=self.conn.add_stream(getattr,self.vessel,"available_thrust")

        flight_info=self.vessel.flight()

        self.speed=self.conn.add_stream(getattr,self.vessel.flight(self.srf_frame),"speed")
        self.velocity=self.conn.add_stream(getattr,flight_info,"velocity")
        self.horizontal_speed=self.conn.add_stream(getattr,self.vessel.flight(self.srf_frame),"horizontal_speed")
        self.vertical_speed=self.conn.add_stream(getattr,self.vessel.flight(self.srf_frame),"vertical_speed")


        self.latitude=self.conn.add_stream(getattr,self.vessel.flight(self.srf_frame),"latitude")
        self.longitude=self.conn.add_stream(getattr,self.vessel.flight(self.srf_frame),"longitude")

        self.pitch=self.conn.add_stream(getattr,self.vessel.flight(),"pitch")
        self.heading=self.conn.add_stream(getattr,self.vessel.flight(self.srf_frame),"heading")

        self.altitude = self.conn.add_stream(getattr, self.vessel.flight(self.srf_frame), 'surface_altitude')

    def kill_horizontal_velocity(self):
        print("Killing Horizontal Velocity")
        self.vessel.auto_pilot.target_pitch_and_heading(0.0,90)
        self.vessel.auto_pilot.engage()
        self.vessel.control.rcs=True
        print("Aiming ship")
        while self.vessel.auto_pilot.heading_error>10 and self.vessel.auto_pilot.pitch_error>10 :
            time.sleep(1)
            print("Error")
            None
        print("Burning")
        #Burn
        self.vessel.control.throttle = 1.0
        while self.horizontal_speed()>self.max_thrust()/self.mass()*0.25:
            None
        self.vessel.control.throttle = 0.0

    def landing_pad_burn(self):
        print("Burning Towards Pad")
        gravity=9.81

        #Coords after horizontal burn
        x=self.latitude()-self.launchpad[0]
        y=self.longitude()-self.launchpad[1]
        z=math.sqrt(x**2+y**2)
        print("Current coords are: "+str(self.longitude)+", "+str(self.latitude))
        #Add on 100m buffer
        time_to_impact=math.sqrt((self.vertical_speed**2)+2*gravity*(self.altitude+100))/gravity+(self.vertical_speed/gravity)
        requried_velocity=z/time_to_impact
        acceleration=self.max_thrust/self.mass
        requried_velocity=(-time_to_impact+math.sqrt(
            (time_to_impact**2)-
            4*
            (1/(2*acceleration))*
            (-(self.vertical_speed**2))
            ))/2*(1/acceleration)

        heading=math.atan(y/x)
        heading=90#TODO Override
        print("Heading is: "+str(heading))
        print("Velocity required is: "+str(requried_velocity))
        print("Distance to impact is: "+str(z))
        print("Time to impact is: "+str(time_to_impact))
        #Speed burn
        self.vessel.auto_pilot.target_pitch_and_heading(0.0,heading)
        print("Adjusting")
        while self.vessel.auto_pilot.pitch_error>10 and self.vessel.auto_pilot.heading_error>10:None
        print("Burning")
        self.vessel.throttle=1.0
        while self.horizontal_speed<requried_velocity*0.9:None
        self.vessel.throttle=0.5
        while self.horizontal_speed<requried_velocity:None
        self.vessel.throttle=0.0

    def cancel_landing_pad(self):
        print("Cancelling Horizontal Velocity")
        gravity=9.81
        fuel_flow=self.max_thrust()/self.specific_impulse()

        burn_time=(self.mass-math.exp(self.horizontal_speed()/(self.gravity*self.specific_impulse())))/(fuel_flow)
        stop_distance=((self.specific_impulse()*burn_time)-((self.specific_impulse()*self.mass())/(fuel_flow)))*(math.log(self.mass()-(fuel_flow*burn_time))-1)
        x=self.latitude()-self.launchpad[0]
        y=self.longitude()-self.launchpad[1]
        z=math.sqrt(x**2+y**2)
        #Wait till need to start burn
        while z>stop_distance:
            x=self.latitude()-self.launchpad[0]
            y=self.longitude()-self.launchpad[1]
            z=math.sqrt(x**2+y**2)
            burn_time=(self.mass()-math.exp(self.horizontal_speed()/(gravity*self.specific_impulse())))/(fuel_flow)
            stop_distance=((self.specific_impulse()*burn_time)-((self.specific_impulse()*self.mass())/(fuel_flow)))*(math.log(self.mass()-(fuel_flow*burn_time))-1)
            
        #Start burn
        self.vessel.thrust=1.0
        time.sleep(burn_time)
        self.vessel.thrust=0.0

    def suicide_burn(self):
        print("Suiciding")
        self.vessel.auto_pilot.target_pitch_and_heading(90.0,90)

        max_velocity=math.sqrt(self.vertical_speed()**2+2*self.gravity*self.altitude())
        time_to_impact=math.sqrt((self.vertical_speed()**2)+2*self.gravity*self.altitude())/self.gravity+(self.vertical_speed()/self.gravity)
        print("Impacting in: "+str(time_to_impact))
        burn_time=(max_velocity/self.max_thrust())*1.1
        spare_time=time_to_impact-burn_time
        print("Approximately "+str(spare_time)+" till action required")


        #Vessel Suicide Burn
        interval=time.time()
        while burn_time<time_to_impact:

            time_to_impact=math.sqrt((self.vertical_speed()**2)+2*gravity*altitude)/gravity+(self.vertical_speed()/gravity)
            
            

            max_velocity=math.sqrt(self.vertical_speed()**2+2*self.gravity*self.altitude())
            try:
                burn_time=((self.mass()-math.exp(max_velocity/(self.gravity*self.specific_impulse())))/
                (self.max_thrust()/self.specific_impulse()))
                
            except OverflowError:
                #print("Burn Time: Infinity")
                None
            time.sleep(0.01)
            if time.time()-interval>5:
                interval=time.time()
                print("Specific Impulse: "+str(self.specific_impulse()))
                print("Current Altitude: "+str(self.altitude()))
                print("Current Velocity: "+str(self.vertical_speed()))
                print("Max Velcity:      "+str(max_velocity))
                print("Impacting in:     "+str(time_to_impact))       
                print("Burn Time:        "+str(burn_time))
        print("Burning")
        self.vessel.control.throttle=1.0
        time.sleep(burn_time)
        self.vessel.control.throttle=0.0



