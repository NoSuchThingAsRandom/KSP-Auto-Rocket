import krpc
import time
import math

class launch:
    def __init__(self,vessel_name,target_stage,target_altitude,target_heading):
        print("Targetting heading of: "+str(target_heading))
        print("Stage separation: "+str(target_stage))
        print("Target Altitude: "+str(target_altitude))
        self.vessel_name=vessel_name
        self.target_heading=target_heading
        self.connect()
        self.streams()
        self.launch(90,target_heading)
        self.burn_till_target_apoapsis(target_stage,target_altitude,30,False)
        
        #Separate Rocket
        time.sleep(0.1)
        self.vessel.control.activate_next_stage()
        time.sleep(0.1)
        self.vessel.control.activate_next_stage()

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

        self.pitch=self.conn.add_stream(getattr,self.vessel.flight(),"pitch")
        self.heading=self.conn.add_stream(getattr,self.vessel.flight(self.srf_frame),"heading")

        self.altitude = self.conn.add_stream(getattr, self.vessel.flight(self.srf_frame), 'surface_altitude')
        self.ut = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
        
        self.apoapsis = self.conn.add_stream(getattr, self.vessel.orbit, 'apoapsis_altitude')
        self.periapsis = self.conn.add_stream(getattr, self.vessel.orbit, 'periapsis_altitude')


    def launch(self,pitch,heading):
        #Pre set controls
        self.vessel.auto_pilot.target_pitch_and_heading(pitch, heading)
        self.vessel.auto_pilot.target_roll=90
        self.vessel.auto_pilot.engage()
        #vessel.auto_pilot.sas=True
        self.vessel.control.throttle = 1
        print("3")
        print("2")
        print("1")
        print('Launch!')
        self.vessel.control.activate_next_stage()
    

    def burn_till_target_apoapsis(self,stage_target,target_apoapsis,min_pitch,boosters):
        current_target_pitch=90
        adjust_size=0.1
        next_adjust=1
        throttle=1.0
        throttle_control=True
        reached_target_pitch=False
        reached_target_min_pitch=False
        while self.apoapsis()<stage_target*0.9:
            if self.speed()>next_adjust:
                #print("Adjusting pitch by: "+str(adjust_size))
                current_target_pitch-=adjust_size
                if adjust_size!=0:
                    self.vessel.auto_pilot.target_pitch_and_heading(float(current_target_pitch),self.target_heading)
                next_adjust+=1
            if self.pitch()<=60 and not reached_target_pitch:
                print("Pitched reached 60")
                adjust_size=0.05
                reached_target_pitch=True
            if self.pitch()<=min_pitch and not reached_target_min_pitch:
                print("Pitch following apoapsis")
                adjust_size=0
                self.vessel.auto_pilot.reference_frame = self.vessel.surface_velocity_reference_frame
                self.vessel.auto_pilot.target_direction = (0, 1, 0)
                reached_target_min_pitch=True


            if self.altitude()>15000.0 and throttle_control:
                throttle_control=False
                throttle=1.0
                self.vessel.control.throttle=throttle
                print("Max Throttle!")
            if throttle*self.max_thrust()>9.81*self.mass()*1.5 and throttle_control:
                    throttle=(9.81*self.mass()*1.5)/self.max_thrust()
                    self.vessel.control.throttle=throttle

            if self.resources.amount('SolidFuel')<0.1 and boosters:
                boosters=False
                print("Separate Boosters")
                self.vessel.control.activate_next_stage()
                time.sleep(0.25)
                self.vessel.control.activate_next_stage()

            fuel_mass=self.vessel.resources.amount("LiquidFuel")*self.vessel.resources.density("LiquidFuel")
            +self.vessel.resources.amount("Oxidizer")*self.vessel.resources.density("Oxidizer")
            delta_v_remaining=self.specific_impulse()*self.gravity*math.exp(self.mass()/(self.mass()-fuel_mass))
            #print("Delta V is: "+str(delta_v_remaining))
            if delta_v_remaining*0.95<self.horizontal_speed()-self.vertical_speed()+math.sqrt(2*self.gravity*self.altitude()):
                print("Reached safe fuel limit")
                self.vessel.throttle=0.0
                self.vessel.auto_pilot.disengage()
                return
        print("10% off target apoapsis")
        self.vessel.control.throttle = 0.5
        while self.apoapsis()<stage_target*0.95:
            time.sleep(0.1)
        print("5% off target apoapsis")
        self.vessel.control.throttle = 0.5
        while self.apoapsis()<stage_target:
            time.sleep(0.1)
        self.vessel.control.throttle = 0
        self.vessel.auto_pilot.disengage()
        print("Reached target apoapsis")


