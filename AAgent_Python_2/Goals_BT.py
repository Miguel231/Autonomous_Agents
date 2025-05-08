import math
import random
import asyncio
import Sensors
from collections import Counter

"""
--------------------------------------------------------------------------------------------------------------------------------------
"""

ROTATION_THRESHOLD = 2 # degrees tolerance for stopping

def angle_delta(current, previous):
	"""
		Returns the smallest positive arc difference between two angles (0..180).
		Example:
			previous=10, current=350 -> 20
			previous=350, current=10 -> 20
	"""
	raw_diff = (current - previous) % 360
	return 360 - raw_diff if raw_diff > 180 else raw_diff

async def execute_turn(a_agent, i_state, direction, angle):
	"""Execute a precise turn using orientation tracking"""
	if angle < 5:
		await a_agent.send_message("action", direction)
		await asyncio.sleep(0.1)
		await a_agent.send_message("action", "nt")
		return

	# print(f"Turning {angle:.1f}° {'left' if direction == 'tl' else 'right'}")
	
	total_rotated = 0 # Total rotation accumulated
	prev_orientation = i_state.rotation['y']
	
	while True:
		await a_agent.send_message("action", direction)
		await asyncio.sleep(0.1)
		
		current_orientation = i_state.rotation['y']
		delta = angle_delta(current_orientation, prev_orientation)
		total_rotated += delta # We simply add how many degrees changed (the "short arc")
		prev_orientation = current_orientation
		# print(f"Rotated: {total_rotated:.2f}° / {angle:.1f}°")
		
		# If we're close enough , stop turning
		if total_rotated >= angle - ROTATION_THRESHOLD:
			await a_agent.send_message("action", "nt")
			await asyncio.sleep(0)
			break

"""
--------------------------------------------------------------------------------------------------------------------------------------
"""

def calculate_distance(point_a, point_b):
    distance = math.sqrt((point_b['x'] - point_a['x']) ** 2 +
                         (point_b['y'] - point_a['y']) ** 2 +
                         (point_b['z'] - point_a['z']) ** 2)
    return distance


class DoNothing:
    """
    Does nothing
    """

    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state = a_agent.i_state

    async def run(self):
        # print("Doing nothing")
        await asyncio.sleep(1)
        return True

class ForwardDist:
    """
        Moves forward a certain distance specified in the parameter "dist".
        If "dist" is -1, selects a random distance between the initial
        parameters of the class "d_min" and "d_max"
    """
    STOPPED = 0
    MOVING = 1
    END = 2

    def __init__(self, a_agent, dist, d_min, d_max):
        self.a_agent = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state = a_agent.i_state
        self.original_dist = dist
        self.target_dist = dist
        self.d_min = d_min
        self.d_max = d_max
        self.starting_pos = a_agent.i_state.position
        self.state = self.STOPPED

    async def run(self):
        try:
            previous_dist = 0.0  # Used to detect if we are stuck
            while True:
                if self.state == self.STOPPED:
                    # starting position before moving
                    self.starting_pos = self.a_agent.i_state.position
                    # Before start moving, calculate the distance we want to move
                    if self.original_dist < 0:
                        self.target_dist = random.randint(self.d_min, self.d_max)
                    else:
                        self.target_dist = self.original_dist
                    # Start moving
                    await self.a_agent.send_message("action", "mf")
                    self.state = self.MOVING
                    # print("TARGET DISTANCE: " + str(self.target_dist))
                elif self.state == self.MOVING:
                    # If we are moving
                    await asyncio.sleep(0.5)  # Wait for a little movement
                    current_dist = calculate_distance(self.starting_pos, self.i_state.position)
                    # print(f"Current distance: {current_dist}")
                    if current_dist >= self.target_dist:  # Check if we already have covered the required distance
                        await self.a_agent.send_message("action", "ntm")
                        self.state = self.STOPPED
                        return True
                    elif previous_dist == current_dist:  # We are not moving
                        # print(f"previous dist: {previous_dist}, current dist: {current_dist}")
                        # print("NOT MOVING")
                        await self.a_agent.send_message("action", "ntm")
                        self.state = self.STOPPED
                        return False
                    previous_dist = current_dist
                else:
                    print("Unknown state: " + str(self.state))
                    return False
        except asyncio.CancelledError:
            print("***** TASK Forward CANCELLED")
            await self.a_agent.send_message("action", "ntm")
            self.state = self.STOPPED


class Turn:
    """
    Repeats the action of turning a random number of degrees in a random
    direction (right or left)
    """
    LEFT = -1
    RIGHT = 1

    SELECTING = 0
    TURNING = 1

    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state = a_agent.i_state

        self.current_heading = 0
        self.new_heading = 0

        self.state = self.SELECTING

    async def run(self):
        try:
            while True:
                if self.state == self.SELECTING:
                    # print("SELECTING NEW TURN")
                    rotation_direction = random.choice([-1, 1])
                    # print(f"Rotation direction: {rotation_direction}")
                    rotation_degrees = random.uniform(1, 180) * rotation_direction
                    # print("Degrees: " + str(rotation_degrees))
                    current_heading = self.i_state.rotation["y"]
                    # print(f"Current heading: {current_heading}")
                    self.new_heading = (current_heading + rotation_degrees) % 360
                    if self.new_heading == 360:
                        self.new_heading = 0.0
                    # print(f"New heading: {self.new_heading}")
                    if rotation_direction == self.RIGHT:
                        await self.a_agent.send_message("action", "tr")
                    else:
                        await self.a_agent.send_message("action", "tl")
                    self.state = self.TURNING
                elif self.state == self.TURNING:
                    # check if we have finished the rotation
                    current_heading = self.i_state.rotation["y"]
                    final_condition = abs(current_heading - self.new_heading)
                    if final_condition < 5:
                        await self.a_agent.send_message("action", "nt")
                        current_heading = self.i_state.rotation["y"]
                        # print(f"Current heading: {current_heading}")
                        # print("TURNING DONE.")
                        self.state = self.SELECTING
                        return True
                await asyncio.sleep(0)
        except asyncio.CancelledError:
            print("***** TASK Turn CANCELLED")
            await self.a_agent.send_message("action", "nt")

"""
--------------------------------------------------------------------------------------------------------------------------------------
"""

class RandomRoam:
	
	#Moves around randomly, changing direction and deciding when to stop,
	#based on predefined probabilities.
	
	STOPPED = 0
	MOVING = 1
	TURNING = 2
	STOP = 3

	def __init__(self, a_agent):
		self.a_agent = a_agent
		self.rc_sensor = a_agent.rc_sensor
		self.i_state = a_agent.i_state
		self.state = self.STOPPED

		# Gaussian parameters for predetermined turn probabilities
		self.turn_mean = 0
		self.turn_stdev = 45

	async def run(self):
		try:
			while True:
				if self.state == self.STOPPED:
					self.state = random.choices([self.TURNING, self.STOP, self.MOVING], weights=(55, 10, 35), k=1)[0]
					print(f"New State: {self.state}")
					await asyncio.sleep(0.5)

				elif self.state == self.MOVING:
					await self.a_agent.send_message("action", "mf")
					print("MOVING")

					# Check for obstacles for a while before changing state
					move_time = random.uniform(1.0, 3.0)
					start_time = asyncio.get_event_loop().time()

					while asyncio.get_event_loop().time() - start_time < move_time:
						hits = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]
						if hits[len(hits)//2]:
							print("Obstacle detected! Stopping and turning.")
							await self.a_agent.send_message("action", "stop")
							self.state = self.TURNING
							break
						await asyncio.sleep(0.1)

					# If no obstacle was detected, randomly choose next state
					if self.state == self.MOVING:
						await self.a_agent.send_message("action", "stop")
						self.state = random.choices([self.TURNING, self.STOP, self.MOVING], weights=(55, 10, 35), k=1)[0]
						print(f"New State: {self.state}")

				elif self.state == self.STOP:
					print("STOPPED")
					await self.a_agent.send_message("action", "stop")
					await asyncio.sleep(random.uniform(1.0, 3.0))  # Random wait time

					self.state = random.choices([self.TURNING, self.STOP, self.MOVING], weights=(55, 10, 35), k=1)[0]
					print(f"New State: {self.state}")

				elif self.state == self.TURNING:
					# Generate a random angle from Gaussian distribution (prioritize small turns)
					raw_angle = random.gauss(self.turn_mean, self.turn_stdev)
					turn_degrees = max(-180, min(180, raw_angle))
					turn_direction = "tr" if turn_degrees < 0 else "tl"
					abs_degrees = abs(turn_degrees)
					
					print(f"Turning {abs_degrees:.1f}° {'left' if turn_direction == 'tl' else 'right'}")
					await self.a_agent.send_message("action", "stop")
					await asyncio.sleep(0.2)
					await execute_turn(self.a_agent, self.i_state, turn_direction, abs_degrees)
					print("Turn completed.")
					
					# Choose next state after turning
					self.state = random.choices([self.STOP, self.MOVING], weights=(30, 70), k=1)[0]
					print(f"New State: {self.state}")
					await asyncio.sleep(0.5)

				else:
					print(f"Unknown state: {self.state}")
					self.state = self.STOPPED
		
		except asyncio.CancelledError:
			print("***** TASK RandomRoam CANCELLED")
			await self.a_agent.send_message("action", "stop")
			self.state = self.STOPPED

class Avoid:
	
	#The Drone advances while avoiding obstacles using distance-weighted ray sensors.
	#Features corner escape and emergency response capabilities.
	
	STOPPED = 0
	MOVING = 1
	TURNING = 2
	STOP = 3

	def __init__(self, a_agent):
		self.a_agent = a_agent
		self.rc_sensor = a_agent.rc_sensor
		self.i_state = a_agent.i_state
		self.state = self.STOPPED
		
		# Parameters
		self.sensitivity = 1.0 # Higher values mean sharper turns when close to obstacles
		self.max_turn_angle = 45 # Maximum angle allowed to turn
		self.distance_decay = 0.5 # Bigger decay means further objects have less influence
		self.emergency_distance = 1 # Distance threshold for emergency response
		
		# Calculate ray angles and responses
		angle_between_rays = self.rc_sensor.max_ray_degrees / self.rc_sensor.rays_per_direction
		self.ray_degrees = [
			angle_between_rays * (i - self.rc_sensor.rays_per_direction) 
			for i in range(self.rc_sensor.num_rays)
		]
		self.ray_turn_degrees = self.calculate_turn_responses()
		# print(self.ray_turn_degrees)
	
	def calculate_turn_responses(self):
		#Calculate optimal turn angles for each ray
		def turn_angle_for_ray(ray_angle):
			# Normalize range [-90, 90] to [-1, 1]. Angles above 90 are ignored
			normalized_pos = max(-1.0, min(1.0, ray_angle / 90.0))
			# Turn to opposite side of the ray. Turn right for center ray
			sign = -1 if normalized_pos > 0 else 1
			# Turn a lot for rays close to the center, less for extreme angles
			magnitude = self.max_turn_angle * math.exp(-normalized_pos**2 / 0.1) # Gaussian of mean 0, std 0.2
			return sign * magnitude * self.sensitivity

		return [turn_angle_for_ray(angle) for angle in self.ray_degrees]
	
	async def run(self):
		try:
			while True:
				if self.state == self.STOPPED:
					await self.a_agent.send_message("action", "mf")
					self.state = self.MOVING
					await asyncio.sleep(0.5)
					
				elif self.state == self.MOVING:
					# Get sensor data
					sensor_hits = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]
					sensor_distances = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.DISTANCE]
					
					# Count hits
					total_hit_count = sum(sensor_hits)
					left_hits = sum(sensor_hits[:len(sensor_hits)//2])
					right_hits = sum(sensor_hits[len(sensor_hits)//2+1:])
					
					# Calculate turn response
					total_turn = 0
					total_weight = 0
					emergency_count = 0
					min_distance = float('inf') # distance to closest object
					
					for i, (hit, distance) in enumerate(zip(sensor_hits, sensor_distances)):
						if hit:
							min_distance = min(min_distance, distance)
							
							if distance <= self.emergency_distance:
								emergency_count += 1
							
							# For bigger distances, weight decreases
							weight = math.exp(1+(-self.distance_decay * distance))
							total_turn += self.ray_turn_degrees[i] * weight
							total_weight += weight
					
					if total_weight == 0:
						await asyncio.sleep(0.1) # to avoid overwhelming the agent
						continue
					
					# Determine response type
					majority = 2/3
					some = 1-majority
					# Emergency if some rays hits are too close or if any ray hit is too close
					emergency = emergency_count >= some * len(sensor_hits) or min_distance < self.emergency_distance * 0.5
					# Corner trap
					corner_trap = (
						total_hit_count / len(sensor_hits) >= majority and # majority of rays hit an object
						abs(left_hits - right_hits) <= 1 and # left and right hits are similar
						total_weight >= majority * len(sensor_hits) * 0.6 # majority of rays detect objects mostly (0.6) close
					)
					
					if corner_trap:
						print("CORNER TRAP DETECTED! Executing escape maneuver")
						direction = "tl" if left_hits <= right_hits else "tr"
						await self.a_agent.send_message("action", "stop")
						await asyncio.sleep(0.1)
						await execute_turn(self.a_agent, self.i_state, direction, 45)
						await self.a_agent.send_message("action", "mf")
					else:
						turn_degrees = abs(total_turn)
						turn_direction = "tr" if total_turn > 0 else "tl"
						
						need_to_stop = emergency or turn_degrees > 30
						if need_to_stop:
							await self.a_agent.send_message("action", "stop")
							await asyncio.sleep(0.1)
							await execute_turn(self.a_agent, self.i_state, turn_direction, turn_degrees)
							await self.a_agent.send_message("action", "mf")
						else:
							await execute_turn(self.a_agent, self.i_state, turn_direction, turn_degrees)
					
					await asyncio.sleep(0.1)
					
				else:
					self.state = self.STOPPED
					await asyncio.sleep(0.5)
		
		except asyncio.CancelledError:
			print("***** TASK Avoid CANCELLED")
			await self.a_agent.send_message("action", "stop")
			await self.a_agent.send_message("action", "nt")
			self.state = self.STOPPED

"""
--------------------------------------------------------------------------------------------------------------------------------------
"""
def flower_count(i_state):
    """
    flowers in the inventory
    """
    return next(
        (item['amount'] for item in i_state.myInventoryList if item['name'] == 'AlienFlower'),
        0 # If not found
    )


# class GetFlower:
#     """
#     Get the flower and check inventory
#     """

#     def __init__(self, a_agent, target_pos: dict, tolerance: float = 1.0):
#         self.a_agent = a_agent
#         self.i_state = a_agent.i_state
#         self.target_pos = target_pos
#         self.tolerance = tolerance

#     async def run(self):
#         try:
#             #move to the flower
#             await self.a_agent.send_message("action",f"walk_to,{self.target_pos['x']},{self.target_pos['y']},{self.target_pos['z']}")

#             #wait
#             while True:
#                 await asyncio.sleep(0.2)
#                 dist = calculate_distance(self.i_state.position, self.target_pos)
#                 if dist <= self.tolerance:
            
#                     return True  # SUCCESS

#         except asyncio.CancelledError:
#             await self.a_agent.send_message("action", "stop")
#             return False


class GetFlower:
	"""
	Orientate to the flower and move forward to it
	"""

	def __init__(self, a_agent):
		self.a_agent = a_agent
		self.i_state = a_agent.i_state
		self.side = 0
		self.turn_map = {
			-1: "tl",
			0: "nt",
			1: "tr"
		}

		# Calculate center-outward ray indices
		num_rays = self.a_agent.rc_sensor.num_rays
		center = num_rays // 2
		self.ray_check_order = [center]  # Start with center
		for offset in range(1, center + 1):
			left = center - offset
			right = center + offset
			if left >= 0:
				self.ray_check_order.append(left)
			if right < num_rays:
				self.ray_check_order.append(right)
		print(f"Ray check order: {self.ray_check_order}")

	async def run(self):
		try:
			while True:
				# Check what side is the flower
				for ray_index in self.ray_check_order:
					ray = tuple(zip(*self.a_agent.rc_sensor.sensor_rays))[ray_index]
					if ray[Sensors.RayCastSensor.HIT] and \
						ray[Sensors.RayCastSensor.OBJECT_INFO]['tag'] == 'AlienFlower':
						if ray[Sensors.RayCastSensor.ANGLE] == 0: # center
							self.side = 0
						elif ray[Sensors.RayCastSensor.ANGLE] < 0: # left
							self.side = -1
						else: # right
							self.side = 1
						break
				before_flower_count = flower_count(self.i_state)
				# Turn to that side until flower is in the center ray
				await self.a_agent.send_message("action", self.turn_map[self.side])
				# Move forward until inventory increases by 1 (this can mf while turning)
				await self.a_agent.send_message("action", "mf")
				await asyncio.sleep(0.1)
				# Check if flower was collected
				current_flower_count = flower_count(self.i_state)
				# print(f"Flower count: {current_flower_count}, Before: {before_flower_count}")
				if current_flower_count > before_flower_count:
					print("Flower collected!")
					await self.a_agent.send_message("action", "stop")
					return
				await asyncio.sleep(0.5)

		except asyncio.CancelledError:
			print("***** TASK GetFlower CANCELLED")
			await self.a_agent.send_message("action", "stop")
			await self.a_agent.send_message("action", "nt")


bitten_astronaut=False


def is_astronaut_bitten():
	global bitten_astronaut	
	return bitten_astronaut

class GetAstronaut: 
	"""
	Orientate to the astronaut and move forward to it
	"""
	def __init__(self, a_agent):
		self.a_agent = a_agent
		#self.tag_object=tag_object
		self.i_state = a_agent.i_state
		self.side = 0
		self.turn_map = {
			-1: "tl",
			0: "nt",
			1: "tr"
		}

		# Calculate center-outward ray indices
		num_rays = self.a_agent.rc_sensor.num_rays
		center = num_rays // 2
		self.ray_check_order = [center]  # Start with center
		for offset in range(1, center + 1):
			left = center - offset
			right = center + offset
			if left >= 0:
				self.ray_check_order.append(left)
			if right < num_rays:
				self.ray_check_order.append(right)
		print(f"Ray check order: {self.ray_check_order}")

	async def run(self):
		try:
			while True:
				# Search for target
				astronautDetected=False
				astronautBitten=False
				for ray_index in self.ray_check_order:
					ray = tuple(zip(*self.a_agent.rc_sensor.sensor_rays))[ray_index]
					if ray[Sensors.RayCastSensor.HIT] and \
						ray[Sensors.RayCastSensor.OBJECT_INFO]['tag'] == "Astronaut":

						if ray[Sensors.RayCastSensor.ANGLE] == 0: # center
							self.side = 0
						elif ray[Sensors.RayCastSensor.ANGLE] < 0: # left
							self.side = -1
						else: # right
							self.side = 1
						
						astronautDetected=True
						print(ray[Sensors.RayCastSensor.DISTANCE])
						if ray[Sensors.RayCastSensor.DISTANCE]<1:
							print(ray[Sensors.RayCastSensor.DISTANCE])
							astronautBitten=True
						else:
							astronautBitten=False

						break
				if astronautDetected:
					await self.a_agent.send_message("action", self.turn_map[self.side])
					await self.a_agent.send_message("action", "mf")

				await asyncio.sleep(0.5)

				if astronautBitten:
					print("Astronaut bitten!") 
					global bitten_astronaut
					if(not bitten_astronaut): bitten_astronaut=True
					return True
				
		except asyncio.CancelledError:
			print(f"***** TASK GetAstronaut CANCELLED")
			await self.a_agent.send_message("action", "stop")
			await self.a_agent.send_message("action", "nt")


class MoveAway:
	"""
    Move the critter away from the astronaut after biting.
    """
	def __init__(self, a_agent):
		self.a_agent=a_agent
		self.i_state = a_agent.i_state

	async def run(self):
		try:
			await self.a_agent.send_message("action", "stop") # All critters wait 
			await asyncio.sleep(0.5)
			for index,ray in enumerate(zip(*self.a_agent.rc_sensor.sensor_rays)):
				if ray[Sensors.RayCastSensor.HIT] and \
				ray[Sensors.RayCastSensor.OBJECT_INFO]['tag'] == 'Astronaut':
					await execute_turn(self.a_agent, self.i_state, "tl", 180) # Only the critters that has detect the astronaut turn around
					await self.a_agent.send_message("action", "mf")
			
			await asyncio.sleep(4) 

			print("MoveAway completed")
			global bitten_astronaut
			if(bitten_astronaut): bitten_astronaut=False

			return True
		
		except asyncio.CancelledError:
			print("***** TASK MoveAway CANCELLED")
			await self.a_agent.send_message("action", "stop")
			await self.a_agent.send_message("action", "nt")


class ReturnAndUnload:
	"""
	Return to base
	"""
	def __init__(self, a_agent):
		self.a_agent = a_agent
		self.rc_sensor = a_agent.rc_sensor
		self.i_state = a_agent.i_state

	async def run(self):
		try:
			while True:
				# Check if we are at base
				if self.i_state.currentNamedLoc == "Base":
					print("At base")
					await self.a_agent.send_message("action", "stop")
					await self.a_agent.send_message("action", "nt")
					await asyncio.sleep(0.2)
					await self.a_agent.send_message("action", "leave,AlienFlower,2")
					print("Unloaded flowers")
					return True
				else:
					print("Returning to base")
					# await self.a_agent.send_message("action", "walk_to,Base")
					await self.a_agent.send_message("action", "teleport_to,Base")
					await asyncio.sleep(0.5)
		except asyncio.CancelledError:
			await self.a_agent.send_message("action", "stop")
			await self.a_agent.send_message("action", "nt")


class HarvestCycle:

    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state   = a_agent.i_state

    async def run(self):
        try:
            while True:
                # --- PHASE 1: GET -------------------------------------------------
                while flower_count(self.i_state) < 2:
                    obj_info = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.OBJECT_INFO]
                    target = next((o["position"] for o in obj_info if o and o["tag"] == "AlienFlower"), None)

                    if target:
                        ok = await GetFlower(self.a_agent, target).run()
                        if not ok:
                            break  
                    else:
                        await RandomRoam(self.a_agent).run()

                # --- PHASE 2: FULL INVENTARY ---------------------------------------
                if flower_count(self.i_state) >= 2:
                    #GO TO BASE
                    await self.a_agent.send_message("action", "teleport_to,Base")
                    
                    while not self.a_agent.at_base():
                        await asyncio.sleep(0.2)

                    
                    for item in self.i_state.myInventoryList:
                        if item["name"] == "AlienFlower":
                            item["amount"] = 0
                    await self.a_agent.send_message("action", "leave,AlienFlower, 2")

        except asyncio.CancelledError:
            await self.a_agent.send_message("action","stop")


# # !!!! I am here
# """PART 3: Scenario Collect-and-run"""
# class EscapeFromCritter:
#     """
#     Flee from any detected 'Critter' object without teleporting.

#     Emergency plan
#     --------------
#     • Stop → 180° turn to the safer side
#     • Dash forward for `panic_dash_time` seconds
#     • Resume ordinary flee logic
#     """

#     safe_distance      = 5.0     # m – ‘far enough’
#     emergency_distance = 1.5     # m – triggers hard-panic dash
#     panic_dash_time    = 1.5     # s – straight run after the 180°
#     max_turn_per_tick  = 60      # deg – cap normal steering
#     grace_period       = 2.0     # s – no critters seen → SUCCESS
#     tick               = 0.1     # s – main loop sleep

#     def __init__(self, a_agent):
#         self.a_agent   = a_agent
#         self.rc_sensor = a_agent.rc_sensor
#         self.i_state   = a_agent.i_state

#     async def run(self):
#         last_seen_time = asyncio.get_event_loop().time()

#         try:
#             while True:
#                 # ---- 1. read sensor rays ----------------------------------
#                 hits      = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]
#                 info      = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.OBJECT_INFO]
#                 angles    = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.ANGLE]
#                 distances = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.DISTANCE]

#                 danger_x = danger_z = 0.0
#                 min_dist = float("inf")

#                 for hit, inf, ang, dist in zip(hits, info, angles, distances):
#                     if hit and inf and inf["tag"] == "CritterMantaRay":
#                         weight   = 1.0 / (dist + 1e-3) ** 2
#                         rad      = math.radians(ang)
#                         danger_x += math.sin(rad) * weight
#                         danger_z += math.cos(rad) * weight
#                         min_dist  = min(min_dist, dist)

#                 sees_critter = min_dist < float("inf")
#                 if sees_critter:
#                     last_seen_time = asyncio.get_event_loop().time()

#                 # ---- 2. clear? --------------------------------------------
#                 if asyncio.get_event_loop().time() - last_seen_time >= self.grace_period:
#                     await self.a_agent.send_message("action", "stop")
#                     await self.a_agent.send_message("action", "nt")
#                     print("[Escape] Safe: no critters in view.")
#                     return True

#                 if not sees_critter:          # nothing new this tick
#                     await asyncio.sleep(self.tick)
#                     continue

#                 # ---- 3. compute flee steering -----------------------------
#                 flee_ang = (math.degrees(math.atan2(-danger_x, -danger_z))) % 360
#                 cur_ang  = self.i_state.rotation["y"]
#                 turn_deg = angle_delta(flee_ang, cur_ang)
#                 turn_dir = "tl" if ((flee_ang - cur_ang) % 360) > 180 else "tr"
#                 turn_deg = min(turn_deg, self.max_turn_per_tick)

#                 # ---- 4. emergency dash ------------------------------------
#                 if min_dist <= self.emergency_distance:
#                     # choose 180° direction biased to side with fewer critters
#                     left_hits  = sum(hits[: len(hits) // 2])
#                     right_hits = sum(hits[len(hits) // 2 + 1 :])
#                     turn_dir   = "tr" if left_hits >= right_hits else "tl"
#                     print(f"[Escape] PANIC! Closest critter {min_dist:.2f} m – 180° {turn_dir}")
#                     await self.a_agent.send_message("action", "stop")
#                     await asyncio.sleep(0.05)
#                     await execute_turn(self.a_agent, self.i_state, turn_dir, 180)
#                     await self.a_agent.send_message("action", "mf")
#                     await asyncio.sleep(self.panic_dash_time)
#                     await self.a_agent.send_message("action", "ntm")
#                     continue  # go back to sensor loop

#                 # ---- 5. normal flee steering ------------------------------
#                 await self.a_agent.send_message("action", "stop")
#                 await asyncio.sleep(0.05)
#                 await execute_turn(self.a_agent, self.i_state, turn_dir, turn_deg)
#                 await self.a_agent.send_message("action", "mf")
#                 await asyncio.sleep(self.tick)

#         except asyncio.CancelledError:
#             print("***** TASK EscapeFromCritter CANCELLED")
#             await self.a_agent.send_message("action", "stop")
#             await self.a_agent.send_message("action", "nt")
#             return False



# class EscapeFromCritter:
#     """
#     One–shot reflex escape:
#         • Decide side   (left, right, or random if dead-ahead)
#         • Snap-turn away (panic_turn_deg)
#         • Dash forward  (panic_dash_time)
#     Then return SUCCESS so the BT can fall back to its normal selector.
#     """

#     # --- Tunables ----------------------------------------------------------
#     side_threshold     = 5.0     # deg – consider |angle| ≤ threshold as “front”
#     panic_turn_deg     = 90      # deg – how much to turn away
#     panic_dash_time    = 2.0     # s   – how long to sprint after turning
#     tick               = 0.05    # s   – inner-loop sleep while searching

#     def __init__(self, a_agent):
#         self.a_agent   = a_agent
#         self.rc_sensor = a_agent.rc_sensor
#         self.i_state   = a_agent.i_state

#     # ----------------------------------------------------------------------
#     async def run(self):
#         try:
#             while True:
#                 # 1) Scan rays for critters --------------------------------
#                 hits      = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]
#                 info      = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.OBJECT_INFO]
#                 angles    = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.ANGLE]
#                 distances = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.DISTANCE]

#                 critter_rays = [
#                     (ang, dist) for hit, inf, ang, dist
#                     in zip(hits, info, angles, distances)
#                     if hit and inf and inf["tag"] == "CritterMantaRay"
#                 ]

#                 if not critter_rays:
#                     # No critter in sight → SUCCESS, escape complete
#                     return True

#                 # 2) Pick the “most dangerous” ray (closest) ---------------
#                 nearest_ang, nearest_dist = min(critter_rays, key=lambda ad: ad[1])

#                 # 3) Decide escape side ------------------------------------
#                 if abs(nearest_ang) <= self.side_threshold:
#                     # Dead ahead → pick the safer side
#                     left_hits  = sum(1 for ang, _ in critter_rays if ang <  -self.side_threshold)
#                     right_hits = sum(1 for ang, _ in critter_rays if ang >  self.side_threshold)

#                     if left_hits == right_hits:           # tie → who’s farther?
#                         avg_left  = sum(dist for ang, dist in critter_rays
#                                         if ang <  -self.side_threshold) / (left_hits  or 1)
#                         avg_right = sum(dist for ang, dist in critter_rays
#                                         if ang >   self.side_threshold) / (right_hits or 1)
#                         turn_dir = "tr" if avg_left >= avg_right else "tl"
#                     else:
#                         turn_dir = "tr" if left_hits >= right_hits else "tl"
#                 else:
#                     # Simple opposite-side reflex
#                     turn_dir = "tr" if nearest_ang < 0 else "tl"

#                 # 4) Snap-turn & dash --------------------------------------
#                 await self.a_agent.send_message("action", "stop")
#                 await asyncio.sleep(0.02)
#                 await execute_turn(self.a_agent, self.i_state,
#                                    turn_dir, self.panic_turn_deg)
#                 await self.a_agent.send_message("action", "mf")
#                 await asyncio.sleep(self.panic_dash_time)
#                 await self.a_agent.send_message("action", "ntm")

#                 # 5) Escape done – return SUCCESS --------------------------
#                 return True

#                 # If you’d rather keep looking for more critters after the
#                 # dash, remove the ‘return True’ above and loop again.

#                 await asyncio.sleep(self.tick)

#         except asyncio.CancelledError:
#             print("***** TASK EscapeFromCritter CANCELLED")
#             await self.a_agent.send_message("action", "stop")
#             await self.a_agent.send_message("action", "nt")
#             return False




class EscapeFromCritter:
    """
    Reflex escape that never stops.
    ───────────────────────────────
    • Decide escape side   (away from nearest / majority critters)
    • Snap-turn *while still moving forward*
    • Dash for `dash_time` seconds
    • Return SUCCESS so BT can fall back to normal behaviours
    """

    # ------------------- Tunables -----------------------------------------
    side_threshold      = 5.0      # deg  → consider |angle| ≤ threshold as "front"
    base_turn_deg       = 90       # deg  → snap-turn for 1 critter
    base_dash_time      = 2.0      # sec  → dash duration for 1 critter
    extra_turn_factor   = 0.5      # each extra critter adds 50 % more turn
    extra_dash_factor   = 0.5      # each extra critter adds 50 % more dash
    turn_tick           = 0.05     # sec  → sleep while turning
    scan_tick           = 0.05     # sec  → idle sleep when no critter

    def __init__(self, a_agent):
        self.a_agent   = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state   = a_agent.i_state

    # ------------------- helpers -----------------------------------------
    async def turn_while_moving(self, direction: str, degrees: float):
        """
        Turn `degrees` on-the-fly (no full stop): keep 'mf' engaged,
        send 'tl' or 'tr' every tick until the rotation is achieved.
        """
        total_rot = 0.0
        prev_yaw  = self.i_state.rotation['y']
        # make sure we’re already moving forward
        await self.a_agent.send_message("action", "mf")

        while total_rot < degrees:
            await self.a_agent.send_message("action", direction)
            await asyncio.sleep(self.turn_tick)
            cur_yaw   = self.i_state.rotation['y']
            total_rot += angle_delta(cur_yaw, prev_yaw)
            prev_yaw  = cur_yaw

        # stop turning (leave "mf" running)
        await self.a_agent.send_message("action", "nt")

    # ------------------- main coroutine ----------------------------------
    async def run(self):
        try:
            while True:
                # 1) Scan rays for critters
                hits      = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]
                info      = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.OBJECT_INFO]
                angles    = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.ANGLE]
                distances = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.DISTANCE]

                critter_rays = [
                    (ang, dist) for hit, inf, ang, dist in
                    zip(hits, info, angles, distances)
                    if hit and inf and inf['tag'] == 'CritterMantaRay'
                ]

                if not critter_rays:
                    # Nothing in sight → SUCCESS
                    return True

                # 2) How many?  Adjust turn / dash if >1
                n_critters   = len(critter_rays)
                turn_deg     = self.base_turn_deg  * (1 + self.extra_turn_factor  * (n_critters - 1))
                dash_time    = self.base_dash_time * (1 + self.extra_dash_factor * (n_critters - 1))

                # 3) Pick side to flee
                # nearest critter:
                nearest_ang, _ = min(critter_rays, key=lambda ad: ad[1])

                if abs(nearest_ang) <= self.side_threshold:          # dead ahead
                    # choose side with fewer critter rays (tie → random)
                    left_count  = sum(1 for ang, _ in critter_rays if ang <  -self.side_threshold)
                    right_count = sum(1 for ang, _ in critter_rays if ang >   self.side_threshold)
                    if left_count == right_count:
                        turn_dir = random.choice(["tl", "tr"])
                    else:
                        turn_dir = "tr" if left_count >= right_count else "tl"
                else:
                    # simple reflex opposite side
                    turn_dir = "tr" if nearest_ang < 0 else "tl"

                # 4) Execute turn-while-moving and dash
                await self.turn_while_moving(turn_dir, turn_deg)
                await asyncio.sleep(dash_time)     # keep dashing forward
                # don’t stop motors here – let other BT nodes decide

                # 5) After dash, re-scan immediately (loop continues)
                await asyncio.sleep(0)             # yield control back to loop

        except asyncio.CancelledError:
            print("***** TASK EscapeFromCritter CANCELLED")
            # Just stop turning – leave forward motion control to caller
            await self.a_agent.send_message("action", "nt")
            return False



