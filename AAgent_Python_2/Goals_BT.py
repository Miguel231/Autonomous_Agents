import math
import random
import asyncio
import Sensors

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
	raw_diff = (current - previous) % 360  # this is for that we always get angle between 0 and 360        
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
            # print("***** TASK Forward CANCELLED")
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
            # print("***** TASK Turn CANCELLED")
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
						# print("CORNER TRAP DETECTED! Executing escape maneuver")
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
			# print("***** TASK Avoid CANCELLED")
			await self.a_agent.send_message("action", "nt")
			self.state = self.STOPPED


def flower_count(i_state):
    """
    flowers in the inventory
    """
    return next(
        (item['amount'] for item in i_state.myInventoryList if item['name'] == 'AlienFlower'),
        0 # If not found
    )


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
		# print(f"Ray check order: {self.ray_check_order}")

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
					# print("Flower collected!")
					await self.a_agent.send_message("action", "stop")
					return
				await asyncio.sleep(0.5)

		except asyncio.CancelledError:
			# print("***** TASK GetFlower CANCELLED")
			# await self.a_agent.send_message("action", "stop")
			await self.a_agent.send_message("action", "nt")


bitten_astronaut=False # global so all critters can act when any of them bites an astronaut


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
		# print(f"Ray check order: {self.ray_check_order}")

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
						# print(ray[Sensors.RayCastSensor.DISTANCE])
						if ray[Sensors.RayCastSensor.DISTANCE]<1:
							# print(ray[Sensors.RayCastSensor.DISTANCE])
							astronautBitten=True
						else:
							astronautBitten=False

						break
				if astronautDetected:
					await self.a_agent.send_message("action", self.turn_map[self.side])
					await self.a_agent.send_message("action", "mf")

				await asyncio.sleep(0.5)

				if astronautBitten:
					# print("Astronaut bitten!") 
					global bitten_astronaut
					if not bitten_astronaut:
						bitten_astronaut=True
					return True
				
		except asyncio.CancelledError:
			# print("***** TASK GetAstronaut CANCELLED")
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

			# print("MoveAway completed")
			global bitten_astronaut
			if bitten_astronaut:
				bitten_astronaut=False

			return True
		
		except asyncio.CancelledError:
			# print("***** TASK MoveAway CANCELLED")
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
					# print("At base")
					await self.a_agent.send_message("action", "stop")
					await self.a_agent.send_message("action", "nt")
					await asyncio.sleep(0.2)
					await self.a_agent.send_message("action", "leave,AlienFlower,2")
					# print("Unloaded flowers")
					return True
				else:
					# print("Returning to base")
					# await self.a_agent.send_message("action", "walk_to,Base")
					await self.a_agent.send_message("action", "teleport_to,Base")
					await asyncio.sleep(0.5)
		except asyncio.CancelledError:
			await self.a_agent.send_message("action", "nt")


class EscapeFromCritter:
    """
    Reflex escape for exactly ONE critter in view.
    Turns away from the critter and dashes forward without stopping.
    """

    front_thresh_deg = 5.0      # |angle| ≤ this → treat as “front”
    turn_deg         = 90       # snap-turn amount
    dash_time        = 0.1      # seconds to keep dashing after the turn
    turn_tick        = 0.05     # sleep while applying turn impulses

    def __init__(self, agent):
        self.agent     = agent
        self.rc_sensor = agent.rc_sensor
        self.i_state   = agent.i_state

    # ---------- helper: turn while still moving forward ------------------
    async def turn_while_moving(self, direction, degrees):
        """Keep 'mf' active; pulse 'tl' or 'tr' until we've rotated `degrees`."""
        total_rot = 0.0
        prev_yaw  = self.i_state.rotation["y"]

        # make sure we’re already driving forward
        # await self.agent.send_message("action", "mf")

        while total_rot < degrees:
            await self.agent.send_message("action", direction)
            await asyncio.sleep(self.turn_tick)
            cur_yaw   = self.i_state.rotation["y"]
            # angle_delta = shortest arc difference (0-180)
			# we are doing way for example delta = cur_yaw - prev_yaw: 5 - 350 = -345
			# this is meaningless, in reality we have turned 15 degrees to the right
	
            delta     = (cur_yaw - prev_yaw) % 360
			# Clamp angle difference to shortest arc (e.g., 270° → 90° turn the other way)
			# Example: from 350° to 80° → (delta = 90, not 270)
            delta     = 360 - delta if delta > 180 else delta
            total_rot += delta
            prev_yaw  = cur_yaw

        # stop turning; keep forward motion
        await self.agent.send_message("action", "nt")

    # ---------- main coroutine ------------------------------------------
    async def run(self):
        try:
            # scan once, react once, then return SUCCESS
            hits      = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]
            info      = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.OBJECT_INFO]
            angles    = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.ANGLE]

            critter_angle = None
            for hit, inf, ang in zip(hits, info, angles):
                if hit and inf and inf["tag"] == "CritterMantaRay":
                    critter_angle = ang          # we found THE critter
                    break

            if critter_angle is None:
                # no critter in sight -> nothing to flee from
                return True

            # decide escape side
            if abs(critter_angle) <= self.front_thresh_deg:       # straight ahead
                turn_dir = random.choice(["tl", "tr"])
            else:
                turn_dir = "tr" if critter_angle < 0 else "tl"    # left critter -> turn right

            # execute snap-turn while moving
            await self.turn_while_moving(turn_dir, self.turn_deg)

            # dash away
            await asyncio.sleep(self.dash_time)   # still in 'mf' state
            await self.agent.send_message("action", "ntm")  # stop forward motor

            return True                            # SUCCESS – escape finished

        except asyncio.CancelledError:
            # print("***** TASK EscapeFromSingleCritter CANCELLED")
            # stop turning impulses; leave forward control to caller
            await self.agent.send_message("action", "nt")
            return False


