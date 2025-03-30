import random
import asyncio
import Sensors
import math

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


class DoNothing:
	"""Does nothing."""
	def __init__(self, a_agent):
		self.a_agent = a_agent

	async def run(self):
		print("Doing nothing")
		await asyncio.sleep(1)
		return True


class ForwardStop:
	"""Moves forward until it finds an obstacle, then stops."""
	STOPPED = 0
	MOVING = 1
	END = 2

	def __init__(self, a_agent):
		self.a_agent = a_agent
		self.rc_sensor = a_agent.rc_sensor
		self.state = self.STOPPED

	async def run(self):
		try:
			while True:
				if self.state == self.STOPPED:
					await self.a_agent.send_message("action", "mf")
					self.state = self.MOVING
				elif self.state == self.MOVING:
					sensor_hits = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]
					if any(ray_hit == 1 for ray_hit in sensor_hits):
						self.state = self.END
						await self.a_agent.send_message("action", "stop")
					else:
						await asyncio.sleep(0.1)
				elif self.state == self.END:
					break
				else:
					print(f"Unknown state: {self.state}")
					return False
		except asyncio.CancelledError:
			print("***** TASK Forward CANCELLED")
			await self.a_agent.send_message("action", "stop")
			self.state = self.STOPPED


class Turn:
	"""
	The drone turns a requested number of degrees using a cumulative approach.
	"""
	def __init__(self, a_agent):
		self.a_agent = a_agent
		self.rc_sensor = a_agent.rc_sensor
		self.i_state = a_agent.i_state

	async def run(self):
		try:
			while True:
				# Pick a turn angle and direction
				angle = random.randint(10, 360)
				direction = random.choice(["tl", "tr"])  # tl = left, tr = right

				print(f"Turning {angle}° {'left' if direction == 'tl' else 'right'}")
				await execute_turn(self.a_agent, self.i_state, direction, angle)

				# Wait 2 seconds before the next turn
				await asyncio.sleep(2)

		except asyncio.CancelledError:
			print("*** TASK Turn CANCELLED")
			await self.a_agent.send_message("action", "stop")

class RandomRoam:
	"""
	Moves around randomly, changing direction and deciding when to stop,
	based on predefined probabilities.
	"""
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
	"""
	The Drone advances while avoiding obstacles using distance-weighted ray sensors.
	Features corner escape and emergency response capabilities.
	"""
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
		"""Calculate optimal turn angles for each ray"""
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
					right_hits = sum(sensor_hits[len(sensor_hits)//2:])
					
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
					# Emergency if 2 or more rays are too close or if any ray is too close
					emergency = emergency_count >= 2 or min_distance < self.emergency_distance * 0.5
					# Corner trap if 3 or more rays hit and total weight is high
					corner_trap = total_hit_count >= 3 and total_weight > 2
					
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
			print("***** TASK Avoid CANCELLED")
			await self.a_agent.send_message("action", "stop")
			await self.a_agent.send_message("action", "nt")
			self.state = self.STOPPED