import random
import asyncio
import Sensors


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

	ROTATION_THRESHOLD = 2  # degrees tolerance for stopping

	def __init__(self, a_agent):
		self.a_agent = a_agent
		self.rc_sensor = a_agent.rc_sensor
		self.i_state = a_agent.i_state

	def angle_delta(self, current, previous):
		"""
		Returns the smallest positive arc difference between two angles (0..180).
		Example:
			previous=10, current=350 -> 20
			previous=350, current=10 -> 20
		"""
		raw_diff = (current - previous) % 360
		# If it's over 180, we flip it to get the smaller side of the circle
		if raw_diff > 180:
			raw_diff = 360 - raw_diff
		return raw_diff

	async def run(self):
		try:
			while True:
				# Pick a turn angle and direction
				angle = 90#random.randint(10, 360)
				direction = random.choice(["tl", "tr"])  # tl = left, tr = right

				print(f"\nTurning {angle}° {'left' if direction == 'tl' else 'right'}")

				# Total rotation accumulated
				total_rotated = 0
				prev_orientation = self.i_state.rotation['y'] # yaw

				while True:
					# Send the turn command
					await self.a_agent.send_message("action", direction)
					# sleep time based on remaining angle
					await asyncio.sleep(0.1)

					current_orientation = self.i_state.rotation['y']
					delta = self.angle_delta(current_orientation, prev_orientation)

					# We simply add how many degrees changed (the "short arc") 
					total_rotated += delta
					prev_orientation = current_orientation

					print(f"Rotated: {total_rotated:.2f}° / {angle}°")

					# If we're close enough , stop turning
					if total_rotated >= angle-self.ROTATION_THRESHOLD or total_rotated > angle:
						await self.a_agent.send_message("action", "nt")  # Stop turning
						await asyncio.sleep(0)
						print("Turn complete.\n")
						break
				
				
				# Wait 2 seconds before the next turn
				await asyncio.sleep(2)

		except asyncio.CancelledError:
			print("*** TASK Turn CANCELLED")
			await self.a_agent.send_message("action", "stop")
			self.state = self.STOPPED

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
		self.state = self.STOPPED
		self.turn_direction = None
		self.turned = 0
		self.num_turns = None  

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
						if any(ray_hit == 1 for ray_hit in self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]):
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
					# Initialize turning parameters
					self.turn_direction = random.choice(["tl", "tr"])
					turn_degrees = random.randint(10, 90)
					self.num_turns = turn_degrees // 15

					print(f"Turning {turn_degrees}° {'left' if self.turn_direction == 'tl' else 'right'}")
					
					await self.a_agent.send_message("action", "stop")
					await asyncio.sleep(0.2)
					
					# Perform the turns
					for _ in range(self.num_turns):
						await self.a_agent.send_message("action", self.turn_direction)
						await asyncio.sleep(0.3)
					
					await self.a_agent.send_message("action", "nt")
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

	def __init__(self, a_agent, sensitivity=1.0, max_turn_angle=30, distance_threshold=0.3, 
				emergency_distance=0.2):
		self.a_agent = a_agent
		self.rc_sensor = a_agent.rc_sensor
		self.i_state = a_agent.i_state
		self.state = self.STOPPED
		
		# Core parameters
		self.sensitivity = sensitivity
		self.max_turn_angle = max_turn_angle
		self.distance_threshold = distance_threshold
		self.emergency_distance = emergency_distance
		self.ROTATION_THRESHOLD = 2
		
		# Calculate ray angles and responses
		angle_between_rays = self.rc_sensor.max_ray_degrees / self.rc_sensor.rays_per_direction
		self.ray_degrees = [
			angle_between_rays * (i - self.rc_sensor.rays_per_direction) 
			for i in range(self.rc_sensor.num_rays)
		]
		self.calculate_turn_responses()
	
	def angle_delta(self, current, previous):
		"""Returns the smallest positive angle difference between two orientations"""
		raw_diff = (current - previous) % 360
		return 360 - raw_diff if raw_diff > 180 else raw_diff
	
	async def execute_turn(self, direction, angle):
		"""Execute a precise turn using orientation tracking"""
		if angle < 5:
			await self.a_agent.send_message("action", direction)
			await asyncio.sleep(0.1)
			await self.a_agent.send_message("action", "nt")
			return

		print(f"Turning {angle:.1f}° {'left' if direction == 'tl' else 'right'}")
		
		total_rotated = 0
		prev_orientation = self.i_state.rotation['y']
		
		while True:
			await self.a_agent.send_message("action", direction)
			await asyncio.sleep(0.1)
			
			current_orientation = self.i_state.rotation['y']
			delta = self.angle_delta(current_orientation, prev_orientation)
			total_rotated += delta
			prev_orientation = current_orientation
			
			if total_rotated >= angle - self.ROTATION_THRESHOLD:
				await self.a_agent.send_message("action", "nt")
				break
	
	def calculate_turn_responses(self):
		"""Calculate optimal turn angles for each ray based on position"""
		def turn_angle_for_ray(ray_angle):
			# Normalize position (-1 to 1)
			normalized_pos = ray_angle / self.rc_sensor.max_ray_degrees
			
			# Base angle calculation with central emphasis
			base_angle = -self.max_turn_angle * (normalized_pos * 0.6)
			central_weight = 1.0 - (abs(normalized_pos) ** 1.8)
			
			return base_angle * (1 + central_weight) * self.sensitivity
		
		self.ray_turn_degrees = [turn_angle_for_ray(angle) for angle in self.ray_degrees]
	
	async def handle_corner_trap(self, left_hits, right_hits):
		"""Execute escape maneuver when trapped in a corner"""
		print("CORNER TRAP DETECTED! Executing escape maneuver")
		direction = "tl" if left_hits <= right_hits else "tr"
		await self.a_agent.send_message("action", "stop")
		await asyncio.sleep(0.2)
		await self.execute_turn(direction, 45)
		await self.a_agent.send_message("action", "mf")
	
	async def handle_obstacle_avoidance(self, turn_direction, turn_degrees, emergency=False):
		"""Handle different obstacle scenarios with appropriate responses"""
		need_to_stop = emergency or turn_degrees > 25
		
		if need_to_stop:
			await self.a_agent.send_message("action", "stop")
			await asyncio.sleep(0.1)
			await self.execute_turn(turn_direction, turn_degrees)
			await self.a_agent.send_message("action", "mf")
		else:
			# Turn while moving for minor adjustments
			if turn_degrees < 8:
				await self.a_agent.send_message("action", turn_direction)
				await asyncio.sleep(0.12)
				await self.a_agent.send_message("action", "nt")
			else:
				turns = max(1, int(turn_degrees // 8))
				for _ in range(min(turns, 2)):
					await self.a_agent.send_message("action", turn_direction)
					await asyncio.sleep(0.15)
				await self.a_agent.send_message("action", "nt")
				
			# Occasionally ensure forward movement
			if random.random() < 0.3:
				await self.a_agent.send_message("action", "mf")
	
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
					
					if not any(sensor_hits):
						# No obstacles - occasionally ensure straight movement
						if random.random() < 0.05:
							await self.a_agent.send_message("action", "nt")
							await self.a_agent.send_message("action", "mf")
						await asyncio.sleep(0.08)
						continue
					
					# Process obstacle data
					total_hit_count = sum(sensor_hits)
					left_hits = sum(sensor_hits[:len(sensor_hits)//2])
					right_hits = sum(sensor_hits[len(sensor_hits)//2:])
					
					# Calculate turn response
					total_turn = 0
					total_weight = 0
					emergency_count = 0
					min_distance = float('inf')
					
					for i, (hit, distance) in enumerate(zip(sensor_hits, sensor_distances)):
						if hit == 1:
							min_distance = min(min_distance, distance)
							
							if distance <= self.emergency_distance:
								emergency_count += 1
							
							weight = min(1.0, self.distance_threshold / max(distance, 0.01))
							total_turn += self.ray_turn_degrees[i] * weight
							total_weight += weight
					
					if total_weight == 0:
						await asyncio.sleep(0.08)
						continue
					
					# Determine response type
					emergency = emergency_count >= 2 or min_distance < self.emergency_distance * 0.7
					corner_trap = total_hit_count >= 3 and abs(total_turn / total_weight) < 5.0
					
					if corner_trap:
						await self.handle_corner_trap(left_hits, right_hits)
					else:
						avg_turn = total_turn / total_weight
						turn_direction = "tr" if avg_turn > 0 else "tl"
						turn_degrees = abs(avg_turn)
						await self.handle_obstacle_avoidance(turn_direction, turn_degrees, emergency)
					
					await asyncio.sleep(0.08)
					
				else:
					self.state = self.STOPPED
					await asyncio.sleep(0.5)
		
		except asyncio.CancelledError:
			print("***** TASK Avoid CANCELLED")
			await self.a_agent.send_message("action", "stop")
			await self.a_agent.send_message("action", "nt")
			self.state = self.STOPPED