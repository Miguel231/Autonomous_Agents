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
	"""Randomly turns left or right between 10 and 360 degrees."""

	def __init__(self, a_agent):
		self.a_agent = a_agent

	async def run(self):
		while True:
			turn_direction = random.choice(["tl", "tr"])
			turn_degrees = random.randint(10, 360)

			print(f"Turning {turn_degrees}° {'left' if turn_direction == 'tl' else 'right'}")

			turns_needed = turn_degrees // 15  # 5 degrees per step

			for _ in range(turns_needed):
				await self.a_agent.send_message("action", turn_direction)
				await asyncio.sleep(0.3)

			await self.a_agent.send_message("action", "nt")
			print("Turn completed. Stopping rotation.")

			await asyncio.sleep(2)


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
	The Drone advances while avoiding obstacles, ensuring it does not collide with objects, including exterior walls.
	Incorporates ray distance to make smarter avoidance decisions.
	"""
	STOPPED = 0
	MOVING = 1
	TURNING = 2
	STOP = 3

	def __init__(self, a_agent, sensitivity=1.0, max_turn_angle=30, center_multiplier=1.2, 
				distance_threshold=0.5, emergency_distance=0.2):
		self.a_agent = a_agent
		self.rc_sensor = a_agent.rc_sensor
		self.state = self.STOPPED
		
		# Adjustable parameters
		self.sensitivity = sensitivity  # Overall sensitivity multiplier
		self.max_turn_angle = max_turn_angle  # Maximum degrees to turn
		self.center_multiplier = center_multiplier  # How much more to turn for central hits
		self.distance_threshold = distance_threshold  # Maximum distance to consider for full turn effect (normalized 0-1)
		self.emergency_distance = emergency_distance  # Distance at which emergency stop and turn is triggered
		
		# Calculate angles of each ray
		angle_between_rays = self.rc_sensor.max_ray_degrees / self.rc_sensor.rays_per_direction
		self.ray_degrees = [
			angle_between_rays * (i - self.rc_sensor.rays_per_direction) for i in range(self.rc_sensor.num_rays)
		]
		
		# Define turn response to each ray hit using a smoother function
		self.calculate_turn_responses()
		
	def calculate_turn_responses(self):
		"""Calculate smooth turn responses for each ray"""
		
		# Function to calculate turn angle based on ray position
		def turn_angle_for_ray(ray_angle):
			# Normalize ray angle to -1 (leftmost) to 1 (rightmost)
			normalized_pos = ray_angle / self.rc_sensor.max_ray_degrees
			
			# Base angle - smoother curve with cubic function
			# Negative values for right rays (turn left), positive for left rays (turn right)
			# Using cubic function for smoother transition with less aggressive values
			base_angle = -self.max_turn_angle * (normalized_pos * 0.6)  # Reduced from 0.8 to make it gentler
			
			# Apply central emphasis using a smoother bell curve
			# This makes central impacts cause larger turns than extreme side impacts
			central_weight = 1.0 - (abs(normalized_pos) ** 1.8)  # Increased from 1.5 for more gradual falloff
			central_adjustment = self.center_multiplier * central_weight
			
			# Calculate final turn angle with sensitivity adjustment
			turn_angle = base_angle * (1 + central_adjustment) * self.sensitivity
			
			return turn_angle
		
		# Calculate turn angle for each ray
		self.ray_turn_degrees = [turn_angle_for_ray(angle) for angle in self.ray_degrees]
		
		# For debugging
		print("Ray degrees:", self.ray_degrees)
		print("Turn response:", self.ray_turn_degrees)

	async def run(self):
		try:
			while True:
				if self.state == self.STOPPED:
					print("Starting to move forward")
					await self.a_agent.send_message("action", "mf")
					self.state = self.MOVING
					await asyncio.sleep(0.5)
					
				elif self.state == self.MOVING:
					sensor_hits = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]
					sensor_distances = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.DISTANCE]
					
					# Count total hit rays for corner detection
					total_hit_count = sum(sensor_hits)
					
					# Check for corner trap (many rays hit with small net turn angle)
					corner_trap = total_hit_count >= 3
					emergency_stop = False
					
					# If any hits detected
					if any(sensor_hits):
						# Calculate turn amount based on all detected rays, weighted by distance
						total_turn = 0
						total_weight = 0
						emergency_count = 0
						
						# Determine if most hits are on left or right for corner escape
						left_hits = sum(sensor_hits[:len(sensor_hits)//2])
						right_hits = sum(sensor_hits[len(sensor_hits)//2:])
						
						for i, (hit, distance) in enumerate(zip(sensor_hits, sensor_distances)):
							if hit == 1:  # If ray hit something
								# Check for emergency stop condition
								if distance <= self.emergency_distance:
									emergency_count += 1
								
								# Calculate distance weight - closer objects have stronger effect
								clamped_distance = max(distance, 0.01)
								distance_weight = min(1.0, self.distance_threshold / clamped_distance)
								
								total_turn += self.ray_turn_degrees[i] * distance_weight
								total_weight += distance_weight
						
						# Emergency stop if multiple rays detect very close obstacles
						if emergency_count >= 2:
							emergency_stop = True
						
						if total_weight > 0:
							# Weighted average of turns based on distance
							avg_turn = total_turn / total_weight
							
							# Handle the corner trap case - where net turn is very small but many obstacles
							if corner_trap and abs(avg_turn) < 5.0:
								print("CORNER TRAP DETECTED! Executing escape maneuver")
								# Choose direction with fewer obstacles
								if left_hits <= right_hits:
									turn_direction = "tl"  # Turn left if fewer obstacles on left
									print("Escaping by turning LEFT")
								else:
									turn_direction = "tr"  # Turn right if fewer obstacles on right
									print("Escaping by turning RIGHT")
								
								# Set a large turn angle to escape the corner
								turn_degrees = 45  # Large enough to escape most corners
								emergency_stop = True  # Treat like an emergency
							else:
								# Normal case - determine turn direction from calculated angle
								turn_direction = "tr" if avg_turn > 0 else "tl"
								turn_degrees = abs(avg_turn)
							
							# Limit maximum turn per iteration to make it smoother
							# Allow larger turns for emergency or corner trap
							turn_degrees = min(turn_degrees, 45 if emergency_stop or corner_trap else 15)
							
							# Calculate turn commands
							turns_needed = max(1, int(turn_degrees // 5))
							
							print(f"Obstacle: Turning {turn_degrees:.1f}° {'right' if turn_direction == 'tr' else 'left'}")
							
							if emergency_stop or corner_trap:
								# Emergency/corner maneuver: stop completely and make a decisive turn
								print("EMERGENCY STOP: Obstacle too close or corner trap")
								await self.a_agent.send_message("action", "stop")
								await asyncio.sleep(0.2)
								
								# Execute turns in smaller increments but more decisively
								for _ in range(min(turns_needed+2, 9)):  # More turns for emergency/corner
									await self.a_agent.send_message("action", turn_direction)
									await asyncio.sleep(0.15)
								
								# Stop turning and carefully resume movement
								await self.a_agent.send_message("action", "nt")
								await asyncio.sleep(0.3)
								await self.a_agent.send_message("action", "mf")
							
							# For moderate turns, keep moving while turning
							elif turn_degrees <= 15:
								# Send turn command while moving
								await self.a_agent.send_message("action", turn_direction)
								await asyncio.sleep(0.15)
								await self.a_agent.send_message("action", "nt")
							else:
								# For sharper turns, slow down briefly
								await self.a_agent.send_message("action", "stop")
								await asyncio.sleep(0.1)
								
								# Execute turns in smaller increments
								for _ in range(min(turns_needed, 3)):
									await self.a_agent.send_message("action", turn_direction)
									await asyncio.sleep(0.15)
								
								# Stop turning and resume movement
								await self.a_agent.send_message("action", "nt")
								await asyncio.sleep(0.1)
								await self.a_agent.send_message("action", "mf")
					else:
						# No hits detected, continue moving forward
						# Occasionally ensure we're moving straight
						if random.random() < 0.05:  # 5% chance each cycle
							await self.a_agent.send_message("action", "nt")
					
					# Small delay between sensor checks
					await asyncio.sleep(0.1)
					
				else:
					print(f"Unknown state: {self.state}")
					self.state = self.STOPPED
					await asyncio.sleep(0.5)
		
		except asyncio.CancelledError:
			print("***** TASK Avoid CANCELLED")
			await self.a_agent.send_message("action", "stop")
			await self.a_agent.send_message("action", "nt")
			self.state = self.STOPPED
