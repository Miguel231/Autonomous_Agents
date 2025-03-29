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
	The Drone advances while avoiding obstacles, ensuring it does not collide with objects, including exterior walls
	"""
	STOPPED = 0
	MOVING = 1
	TURNING = 2
	STOP = 3

	def __init__(self, a_agent):
		self.a_agent = a_agent
		self.rc_sensor = a_agent.rc_sensor
		self.state = self.STOPPED
		angle_between_rays = self.rc_sensor.max_ray_degrees / self.rc_sensor.rays_per_direction
		self.ray_degrees = [
			angle_between_rays * (i - self.rc_sensor.rays_per_direction) for i in range(self.rc_sensor.num_rays)
		]
		# Define turn response to each ray hit
		# We'll use a continuous function that:
		# - Makes left rays turn right (positive values)
		# - Makes right rays turn left (negative values)
		# - Makes central impacts cause large turns
		# - Makes extreme side impacts cause smaller turning adjustments
		
		# Parameters for our turn function
		max_turn_angle = 45  # Maximum degrees to turn
		center_multiplier = 1.5  # How much more to turn for central hits
		
		# Function to calculate turn angle based on ray position
		def turn_angle_for_ray(ray_angle):
			# Normalize ray angle to -1 (leftmost) to 1 (rightmost)
			normalized_pos = ray_angle / self.rc_sensor.max_ray_degrees
			
			# Sigmoid-like function with stronger response in middle
			# For ray on right side (positive angle), turn left (negative angle)
			base_angle = -max_turn_angle * normalized_pos
			
			# Amplify turn for central hits using a bell curve
			central_factor = center_multiplier * (1 - abs(normalized_pos))
			turn_angle = base_angle * (1 + central_factor)
			
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
					
					# If any hits detected
					if any(sensor_hits):
						# Calculate turn amount based on all detected rays
						total_turn = 0
						num_hits = 0
						
						for i, hit in enumerate(sensor_hits):
							if hit == 1:  # If ray hit something
								total_turn += self.ray_turn_degrees[i]
								num_hits += 1
						
						if num_hits > 0:
							# Average the turn responses for all hits
							avg_turn = total_turn / num_hits
							
							# Determine turn direction
							turn_direction = "tr" if avg_turn > 0 else "tl"
							turn_degrees = abs(avg_turn)
							
							# Calculate how many turn commands to send
							# Each turn command is approximately 5-15 degrees
							turns_needed = max(1, int(turn_degrees // 15))
							
							print(f"Obstacle detected! Turning {turn_degrees:.1f}° {'right' if turn_direction == 'tr' else 'left'}")
							
							# Send a single turn command, but keep moving forward
							await self.a_agent.send_message("action", turn_direction)
							await asyncio.sleep(0.2)  # Short delay between commands
							
							# If the turn is significant, might need to slow down
							if turn_degrees > 30:
								# Temporarily slow down for sharper turns
								await self.a_agent.send_message("action", "stop")
								await asyncio.sleep(0.1)
								
								# Execute additional turns if needed
								for _ in range(min(turns_needed-1, 2)):  # Limit to at most 3 turns at once
									await self.a_agent.send_message("action", turn_direction)
									await asyncio.sleep(0.2)
								
								# Resume movement
								await self.a_agent.send_message("action", "mf")
							
							# Stop turning
							await self.a_agent.send_message("action", "nt")
					else:
						# No hits detected, continue moving forward
						# Occasionally send "nt" to ensure no residual turning
						if random.random() < 0.1:  # 10% chance each cycle
							await self.a_agent.send_message("action", "nt")
					
					# Small delay between sensor checks to avoid overwhelming the system
					await asyncio.sleep(0.1)
					
				else:
					print(f"Unknown state: {self.state}")
					self.state = self.STOPPED
					await asyncio.sleep(0.5)
		
		except asyncio.CancelledError:
			print("***** TASK Avoid CANCELLED")
			await self.a_agent.send_message("action", "stop")
			await self.a_agent.send_message("action", "nt")  # Ensure we're not turning
			self.state = self.STOPPED
