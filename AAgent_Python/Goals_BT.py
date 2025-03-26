import math
import random
import asyncio
import Sensors
from collections import Counter


class DoNothing:
    """
    Does nothing
    """
    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state = a_agent.i_state

    async def run(self):
        print("Doing nothing")
        await asyncio.sleep(1)
        return True

class ForwardStop:
    """
        Moves forward till it finds an obstacle. Then stops.
    """
    STOPPED = 0
    MOVING = 1
    END = 2

    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state = a_agent.i_state
        self.state = self.STOPPED

    async def run(self):
        try:
            while True:
                if self.state == self.STOPPED:
                    # Start moving
                    await self.a_agent.send_message("action", "mf")
                    self.state = self.MOVING
                elif self.state == self.MOVING:
                    sensor_hits = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]
                    if any(ray_hit == 1 for ray_hit in sensor_hits):
                        self.state = self.END
                        await self.a_agent.send_message("action", "stop")
                    else:
                        await asyncio.sleep(0)
                elif self.state == self.END:
                    break
                else:
                    print("Unknown state: " + str(self.state))
                    return False
        except asyncio.CancelledError:
            print("***** TASK Forward CANCELLED")
            await self.a_agent.send_message("action", "stop")
            self.state = self.STOPPED

class Turn:
    """
    The drone randomly selects a degree of turn between 10 and 360,
    along with a direction (left or right), and executes the turn accordingly.
    Upon completion, the drone selects a new turn and repeats the process.
    """

    def __init__(self, a_agent):
        self.a_agent = a_agent

    async def run(self):
        while True:  
            # Choose a random direction to turn
            turn_direction = random.choice(["tl", "tr"])
            turn_degrees = random.randint(10, 360)

            print(f"Turning {turn_degrees} degrees to the { 'left' if turn_direction == 'tl' else 'right' }")

            current_rotation = self.a_agent.i_state.rotation["y"]
            new_rotation = self.calculate_new_rotation(current_rotation, turn_degrees, turn_direction)

            print(f"Current rotation: {current_rotation}° -> New rotation: {new_rotation}°")

            turns_needed = turn_degrees // 5  #5 degrees per turn

            for _ in range(turns_needed):
                await self.a_agent.send_message("action", turn_direction)
                await asyncio.sleep(0.3)

            await self.a_agent.send_message("action", "nt")
            print("Turn completed. Stopping rotation.")

            await asyncio.sleep(2)

    def calculate_new_rotation(self, current_rotation, turn_degrees, direction):
        if direction == "tr":  #Right (more degrees)
            new_rotation = (current_rotation + turn_degrees) % 360
        else:  #Left (less degrees)
            new_rotation = (current_rotation - turn_degrees) % 360

        return new_rotation


