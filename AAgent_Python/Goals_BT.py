import math
import random
import asyncio
import Sensors
from collections import Counter


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

            turns_needed = turn_degrees // 5  # 5 degrees per step

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

    async def update(self):
        if self.state == self.STOPPED:
            self.state = random.choices([self.TURNING, self.STOP, self.MOVING], weights=(55, 10, 35), k=1)[0]
            print(f"New State: {self.state}")
            await asyncio.sleep(2)

        elif self.state == self.MOVING:
            await self.a_agent.send_message("action", "mf")
            print("MOVING")

            if any(ray_hit == 1 for ray_hit in self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]):
                print("Obstacle detected! Stopping and turning.")
                await self.a_agent.send_message("action", "stop")
                self.state = self.TURNING
            else:
                self.state = random.choices([self.TURNING, self.STOP, self.MOVING], weights=(55, 10, 35), k=1)[0]

            await asyncio.sleep(2)

        elif self.state == self.STOP:
            print("STOPPED")
            await self.a_agent.send_message("action", "stop")
            await asyncio.sleep(2)

            self.state = random.choices([self.TURNING, self.STOP, self.MOVING], weights=(55, 10, 35), k=1)[0]
            print(f"New State: {self.state}")
            await asyncio.sleep(0.1)

        elif self.state == self.TURNING:
            if self.turn_direction is None or self.num_turns is None:
                await self.a_agent.send_message("action", "stop")
                await asyncio.sleep(0.1)

                self.turn_direction = random.choice(["tl", "tr"])  
                turn_degrees = random.randint(10, 360)  
                self.num_turns = turn_degrees // 5  

                print(f"Turning {turn_degrees}° {'left' if self.turn_direction == 'tl' else 'right'}")

            for _ in range(self.num_turns):
                await self.a_agent.send_message("action", self.turn_direction)
                await asyncio.sleep(0.3)

            await self.a_agent.send_message("action", "nt")
            print("Turn completed.")

            self.turn_direction = None  
            self.num_turns = None  
            self.state = random.choices([self.TURNING, self.STOP, self.MOVING], weights=(55, 10, 35), k=1)[0]

            print(f"New State: {self.state}")
            await asyncio.sleep(0.1)

        else:
            print(f"Unknown state: {self.state}")
