import json
import multiprocessing
import subprocess
import signal
import sys
import asyncio
from AAgent_BT import AAgent


def load_config(json_file):
    with open(json_file, 'r') as file:
        return json.load(file)


def start_agents(config_file):
    config = load_config(config_file)

    all_agents = []
    packs = config.get('packs', [])
    for pack in packs:
        agent_config_file = pack.get("agent_config_file", "")
        num_agents = pack.get("num_agents", 1)

        # Create multiple AAgent instances
        agents_in_pack = [AAgent(agent_config_file) for _ in range(num_agents)]

        all_agents.extend(agents_in_pack)

    async def run_all_agents():
        tasks = []
        for agent in all_agents:
            task = asyncio.create_task(agent.run())
            tasks.append(task)

        await asyncio.wait(tasks, return_when=asyncio.ALL_COMPLETED)

    try:
        asyncio.run(run_all_agents())
    except KeyboardInterrupt:
        print("Shutting down...")

    print("Bye!!!")


if __name__ == "__main__":
    if len(sys.argv) < 1:
        print("Usage: python Spawner.py <init_file.json>")
    else:
        init_file = sys.argv[1]
        start_agents(init_file)
