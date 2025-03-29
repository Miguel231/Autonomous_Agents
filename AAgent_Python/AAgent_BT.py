import sys
from collections import deque

import aiohttp
import asyncio
import json
import Sensors
import Goals_BT

import tkinter as tk
from threading import Thread
import queue
import copy

# Agent TK GUI
gui_blackboard = queue.Queue()
exit_gui = False


# Agent TK GUI
class AAgent_interface:
    def __init__(self, aa_name):
        self.gui_root = tk.Tk()
        self.gui_root.title("AAGENT: " + aa_name)

        self.text = tk.Text(self.gui_root, width=40, height=20, wrap="word")
        self.text.pack(expand=True, fill="both")

        self.gui_root.geometry("600x400")
        self.update_values()

    def update_values(self):
        try:
            data_received = gui_blackboard.get_nowait()
            self.text.delete("1.0", tk.END)
            for key, value in data_received.items():
                self.text.insert(tk.END, f"{key}: {value}\n")
        except queue.Empty:
            pass
        self.gui_root.after(100, self.update_values)
        if exit_gui:
            self.gui_root.quit()

    def start(self):
        self.gui_root.mainloop()


class InternalState:
    """
    Internal state
        Stores the internal state of the agent.
            isRotatingRight: <bool>
            isRotatingLeft: <bool>
            movingForwards: <bool>
            movingBackwards: <bool>
            speed: <float> Speed of the agent
            position: <dict { "x": <float>, "y": <float>, "z": <float> } Position using world coordinates
            rotation: <dict { "x": <float>, "y": <float>, "z": <float> } Rotation y - Yaw, x - Pitch, z - Roll
    """

    def __init__(self):
        self.isRotatingRight = False
        self.isRotatingLeft = False
        self.movingForwards = False
        self.movingBackwards = False
        self.speed = 0.0
        self.position = {"x": 0, "y": 0, "z": 0}
        self.rotation = {"x": 0, "y": 0, "z": 0}

        self.emergency = False
        self.emergencyType = ""
        self.emergencyLoc = ""
        self.namedLoc = ""
        self.onRoute = False
        self.targetNamedLoc = ""
        self.nearbyContainerInventory = False
        self.nearbyContainerInventoryList = []

    def update_internal_state(self, i_state_dict):
        self.isRotatingRight = i_state_dict["isRotatingRight"]
        self.isRotatingLeft = i_state_dict["isRotatingLeft"]
        self.movingForwards = i_state_dict["movingForwards"]
        self.movingBackwards = i_state_dict["movingBackwards"]
        self.speed = i_state_dict["speed"]
        self.position = i_state_dict["position"]
        self.rotation = i_state_dict["rotation"]

        self.emergency = i_state_dict["emergency"]
        self.emergencyType = i_state_dict["emergencyType"]
        self.emergencyLoc = i_state_dict["emergencyLoc"]
        self.namedLoc = i_state_dict["namedLoc"]
        self.onRoute = i_state_dict["onRoute"]
        self.targetNamedLoc = i_state_dict["targetNamedLoc"]
        self.nearbyContainerInventory = i_state_dict["nearbyContainerInventory"]
        self.nearbyContainerInventoryList = i_state_dict["nearbyContainerInventoryList"]

        # Agent TK GUI
        if gui_blackboard.empty():
            gui_blackboard.put(copy.deepcopy(i_state_dict))


class AAgent:
    # Constants that define the state of the simulation
    ON_HOLD = 0
    RUNNING = 1

    def __init__(self, config_file_path: str):
        # Read the agent configuration file and put the info in the 'config' dictionary.
        with open(config_file_path, 'r') as file:
            config_data = file.read()
            self.config = json.loads(config_data)
        # Extract the parameters of the agent from the config dictionary
        self.AgentParameters = self.config['AgentParameters']
        self.python_gui_monitor = self.config['Misc']['python_gui_monitor']

        # URL to connect with Unity
        self.url = f"ws://{self.config['Server']['host']}:{self.config['Server']['port']}/"

        # Agent sensors
        self.rc_sensor = Sensors.RayCastSensor(self.AgentParameters['ray_perception_sensor_param'])

        # Agent internal state
        self.i_state = InternalState()

        # Misc. variables
        # variables used for the websocket connection
        self.session = None
        self.ws = None
        # State of the simulation: ON_HOLD | RUNNING
        self.simulation_state = self.ON_HOLD
        # Asyncio exit event used to notify the tasks that they have to finish
        self.exit_event = asyncio.Event()
        # Flag that confirms the connection with Unity is fully operative and that Unity is waiting for messages
        self.connection_ready = False

        # Reference to the possible goals the agent can execute
        self.goals = {
            "DoNothing": Goals_BT.DoNothing(self),
            "ForwardStop": Goals_BT.ForwardStop(self),
            "Turn": Goals_BT.Turn(self),
            "RandomRoam": Goals_BT.RandomRoam(self),
            "Avoid": Goals_BT.Avoid(self),
        }

        # Reference to the possible behaviour trees the agent can execute
        self.bts = {
        }

        # Active goal
        self.currentGoal = None

        # Active behaviour tree
        self.currentBT = None

        # Individual actions pending execution
        self.pendingActions = deque()

    async def open_websocket(self):
        """
        Establishes the connection with Unity using a websocket. After that, it sends the initial parameters of the
        agent, obtained previously from the configuration file.
        """
        try:
            self.session = aiohttp.ClientSession()
            print("Connecting to: " + self.url)
            self.ws = await self.session.ws_connect(self.url)
            print("Connected to WebSocket server")
            param_json = json.dumps(self.AgentParameters)
            print("Sending the initial parameters: " + param_json)
            await self.send_message("initial_params", param_json)
        except aiohttp.ClientError:
            print("Failed connection")
            self.exit_event.set()

    async def close_websocket(self):
        """
        Properly close the websocket connection.
        """
        if self.ws:
            await self.ws.close()
        if self.session:
            await self.session.close()
        print("WebSocket connection properly closed")

    async def send_message(self, msg_type: str, msg_content: str):
        """
        Sends a message in json format of type 'msg_type' and with content 'msg_content' to Unity
        :param msg_type: General type of the message.
        :param msg_content: Content of the message
        """
        msg = {"type": msg_type, "content": msg_content}
        msg_json = json.dumps(msg)
        if msg_type == "action":
            print(msg_content)
        await self.ws.send_str(msg_json)

    async def receive_messages(self):
        """
        Gets the messages that arrive from Unity through the websocket. If the message is not a 'close' message or
        an error, it calls the function 'process_incoming_message() to process it.
        """
        try:
            # With this loop, we will repeatedly await the next value produced by iterating over self.ws.
            # At each iteration, the event loop will suspend execution until a new value becomes available
            # from self.ws. The loop continues iterating over self.ws till the websocket is closed.
            async for msg in self.ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    self.process_incoming_message(msg.data)
                elif msg.type == aiohttp.WSMsgType.CLOSED:
                    print("Connection closed by Unity")
                    break
                elif msg.type == aiohttp.WSMsgType.ERROR:
                    print(f"WebSocket connection closed with error: {self.ws.exception()}")
                    break
        except Exception as e:
            print(f"Connection failed: {e}")
        finally:
            print("Finishing receive_messages")
            self.exit_event.set()

    def process_incoming_message(self, msg_data: str):
        """
        Processes the message 'msg_data' received from Unity. It is expected to be in json format.
        :param msg_data: Message received in json format.
        """
        try:
            msg_dict = json.loads(msg_data)

            if msg_dict["Type"] == "sensor":
                self.rc_sensor.set_perception(msg_dict["Content"][0])
                self.i_state.update_internal_state(msg_dict["Content"][1])
            elif msg_dict["Type"] == "sim_control":
                if msg_dict["Content"] == "connection_ready":
                    self.connection_ready = True
                elif msg_dict["Content"] == "on_hold":
                    self.simulation_state = self.ON_HOLD
                    print("ON HOLD")
                elif msg_dict["Content"] == "start":
                    self.simulation_state = self.RUNNING
                    print("RUNNING")
                elif msg_dict["Content"] == "error":
                    print("Error creating the agent in Unity.")
                    self.exit_event.set()
                else:
                    print("Received unknown message - Type: " + msg_dict["Type"] + "- Content: " + msg_dict["Content"])
            elif msg_dict["Type"] == "agent_control":
                # These kind of messages have the format
                # command:data
                try:
                    command, data = msg_dict["Content"].split(":")
                    if command == "action":
                        if self.currentBT:  # If there is a BT running
                            self.bts[self.currentBT].stop_behaviour_tree()
                            self.currentBT = None
                        if self.currentGoal:  # If there is a single Goal running
                            self.currentGoal = None
                        self.pendingActions.append(data)
                    elif command == "goal":
                        self.pendingActions.append("stop")  # Just in case there is translation action running
                        self.pendingActions.append("nt")  # Just in case there is rotation action running
                        if self.currentBT:  # If there is a BT running
                            self.bts[self.currentBT].stop_behaviour_tree()
                            self.currentBT = None
                        self.currentGoal = data
                    elif command == "bt":
                        self.pendingActions.append("stop")  # Just in case there is an action running
                        self.pendingActions.append("nt")  # Just in case there is rotation action running
                        if self.currentGoal:  # If there is a single Goal running
                            self.currentGoal = None
                        self.currentBT = data
                    else:
                        print("Agent_control message with an unknown command: " + msg_dict["content"])
                except Exception as e:
                    print(f"Exception1: {e}")
                    print(f"Message: {msg_data}")
            else:
                print("Received unknown message - Type: " + msg_dict["Type"] + "- Content: " + msg_dict["Content"])
        except json.JSONDecodeError:
            print(f"Failed JSON decoding of the received message: {msg_data}")
        except Exception as e:
            print(f"Exception2: {e}")
            raise e

    async def main_loop(self):
        # Keep going while there is not an event to exit
        while not self.exit_event.is_set():
            # Control if we are on hold (simulation paused from Unity)
            if self.simulation_state == self.ON_HOLD:
                # Just wait, but allow the other tasks keep running
                await asyncio.sleep(0)
            else:
                # Here is where we perform the agent actions
                # It can be the case we are executing a single action, a simple goal or a behaviour tree
                try:
                    if len(self.pendingActions) > 0:
                        # Single action
                        await self.send_message("action", self.pendingActions.popleft())
                    elif self.currentGoal:
                        # We are running a simple goal
                        await self.goals[self.currentGoal].run()
                    elif self.currentBT:
                        # We are running a behaviour tree
                        await self.bts[self.currentBT].tick()
                    else:
                        await asyncio.sleep(0)
                except Exception as e:
                    print("Execution failed.")
                    print(f"Exception3: {e}")
                    self.exit_event.set()
        print("Finishing main_loop")

    async def run(self):
        try:
            # Create the connection task, that will manage the connection with Unity,
            # and the exit_event task, that will be used to exit if there is an error
            connect_task = asyncio.create_task(self.open_websocket())
            awaited_exit_event = asyncio.create_task(self.exit_event.wait())

            # Wait for the connection with Unity to be ready or the exit event, what comes first
            await asyncio.wait([connect_task, awaited_exit_event], return_when=asyncio.FIRST_COMPLETED)

            if not self.exit_event.is_set():
                # Now that the connection is established, create the task to start receiving messages from Unity
                # We are not awaiting this task because it has to run forever till the main loop finishes
                asyncio.create_task(self.receive_messages())
                # Wait for the flag "connection_ready" to be True. If it is true, it means we have received an ack
                # from Unity saying that the connection is fully established and Unity is ready to receive messages
                while not self.connection_ready:
                    await asyncio.sleep(0)
                print("Connection with Unity fully established")

                # We are ready now  to start the main loop of the agent
                await self.main_loop()
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt received! Initiating shutdown...")
            self.exit_event.set()
        except Exception as e:
            print(f"Unexpected error: {e}")
            self.exit_event.set()
        finally:
            # Clean the websocket connection
            await self.close_websocket()
            print("Connection with Unity closed")

# Agent TK GUI
def run_tk(aa_name):
    app = AAgent_interface(aa_name)
    app.start()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python AAgent_BT.py <init_file.json>")
    else:
        # Get the name of the file with the initial parameters
        init_file = sys.argv[1]

        # Creates an instance of the AAgent_Python class.
        my_AAgent = AAgent(init_file)

        # Agent TK GUI
        # Start the python gui monitor interface
        if my_AAgent.python_gui_monitor:
            agent_name = my_AAgent.AgentParameters["name"]
            tk_thread = Thread(target=run_tk, args=(agent_name,))
            tk_thread.start()

        # Run the AAgent. It creates a new event loop, runs the my_AAgent.run()
        # coroutine in that event loop, and then closes the event loop when the coroutine completes.
        asyncio.run(my_AAgent.run())

        # Close the agent TK GUI
        if my_AAgent.python_gui_monitor:
            exit_gui = True
            tk_thread.join()

        print("Bye!!!")
