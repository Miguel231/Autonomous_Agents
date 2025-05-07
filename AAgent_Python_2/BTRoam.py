import asyncio
import random
import py_trees
import py_trees as pt
from py_trees import common
import Goals_BT
import Sensors


class BN_DoNothing(pt.behaviour.Behaviour):
    def __init__(self, aagent):
        self.my_agent = aagent
        self.my_goal = None
        # print("Initializing BN_DoNothing")
        super(BN_DoNothing, self).__init__("BN_DoNothing")

    def initialise(self):
        self.my_goal = asyncio.create_task(Goals_BT.DoNothing(self.my_agent).run())

    def update(self):
        if not self.my_goal.done():
            return pt.common.Status.RUNNING
        else:
            if self.my_goal.result():
                # print("BN_DoNothing completed with SUCCESS")
                return pt.common.Status.SUCCESS
            else:
                # print("BN_DoNothing completed with FAILURE")
                return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        # Finishing the behaviour, therefore we have to stop the associated task
        self.my_goal.cancel()


class BN_ForwardRandom(pt.behaviour.Behaviour):
    def __init__(self, aagent):
        self.my_goal = None
        # print("Initializing BN_ForwardRandom")
        super(BN_ForwardRandom, self).__init__("BN_ForwardRandom")
        self.logger.debug("Initializing BN_ForwardRandom")
        self.my_agent = aagent

    def initialise(self):
        self.logger.debug("Create Goals_BT.ForwardDist task")
        self.my_goal = asyncio.create_task(Goals_BT.ForwardDist(self.my_agent, -1, 1, 5).run())

    def update(self):
        if not self.my_goal.done():
            return pt.common.Status.RUNNING
        else:
            if self.my_goal.result():
                self.logger.debug("BN_ForwardRandom completed with SUCCESS")
                # print("BN_ForwardRandom completed with SUCCESS")
                return pt.common.Status.SUCCESS
            else:
                self.logger.debug("BN_ForwardRandom completed with FAILURE")
                # print("BN_ForwardRandom completed with FAILURE")
                return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        # Finishing the behaviour, therefore we have to stop the associated task
        self.logger.debug("Terminate BN_ForwardRandom")
        self.my_goal.cancel()


class BN_TurnRandom(pt.behaviour.Behaviour):
    def __init__(self, aagent):
        self.my_goal = None
        # print("Initializing BN_TurnRandom")
        super(BN_TurnRandom, self).__init__("BN_TurnRandom")
        self.my_agent = aagent

    def initialise(self):
        self.my_goal = asyncio.create_task(Goals_BT.Turn(self.my_agent).run())

    def update(self):
        if not self.my_goal.done():
            return pt.common.Status.RUNNING
        else:
            res = self.my_goal.result()
            if res:
                # print("BN_Turn completed with SUCCESS")
                return pt.common.Status.SUCCESS
            else:
                # print("BN_Turn completed with FAILURE")
                return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        # Finishing the behaviour, therefore we have to stop the associated task
        self.logger.debug("Terminate BN_TurnRandom")
        self.my_goal.cancel()

"""
class BN_DetectFlower(pt.behaviour.Behaviour):
    def __init__(self, aagent):
        self.my_goal = None
        # print("Initializing BN_DetectFlower")
        super(BN_DetectFlower, self).__init__("BN_DetectFlower")
        self.my_agent = aagent

    def initialise(self):
        pass

    def update(self):
        sensor_obj_info = self.my_agent.rc_sensor.sensor_rays[Sensors.RayCastSensor.OBJECT_INFO]
        for index, value in enumerate(sensor_obj_info):
            if value:  # there is a hit with an object
                if value["tag"] == "AlienFlower":  # If it is a flower
                    # print("Flower detected!")
                    # print("BN_DetectFlower completed with SUCCESS")
                    return pt.common.Status.SUCCESS
        # print("No flower...")
        # print("BN_DetectFlower completed with FAILURE")
        return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        pass
"""


class BN_DetectObject(pt.behaviour.Behaviour):
    def __init__(self, aagent, tag_object):
        self.my_goal = None
        super(BN_DetectObject, self).__init__("BN_Detect{tag_object}")
        self.my_agent = aagent
        self.tag_object=tag_object

    def initialise(self):
        pass

    def update(self):
        sensor_obj_info = self.my_agent.rc_sensor.sensor_rays[Sensors.RayCastSensor.OBJECT_INFO]
        for index, value in enumerate(sensor_obj_info):
            if value:  # there is a hit with an object
                if value["tag"] ==self.tag_object:  
                    return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

    def terminate(self, new_status: common.Status):
        pass


class BN_Roam(pt.behaviour.Behaviour):
	def __init__(self, aagent):
		self.my_goal = None
		super(BN_Roam, self).__init__("BN_Roam")
		self.logger.debug("Initializing BN_Roam")
		self.my_agent = aagent

	def initialise(self):
		self.logger.debug("Create Goals_BT.Avoid task")
		self.my_goal = asyncio.create_task(Goals_BT.Avoid(self.my_agent).run())

	def update(self):
		if not self.my_goal.done():
			return pt.common.Status.RUNNING
		else:
			if self.my_goal.result():
				self.logger.debug("BN_Roam completed with SUCCESS")
				# print("BN_Roam completed with SUCCESS")
				return pt.common.Status.SUCCESS
			else:
				self.logger.debug("BN_Roam completed with FAILURE")
				# print("BN_Roam completed with FAILURE")
				return pt.common.Status.FAILURE

	def terminate(self, new_status: common.Status):
		# Finishing the behaviour, therefore we have to stop the associated task
		self.logger.debug("Terminate BN_Roam")
		self.my_goal.cancel()


class BN_MoveToFlower(pt.behaviour.Behaviour):
	def __init__(self, aagent):
		self.my_goal = None
		super(BN_MoveToFlower, self).__init__("BN_MoveToFlower")
		self.logger.debug("Initializing BN_MoveToFlower")
		self.my_agent = aagent

	def initialise(self):
		self.logger.debug("Create Goals_BT.GetFlower task")
		self.my_goal = asyncio.create_task(Goals_BT.GetFlower(self.my_agent).run())

	def update(self):
		if not self.my_goal.done():
			return pt.common.Status.RUNNING
		else:
			if self.my_goal.result():
				self.logger.debug("BN_MoveToFlower completed with SUCCESS")
				# print("BN_MoveToFlower completed with SUCCESS")
				return pt.common.Status.SUCCESS
			else:
				self.logger.debug("BN_MoveToFlower completed with FAILURE")
				# print("BN_MoveToFlower completed with FAILURE")
				return pt.common.Status.FAILURE

	def terminate(self, new_status: common.Status):
		# Finishing the behaviour, therefore we have to stop the associated task
		self.logger.debug("Terminate BN_MoveToFlower")
		self.my_goal.cancel()



class BN_MoveToAstronaut(pt.behaviour.Behaviour):
	def __init__(self, aagent):
		self.my_goal = None
		super(BN_MoveToAstronaut, self).__init__("BN_MoveToAstronaut")
		self.logger.debug("Initializing BN_MoveToAstronaut")
		self.my_agent = aagent
		#self.tag_object=tag_object

	def initialise(self):
		self.logger.debug("Create Goals_BT.GetAstronaut task")
		self.my_goal = asyncio.create_task(Goals_BT.GetAstronaut(self.my_agent).run())

	def update(self):
		if not self.my_goal.done():
			return pt.common.Status.RUNNING
		else:
			if self.my_goal.result():
				self.logger.debug("BN_MoveToAstronaut completed with SUCCESS")
				return pt.common.Status.SUCCESS
			else:
				self.logger.debug("BN_MoveToAstronaut completed with FAILURE")
				return pt.common.Status.FAILURE

	def terminate(self, new_status: common.Status):
		# Finishing the behaviour, therefore we have to stop the associated task
		self.logger.debug("Terminate BN_MoveToAstronaut")
		self.my_goal.cancel()


class BN_RetreatAfterBite(pt.behaviour.Behaviour):
	def __init__(self, aagent):
		self.my_goal = None
		super(BN_RetreatAfterBite, self).__init__("BN_RetreatAfterBite")
		self.logger.debug("Initializing BN_MoveAway")
		self.my_agent = aagent
		#self.tag_object=tag_object

	def initialise(self):
		self.logger.debug("Create Goals_BT.MoveAway task")
		self.my_goal = asyncio.create_task(Goals_BT.MoveAway(self.my_agent).run())

	def update(self):
		if not self.my_goal.done():
			return pt.common.Status.RUNNING
		else:
			if self.my_goal.result():
				self.logger.debug("BN_RetreatAfterBite completed with SUCCESS")
				# print("BN_MoveToFlower completed with SUCCESS")
				return pt.common.Status.SUCCESS
			else:
				self.logger.debug("BN_RetreatAfterBite completed with FAILURE")
				# print("BN_MoveToFlower completed with FAILURE")
				return pt.common.Status.FAILURE

	def terminate(self, new_status: common.Status):
		# Finishing the behaviour, therefore we have to stop the associated task
		self.logger.debug("Terminate BN_MoveToFlower")
		self.my_goal.cancel()


class BN_AstronautBitten(pt.behaviour.Behaviour):
	def __init__(self, aagent):
		super(BN_AstronautBitten, self).__init__("BN_AtronautBitten")
		self.logger.debug("Initializing BN_AtronautBitten")
		self.my_agent = aagent

	def initialise(self):
		pass

	def update(self):
		if Goals_BT.is_astronaut_bitten():
			return pt.common.Status.SUCCESS
		return pt.common.Status.FAILURE

	def terminate(self, new_status: common.Status):
		# Finishing the behaviour, therefore we have to stop the associated task
		self.logger.debug("Terminate BN_AtronautBitten")


class BN_Full(pt.behaviour.Behaviour):
	def __init__(self, aagent):
		super(BN_Full, self).__init__("BN_Full")
		self.logger.debug("Initializing BN_Full")
		self.my_agent = aagent

	def initialise(self):
		pass

	def update(self):
		flower_count = Goals_BT.flower_count(self.my_agent.i_state)
		if flower_count >= 2:
			return pt.common.Status.SUCCESS
		return pt.common.Status.FAILURE

	def terminate(self, new_status: common.Status):
		# Finishing the behaviour, therefore we have to stop the associated task
		self.logger.debug("Terminate BN_Full")


class BN_ReturnAndUnload(pt.behaviour.Behaviour):
	def __init__(self, aagent):
		self.my_goal = None
		super(BN_ReturnAndUnload, self).__init__("BN_ReturnBase")
		self.logger.debug("Initializing BN_ReturnBase")
		self.my_agent = aagent

	def initialise(self):
		self.logger.debug("Create Goals_BT.ReturnBase task")
		self.my_goal = asyncio.create_task(Goals_BT.ReturnAndUnload(self.my_agent).run())

	def update(self):
		if not self.my_goal.done():
			return pt.common.Status.RUNNING
		else:
			if self.my_goal.result():
				self.logger.debug("BN_ReturnBase completed with SUCCESS")
				# print("BN_ReturnBase completed with SUCCESS")
				return pt.common.Status.SUCCESS
			else:
				self.logger.debug("BN_ReturnBase completed with FAILURE")
				# print("BN_ReturnBase completed with FAILURE")
				return pt.common.Status.FAILURE

	def terminate(self, new_status: common.Status):
		# Finishing the behaviour, therefore we have to stop the associated task
		self.logger.debug("Terminate BN_ReturnBase")
		self.my_goal.cancel()

"""
class BTRoam:
    def __init__(self, aagent):
        # py_trees.logging.level = py_trees.logging.Level.DEBUG

        self.aagent = aagent

        # VERSION 1
        # self.root = pt.composites.Sequence(name="Sequence", memory=True)
        # self.root.add_children([BN_TurnRandom(aagent),
        #                         BN_ForwardRandom(aagent),
        #                         BN_DoNothing(aagent)])

        # VERSION 2
        self.root = pt.composites.Parallel("Parallel", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
        self.root.add_children([BN_ForwardRandom(aagent), BN_TurnRandom(aagent)])

        # VERSION 3 (with DetectFlower)
        # detection = pt.composites.Sequence(name="DetectFlower", memory=True)
        # detection.add_children([BN_DetectFlower(aagent), BN_DoNothing(aagent)])
        #
        # roaming = pt.composites.Parallel("Parallel", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
        # roaming.add_children([BN_ForwardRandom(aagent), BN_TurnRandom(aagent)])
        #
        # self.root = pt.composites.Selector(name="Selector", memory=False)
        # self.root.add_children([detection, roaming])

        self.behaviour_tree = pt.trees.BehaviourTree(self.root)

    # Function to set invalid state for a node and its children recursively
    def set_invalid_state(self, node):
        node.status = pt.common.Status.INVALID
        for child in node.children:
            self.set_invalid_state(child)

    def stop_behaviour_tree(self):
        # Setting all the nodes to invalid, we force the associated asyncio tasks to be cancelled
        self.set_invalid_state(self.root)

    async def tick(self):
        self.behaviour_tree.tick()
        await asyncio.sleep(0)
"""

class AstronautBT:
	def __init__(self, aagent):
		# py_trees.logging.level = py_trees.logging.Level.DEBUG

		self.aagent = aagent
		
		unloading = pt.composites.Sequence(name="Unload Sequence", memory=False)
		unloading.add_children([BN_Full(aagent), BN_ReturnAndUnload(aagent)])

		get_flower = pt.composites.Sequence(name="Get Flower", memory=False)
		get_flower.add_children([BN_DetectObject(aagent, "AlienFlower"), BN_MoveToFlower(aagent)])

		roaming = pt.composites.Selector(name="Roaming", memory=False)
		roaming.add_children([get_flower, BN_Roam(aagent)])

		self.root = pt.composites.Selector(name="Selector", memory=False)
		self.root.add_children([unloading, roaming])

		self.behaviour_tree = pt.trees.BehaviourTree(self.root)

	# Function to set invalid state for a node and its children recursively
	def set_invalid_state(self, node):
		node.status = pt.common.Status.INVALID
		for child in node.children:
			self.set_invalid_state(child)

	def stop_behaviour_tree(self):
		# Setting all the nodes to invalid, we force the associated asyncio tasks to be cancelled
		self.set_invalid_state(self.root)
		print("Stopping behaviour tree")

	async def tick(self):
		self.behaviour_tree.tick()
		await asyncio.sleep(0)


class CritterBT:
	def __init__(self, aagent):

		self.aagent = aagent
		
		move_away = pt.composites.Sequence(name="Move Away Sequence", memory=False)
		move_away.add_children([BN_AstronautBitten(aagent), BN_RetreatAfterBite(aagent)])

		get_astronaut = pt.composites.Sequence(name="Get Astronaut", memory=False)
		get_astronaut.add_children([BN_DetectObject(aagent, "Astronaut"), BN_MoveToAstronaut(aagent)])

		self.root = pt.composites.Selector(name="Selector", memory=False)
		self.root.add_children([move_away, get_astronaut, BN_Roam(aagent)])


		self.behaviour_tree = pt.trees.BehaviourTree(self.root)

	# Function to set invalid state for a node and its children recursively
	def set_invalid_state(self, node):
		node.status = pt.common.Status.INVALID
		for child in node.children:
			self.set_invalid_state(child)

	def stop_behaviour_tree(self):
		# Setting all the nodes to invalid, we force the associated asyncio tasks to be cancelled
		self.set_invalid_state(self.root)
		print("Stopping behaviour tree")

	async def tick(self):
		self.behaviour_tree.tick()
		await asyncio.sleep(0)