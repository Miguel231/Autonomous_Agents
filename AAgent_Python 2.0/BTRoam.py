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
