# Python 2.7 (PLACEHOLDER)

class QueueManager(object):
    def __init__(self):
        self.queue = []
        self.robot_pose = None

    def update_robot_pose(self, pose):
        self.robot_pose = pose

    def add_person(self):
        # (PLACEHOLDER) ETA calculation
        position = len(self.queue) + 1
        eta = position * 5  # 5 minutes per tour placeholder
        wait_location = "Lobby"
        self.queue.append(position)
        return eta, wait_location

    def status(self):
        return "Queue length: %d" % len(self.queue)

    def cancel(self):
        if self.queue:
            self.queue.pop(0)
            return "Removed first person in line."
        return "Queue empty."

