# Python 2.7

class QueueManager(object):
    def __init__(self):
        self.queue = []
        self.robot_pose = None
        self.minutes_per_person = 5

    def update_robot_pose(self, pose):
        self.robot_pose = pose

    def _eta_for_position(self, pos):
        return pos * self.minutes_per_person

    def add_person(self, nearest_landmark=None):
        position = len(self.queue) + 1
        self.queue.append(position)

        eta = self._eta_for_position(position)

        if nearest_landmark:
            wait_location = nearest_landmark["name"]
        else:
            wait_location = "Main Lobby"

        return eta, wait_location

    def status(self):
        count = len(self.queue)
        eta = self._eta_for_position(count) if count > 0 else 0

        return "People in queue: %d | Estimated wait: %d minutes" % (
            count, eta
        )

    def cancel(self):
        if self.queue:
            self.queue.pop(0)
            return "You have left the queue."
        return "Queue is empty."

