import cotask
import task_share
from Line_Follow import Follow_Class
import time
from pyb import Pin

class TaskManager:
    """
    Manages tasks for the Line Following robot.

    Attributes:
        fun1 (Follow_Class): An instance of the Follow_Class for line following.
        task1 (cotask.Task): Provided function that allows multiple tasks to run cooperatively.

    Methods:
        run_tasks(): Runs the tasks in the task list.

    """
    def __init__(self):
        self.fun1 = Follow_Class()
        self.task1 = cotask.Task(
            self.fun1.Follow(),
            name='Task 1',
            priority=1,
            period=100,
            profile=True,
            trace=True
        )

    def run_tasks(self):
        """
        Runs the tasks in the task list. For this application, there is only one task

        """
        cotask.task_list.append(self.task1)
        while True:
            try:
                cotask.task_list.pri_sched()
            except KeyboardInterrupt:
                break


if __name__ == "__main__":
    manager = TaskManager()
    manager.run_tasks()
