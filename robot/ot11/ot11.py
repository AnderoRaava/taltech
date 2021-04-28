"""OT11 - World model."""
import math
import PiBot


class Robot:
    """The robot class."""

    def __init__(self, initial_odometry=[0, 0, 0]):
        """
        Initialize variables.

        Arguments:
          initial_odometry -- Initial odometry(start position and angle),
                              [x, y, yaw] in [meters, meters, radians]
        """
        self.robot = PiBot.PiBot()
        self.objects = None
        self.fov = self.robot.CAMERA_FIELD_OF_VIEW
        self.resolution = self.robot.CAMERA_RESOLUTION
        self.left_encoder = []
        self.right_encoder = []
        self.current_time = 0
        self.previous_time = 0
        self.odometry = initial_odometry
        self.all_objects = {}

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set the API reference."""
        self.robot = robot

    def calculate_odometry(self):
        """Calculate encoder odometry."""
        time = self.current_time - self.previous_time
        speed_sum = self.right_wheel() + self.left_wheel()
        x = self.robot.WHEEL_DIAMETER / 4 * speed_sum * math.cos(self.odometry[2])
        y = self.robot.WHEEL_DIAMETER / 4 * speed_sum * math.sin(self.odometry[2])
        self.odometry[0] += x * time
        self.odometry[1] += y * time

    def left_wheel(self):
        """Get wheel speed."""
        if self.current_time - self.previous_time != 0:
            return math.radians((self.left_encoder[-1] - self.left_encoder[-2])
                                / (self.current_time - self.previous_time))
        return 0

    def right_wheel(self):
        """Get wheel speed."""
        if self.current_time - self.previous_time != 0:
            return math.radians((self.right_encoder[-1] - self.right_encoder[-2])
                                / (self.current_time - self.previous_time))
        return 0

    def update_world(self) -> None:
        """Update the world model (insert objects into memory)."""
        # If robot doesn't see any object, return None
        if self.objects is None or self.objects == []:
            return None
        # If robot sees objects, add all object with absolute coordinates and current distance between object and robot.
        else:
            for o in self.objects:
                x = self.odometry[0] + o[1][0]
                y = self.odometry[1] + o[1][1]
                r = math.sqrt((o[1][0]) ** 2 + (o[1][1]) ** 2)
                self.all_objects[o[0]] = [x, y, r]

    def get_closest_object_angle(self) -> float:
        """
        Return the angle of the closest object.

        This method returns the angle of the object that is
        the shortest Euclidean distance
        (i.e., the closest object angle w.r.t. the robot heading).

        Returns:
          The normalized (range [0..2*pi]) angle (radians) to the
          closest object w.r.t. the robot heading following
          the right-hand rule.
          E.g., 0 if object is straight ahead,
                1.57 if the object is 90 degrees to the left of the robot.
                3.14 if the closest object is 180 degrees from the robot.
                4.71 if the objectis 90 degrees to the right of the robot.
          None if no objects have been detected.
        """
        # Add all new objects to all objects list.
        self.update_world()

        # If list is empty return None
        if len(self.all_objects.keys()) < 1:
            return None

        answer = None
        # Calculate new distance from object to robot
        for k in self.all_objects.keys():
            ob = self.all_objects[k]
            x = ob[0] - self.odometry[0]
            y = ob[1] - self.odometry[1]
            r = math.sqrt(x ** 2 + y ** 2)
            self.all_objects[k] = [ob[0], ob[1], r]

        # Find the shortest distance
        for b in self.all_objects.values():
            if answer is None:
                answer = b
            elif answer[2] > b[2]:
                answer = b

        # Calculate degree in radians to the nearest object
        x = answer[0] - self.odometry[0]
        y = answer[1] - self.odometry[1]
        if x == 0 and y > 0:
            a = math.pi/2
        elif x == 0 and y < 0:
            a = 1.5 * math.pi
        elif y == 0 and x > 0:
            a = 0
        elif y == 0 and x < 0:
            a = math.pi
        else:
            a = math.atan(y / x)
            if x < 0 and y > 0:
                a = math.pi - a
            if x < 0 and y < 0:
                a = math.pi + a
            if x > 0 and y < 0:
                a = 2 * math.pi - a
        return a

    def sense(self):
        """SPA architecture sense block."""
        self.objects = self.robot.get_camera_objects()
        self.current_time = self.robot.get_time()
        self.left_encoder.append(self.robot.get_left_wheel_encoder())
        self.right_encoder.append(self.robot.get_right_wheel_encoder())
        self.calculate_odometry()
        self.previous_time = self.current_time

    def spin(self):
        """Spin loop."""
        for _ in range(100):
            self.sense()
            print(f"objects = {self.robot.get_camera_objects()}")
            self.robot.sleep(0.05)


def main():
    """The main entry point."""
    robot = Robot()
    robot.spin()


if __name__ == "__main__":
    main()
