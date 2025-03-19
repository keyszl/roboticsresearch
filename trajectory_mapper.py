import sys, datetime, threading
import runner
import trajectories
import rclpy

import ir_bump_turn_odom


class TrajectoryMapper(ir_bump_turn_odom.IrBumpTurnBot):
    def __init__(self, namespace, ir_limit):
        super().__init__(namespace, ir_limit)
        self.reset_odom()
        self.map = trajectories.TrajectoryMap()

    def timer_callback(self):
        super().timer_callback()
        p = self.last_x_y()
        if p is not None:
            self.map.update(p[0], p[1])


if __name__ == '__main__':
    rclpy.init()
    ir_limit = 50 if len(sys.argv) < 3 else int(sys.argv[2])

    bot = TrajectoryMapper(f'/{sys.argv[1]}', ir_limit)

    executor = rclpy.executors.MultiThreadedExecutor()
    bot.add_self_recursive(executor)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    quit = False
    while not quit:
        print("Options")
        print("1: Pause")
        print("2: Pause and name location")
        print("3: Resume")
        print("4: Quit")
        option = input("Enter choice: ")
        if option.isdigit():
            option = int(option)
            if option <= 2:
                bot.pause()
                if option == 2:
                    name = input("Enter location name: ")
                    if len(name) > 0:
                        bot.map.assign_location_name(name)
                        print(f"{name} assigned to {bot.map.current}")
                    else:
                        print("No name assignment performed")
            elif option == 3:
                bot.resume()
            elif option == 4:
                quit = True
    rclpy.shutdown()
    executor_thread.join()
    with open(f"trajectory_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}", 'w') as file:
        file.write(f"{bot.map}\n")
