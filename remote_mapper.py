# sudo chmod 666 /dev/video*

import threading, subprocess, curses, pickle, datetime

from pyhop_anytime import *
from anyhop_ros.curses_vision_demo import video_capture, display_frame

from queue import Queue

from geometry_msgs.msg import Pose
from runner import RemoteNode, drain_queue, spin_thread_simpler
from occupancy_grid import PathwayGrid


def reset_pos(bot):
    call = f'ros2 service call /{bot}/reset_pose irobot_create_msgs/srv/ResetPose '
    call += '"{pose:{position:{x: 0.0, y: 0.0, z: 0.0}, orientation:{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"'
    return subprocess.run(call, shell=True, capture_output=True)


# From: https://stackoverflow.com/questions/21784625/how-to-input-a-word-in-ncurses-screen
def my_raw_input(stdscr, row, col, prompt_string):
    curses.echo()
    stdscr.addstr(row, col, prompt_string)
    stdscr.refresh()
    text = stdscr.getstr(row, col + 1 + len(prompt_string), 20)
    return text


class Runner:
    def __init__(self, stdscr):
        self.map = PathwayGrid(0.5)
        self.bot = sys.argv[1]

        self.height, self.width = stdscr.getmaxyx()

        self.info_window = curses.newwin(8, self.width, 0, 0)
        self.image_window = curses.newwin(self.height - 10, self.width, 10, 0)
        self.robot_threaddscr = stdscr

        self.running = threading.Event()
        self.cmd_queue = Queue()
        self.pos_queue = Queue()
        self.bump_queue = Queue()
        self.image_queue = Queue()
        self.ir_queue = Queue()
        self.last_cmd = None
        self.last_time = None

    def main_loop(self):
        self.running.set()
        self.robot_thread = threading.Thread(target=spin_thread_simpler,
                                             args=(self.running, lambda: RemoteNode(self.cmd_queue, self.pos_queue,
                                                                                    self.ir_queue, self.bump_queue,
                                                                                    f"/{self.bot}")))

        self.info_window.addstr(0, 0, 'WASD to move; R to reset position; Q to quit')
        self.info_window.refresh()

        self.robot_threaddscr.nodelay(True)

        self.capture_thread = threading.Thread(target=video_capture, args=(self.running, self.image_queue, 0), daemon=True)

        self.robot_thread.start()
        self.capture_thread.start()

        while self.running.is_set(): 
            try:
                self.handle_key()
            except curses.error:
                pass
            self.handle_position()
            self.handle_image()

        self.capture_thread.join()
        self.robot_thread.join()
    
        with open(f"map_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}", 'wb') as file:
            pickle.dump(self.map, file)

    def handle_image(self):
        frame = drain_queue(self.image_queue)
        if frame is not None:
            display_frame(frame, self.image_window)
            self.image_window.refresh()

    def handle_key(self):
        k = self.robot_threaddscr.getkey()
        curses.flushinp()
        if k == 'q':
            self.running.clear()
        elif k == 'r':
            self.info_window.addstr(1, 0, f"Waiting for reset...{' ' * 30}")
            result = reset_pos(self.bot)
            if result.returncode == 0:
                self.info_window.addstr(1, 0, "Reset complete.     ")
            else:
                self.info_window.addstr(1, 0, "Trouble with reset. ")
            self.info_window.refresh()
        else:
            self.cmd_queue.put(k)
            if self.last_time is not None:
                self.last_cmd = f"Sent {k}: {self.last_time:.2f} s          "
            else:
                self.last_cmd = f"Sent {k}: no timestamp"

    def handle_position(self):
        self.info_window.addstr(0, 0, 'WASD to move; R to reset position; Q to quit')
        time_update = False
        ack = None
        pose = None
        while not self.pos_queue.empty():
            pos = self.pos_queue.get()
            if type(pos) == float:
                self.last_time = pos
                time_update = True
            elif type(pos) == str:
                ack = f"Executed {pos}: {self.last_time:.2f} s"
            elif type(pos) == Pose:
                pose = pos
        if time_update:
            if self.last_cmd is not None:
                self.info_window.addstr(1, 0, self.last_cmd)
            self.info_window.addstr(2, 0, f"{self.last_time:.2f} s")
        if ack:
            self.info_window.addstr(1, 0, ack)
            self.last_cmd = None
        if pose:
            p = pose.position
            h = pose.orientation
            self.info_window.addstr(3, 0, f"Position:    ({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})        ")
            self.info_window.addstr(4, 0, f"Orientation: ({h.x:6.2f}, {h.y:6.2f}, {h.z:6.2f}, {h.w:6.2f})        ")
            self.map.visit(p.x, p.y)

        ir = drain_queue(self.ir_queue)
        if ir:
            self.info_window.addstr(5, 0, f"ir: {ir}{' ' * 20}")
        bump = drain_queue(self.bump_queue)
        if bump:
            self.info_window.addstr(6, 0, f"{bump}{' ' * 20}")
        self.info_window.refresh()

        
def run_runner(stdscr):
    r = Runner(stdscr)
    curses.curs_set(0)
    stdscr.clear()
    r.main_loop()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: remote_mapper robot_name")
    else:
        curses.wrapper(run_runner)
