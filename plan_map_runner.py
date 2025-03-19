import pickle, curses, threading, queue, sys
import rclpy
from nav_msgs.msg import Odometry
from runner import GoToNode, drain_queue, spin_thread


def print_odometry(stdscr, msg: Odometry):
    p = msg.pose.pose.position
    h = msg.pose.pose.orientation
    stdscr.addstr(2, 0, f"Position:    ({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})")
    stdscr.addstr(3, 0, f"Orientation: ({h.x:6.2f}, {h.y:6.2f}, {h.z:6.2f}, {h.w:6.2f})")


def main(stdscr):
    with open(sys.argv[2], 'rb') as f:
        map_data = pickle.load(f)
        run_robot_map(stdscr, sys.argv[1], map_data)


def run_robot_map(stdscr, robot, map_data):
    map_graph = map_data.square_graph()
    stdscr.nodelay(True)
    stdscr.clear()
    curses.curs_set(0)

    finished = threading.Event()
    ros_ready = threading.Event()
    active = threading.Event()
    pos_queue = queue.Queue()
    cmd_queue = queue.Queue()
    status_queue = queue.Queue()
    current_input = ''
    
    st = threading.Thread(target=spin_thread, args=(finished, ros_ready, lambda: GoToNode(pos_queue, cmd_queue, status_queue, active, robot)))
    st.start()

    current_location = None
    next_step = None
    goal = None

    stdscr.addstr(0, 0, '"quit" to quit, "stop" to stop, "go [name]" to go to a location; "reset" to reset odometry; "see [name]" to see coordinate')
    map_str = map_data.square_name_str()
    for i, line in enumerate(map_str.split('\n')):
        stdscr.addstr(8 + i, 0, line)
    stdscr.refresh()
    
    while True:
        try:
            k = stdscr.getkey()
            curses.flushinp()
            if k == '\n':
                if current_input == 'quit':
                    break
                elif current_input.startswith('see'):
                    parts = current_input.split()
                    if len(parts) >= 2:
                        stdscr.addstr(5, 0, f"{map_graph.node_value(parts[1])}" if parts[1] in map_graph else 'Unrecognized')
                    else:
                        stdscr.addstr(5, 0, "see what?")
                elif current_input == 'stop':
                    drain_queue(cmd_queue)
                    active.clear()
                elif current_input == 'reset':
                    drain_queue(cmd_queue)
                    active.clear()
                    cmd_queue.put('reset')
                elif current_input.startswith("go"):
                    parts = current_input.split()
                    if len(parts) >= 2:
                        if parts[1] in map_graph:
                            goal = parts[1]
                            next_step = map_graph.next_step_from_to(current_location, goal)
                            cmd_queue.put(map_graph.node_value(next_step))
                            stdscr.addstr(5, 0, f'sent request "{current_input}"                ')
                        else:
                            stdscr.addstr(5, 0, f'Unknown location: {parts[1]}')
                    else:
                        stdscr.addstr(5, 0, 'go where?')
                else:
                    stdscr.addstr(5, 0, f'Unrecognized input: "{current_input}"')
                current_input = ''
            elif k == '\b':
                current_input = current_input[:-1]
            else:
                current_input += k
        except curses.error as e:
            if str(e) != 'no input':
                stdscr.addstr(5, 0, traceback.format_exc())

        if ros_ready.is_set():
            stdscr.addstr(4, 0, "ROS2 ready")

        p = drain_queue(pos_queue)
        if p:
            print_odometry(stdscr, p)
            current_location, _ = map_graph.closest_node(p.pose.pose.position.x, p.pose.pose.position.y)

        s = drain_queue(status_queue)
        if s:
            stdscr.addstr(6, 0, f"{s}                                                ")
            if s == 'Stopping':
                if current_location != goal:
                    next_step = map_graph.next_step_from_to(current_location, goal)
                    cmd_queue.put(map_graph.node_value(next_step))
                    stdscr.addstr(5, 0, f'Sent next step: {next_step}')
        stdscr.addstr(1, 0, f"> {current_input}                                 ")
        stdscr.addstr(7, 0, f"{'active  ' if active.is_set() else 'inactive'}")
        stdscr.addstr(4, 0, f"@{current_location}; heading to {goal} via {next_step}        ")
        stdscr.refresh()
    finished.set()
    st.join()
    

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python3 plan_map_runner.py robot pickled_map_file")
    else:
        curses.wrapper(main)
