import socket
import sys
import keyboard
import time
import panda_py
import pickle

if len(sys.argv) != 4:
    raise ValueError("Provide robot ip, follower ip and follower port")

leader_robot = panda_py.Panda('172.22.2.3')
leader_robot.move_to_start()
leader_robot.teaching_mode(active = True)

ROBOT_IP = sys.argv[1]
FOLLOWER_IP = sys.argv[2]
FOLLOWER_PORT = int(sys.argv[3])

frequency = 10 #1 message a second

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print('socket running')
# print('press q to close')

while True:

    # if keyboard.is_pressed('q'):
    #     print('Exiting...')
    #     break
    leader_state = leader_robot.get_state()
    state_data = leader_state.q + leader_state.dq
    message = pickle.dumps(state_data)
    sock.sendto(message, (FOLLOWER_IP, FOLLOWER_PORT))

    time.sleep(1 / frequency)

sock.close()