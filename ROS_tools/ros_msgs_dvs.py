import rosbag
import sys
if len(sys.argv) < 2:
  print("Usage: {} dataset_name".format(sys.argv[0]))
  exit(1)
file_name = sys.argv[1]
bag = rosbag.Bag('{}.bag'.format(file_name), 'r')
for topic, msg, t in bag.read_messages(topics=['/dvs/events']):
  print(msg)
bag.close()

