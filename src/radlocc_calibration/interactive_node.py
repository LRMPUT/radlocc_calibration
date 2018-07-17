from __future__ import print_function

from collect_node import CollectNode

class InteractiveNode(CollectNode):

    def __init__(self, image_topic, laserscan_topic, data_saver):
        CollectNode.__init__(self, image_topic, laserscan_topic, data_saver)

    def start(self):
        print("Press <enter> to capture and <q> to exit")
        i = 0
        try:
            while raw_input("[{}] another one? ".format(i)) != 'q':
                self.run()
                i += 1
            self.finish()
        except KeyboardInterrupt:
            print("cancelled")
            

    