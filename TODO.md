[] implement the frame payload type

## Assumptions
[] action messages arrive in order

## Issues
[] Out-of-order reception: if node A sends (x1,x2,x3) to node B, and we are set to deliver all messages until success, then when node B receives out of order (x3,x1,x2), node A will be deadlocked, because it fails to deliver x1 and x2 and will keep retrying. The solution is to make use of the ABORTED state of the goal handle. Node B can accepts x1 if buffer allows, but because it finds x1 is out of order, it will abort it, and node A will be aware of that through result_callback.
[] Out-of-order delivery and reception: in local machine, we know there will be no message lost, so all nodes can be set to "no-lost-assumed" mode, and they will use tbb::sequencer_node to receive and send messages, using +1 increment frame number to order the messages. This only works in environment where message lost is not possible, otherwise all nodes will be deadlocked.