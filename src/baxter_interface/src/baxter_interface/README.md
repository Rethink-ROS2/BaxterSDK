# baxter_interface migration

## QOS Profiles

ROS 1

```python
# No QoS profiles in ROS 1, but you can set queue_size for publishers and subscribers
pub = rospy.Publisher('chatter', String, queue_size=10)
sub = rospy.Subscriber('chatter', String, callback, queue_size=10)
```

ROS 2
https://docs.ros.org/en/kilted/Concepts/Intermediate/About-Quality-of-Service-Settings.html

```python
# In ROS 2, you can specify QoS profiles for publishers and subscribers
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

state_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

pub = node.create_publisher(String, 'chatter', state_qos)
sub = node.create_subscription(String, 'chatter', callback, state_qos)

```

## Publishers
ROS 1

```python
pub = rospy.Publisher('chatter', String)
# or
pub = rospy.Publisher('chatter', String, queue_size=10)
```

ROS 2

```python
pub = node.create_publisher(String, 'chatter', rclpy.qos.QoSProfile())
# or
pub = node.create_publisher(String, 'chatter', 10)
```

---

### Subscribers
ROS 1

```python
sub = rospy.Subscriber('chatter', String, callback)
# or
sub = rospy.Subscriber('chatter', String, callback, queue_size=10)
```

ROS 2

```python
sub = node.create_subscription(String, 'chatter', callback, rclpy.qos.QoSProfile())
# or
sub = node.create_subscription(String, 'chatter', callback, 10)
```

---

### Services

```python
srv = rospy.Service('add_two_ints', AddTwoInts, add_two_ints_callback)
```

ROS 2

```python
srv = node.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)
```

---

#### Service Clients

```python
rospy.wait_for_service('add_two_ints')
add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
resp = add_two_ints(req)
```

ROS 2

```python
add_two_ints = node.create_client(AddTwoInts, 'add_two_ints')
while not add_two_ints.wait_for_service(timeout_sec=1.0):
    node.get_logger().info('service not available, waiting again...')
resp = add_two_ints.call_async(req)
rclpy.spin_until_future_complete(node, resp)
```
